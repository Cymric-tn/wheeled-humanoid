// Arduino Mega 2560: hardware layer of the wheeled humanoid.
// Reads every sensor, runs RobotCore every 10 ms, writes the motor drivers and the servo driver.
// All decisions (profiles, obstacle stop, gestures, safety) are in lib/robot_core, which is the
// same code the host tests and the digital twin run.
#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_PWMServoDriver.h>
#include "hw_config.h"
#include "robot_core.h"

static RobotCore core;
static CoreInputs in;
static CoreOutputs out;
static VL53L0X tof[2];
static bool tofOk[2] = {false, false};
static Adafruit_PWMServoDriver pca(PCA_ADDR);
static bool pcaOk = false, mpuOk = false;
static float gyroBias = 0;
static uint16_t servoTicks[8];          // last value written to each channel (0xFFFF = off)
static bool oeEnabled = false;
static uint32_t loopOverruns = 0;

// ------------------------------------------------------------------ encoders (4x decoding)
static volatile int32_t encCountL = 0, encCountR = 0;
static volatile uint8_t encStateL = 0, encStateR = 0;
static const int8_t QDEC[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

static void isrEncL() {
  uint8_t s = (digitalRead(PIN_ENC_L_A) << 1) | digitalRead(PIN_ENC_L_B);
  encCountL += QDEC[(encStateL << 2) | s];
  encStateL = s;
}
static void isrEncR() {
  uint8_t s = (digitalRead(PIN_ENC_R_A) << 1) | digitalRead(PIN_ENC_R_B);
  encCountR += QDEC[(encStateR << 2) | s];
  encStateR = s;
}
static void readEncoders(int32_t& l, int32_t& r) {
  noInterrupts();
  l = encCountL;
  r = encCountR;
  interrupts();
  l *= ENC_DIR_L;
  r *= ENC_DIR_R;
}

// ------------------------------------------------------------------ motors
static void setMotor(uint8_t rpwm, uint8_t lpwm, float duty) {
  int v = (int)(fabs(duty) * 255.0f + 0.5f);
  if (v > 255) v = 255;
  if (duty >= 0) {
    analogWrite(lpwm, 0);
    analogWrite(rpwm, v);
  } else {
    analogWrite(rpwm, 0);
    analogWrite(lpwm, v);
  }
}
static void writeMotors(const CoreOutputs& o) {
  digitalWrite(PIN_L_EN, o.drive_enable ? HIGH : LOW);
  digitalWrite(PIN_R_EN, o.drive_enable ? HIGH : LOW);
  setMotor(PIN_L_RPWM, PIN_L_LPWM, o.drive_enable ? o.duty[0] * MOTOR_DIR_L : 0);
  setMotor(PIN_R_RPWM, PIN_R_LPWM, o.drive_enable ? o.duty[1] * MOTOR_DIR_R : 0);
}

// ------------------------------------------------------------------ servos
static uint16_t degToTicks(uint8_t ch, float deg) {
  float us = SERVO_CENTER_US[ch] + SERVO_DIR[ch] * deg * SERVO_US_PER_DEG[ch];
  if (us < PULSE_MIN) us = PULSE_MIN;
  if (us > PULSE_MAX) us = PULSE_MAX;
  return (uint16_t)(us * 4096.0f / 20000.0f + 0.5f);  // 50 Hz frame = 20000 us = 4096 ticks
}
static void writeServo(uint8_t ch, uint16_t ticks) {
  if (servoTicks[ch] == ticks) return;  // the I2C bus is shared: write only changes
  servoTicks[ch] = ticks;
  if (ticks == 0xFFFF) pca.setPWM(ch, 0, 4096);  // full off: no pulse, the servo goes limp
  else pca.setPWM(ch, 0, ticks);
}
static void writeServos(const CoreOutputs& o, bool frame) {
  bool any = false;
  for (uint8_t ch = 0; ch < 8; ch++) any |= o.servo_on[ch];
  if (any != oeEnabled) {
    digitalWrite(PIN_PCA_OE, any ? LOW : HIGH);
    oeEnabled = any;
  }
  if (!pcaOk || !frame) return;  // servos refresh at 50 Hz, every second control step
  for (uint8_t ch = 0; ch < 8; ch++) writeServo(ch, o.servo_on[ch] ? degToTicks(ch, o.joint_deg[ch]) : 0xFFFF);
}

// ------------------------------------------------------------------ MPU-6050 (gyro z only)
static bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}
static bool mpuGyroZ(float& dps) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47);  // GYRO_ZOUT_H
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDR, 2) != 2) return false;
  int16_t raw = (Wire.read() << 8) | Wire.read();
  dps = raw / 131.0f;  // +-250 deg/s range
  return true;
}
static void setupMpu() {
  mpuOk = mpuWrite(0x6B, 0x01)      // wake up, clock from the gyro PLL
          && mpuWrite(0x1A, 0x03)   // low-pass 44 Hz
          && mpuWrite(0x1B, 0x00);  // +-250 deg/s
  if (!mpuOk) return;
  delay(100);
  float sum = 0, v;
  int n = 0;
  for (int i = 0; i < 200; i++) {  // 1 s of samples, robot standing still
    if (mpuGyroZ(v)) { sum += v; n++; }
    delay(5);
  }
  gyroBias = n ? sum / n : 0;
}

// ------------------------------------------------------------------ VL53L0X (non-blocking reads)
static void setupTof() {
  pinMode(PIN_TOF1_XSHUT, OUTPUT);
  pinMode(PIN_TOF2_XSHUT, OUTPUT);
  digitalWrite(PIN_TOF1_XSHUT, LOW);  // both in reset
  digitalWrite(PIN_TOF2_XSHUT, LOW);
  delay(10);
  pinMode(PIN_TOF1_XSHUT, INPUT);     // release no. 1 (its board pulls XSHUT up), re-address it
  delay(10);
  tof[0].setTimeout(30);
  if (tof[0].init()) {
    tof[0].setAddress(TOF1_ADDR);
    tofOk[0] = true;
  }
  pinMode(PIN_TOF2_XSHUT, INPUT);     // release no. 2, keeps 0x29
  delay(10);
  tof[1].setTimeout(30);
  tofOk[1] = tof[1].init();
  for (uint8_t i = 0; i < 2; i++)
    if (tofOk[i]) tof[i].startContinuous(0);
}
// Same registers as VL53L0X::readRangeContinuousMillimeters(), without waiting.
static bool tofPoll(uint8_t i, int16_t& mm) {
  if (!tofOk[i]) return false;
  if ((tof[i].readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) == 0) return false;
  uint16_t r = tof[i].readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 10);
  tof[i].writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);
  mm = (r >= 8190) ? -1 : (int16_t)r;  // 8190/8191: nothing in range
  return true;
}

// ------------------------------------------------------------------ HC-SR04
static int16_t sonarRead(uint8_t trig, uint8_t echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  unsigned long us = pulseIn(echo, HIGH, SONAR_TIMEOUT_US);  // blocks at most ~6 ms
  if (us == 0) return -1;
  return (int16_t)(us * 0.1715f);  // 343 m/s, there and back
}

// ------------------------------------------------------------------ serial
static char lineBuf[64];
static uint8_t lineLen = 0;
static uint32_t calMotorUntil = 0;
static float calDuty[2] = {0, 0};

// Bench calibration commands, handled here because they bypass the controller.
// Accepted only while the controller is idle and has no fault.
static bool handleCal(char* line) {
  if (strncmp(line, "CAL", 3) != 0 && strncmp(line, "cal", 3) != 0) return false;
  char* t = strtok(line + 3, " ");
  if (!t) { Serial.println(F("ERR CAL SYNTAX ENC|MOTOR dl dr|PULSE ch us|INFO")); return true; }
  for (char* c = t; *c; c++) *c = toupper(*c);
  if (!strcmp(t, "INFO")) {
    Serial.print(F("CAL INFO tof1=")); Serial.print(tofOk[0]);
    Serial.print(F(" tof2=")); Serial.print(tofOk[1]);
    Serial.print(F(" mpu=")); Serial.print(mpuOk);
    Serial.print(F(" gbias=")); Serial.print(gyroBias, 3);
    Serial.print(F(" pca=")); Serial.print(pcaOk);
    Serial.print(F(" overruns=")); Serial.println(loopOverruns);
    return true;
  }
  if (!strcmp(t, "ENC")) {
    int32_t l, r;
    readEncoders(l, r);
    Serial.print(F("CAL ENC ")); Serial.print(l); Serial.print(' '); Serial.println(r);
    return true;
  }
  if (core.faults() || core.drive().busy() || core.demoRunning()) { Serial.println(F("ERR CAL BUSY")); return true; }
  if (!strcmp(t, "MOTOR")) {  // CAL MOTOR 0.2 0.2 : raw duty for 1 s, robot convention, wheels off the floor
    char* a = strtok(0, " ");
    char* b = strtok(0, " ");
    if (!a || !b) { Serial.println(F("ERR CAL MOTOR dl dr")); return true; }
    calDuty[0] = constrain(atof(a), -0.5, 0.5);
    calDuty[1] = constrain(atof(b), -0.5, 0.5);
    calMotorUntil = millis() + 1000;
    Serial.println(F("OK CAL MOTOR 1 s"));
    return true;
  }
  if (!strcmp(t, "PULSE")) {  // CAL PULSE 3 1500 : raw pulse on one channel, to find the centres
    char* a = strtok(0, " ");
    char* b = strtok(0, " ");
    if (!a || !b || atoi(a) < 0 || atoi(a) > 7) { Serial.println(F("ERR CAL PULSE ch us")); return true; }
    int us = constrain(atoi(b), PULSE_MIN, PULSE_MAX);
    digitalWrite(PIN_PCA_OE, LOW);
    oeEnabled = true;
    writeServo(atoi(a), (uint16_t)(us * 4096L / 20000L));
    Serial.println(F("OK CAL PULSE"));
    return true;
  }
  Serial.println(F("ERR CAL UNKNOWN"));
  return true;
}

static void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      lineBuf[lineLen] = 0;
      if (lineLen && !handleCal(lineBuf)) core.handleLine(lineBuf);
      lineLen = 0;
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    }
  }
}

static void flushMessages() {
  char buf[96];
  while (core.popMessage(buf, sizeof(buf))) Serial.println(buf);
}

// ------------------------------------------------------------------ setup / loop
void setup() {
  // outputs safe before anything else: drivers off, servos released
  pinMode(PIN_L_EN, OUTPUT);
  pinMode(PIN_R_EN, OUTPUT);
  digitalWrite(PIN_L_EN, LOW);
  digitalWrite(PIN_R_EN, LOW);
  pinMode(PIN_PCA_OE, OUTPUT);
  digitalWrite(PIN_PCA_OE, HIGH);
  const uint8_t pwmPins[] = {PIN_L_RPWM, PIN_L_LPWM, PIN_R_RPWM, PIN_R_LPWM};
  for (uint8_t p : pwmPins) {
    pinMode(p, OUTPUT);
    analogWrite(p, 0);
  }
  // motor PWM at 3.9 kHz instead of 490 Hz (timer 3: pin 5, timer 4: pins 6, 7, 8)
  TCCR3B = (TCCR3B & 0xF8) | 0x02;
  TCCR4B = (TCCR4B & 0xF8) | 0x02;

  Serial.begin(115200);
  pinMode(PIN_ESTOP_SENSE, INPUT_PULLUP);
  pinMode(PIN_SONAR_L_TRIG, OUTPUT);
  pinMode(PIN_SONAR_R_TRIG, OUTPUT);
  pinMode(PIN_SONAR_L_ECHO, INPUT);
  pinMode(PIN_SONAR_R_ECHO, INPUT);
  const uint8_t encPins[] = {PIN_ENC_L_A, PIN_ENC_L_B, PIN_ENC_R_A, PIN_ENC_R_B};
  for (uint8_t p : encPins) pinMode(p, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isrEncL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_B), isrEncL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isrEncR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_B), isrEncR, CHANGE);

  Wire.begin();
  Wire.setClock(400000);  // 8 servo channels + 2 ToF + IMU share the bus
  Wire.setWireTimeout(3000, true);  // a stuck bus must not hang the control loop
  Wire.beginTransmission(PCA_ADDR);
  pcaOk = Wire.endTransmission() == 0;
  if (pcaOk) {
    pca.begin();
    pca.setOscillatorFrequency(PCA_OSC_HZ);
    pca.setPWMFreq(50);
    Wire.setClock(400000);
    for (uint8_t ch = 0; ch < 16; ch++) pca.setPWM(ch, 0, 4096);
  }
  for (uint8_t ch = 0; ch < 8; ch++) servoTicks[ch] = 0xFFFF;
  setupMpu();
  setupTof();

  core.begin();
  flushMessages();
  Serial.print(F("EVT HW tof1=")); Serial.print(tofOk[0]);
  Serial.print(F(" tof2=")); Serial.print(tofOk[1]);
  Serial.print(F(" mpu=")); Serial.print(mpuOk);
  Serial.print(F(" pca=")); Serial.println(pcaOk);
}

void loop() {
  static uint32_t next = micros();
  static uint32_t lastSonar = 0;
  static uint8_t sonarSide = 0;
  static uint8_t frame = 0;

  readSerial();
  if ((int32_t)(micros() - next) < 0) return;
  next += 10000UL;
  if ((int32_t)(micros() - next) > 0) {  // more than one period late: count it and resynchronise
    loopOverruns++;
    next = micros() + 10000UL;
  }

  memset(&in, 0, sizeof(in));
  readEncoders(in.encL, in.encR);
  int16_t mm;
  for (uint8_t i = 0; i < 2; i++)
    if (tofPoll(i, mm)) { in.range_mm[i] = mm; in.range_new[i] = 1; }
  if (millis() - lastSonar >= SONAR_PERIOD_MS) {
    lastSonar = millis();
    uint8_t i = 2 + sonarSide;
    in.range_mm[i] = sonarSide ? sonarRead(PIN_SONAR_R_TRIG, PIN_SONAR_R_ECHO)
                               : sonarRead(PIN_SONAR_L_TRIG, PIN_SONAR_L_ECHO);
    in.range_new[i] = 1;
    sonarSide ^= 1;
  }
  float dps;
  in.gyro_z = (mpuOk && mpuGyroZ(dps)) ? GYRO_Z_DIR * (dps - gyroBias) * 0.01745329f : 0;
  in.vbat = analogRead(PIN_VBAT) * (5.0f / 1023.0f) * VBAT_DIVIDER * VBAT_CAL;
  in.estop = digitalRead(PIN_ESTOP_SENSE) == LOW;

  core.step(in, out);

  if (calMotorUntil && (int32_t)(millis() - calMotorUntil) < 0 && !core.faults()) {
    out.drive_enable = 1;
    out.duty[0] = calDuty[0];
    out.duty[1] = calDuty[1];
  } else {
    calMotorUntil = 0;
  }
  writeMotors(out);
  writeServos(out, (frame++ & 1) == 0);
  flushMessages();
}
