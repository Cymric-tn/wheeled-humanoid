// Pins and calibration of the real hardware. Pin numbers follow the wiring diagram
// (wiring/wheeled_humanoid_wiring.pdf, sheet 2). Values marked CALIBRATE are set during the
// first bench session with the commands of the CAL group (see README, "Calibration").
#pragma once
#include <Arduino.h>

// ---- motors (BTS7960: RPWM drives forward, LPWM reverse; R_EN and L_EN joined)
const uint8_t PIN_L_RPWM = 5, PIN_L_LPWM = 6, PIN_L_EN = 30;
const uint8_t PIN_R_RPWM = 7, PIN_R_LPWM = 8, PIN_R_EN = 31;
// CALIBRATE: +1 or -1 so that a positive duty rolls the robot forward. The motors are mirror images,
// so one of them is normally -1.
const int8_t MOTOR_DIR_L = -1, MOTOR_DIR_R = +1;

// ---- encoders (phase A and B, all four on external-interrupt pins)
const uint8_t PIN_ENC_L_A = 2, PIN_ENC_L_B = 18;
const uint8_t PIN_ENC_R_A = 3, PIN_ENC_R_B = 19;
// CALIBRATE: +1 or -1 so that the counts grow when the wheel rolls forward.
const int8_t ENC_DIR_L = -1, ENC_DIR_R = +1;

// ---- range sensors
const uint8_t PIN_SONAR_L_TRIG = 22, PIN_SONAR_L_ECHO = 23;
const uint8_t PIN_SONAR_R_TRIG = 24, PIN_SONAR_R_ECHO = 25;
const uint8_t PIN_TOF1_XSHUT = 26, PIN_TOF2_XSHUT = 27;  // never drive XSHUT high (2.8 V input)
const uint8_t TOF1_ADDR = 0x30;                          // ToF no. 1 is re-addressed at boot; no. 2 keeps 0x29
const unsigned long SONAR_TIMEOUT_US = 6000;              // about 1 m: farther echoes count as "nothing"
const uint16_t SONAR_PERIOD_MS = 60;                      // one sonar every 60 ms, left and right alternate

// ---- PCA9685 servo driver
const uint8_t PIN_PCA_OE = 28;          // low = outputs on, high = all servos released
const uint8_t PCA_ADDR = 0x40;
const uint32_t PCA_OSC_HZ = 25000000;   // CALIBRATE: many PCA9685 boards run near 26-27 MHz;
                                        // measure a 1500 us pulse and correct this value
// Servo calibration per channel (joint order = channel order, see config.h).
// pulse_us = CENTER + DIR * angle_deg * US_PER_DEG, clamped to [PULSE_MIN, PULSE_MAX].
// CALIBRATE: CENTER is the pulse at the CAD pose (arm hanging, head forward); DIR makes the
// joint move in the twin's positive direction.
const int16_t SERVO_CENTER_US[8] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
const int8_t SERVO_DIR[8] = {+1, +1, +1, -1, -1, -1, +1, +1};
const float SERVO_US_PER_DEG[8] = {11.11f, 11.11f, 11.11f, 11.11f, 11.11f, 11.11f, 11.11f, 11.11f};
const int16_t PULSE_MIN = 500, PULSE_MAX = 2500;

// ---- MPU-6050
const uint8_t MPU_ADDR = 0x68;
const int8_t GYRO_Z_DIR = +1;           // CALIBRATE: + when the robot turns left (seen from above)

// ---- battery and emergency stop
const uint8_t PIN_VBAT = A0;
const float VBAT_DIVIDER = (10.0f + 3.3f) / 3.3f;  // R1 10 k, R2 3.3 k (wiring sheet 1)
const float VBAT_CAL = 1.000f;          // CALIBRATE against a multimeter
const uint8_t PIN_ESTOP_SENSE = 29;     // optional spare NO contact to GND; LOW = pressed.
                                        // Not wired: the pull-up reads HIGH = released.
