#include "robot_core.h"
#include <ctype.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

using namespace cfg;

static const float DEG = 0.01745329f;
#define FW_VERSION "1.0"

// The minimum demonstration (spec book, project 2): move, turn, stop before an obstacle,
// arm movement, head gesture. Each step starts when the previous one has finished.
static const char* const DEMO_STEPS[] = {
    "SERVO ON", "FWD 100", "BACK 30", "TURN 90", "FWD 300", "WAVE R", "NOD", "SHAKE", "REST",
};
static const int8_t DEMO_LEN = sizeof(DEMO_STEPS) / sizeof(DEMO_STEPS[0]);

static bool parseNum(const char* s, float& v) {
  if (!s) return false;
  char* end;
  v = (float)strtod(s, &end);
  return end != s && *end == 0 && !isnan(v);
}

static long r(float v) { return (long)(v < 0 ? v - 0.5f : v + 0.5f); }

// ---------------------------------------------------------------- output queue
void RobotCore::send(const char* fmt, ...) {
  if (qCount_ == QN) {  // full: drop the oldest so the newest state always gets out
    qHead_ = (qHead_ + 1) % QN;
    qCount_--;
  }
  char* dst = q_[(qHead_ + qCount_) % QN];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(dst, QL, fmt, ap);
  va_end(ap);
  qCount_++;
}

bool RobotCore::popMessage(char* buf, uint8_t size) {
  if (!qCount_) return false;
  strncpy(buf, q_[qHead_], size - 1);
  buf[size - 1] = 0;
  qHead_ = (qHead_ + 1) % QN;
  qCount_--;
  return true;
}

// ---------------------------------------------------------------- life cycle
void RobotCore::begin() {
  now_ = 0;
  faults_ = F_NONE;
  obstacle_.reset();
  gest_.reset();
  drive_.reset(0, 0);
  for (uint8_t j = 0; j < NJ; j++) servo_on_[j] = 0;
  servos_requested_ = false;
  demo_step_ = -1;
  qHead_ = qCount_ = 0;
  vbat_ = 0;
  drive_cmd_[0] = 0;
  gest_reported_ = true;
  send("READY humanoid-fw " FW_VERSION);
}

void RobotCore::startServos() {
  if (servos_requested_) return;
  servos_requested_ = true;
  servo_start_ms_ = now_;
}

void RobotCore::finishDrive(Drive::Result res) {
  if (res == Drive::NONE || !drive_cmd_[0]) return;
  const char* how = res == Drive::DONE_OK ? "ok" : res == Drive::DONE_BLOCKED ? "blocked" : "stopped";
  if (strcmp(drive_cmd_, "VEL") != 0) {
    send("DONE %s %s x=%ld y=%ld th=%ld", drive_cmd_, how, r(drive_.x() * 1000), r(drive_.y() * 1000),
         r(drive_.heading() / DEG * 10));
  }
  drive_cmd_[0] = 0;
}

void RobotCore::abortDemo(const char* why) {
  if (demo_step_ < 0) return;
  demo_step_ = -1;
  send("DONE DEMO %s", why);
}

void RobotCore::enterFault(uint8_t f, const char* why) {
  bool first = faults_ == F_NONE;
  faults_ |= f;
  drive_.halt();
  finishDrive(drive_.takeResult());
  if (gest_.busy() && !gest_reported_) {
    send("DONE %s stopped", gest_name_);
    gest_reported_ = true;
  }
  gest_.stop();
  servos_requested_ = false;
  abortDemo("aborted");
  if (first || f == F_ESTOP) send("EVT FAULT %s", why);
}

// ---------------------------------------------------------------- commands
void RobotCore::handleLine(const char* line) {
  char buf[64];
  strncpy(buf, line, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = 0;
  command(buf, false);
}

void RobotCore::command(char* line, bool fromDemo) {
  char* tok[6];
  uint8_t n = 0;
  for (char* t = strtok(line, " \t\r\n,"); t && n < 6; t = strtok(0, " \t\r\n,")) tok[n++] = t;
  if (!n) return;
  for (char* c = tok[0]; *c; c++) *c = toupper(*c);
  const char* cmd = tok[0];
  float a = 0, b = 0, c = 0;

  // ---- queries and settings: always allowed, never abort the demo
  if (!strcmp(cmd, "PING")) { send("PONG"); return; }
  if (!strcmp(cmd, "STATE?") || !strcmp(cmd, "STATE")) { sendState(); return; }
  if (!strcmp(cmd, "STREAM")) {
    if (n != 2 || !parseNum(tok[1], a) || a < 0 || a > 20) { send("ERR STREAM RANGE 0..20 Hz"); return; }
    stream_period_ms_ = a > 0 ? (uint16_t)(1000.0f / a) : 0;
    stream_age_ms_ = 0;
    send("OK STREAM");
    return;
  }
  if (!strcmp(cmd, "HELP")) {
    send("CMDS FWD BACK TURN VEL STOP ARM HEAD WAVE POINT NOD SHAKE REST SERVO DEMO RESET");
    send("CMDS STATE? STREAM PING");
    return;
  }
  if (!strcmp(cmd, "STOP")) {
    abortDemo("stopped");
    drive_.stop(Drive::DONE_STOPPED, A_MAX);
    if (gest_.busy() && !gest_reported_) {
      send("DONE %s stopped", gest_name_);
      gest_reported_ = true;
    }
    gest_.stop();
    send("OK STOP");
    return;
  }
  if (!strcmp(cmd, "RESET")) {
    if (estop_prev_) { send("ERR RESET ESTOP_PRESSED"); return; }
    faults_ = F_NONE;
    lowbat_ms_ = 0;
    send("OK RESET");
    return;
  }

  // ---- everything below moves the robot
  if (!fromDemo && demo_step_ >= 0) abortDemo("interrupted");
  if (faults_) { send("ERR %s FAULT %d", cmd, faults_); return; }

  if (!strcmp(cmd, "DEMO")) {
    demo_step_ = 0;
    demo_wait_ = false;
    send("OK DEMO");
    return;
  }
  if (!strcmp(cmd, "FWD") || !strcmp(cmd, "BACK") || !strcmp(cmd, "TURN")) {
    bool turn = cmd[0] == 'T';
    float lim = turn ? 720 : 500;
    if (n != 2 || !parseNum(tok[1], a) || fabsf(a) > lim || a == 0) {
      send("ERR %s RANGE %s", cmd, turn ? "-720..720 deg" : "1..500 cm");
      return;
    }
    if (!strcmp(cmd, "BACK")) a = -fabsf(a);
    if (!turn && a > 0 && blocked_) { send("ERR FWD BLOCKED %d", obst_mm_); return; }
    if (drive_.busy()) {  // a new move replaces the current one
      drive_.halt();
      finishDrive(Drive::DONE_STOPPED);
      drive_.takeResult();
    }
    if (turn) drive_.startTurn(a * DEG);
    else drive_.startDistance(a * 0.01f);
    strncpy(drive_cmd_, cmd, sizeof(drive_cmd_) - 1);
    send("OK %s", cmd);
    return;
  }
  if (!strcmp(cmd, "VEL")) {  // teleop stream: mm/s, deg/s. Must repeat within 500 ms.
    if (n != 3 || !parseNum(tok[1], a) || !parseNum(tok[2], b)) { send("ERR VEL SYNTAX"); return; }
    if (a > 0 && blocked_) a = 0;  // turning and reversing stay allowed
    if (drive_.busy() && strcmp(drive_cmd_, "VEL") != 0) {
      drive_.halt();
      finishDrive(Drive::DONE_STOPPED);
      drive_.takeResult();
    }
    drive_.setVelocity(a * 0.001f, b * DEG);
    strcpy(drive_cmd_, "VEL");
    vel_age_ms_ = 0;
    return;
  }

  // ---- servos
  if (!strcmp(cmd, "SERVO")) {
    if (n == 2 && !strcmp(tok[1], "ON")) { startServos(); send("OK SERVO ON"); return; }
    if (n == 2 && !strcmp(tok[1], "OFF")) {
      gest_.stop();
      servos_requested_ = false;
      send("OK SERVO OFF");
      return;
    }
    send("ERR SERVO SYNTAX ON|OFF");
    return;
  }
  GestureId g = G_NONE;
  uint8_t side = 1;
  if (!strcmp(cmd, "WAVE")) g = G_WAVE;
  else if (!strcmp(cmd, "POINT")) g = G_POINT;
  else if (!strcmp(cmd, "NOD")) g = G_NOD;
  else if (!strcmp(cmd, "SHAKE")) g = G_SHAKE;
  else if (!strcmp(cmd, "REST")) g = G_REST;
  float target[NJ];
  for (uint8_t j = 0; j < NJ; j++) target[j] = NAN;
  if (g == G_NONE) {
    if (!strcmp(cmd, "ARM")) {
      if (n != 5 || (toupper(tok[1][0]) != 'L' && toupper(tok[1][0]) != 'R') || !parseNum(tok[2], a) ||
          !parseNum(tok[3], b) || !parseNum(tok[4], c)) {
        send("ERR ARM SYNTAX ARM L|R pitch roll elbow");
        return;
      }
      uint8_t base = toupper(tok[1][0]) == 'L' ? L_PITCH : R_PITCH;
      target[base] = a;
      target[base + 1] = b;
      target[base + 2] = c;
    } else if (!strcmp(cmd, "HEAD")) {
      if (n != 3 || !parseNum(tok[1], a) || !parseNum(tok[2], b)) { send("ERR HEAD SYNTAX HEAD pan tilt"); return; }
      target[HEAD_PAN] = a;
      target[HEAD_TILT] = b;
    } else {
      send("ERR %s UNKNOWN", cmd);
      return;
    }
  } else if (n >= 2) {
    side = toupper(tok[1][0]) == 'L' ? 0 : 1;
  }
  if (gest_.busy() && !gest_reported_) send("DONE %s stopped", gest_name_);
  startServos();
  if (g == G_NONE) gest_.pose(target);
  else gest_.play(g, side);
  strncpy(gest_name_, g == G_NONE ? cmd : Gestures::name(g), sizeof(gest_name_) - 1);
  gest_reported_ = false;
  send("OK %s", gest_name_);
}

// ---------------------------------------------------------------- demo
void RobotCore::demoTick() {
  if (demo_step_ < 0) return;
  if (demo_wait_) {
    bool servosSettling = servos_requested_ && now_ - servo_start_ms_ < 2u * SERVO_GROUP_DELAY_MS;
    if (drive_.busy() || gest_.busy() || servosSettling) return;
    demo_wait_ = false;
    demo_step_++;
  }
  if (demo_step_ >= DEMO_LEN) {
    demo_step_ = -1;
    send("DONE DEMO ok");
    return;
  }
  char buf[24];
  strncpy(buf, DEMO_STEPS[demo_step_], sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = 0;
  send("EVT DEMO %d %s", demo_step_ + 1, buf);
  command(buf, true);
  demo_wait_ = true;
}

// ---------------------------------------------------------------- telemetry
void RobotCore::sendState() {
  const char* st = faults_ ? "FAULT" : (drive_.busy() || gest_.busy() || demo_step_ >= 0) ? "BUSY" : "IDLE";
  send("STATE st=%s f=%d x=%ld y=%ld th=%ld gth=%ld v=%ld obst=%d sens=%d vbat=%ld drv=%s gest=%s srv=%d demo=%d",
       st, faults_, r(drive_.x() * 1000), r(drive_.y() * 1000), r(drive_.heading() / DEG * 10),
       r(drive_.gyroHeading() / DEG * 10), r(0.5f * (drive_.speedL() + drive_.speedR()) * 1000), obst_mm_,
       obstacle_.freshMask(), r(vbat_ * 1000), drive_cmd_[0] ? drive_cmd_ : "-",
       gest_.busy() ? gest_name_ : "-", servos_requested_ ? 1 : 0, demo_step_ + 1);
}

// ---------------------------------------------------------------- 100 Hz step
void RobotCore::step(const CoreInputs& in, CoreOutputs& out) {
  if (now_ == 0) {  // first step: take the encoder counts as zero
    drive_.reset(in.encL, in.encR);
    vbat_ = in.vbat;
  }
  now_ += DT_MS;

  // ---- emergency stop sense line and battery
  if (in.estop && !estop_prev_) enterFault(F_ESTOP, "ESTOP");
  if (!in.estop && estop_prev_) send("EVT ESTOP_RELEASED send RESET");
  estop_prev_ = in.estop;

  vbat_ += 0.02f * (in.vbat - vbat_);  // 0.5 s time constant
  if (vbat_ < VBAT_ABSENT) {
    if (!vbat_absent_reported_) send("EVT VBAT_ABSENT");
    vbat_absent_reported_ = true;
    lowbat_ms_ = 0;
  } else {
    vbat_absent_reported_ = false;
    if (vbat_ < VBAT_CUTOFF) {
      lowbat_ms_ += DT_MS;
      if (lowbat_ms_ >= VBAT_CUTOFF_MS && !(faults_ & F_LOWBAT)) enterFault(F_LOWBAT, "LOWBAT");
    } else {
      lowbat_ms_ = 0;
    }
    if (vbat_ < VBAT_WARN && !vbat_warned_) { send("EVT VBAT_LOW %ld", r(vbat_ * 1000)); vbat_warned_ = true; }
    if (vbat_ > VBAT_WARN + 0.2f) vbat_warned_ = false;
  }

  // ---- obstacle
  obstacle_.tick(DT_MS);
  for (uint8_t i = 0; i < 4; i++)
    if (in.range_new[i]) obstacle_.push(i, in.range_mm[i]);
  obst_mm_ = obstacle_.nearest();
  bool wasBlocked = blocked_;
  if (obst_mm_ < 0) blocked_ = true;                       // no sensor: never drive forward blind
  else if (obst_mm_ < OBST_BRAKE_MM) blocked_ = true;
  else if (obst_mm_ > OBST_CLEAR_MM) blocked_ = false;
  if (blocked_ && drive_.movingForward() &&
      (drive_.mode() == Drive::DISTANCE || drive_.mode() == Drive::VELOCITY)) {
    drive_.stop(Drive::DONE_BLOCKED, OBST_BRAKE_DECEL);
    if (obst_mm_ < 0) send("EVT NO_RANGE");
    else send("EVT BLOCKED %d", obst_mm_);
  }
  if (wasBlocked && !blocked_) send("EVT CLEAR %d", obst_mm_);

  // ---- VEL watchdog
  if (drive_.mode() == Drive::VELOCITY) {
    vel_age_ms_ += DT_MS;
    if (vel_age_ms_ > VEL_TIMEOUT_MS) {
      drive_.stop(Drive::DONE_STOPPED, A_MAX);
      send("EVT VEL_TIMEOUT");
    }
  }

  // ---- drive
  bool wasBusy = drive_.busy();
  drive_.update(in.encL, in.encR, in.gyro_z, DT, out.duty[0], out.duty[1]);
  if (wasBusy && !drive_.busy()) {  // report only once the wheels have stopped
    if (!strcmp(drive_cmd_, "TURN") && drive_.lastSlipDeg() > SLIP_LIMIT_DEG)
      send("EVT SLIP %ld", r(drive_.lastSlipDeg()));
    finishDrive(drive_.takeResult());
  }
  out.drive_enable = faults_ ? 0 : 1;
  if (faults_) out.duty[0] = out.duty[1] = 0;

  // ---- servos
  gest_.update(DT);
  if (!gest_.busy() && !gest_reported_) {
    send("DONE %s ok", gest_name_);
    gest_reported_ = true;
  }
  const float* q = gest_.position();
  for (uint8_t j = 0; j < NJ; j++) {
    out.joint_deg[j] = q[j];
    servo_on_[j] = servos_requested_ && (now_ - servo_start_ms_) >= SERVO_STRIP[j] * SERVO_GROUP_DELAY_MS;
    out.servo_on[j] = servo_on_[j];
  }

  demoTick();

  if (stream_period_ms_) {
    stream_age_ms_ += DT_MS;
    if (stream_age_ms_ >= stream_period_ms_) {
      stream_age_ms_ = 0;
      sendState();
    }
  }
}
