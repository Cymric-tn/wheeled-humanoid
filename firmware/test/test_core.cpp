// Host unit tests for the controller core (no Arduino, no simulator).
// A first-order motor model closes the loop: wheel speed follows duty x no-load speed with a
// 0.2 s time constant (the config.h estimate); one test varies it from 0.1 to 0.35 s. Build and run:  make -C firmware test
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "robot_core.h"

static int fails = 0, checks = 0;
#define CHECK(cond, ...)                                       \
  do {                                                         \
    checks++;                                                  \
    printf((cond) ? "  ok   " : "  FAIL "); printf(__VA_ARGS__); printf("\n"); \
    if (!(cond)) fails++;                                      \
  } while (0)

struct Plant {
  double tau = 0.2;        // s, wheel time constant (config.h estimate)
  double wL = 0, wR = 0;   // wheel speed rad/s
  double qL = 0, qR = 0;   // wheel angle rad
  double wallMm = 5000;    // distance from the front sensors to an obstacle
  double x = 0, th = 0;    // true pose (straight-line only for the wall)
  float vbat = 11.4f;
  bool estop = false;
  int tick = 0;
  void step(const CoreOutputs& o, double dt) {
    const double wmax = cfg::WHEEL_NOLOAD;
    double tL = o.drive_enable ? o.duty[0] * wmax : 0, tR = o.drive_enable ? o.duty[1] * wmax : 0;
    wL += (tL - wL) * dt / tau;
    wR += (tR - wR) * dt / tau;
    qL += wL * dt;
    qR += wR * dt;
    double v = 0.5 * (wL + wR) * cfg::WHEEL_RADIUS;
    wallMm -= v * dt * 1000;
    x += v * dt;
    th += (wR - wL) * cfg::WHEEL_RADIUS / cfg::TRACK * dt;
  }
  CoreInputs inputs() {
    CoreInputs in;
    memset(&in, 0, sizeof(in));
    double cpr = cfg::COUNTS_PER_REV / (2 * M_PI);
    in.encL = (int32_t)floor(qL * cpr);
    in.encR = (int32_t)floor(qR * cpr);
    tick++;
    if (tick % 3 == 0) {  // ToF every 30 ms
      int16_t mm = wallMm < cfg::TOF_MAX_MM ? (int16_t)wallMm : -1;
      in.range_mm[0] = in.range_mm[1] = mm;
      in.range_new[0] = in.range_new[1] = 1;
    }
    if (tick % 6 == 0) {  // sonar every 60 ms
      int16_t mm = wallMm < cfg::SONAR_MAX_MM ? (int16_t)wallMm : -1;
      in.range_mm[2] = in.range_mm[3] = mm;
      in.range_new[2] = in.range_new[3] = 1;
    }
    in.gyro_z = (float)((wR - wL) * cfg::WHEEL_RADIUS / cfg::TRACK);
    in.vbat = vbat;
    in.estop = estop;
    return in;
  }
};

struct Rig {
  RobotCore core;
  Plant plant;
  CoreOutputs out;
  char last[128][96];
  int nlast = 0;
  Rig() { core.begin(); memset(&out, 0, sizeof(out)); drain(); }
  void drain() {
    char buf[96];
    while (core.popMessage(buf, sizeof(buf))) {
      if (nlast < 128) strcpy(last[nlast++], buf);
    }
  }
  bool saw(const char* prefix) {
    for (int i = 0; i < nlast; i++) if (!strncmp(last[i], prefix, strlen(prefix))) return true;
    return false;
  }
  void cmd(const char* c) { nlast = 0; core.handleLine(c); drain(); }
  // run until a DONE message starting with `done` (or the timeout); returns seconds taken
  double runUntil(const char* done, double timeout) {
    for (int i = 0; i < timeout * 100; i++) {
      CoreInputs in = plant.inputs();
      core.step(in, out);
      plant.step(out, 0.01);
      drain();
      if (done && saw(done)) return (i + 1) * 0.01;
    }
    return -1;
  }
};

static void settle(Rig& r) { r.runUntil(0, 1.0); r.nlast = 0; }

int main() {
  printf("controller core, host tests\n");
  {
    Rig r;
    settle(r);
    r.cmd("FWD 200");
    CHECK(r.saw("OK FWD"), "FWD accepted");
    double t = r.runUntil("DONE FWD", 20);
    double dist = r.plant.x;
    CHECK(t > 0 && fabs(dist - 2.0) < 0.02, "FWD 200: true distance %.3f m in %.1f s (error under 2 cm)", dist, t);
    CHECK(fabs(r.core.drive().x() - dist) < 0.01, "encoder odometry %.3f m matches", r.core.drive().x());
    r.cmd("BACK 50");
    r.runUntil("DONE BACK", 10);
    CHECK(fabs(r.plant.x - 1.5) < 0.02, "BACK 50: at %.3f m", r.plant.x);
  }
  {
    Rig r;
    settle(r);
    r.cmd("TURN 90");
    double t = r.runUntil("DONE TURN", 10);
    double deg = r.plant.th * 180 / M_PI;
    CHECK(t > 0 && fabs(deg - 90) < 2, "TURN 90: true heading %.2f deg in %.1f s", deg, t);
    CHECK(fabs(r.plant.x) < 0.01, "turn on the spot: centre moved %.1f mm", r.plant.x * 1000);
  }
  {  // robustness: the real motor time constant is not known yet
    double worst = 0, worstTurn = 0;
    const double taus[] = {0.1, 0.15, 0.25, 0.35};
    for (double tau : taus) {
      Rig r;
      r.plant.tau = tau;
      settle(r);
      r.cmd("FWD 150");
      r.runUntil("DONE FWD", 20);
      worst = fmax(worst, fabs(r.plant.x - 1.5));
      r.cmd("TURN -90");
      r.runUntil("DONE TURN", 20);
      worstTurn = fmax(worstTurn, fabs(r.plant.th * 180 / M_PI + 90));
    }
    CHECK(worst < 0.01 && worstTurn < 1.0, "motor time constant 0.1 to 0.35 s: worst error %.1f mm, %.2f deg", worst * 1000, worstTurn);
  }
  {  // peak speed and acceleration respect the limits
    Rig r;
    settle(r);
    r.cmd("FWD 300");
    double vmax = 0, amax = 0, vprev = 0;
    for (int i = 0; i < 1500; i++) {
      CoreInputs in = r.plant.inputs();
      r.core.step(in, r.out);
      r.plant.step(r.out, 0.01);
      r.drain();
      double v = 0.5 * (r.plant.wL + r.plant.wR) * cfg::WHEEL_RADIUS;
      vmax = fmax(vmax, v);
      if (i > 0) amax = fmax(amax, fabs(v - vprev) / 0.01);
      vprev = v;
    }
    CHECK(vmax <= cfg::V_MAX * 1.05, "speed cap: peak %.3f m/s (cap %.2f)", vmax, cfg::V_MAX);
    // the profile is limited to A_MAX; the wheels briefly exceed it (about 0.1 s) while the
    // position loop catches up with the motor lag at start-up. Tip-over needs 6.7 m/s^2.
    CHECK(amax <= cfg::A_MAX * 1.7, "acceleration: peak %.2f m/s^2 (profile %.2f, tip-over 6.7)", amax, cfg::A_MAX);
  }
  {  // obstacle stop from full speed
    Rig r;
    r.plant.wallMm = 1500;
    settle(r);
    r.cmd("FWD 400");
    double t = r.runUntil("DONE FWD", 20);
    CHECK(r.saw("DONE FWD blocked"), "FWD ends 'blocked' at the wall");
    CHECK(r.plant.wallMm > 300 && r.plant.wallMm < 450, "stops %.0f mm from the wall (limit 300 mm)", r.plant.wallMm);
    r.cmd("FWD 50");
    CHECK(r.saw("ERR FWD BLOCKED"), "new forward move refused while blocked");
    r.cmd("BACK 30");
    CHECK(r.saw("OK BACK"), "reversing still allowed");
    (void)t;
  }
  {  // no range sensor -> no forward motion
    Rig r;
    settle(r);
    RobotCore c;
    c.begin();
    CoreOutputs o;
    CoreInputs in;
    memset(&in, 0, sizeof(in));
    in.vbat = 11.4f;
    for (int i = 0; i < 100; i++) c.step(in, o);
    c.handleLine("FWD 50");
    char b[96];
    bool refused = false;
    while (c.popMessage(b, sizeof(b))) if (!strncmp(b, "ERR FWD BLOCKED", 15)) refused = true;
    CHECK(refused, "with every range sensor silent, FWD is refused");
  }
  {  // emergency stop and low battery
    Rig r;
    settle(r);
    r.cmd("FWD 200");
    r.runUntil(0, 1.0);
    r.plant.estop = true;
    r.nlast = 0;
    r.runUntil(0, 0.05);
    CHECK(r.saw("EVT FAULT ESTOP") && r.out.drive_enable == 0 && r.out.duty[0] == 0, "E-stop: fault, drivers disabled");
    r.cmd("FWD 10");
    CHECK(r.saw("ERR FWD FAULT"), "motion refused during the fault");
    r.cmd("RESET");
    CHECK(r.saw("ERR RESET ESTOP_PRESSED"), "RESET refused while the button is pressed");
    r.plant.estop = false;
    r.runUntil(0, 0.05);
    r.cmd("RESET");
    CHECK(r.saw("OK RESET") && r.core.faults() == 0, "RESET after release");
    r.plant.vbat = 9.5f;
    r.nlast = 0;
    double t = r.runUntil("EVT FAULT LOWBAT", 6);
    CHECK(t > 2.0 && t < 4.0, "low battery fault after %.2f s below 9.9 V", t);
  }
  {  // VEL watchdog
    Rig r;
    settle(r);
    r.cmd("VEL 200 0");
    r.runUntil(0, 0.3);
    double v1 = 0.5 * (r.plant.wL + r.plant.wR) * cfg::WHEEL_RADIUS;
    r.nlast = 0;
    double t = r.runUntil("EVT VEL_TIMEOUT", 2);
    CHECK(v1 > 0.05 && t > 0.1 && t < 0.3, "VEL stream stops %.2f s after the last command", t + 0.3);
  }
  {  // gestures: peak servo speed and the clearance rule
    Rig r;
    settle(r);
    r.cmd("WAVE R");
    double peak = 0, prev[cfg::NJ];
    memcpy(prev, r.core.gestures().position(), sizeof(prev));
    double t = -1;
    for (int i = 0; i < 2000; i++) {
      CoreInputs in = r.plant.inputs();
      r.core.step(in, r.out);
      r.drain();
      const float* q = r.core.gestures().position();
      for (int j = 0; j < cfg::NJ; j++) {
        peak = fmax(peak, fabs(q[j] - prev[j]) / 0.01);
        prev[j] = q[j];
      }
      if (r.saw("DONE WAVE ok")) { t = (i + 1) * 0.01; break; }
    }
    CHECK(t > 0, "WAVE completes in %.1f s", t);
    CHECK(peak <= cfg::SERVO_PEAK_DPS * 1.02, "servo peak speed %.1f deg/s (limit %.0f)", peak, cfg::SERVO_PEAK_DPS);
    CHECK(r.out.servo_on[cfg::R_PITCH] && r.out.servo_on[cfg::L_PITCH], "both servo strips enabled");
    float q[cfg::NJ] = {0, -40, 0, 60, -40, 0, 0, 0};
    Gestures::applyLimits(q);
    CHECK(q[cfg::L_ROLL] == cfg::ROLL_MIN_ARM_DOWN && q[cfg::R_ROLL] == -30,
          "roll limited to -15 deg only while the arm hangs (%.0f, %.0f)", q[cfg::L_ROLL], q[cfg::R_ROLL]);
  }
  {  // servo strips start one after the other
    Rig r;
    settle(r);
    r.cmd("SERVO ON");
    r.runUntil(0, 0.1);
    bool aOnly = r.out.servo_on[0] && !r.out.servo_on[3];
    r.runUntil(0, 0.3);
    CHECK(aOnly && r.out.servo_on[3], "strip A first, strip B %d ms later", cfg::SERVO_GROUP_DELAY_MS);
  }
  {  // protocol errors
    Rig r;
    r.cmd("FWD 9999");
    CHECK(r.saw("ERR FWD RANGE"), "FWD out of range rejected");
    r.cmd("ARM X 1 2 3");
    CHECK(r.saw("ERR ARM SYNTAX"), "ARM with a bad side rejected");
    r.cmd("FLY");
    CHECK(r.saw("ERR FLY UNKNOWN"), "unknown command rejected");
    r.cmd("state?");
    CHECK(r.saw("STATE st="), "STATE? answers (commands are case-insensitive)");
  }
  {  // full demonstration with a wall 1.6 m ahead after the turn is not modelled here:
     // the plant is one-dimensional, so the wall is placed for the last forward leg only.
    Rig r;
    settle(r);
    r.plant.wallMm = 99999;
    r.cmd("DEMO");
    bool wallSet = false;
    double t = 0;
    for (int i = 0; i < 9000; i++) {
      CoreInputs in = r.plant.inputs();
      r.core.step(in, r.out);
      r.plant.step(r.out, 0.01);
      r.drain();
      if (!wallSet && r.saw("EVT DEMO 5 FWD 300")) { r.plant.wallMm = 1200; wallSet = true; }
      if (r.saw("DONE DEMO")) { t = (i + 1) * 0.01; break; }
      if (r.nlast > 100) r.nlast = 0;
    }
    CHECK(r.saw("DONE DEMO ok"), "DEMO runs to the end in %.1f s", t);
  }
  printf("\n%d/%d checks passed\n", checks - fails, checks);
  return fails ? 1 : 0;
}
