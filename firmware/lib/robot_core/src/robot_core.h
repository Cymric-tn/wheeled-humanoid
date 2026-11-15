// Robot controller, independent of the hardware. The same code runs:
//   - on the Arduino Mega (src/main.cpp reads the sensors and drives the outputs),
//   - in the host unit tests (test/test_core.cpp),
//   - inside the MuJoCo digital twin (sim/firmware_sim.py, through sim_api.cpp).
// It is stepped every 10 ms and talks the text protocol described in protocol.md.
#pragma once
#include <stdint.h>
#include "config.h"
#include "drive.h"
#include "gestures.h"
#include "obstacle.h"

struct CoreInputs {
  int32_t encL, encR;            // cumulative encoder counts, robot convention (+ = forward)
  int16_t range_mm[4];           // ToF L, ToF R, sonar L, sonar R; <= 0 = no echo
  uint8_t range_new[4];          // 1 when range_mm holds a new reading this step
  float gyro_z;                  // rad/s, + = turning left
  float vbat;                    // V at the battery (before the emergency stop)
  uint8_t estop;                 // 1 while the emergency stop is pressed (optional sense line)
};

struct CoreOutputs {
  float duty[2];                 // left, right wheel, [-1, 1], robot convention
  uint8_t drive_enable;          // BTS7960 enable lines
  float joint_deg[cfg::NJ];      // servo targets, twin convention
  uint8_t servo_on[cfg::NJ];     // 0 = no pulse (servo limp)
};

class RobotCore {
 public:
  enum State : uint8_t { ST_IDLE, ST_BUSY, ST_FAULT };
  enum Fault : uint8_t { F_NONE = 0, F_ESTOP = 1, F_LOWBAT = 2 };

  void begin();
  void step(const CoreInputs& in, CoreOutputs& out);
  void handleLine(const char* line);
  // Next message for the serial port (without newline). Returns false when empty.
  bool popMessage(char* buf, uint8_t size);

  // read-only access for tests and the simulator
  const Drive& drive() const { return drive_; }
  const Gestures& gestures() const { return gest_; }
  int16_t obstacleMm() const { return obst_mm_; }
  uint8_t faults() const { return faults_; }
  bool demoRunning() const { return demo_step_ >= 0; }
  uint32_t timeMs() const { return now_; }

 private:
  void send(const char* fmt, ...);
  void command(char* line, bool fromDemo);
  void finishDrive(Drive::Result r);
  void startServos();
  void enterFault(uint8_t f, const char* why);
  void demoTick();
  void abortDemo(const char* why);
  void sendState();

  Drive drive_;
  Gestures gest_;
  Obstacle obstacle_;
  uint32_t now_ = 0;
  int16_t obst_mm_ = -1;
  bool blocked_ = false;
  uint8_t faults_ = F_NONE;
  bool estop_prev_ = false;
  float vbat_ = 0;
  uint16_t lowbat_ms_ = 0;
  bool vbat_warned_ = false, vbat_absent_reported_ = false;
  uint16_t vel_age_ms_ = 0;
  char drive_cmd_[8] = "";
  bool gest_reported_ = true;
  char gest_name_[8] = "";
  // servos
  bool servos_requested_ = false;
  uint32_t servo_start_ms_ = 0;
  uint8_t servo_on_[cfg::NJ];
  // demo
  int8_t demo_step_ = -1;
  bool demo_wait_ = false;
  // telemetry
  uint16_t stream_period_ms_ = 0, stream_age_ms_ = 0;
  // output queue
  static const uint8_t QN = 6, QL = 96;
  char q_[QN][QL];
  uint8_t qHead_ = 0, qCount_ = 0;
};
