// C interface to the controller for the digital twin (sim/firmware_sim.py, via ctypes).
// Compiled only on the host: the Arduino build skips it.
#ifndef ARDUINO
#include <string.h>
#include "robot_core.h"

extern "C" {

void* core_new() {
  RobotCore* c = new RobotCore();
  c->begin();
  return c;
}
void core_free(void* c) { delete static_cast<RobotCore*>(c); }
void core_line(void* c, const char* line) { static_cast<RobotCore*>(c)->handleLine(line); }
void core_step(void* c, const CoreInputs* in, CoreOutputs* out) { static_cast<RobotCore*>(c)->step(*in, *out); }
int core_pop(void* c, char* buf, int size) { return static_cast<RobotCore*>(c)->popMessage(buf, (uint8_t)size) ? 1 : 0; }
// Internal estimates, for logging: obstacle mm, odometry x, y (m), heading, gyro heading (rad),
// wheel speeds L, R (m/s), drive busy, gesture busy, demo running, faults.
void core_probe(void* c, float* p) {
  const RobotCore* k = static_cast<RobotCore*>(c);
  p[0] = k->obstacleMm();
  p[1] = k->drive().x();
  p[2] = k->drive().y();
  p[3] = k->drive().heading();
  p[4] = k->drive().gyroHeading();
  p[5] = k->drive().speedL();
  p[6] = k->drive().speedR();
  p[7] = k->drive().busy();
  p[8] = k->gestures().busy();
  p[9] = k->demoRunning();
  p[10] = k->faults();
}
int core_sizes(int* in_size, int* out_size) {
  *in_size = (int)sizeof(CoreInputs);
  *out_size = (int)sizeof(CoreOutputs);
  return 1;
}
}
#endif
