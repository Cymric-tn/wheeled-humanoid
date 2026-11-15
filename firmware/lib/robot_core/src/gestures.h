// Servo motion: keyframes in joint space, played with a minimum-jerk profile.
// No servo is ever commanded to jump. Every move lasts long enough for its largest joint change to
// peak at SERVO_PEAK_DPS, which keeps current peaks low and stops a 75 cm structure from shaking.
#pragma once
#include <stdint.h>
#include "config.h"

// One keyframe: a target for each joint (or KEEP), then a pause.
struct Keyframe {
  int16_t deg[cfg::NJ];
  uint16_t pause_ms;
};
const int16_t KEEP = -32768;

enum GestureId : uint8_t { G_NONE, G_REST, G_WAVE, G_POINT, G_NOD, G_SHAKE, G_POSE };

class Gestures {
 public:
  void reset();
  // Start a stored gesture. side: 0 = left arm, 1 = right arm (ignored for head gestures).
  void play(GestureId id, uint8_t side);
  // Move to a pose (degrees; NaN keeps the joint).
  void pose(const float target[cfg::NJ]);
  void stop();                      // freeze where the joints are now
  void update(float dt);
  bool busy() const { return id_ != G_NONE; }
  GestureId current() const { return id_; }
  const float* position() const { return pos_; }
  static const char* name(GestureId id);

  // Joint limits (config.h), checked for clearance on the CAD solids. Public for the tests.
  static void applyLimits(float q[cfg::NJ]);

 private:
  void startSegment(const float target[cfg::NJ]);
  bool nextKey();

  GestureId id_ = G_NONE;
  uint8_t side_ = 1;
  const Keyframe* seq_ = 0;
  uint8_t seqLen_ = 0, seqIdx_ = 0;
  float pos_[cfg::NJ];
  float from_[cfg::NJ], to_[cfg::NJ];
  float u_ = 1, T_ = 0;
  float pause_ = 0;
  bool inPause_ = false;
  float poseTarget_[cfg::NJ];
};

// Minimum-jerk position profile s(u) = 10u^3 - 15u^4 + 6u^5, u in [0, 1].
// Peak speed is 1.875 * distance / duration, at u = 0.5.
inline float minJerk(float u) {
  if (u <= 0) return 0;
  if (u >= 1) return 1;
  return u * u * u * (10.0f + u * (-15.0f + 6.0f * u));
}
