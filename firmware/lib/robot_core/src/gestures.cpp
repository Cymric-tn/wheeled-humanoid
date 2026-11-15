#include "gestures.h"
#include <math.h>

using namespace cfg;

// Gestures are written for one arm in the twin convention, which is the same for both arms
// (pitch + forward, roll + outward, elbow + bends forward). Columns: arm pitch, arm roll, elbow,
// other arm pitch, other arm roll, other arm elbow, head pan, head tilt. Head pan is mirrored
// for the left arm so the robot looks at the waving hand.
#define K KEEP
static const Keyframe WAVE[] = {
    {{0, 70, 60, K, K, K, -30, -10}, 0},
    {{0, 70, 95, K, K, K, K, K}, 0},
    {{0, 70, 40, K, K, K, K, K}, 0},
    {{0, 70, 95, K, K, K, K, K}, 0},
    {{0, 70, 40, K, K, K, K, K}, 0},
    {{0, 70, 95, K, K, K, K, K}, 0},
    {{0, 70, 60, K, K, K, K, K}, 200},
    {{0, 0, 0, K, K, K, 0, 0}, 0},
};
static const Keyframe POINT[] = {
    {{85, 0, 0, K, K, K, -20, 5}, 1500},
    {{0, 0, 0, K, K, K, 0, 0}, 0},
};
static const Keyframe NOD[] = {
    {{K, K, K, K, K, K, 0, 20}, 0},
    {{K, K, K, K, K, K, 0, -10}, 0},
    {{K, K, K, K, K, K, 0, 20}, 0},
    {{K, K, K, K, K, K, 0, -10}, 0},
    {{K, K, K, K, K, K, 0, 0}, 0},
};
static const Keyframe SHAKE[] = {
    {{K, K, K, K, K, K, 30, 0}, 0},
    {{K, K, K, K, K, K, -30, 0}, 0},
    {{K, K, K, K, K, K, 30, 0}, 0},
    {{K, K, K, K, K, K, -30, 0}, 0},
    {{K, K, K, K, K, K, 0, 0}, 0},
};
static const Keyframe REST[] = {
    {{0, 0, 0, 0, 0, 0, 0, 0}, 0},
};
#undef K

#define LEN(a) (uint8_t)(sizeof(a) / sizeof(a[0]))

const char* Gestures::name(GestureId id) {
  switch (id) {
    case G_REST: return "REST";
    case G_WAVE: return "WAVE";
    case G_POINT: return "POINT";
    case G_NOD: return "NOD";
    case G_SHAKE: return "SHAKE";
    case G_POSE: return "POSE";
    default: return "NONE";
  }
}

void Gestures::reset() {
  for (uint8_t j = 0; j < NJ; j++) pos_[j] = from_[j] = to_[j] = 0;
  id_ = G_NONE;
  u_ = 1;
}

void Gestures::applyLimits(float q[NJ]) {
  for (uint8_t j = 0; j < NJ; j++) {
    if (q[j] < J_MIN[j]) q[j] = J_MIN[j];
    if (q[j] > J_MAX[j]) q[j] = J_MAX[j];
  }
}

void Gestures::startSegment(const float target[NJ]) {
  float maxd = 0;
  for (uint8_t j = 0; j < NJ; j++) {
    from_[j] = pos_[j];
    to_[j] = isnan(target[j]) ? pos_[j] : target[j];
  }
  applyLimits(to_);
  for (uint8_t j = 0; j < NJ; j++) {
    float d = fabsf(to_[j] - from_[j]);
    if (d > maxd) maxd = d;
  }
  T_ = 1.875f * maxd / SERVO_PEAK_DPS;
  if (T_ < SERVO_MIN_MOVE_S) T_ = SERVO_MIN_MOVE_S;
  u_ = 0;
}

// Load the next keyframe of the sequence. Returns false at the end.
bool Gestures::nextKey() {
  if (seqIdx_ >= seqLen_) return false;
  const Keyframe& k = seq_[seqIdx_++];
  float t[NJ];
  // map "this arm / other arm" columns to left / right joints
  const uint8_t armA = side_ ? R_PITCH : L_PITCH;
  const uint8_t armB = side_ ? L_PITCH : R_PITCH;
  for (uint8_t c = 0; c < NJ; c++) {
    uint8_t j;
    if (c < 3) j = armA + c;
    else if (c < 6) j = armB + (c - 3);
    else j = c;
    float v = (k.deg[c] == KEEP) ? NAN : (float)k.deg[c];
    if (j == HEAD_PAN && !isnan(v) && side_ == 0) v = -v;  // look toward the left hand
    t[j] = v;
  }
  startSegment(t);
  pause_ = k.pause_ms * 0.001f;
  inPause_ = false;
  return true;
}

void Gestures::play(GestureId id, uint8_t side) {
  side_ = side;
  seqIdx_ = 0;
  switch (id) {
    case G_WAVE: seq_ = WAVE; seqLen_ = LEN(WAVE); break;
    case G_POINT: seq_ = POINT; seqLen_ = LEN(POINT); break;
    case G_NOD: seq_ = NOD; seqLen_ = LEN(NOD); break;
    case G_SHAKE: seq_ = SHAKE; seqLen_ = LEN(SHAKE); break;
    default: seq_ = REST; seqLen_ = LEN(REST); id = G_REST; side_ = 1; break;
  }
  id_ = id;
  nextKey();
}

void Gestures::pose(const float target[NJ]) {
  for (uint8_t j = 0; j < NJ; j++) poseTarget_[j] = target[j];
  seq_ = 0;
  seqLen_ = seqIdx_ = 0;
  id_ = G_POSE;
  startSegment(poseTarget_);
  pause_ = 0;
  inPause_ = false;
}

void Gestures::stop() {
  id_ = G_NONE;
  u_ = 1;
  for (uint8_t j = 0; j < NJ; j++) from_[j] = to_[j] = pos_[j];
}

void Gestures::update(float dt) {
  if (id_ == G_NONE) return;
  if (!inPause_) {
    u_ += dt / T_;
    float s = minJerk(u_);
    for (uint8_t j = 0; j < NJ; j++) pos_[j] = from_[j] + (to_[j] - from_[j]) * s;
    applyLimits(pos_);
    if (u_ >= 1) inPause_ = true;
    return;
  }
  pause_ -= dt;
  if (pause_ > 0) return;
  if (!nextKey()) id_ = G_NONE;
}
