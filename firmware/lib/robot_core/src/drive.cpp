#include "drive.h"
#include <math.h>
#include "config.h"

using namespace cfg;

static const float M_PER_COUNT = 2.0f * 3.14159265f * WHEEL_RADIUS / COUNTS_PER_REV;

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static float signf(float v) { return v < 0 ? -1.0f : 1.0f; }

void Drive::reset(int32_t countL, int32_t countR) {
  lastL_ = countL;
  lastR_ = countR;
  mode_ = IDLE;
  result_ = NONE;
  lin_ = turn_ = linT_ = turnT_ = 0;
  vL_ = vR_ = iL_ = iR_ = 0;
  x_ = y_ = th_ = gth_ = odo_ = 0;
}

float Drive::ramp(float cur, float target, float step) {
  if (target > cur + step) return cur + step;
  if (target < cur - step) return cur - step;
  return target;
}

void Drive::startDistance(float metres) {
  mode_ = DISTANCE;
  result_ = NONE;
  target_ = metres;
  sL_ = sR_ = sRefL_ = sRefR_ = 0;
  turn_ = 0;
}

void Drive::startTurn(float radians) {
  mode_ = TURN;
  result_ = NONE;
  target_ = radians * TRACK * 0.5f;  // arc each wheel travels (right wheel +, left wheel -)
  sL_ = sR_ = sRefL_ = sRefR_ = 0;
  lin_ = 0;
  turnStartTh_ = th_;
  turnStartG_ = gth_;
}

void Drive::setVelocity(float v, float w) {
  if (mode_ != VELOCITY) {
    result_ = NONE;
    if (mode_ == IDLE) sL_ = sR_ = sRefL_ = sRefR_ = 0;
  }
  mode_ = VELOCITY;
  linT_ = clampf(v, -V_TELEOP_MAX, V_TELEOP_MAX);
  turnT_ = clampf(w, -W_TELEOP_MAX, W_TELEOP_MAX) * TRACK * 0.5f;
}

void Drive::stop(Result why, float decel) {
  if (mode_ == IDLE) return;
  mode_ = STOPPING;
  result_ = why;
  linT_ = turnT_ = 0;
  decel_ = decel;
  hold_ = 0;
}

void Drive::halt() {
  if (mode_ != IDLE && result_ == NONE) result_ = DONE_STOPPED;
  mode_ = IDLE;
  lin_ = turn_ = linT_ = turnT_ = 0;
  iL_ = iR_ = 0;
  sL_ = sR_ = sRefL_ = sRefR_ = 0;
}

float Drive::wheelPI(float ref, float acc, float meas, float& integ, float dt) {
  float e = ref - meas;
  float duty = FF * (ref + TAU_MOTOR * acc) + KP * e + integ;
  // integrate only when the output is not saturated in the direction of the error (anti-windup)
  if (!((duty >= 1.0f && e > 0) || (duty <= -1.0f && e < 0))) {
    integ = clampf(integ + KI * e * dt, -I_LIMIT, I_LIMIT);
  }
  return clampf(duty, -1.0f, 1.0f);
}

void Drive::update(int32_t countL, int32_t countR, float gyroZ, float dt, float& dutyL, float& dutyR) {
  // ---- measure
  float dl = (countL - lastL_) * M_PER_COUNT;
  float dr = (countR - lastR_) * M_PER_COUNT;
  lastL_ = countL;
  lastR_ = countR;
  vL_ += SPEED_ALPHA * (dl / dt - vL_);
  vR_ += SPEED_ALPHA * (dr / dt - vR_);
  sL_ += dl;
  sR_ += dr;
  float d = 0.5f * (dl + dr);
  float dth = (dr - dl) / TRACK;
  x_ += d * cosf(th_ + 0.5f * dth);
  y_ += d * sinf(th_ + 0.5f * dth);
  th_ += dth;
  gth_ += gyroZ * dt;
  odo_ += fabsf(d);

  // ---- profile: path speed lin_ and turn speed turn_ (per wheel), ramp-limited
  float step = A_MAX * dt;
  bool finished = false;
  if (mode_ == DISTANCE || mode_ == TURN) {
    bool turn = mode_ == TURN;
    float progress = 0.5f * (turn ? (sRefR_ - sRefL_) : (sRefL_ + sRefR_)) * signf(target_);
    float remaining = fabsf(target_) - progress;
    // plan the braking at 80 % of the limit: the margin lets the loops catch up with the motor lag
    float vdes = sqrtf(2.0f * 0.8f * A_MAX * (remaining > 0 ? remaining : 0));
    float vcap = turn ? V_TURN : V_MAX;
    if (vdes > vcap) vdes = vcap;
    if (vdes < V_CREEP) vdes = V_CREEP;
    float& ref = turn ? turn_ : lin_;
    ref = ramp(ref, vdes * signf(target_), step);
    if (fabsf(ref) * dt >= remaining) {  // this step reaches the target: land on it exactly
      finished = true;
      ref = 0;
    }
  } else if (mode_ == VELOCITY) {
    lin_ = ramp(lin_, linT_, step);
    turn_ = ramp(turn_, turnT_, step);
  } else if (mode_ == STOPPING) {
    float s = decel_ * dt;
    lin_ = ramp(lin_, 0, s);
    turn_ = ramp(turn_, 0, s);
  }

  // ---- position references of each wheel (the integral of the profile)
  float vRefL = lin_ - turn_, vRefR = lin_ + turn_;
  sRefL_ += vRefL * dt;
  sRefR_ += vRefR * dt;
  if (finished) {
    if (mode_ == DISTANCE) sRefL_ = sRefR_ = target_;
    else { sRefL_ = -target_; sRefR_ = target_; }
    if (mode_ == TURN) slip_deg_ = 0;  // measured once the wheels have settled, below
    endTurn_ = mode_ == TURN;
    mode_ = STOPPING;
    result_ = DONE_OK;
    decel_ = A_MAX;
    hold_ = 0;
  }
  if (mode_ == STOPPING) {
    hold_ += dt;
    bool still = fabsf(vL_) < 0.01f && fabsf(vR_) < 0.01f;
    bool onTarget = fabsf(sRefL_ - sL_) < POS_HOLD_TOL && fabsf(sRefR_ - sR_) < POS_HOLD_TOL;
    if (lin_ == 0 && turn_ == 0 && still && (onTarget || hold_ > HOLD_TIMEOUT)) {
      if (endTurn_) slip_deg_ = fabsf((th_ - turnStartTh_) - (gth_ - turnStartG_)) * 57.29578f;
      endTurn_ = false;
      mode_ = IDLE;
      if (result_ == NONE) result_ = DONE_STOPPED;
    }
  }

  // ---- wheel loops: speed reference + position correction, then PI with feed-forward
  if (mode_ == IDLE) {
    iL_ = iR_ = 0;
    prevL_ = prevR_ = accFL_ = accFR_ = 0;
    dutyL = dutyR = 0;
    return;
  }
  float cmdL = vRefL + clampf(KPOS * (sRefL_ - sL_), -POS_CORR_MAX, POS_CORR_MAX);
  float cmdR = vRefR + clampf(KPOS * (sRefR_ - sR_), -POS_CORR_MAX, POS_CORR_MAX);
  // acceleration feed-forward, low-passed (0.1 s) so a change of profile phase does not jerk the robot
  float accL = clampf((vRefL - prevL_) / dt, -1.2f * A_MAX, 1.2f * A_MAX);
  float accR = clampf((vRefR - prevR_) / dt, -1.2f * A_MAX, 1.2f * A_MAX);
  prevL_ = vRefL;
  prevR_ = vRefR;
  accFL_ += (accL - accFL_) * dt / 0.1f;
  accFR_ += (accR - accFR_) * dt / 0.1f;
  dutyL = wheelPI(cmdL, accFL_, vL_, iL_, dt);
  dutyR = wheelPI(cmdR, accFR_, vR_, iR_, dt);
}
