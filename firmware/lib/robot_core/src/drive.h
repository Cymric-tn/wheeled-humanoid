// Differential drive: odometry, speed profiles, and per wheel a position reference (the integral of the
// profile) tracked by a speed loop (PI with feed-forward). Tracking position, not only speed, makes
// the robot stop on the target even though the motor lags, and keeps the two wheels in step.
// Speeds are in the robot convention: positive wheel speed moves the robot forward.
// Motor and encoder polarity are handled in the hardware layer (main.cpp), not here.
#pragma once
#include <stdint.h>

class Drive {
 public:
  enum Mode : uint8_t { IDLE, DISTANCE, TURN, VELOCITY, STOPPING };
  enum Result : uint8_t { NONE, DONE_OK, DONE_STOPPED, DONE_BLOCKED };

  void reset(int32_t countL, int32_t countR);
  void startDistance(float metres);          // signed: + forward
  void startTurn(float radians);             // signed: + left (counter-clockwise from above)
  void setVelocity(float v, float w);        // teleop: m/s and rad/s
  void stop(Result why, float decel);        // ramp to zero
  void halt();                               // zero output now (faults)

  // One control step. Returns the duty for each wheel in [-1, 1].
  void update(int32_t countL, int32_t countR, float gyroZ, float dt, float& dutyL, float& dutyR);

  Mode mode() const { return mode_; }
  bool busy() const { return mode_ != IDLE; }
  bool movingForward() const { return lin_ > 0.005f || (mode_ == DISTANCE && target_ > 0); }
  Result takeResult() { Result r = result_; result_ = NONE; return r; }

  // odometry (encoders) and gyro heading
  float x() const { return x_; }
  float y() const { return y_; }
  float heading() const { return th_; }
  float gyroHeading() const { return gth_; }
  float odometer() const { return odo_; }
  float speedL() const { return vL_; }
  float speedR() const { return vR_; }
  float lastSlipDeg() const { return slip_deg_; }

 private:
  static float ramp(float cur, float target, float step);
  float wheelPI(float ref, float acc, float meas, float& integ, float dt);

  Mode mode_ = IDLE;
  Result result_ = NONE;
  int32_t lastL_ = 0, lastR_ = 0;
  float sL_ = 0, sR_ = 0;          // wheel travel since the move started, m
  float sRefL_ = 0, sRefR_ = 0;    // where each wheel should be: integral of the profile, m
  float hold_ = 0;                 // s spent settling in STOPPING
  bool endTurn_ = false;
  float target_ = 0;               // DISTANCE: path length, TURN: arc per wheel (signed)
  float lin_ = 0, turn_ = 0;       // ramped references: path speed and turn speed per wheel, m/s
  float linT_ = 0, turnT_ = 0;     // targets of those references (VELOCITY, STOPPING)
  float decel_ = 0;
  float vL_ = 0, vR_ = 0;          // measured wheel speeds, m/s
  float iL_ = 0, iR_ = 0;          // integrators
  float prevL_ = 0, prevR_ = 0;    // previous profile references, for the acceleration feed-forward
  float accFL_ = 0, accFR_ = 0;    // filtered profile accelerations
  float x_ = 0, y_ = 0, th_ = 0, gth_ = 0, odo_ = 0;
  float turnStartTh_ = 0, turnStartG_ = 0, slip_deg_ = 0;
};
