// Tunable values of the controller. Units: SI unless the name says otherwise.
// "MEASURE" marks a design value to replace with the value measured on the real robot
// during the build phase (tests T1 to T6 in the design book).
#pragma once
#include <stdint.h>

namespace cfg {

// ---- timing
const float DT = 0.01f;                 // control period, s (100 Hz loop)
const uint16_t DT_MS = 10;

// ---- base geometry (CAD values)
const float WHEEL_RADIUS = 0.065f;      // m, 130 mm wheel
const float TRACK = 0.300f;             // m, wheel centre to wheel centre. MEASURE: calibrate with TURN 360
const int32_t COUNTS_PER_REV = 1320;    // encoder counts per wheel turn, 4x decoding.
                                        // MEASURE: 11 pulses x 30:1 gearbox x 4 edges is typical for
                                        // 12 V 200 rpm encoder gearmotors; turn a wheel 10 times and count.
const float WHEEL_NOLOAD = 20.94f;      // rad/s, 200 rpm at 12 V (kit list)

// ---- motion limits (design book, section 1.4.2)
const float V_MAX = 0.40f;              // m/s, 29 % of the top speed
const float A_MAX = 0.50f;              // m/s^2, ramp limit on every speed reference
const float V_TURN = 0.12f;             // m/s per wheel when turning on the spot (0.8 rad/s)
const float V_CREEP = 0.01f;            // m/s, minimum profile speed until the target is reached
const float V_TELEOP_MAX = 0.30f;       // m/s, VEL command limit
const float W_TELEOP_MAX = 1.20f;       // rad/s, VEL command limit

// ---- wheel loops
const float FF = 1.0f / (WHEEL_NOLOAD * WHEEL_RADIUS);  // duty per m/s (feed-forward)
// Mechanical time constant of a wheel: J / k with J = m r^2 / 2 per wheel (4.8 kg robot, r = 65 mm)
// and k = stall torque / no-load speed = 0.98 N.m / 20.9 rad/s, so about 0.2 s. MEASURE with a step.
const float TAU_MOTOR = 0.20f;          // s, acceleration feed-forward
const float KP = 2.0f;                  // duty per m/s of speed error
const float KI = 10.0f;                 // duty per (m/s * s)
const float I_LIMIT = 0.4f;             // integrator clamp, duty
const float KPOS = 4.0f;                // m/s per m of position error (each wheel tracks its reference)
const float POS_CORR_MAX = 0.10f;       // m/s, clamp on that correction
const float POS_HOLD_TOL = 0.001f;      // m, a move ends when both wheels are this close and still
const float HOLD_TIMEOUT = 1.5f;        // s, or after this long at rest
const float SPEED_ALPHA = 0.3f;         // low-pass on the measured wheel speed

// ---- obstacle stop (design book, sections 1.1 and 4.1)
const int16_t OBST_BRAKE_MM = 550;      // braking starts here
const int16_t OBST_CLEAR_MM = 650;      // forward motion allowed again above this (hysteresis)
const int16_t RANGE_MIN_MM = 20;        // closer readings are treated as invalid
const int16_t TOF_MAX_MM = 1200;        // VL53L0X default mode
const int16_t SONAR_MAX_MM = 1000;      // set by the echo timeout in main.cpp
const uint16_t RANGE_STALE_MS = 500;    // a sensor silent this long is ignored
const float OBST_BRAKE_DECEL = 0.5f;    // m/s^2, obstacle braking. Above about 0.6 the robot pitches onto its front caster (book 1.4.1)

// ---- safety
const float VBAT_CUTOFF = 9.9f;         // V, 3.3 V per cell
const float VBAT_WARN = 10.5f;          // V
const float VBAT_ABSENT = 5.0f;         // V, below this the divider reads nothing (USB-only bench)
const uint16_t VBAT_CUTOFF_MS = 2000;   // must stay low this long (servo current dips)
const uint16_t VEL_TIMEOUT_MS = 500;    // VEL stream watchdog
const float SLIP_LIMIT_DEG = 10.0f;     // encoder heading vs gyro heading, per turn

// ---- servos
const float SERVO_PEAK_DPS = 90.0f;     // deg/s, peak speed of every interpolated move
const float SERVO_MIN_MOVE_S = 0.25f;   // s, shortest move
const uint16_t SERVO_GROUP_DELAY_MS = 300;  // strip A, then strip B, on SERVO ON

// Joint order = PCA9685 channel order (wiring sheet 3).
enum Joint { L_PITCH = 0, L_ROLL, L_ELBOW, R_PITCH, R_ROLL, R_ELBOW, HEAD_PAN, HEAD_TILT, NJ };

// Joint limits in degrees, in the digital-twin convention (sim/README.md):
// pitch + raises the arm forward, roll + swings the arm outward, elbow + bends the forearm forward,
// pan + turns the head left, tilt + looks down. Zero = CAD pose (arms down, head forward).
const float J_MIN[NJ] = {-45, -30, 0, -45, -30, 0, -90, -30};
const float J_MAX[NJ] = {180, 120, 135, 180, 120, 135, 90, 30};
// The forearm can reach the torso side plate when the arm hangs and rolls inward.
const float ROLL_MIN_ARM_DOWN = -15.0f; // deg, applies while pitch is below:
const float ROLL_RULE_PITCH = 45.0f;    // deg
// Servo strip of each channel (wiring sheet 3): 0 = strip A, 1 = strip B.
const uint8_t SERVO_STRIP[NJ] = {0, 0, 0, 1, 1, 1, 0, 1};

}  // namespace cfg
