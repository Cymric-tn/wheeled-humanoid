# Serial protocol

The Arduino Mega, the digital twin (`sim/firmware_sim.py --tcp`) and the Pi service all use the same protocol: text lines at 115200 baud, ending in `\n`. Commands are case-insensitive. A person can type them in a serial monitor (`pio device monitor`) or with `nc localhost 5555` against the twin.

## Commands

| Command | Effect | Replies |
|---|---|---|
| `FWD <cm>` / `BACK <cm>` | Drive 1 to 500 cm at the capped speed (0.40 m/s, 0.5 m/s²) | `OK FWD`, then `DONE FWD ok\|blocked\|stopped x= y= th=` |
| `TURN <deg>` | Turn on the spot, −720 to 720°, positive = left | `OK TURN`, then `DONE TURN ...` |
| `VEL <mm/s> <deg/s>` | Teleop. It must be repeated within 500 ms, or the robot stops (`EVT VEL_TIMEOUT`) | errors only |
| `STOP` | Ramp the wheels to zero, freeze the servos, abort the demo | `OK STOP` |
| `ARM L\|R <pitch> <roll> <elbow>` | Move one arm to joint angles in degrees (twin convention) | `OK ARM`, then `DONE ARM ok` |
| `HEAD <pan> <tilt>` | Move the head | `OK HEAD`, then `DONE HEAD ok` |
| `WAVE [L\|R]`, `POINT [L\|R]`, `NOD`, `SHAKE`, `REST` | Stored gestures (default arm: right) | `OK WAVE`, then `DONE WAVE ok` |
| `SERVO ON\|OFF` | Power the servo outputs (strip A, then strip B 300 ms later) or release them | `OK SERVO ON` |
| `DEMO` | Minimum demonstration: `SERVO ON`, `FWD 100`, `BACK 30`, `TURN 90`, `FWD 300` (stops at the obstacle), `WAVE R`, `NOD`, `SHAKE`, `REST` | `OK DEMO`, `EVT DEMO <n> <step>` for each step, `DONE DEMO ok\|stopped\|interrupted\|aborted` |
| `RESET` | Clear a fault (refused while the E-stop is pressed) | `OK RESET` |
| `STATE?` | One state line | `STATE ...` |
| `STREAM <hz>` | Send `STATE` lines periodically, 0 to 20 Hz | `OK STREAM` |
| `PING`, `HELP` | | `PONG`, `CMDS ...` |

A new drive command replaces the current one, which then reports `DONE <cmd> stopped`. The same applies to a new gesture. Any motion command typed during `DEMO` interrupts the demo.

Bench-only commands are handled in `main.cpp`, so they exist only on the real Mega. They are accepted only while the robot is idle, and the Pi web page cannot send them:

| Command | Use |
|---|---|
| `CAL INFO` | Which devices answered at boot (ToF, IMU, PCA9685), gyro bias, loop overruns |
| `CAL ENC` | Raw encoder counts: turn each wheel by hand to check direction and counts per turn |
| `CAL MOTOR <dl> <dr>` | Raw duty (at most ±0.5) for 1 s, with the wheels off the floor: checks motor direction |
| `CAL PULSE <ch> <us>` | Raw servo pulse on one channel: finds each servo's centre and direction |

## Messages from the controller

| Message | Meaning |
|---|---|
| `READY humanoid-fw 1.0` | After reset (opening the USB port resets the Mega) |
| `EVT HW tof1= tof2= mpu= pca=` | Which devices answered at boot (1 = present) |
| `EVT BLOCKED <mm>` | Obstacle under 550 mm while driving forward: braking at 0.5 m/s² |
| `EVT CLEAR <mm>` | Path clear again (over 650 mm) |
| `EVT NO_RANGE` | No range sensor has reported for 500 ms: forward motion is refused |
| `EVT FAULT ESTOP` / `EVT FAULT LOWBAT` | Drivers disabled, servos released, demo aborted |
| `EVT ESTOP_RELEASED send RESET` | The button is released; the fault stays until `RESET` |
| `EVT VBAT_LOW <mV>` / `EVT VBAT_ABSENT` | Below 10.5 V / the divider reads nothing (USB-only bench) |
| `EVT SLIP <deg>` | After a turn, the encoder and gyro headings differ by more than 10° |
| `EVT VEL_TIMEOUT` | The teleop stream stopped |
| `ERR <cmd> <reason>` | `RANGE`, `SYNTAX`, `UNKNOWN`, `BLOCKED <mm>`, `FAULT <bits>` |

`STATE` fields: `st` (IDLE, BUSY, FAULT), `f` (fault bits: 1 = E-stop, 2 = low battery), `x`, `y` (mm, odometry), `th`, `gth` (heading from the encoders and from the gyro, 0.1°), `v` (mm/s), `obst` (mm, −1 = no sensor), `sens` (bit mask of the fresh range sensors), `vbat` (mV), `drv`, `gest` (running command), `srv` (servos on), `demo` (step number, 0 = none).
