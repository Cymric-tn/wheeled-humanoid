#!/usr/bin/env python3
"""Smoke test for the digital twin. Needs:  pip install mujoco numpy

Checks: both model files load, nothing intersects at rest, the robot settles on
its wheels, every servo reaches its target in the documented direction, and the
wheels drive the robot forward and turn it left.
Run from the V2 folder:   python3 sim/test_twin.py
"""
import os
import numpy as np
import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
fails = []


def check(cond, msg):
    print(("  ok   " if cond else "  FAIL ") + msg)
    if not cond:
        fails.append(msg)


# 1. URDF parses (MuJoCo's URDF importer; ROS tools read the same file)
urdf = mujoco.MjModel.from_xml_path(os.path.join(HERE, "wheeled_humanoid_v2.urdf"))
print(f"URDF: {urdf.nbody} bodies, {urdf.njnt} joints (MuJoCo fixes the URDF root to the world, so base mass is not counted here)")

m = mujoco.MjModel.from_xml_path(os.path.join(HERE, "scene.xml"))
d = mujoco.MjData(m)
print(f"MJCF: {m.nbody} bodies, {m.njnt} joints, {m.nu} actuators, mass {sum(m.body_mass):.3f} kg")
aid = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i): i for i in range(m.nu)}


def jpos(name):
    return d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)]]


def body_pos(name):
    return d.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name)].copy()


# 2. no self-intersection at the CAD pose
mujoco.mj_forward(m, d)
floor = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "floor")
self_contacts = [(mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[c.geom1]),
                  mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[c.geom2]))
                 for c in d.contact[:d.ncon] if floor not in (c.geom1, c.geom2)]
check(not self_contacts, f"no self-collision at rest {self_contacts or ''}")

# 3. settle
mujoco.mj_resetData(m, d)
for _ in range(1500):
    mujoco.mj_step(m, d)
z = body_pos("base_link")[2]
rp = d.qpos[3:7]
tilt = np.degrees(2 * np.arccos(min(1.0, abs(rp[0]))))
check(abs(z) < 0.01 and tilt < 3, f"settles on wheels+caster: base z={z*1000:.1f} mm, tilt={tilt:.2f} deg")

check(abs(d.sensor("tof_left").data[0] - (1.98 - 0.18 - body_pos("base_link")[0])) < 0.02, f"front ToF sees the wall 1.8 m ahead: {d.sensor('tof_left').data[0]:.3f} m")
# 4. each servo: reach target + direction check (world-frame motion of a child point)
tests = {  # actuator: (target rad, body to watch, axis index, expected sign)
    "l_sh_pitch": (0.8, "forearm_left_link", 0, +1),   # arm forward  -> +x
    "r_sh_pitch": (0.8, "forearm_right_link", 0, +1),
    "l_sh_roll": (0.6, "forearm_left_link", 1, +1),    # outward      -> +y
    "r_sh_roll": (0.6, "forearm_right_link", 1, -1),   # outward      -> -y
    "l_elbow": (0.8, "forearm_left_link", 0, +1),      # forearm fwd  -> +x (watch COM)
    "r_elbow": (0.8, "forearm_right_link", 0, +1),
    "head_pan": (0.6, "head_link", 1, +1),             # turn left: head COM (ahead of axis) -> +y
    "head_tilt": (0.4, "head_link", 2, -1),            # look down: head COM drops
}
for act, (tgt, body, ax, sgn) in tests.items():
    mujoco.mj_resetData(m, d)
    for _ in range(500):
        mujoco.mj_step(m, d)
    bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, body)
    p0 = d.xipos[bid].copy()
    d.ctrl[:] = 0
    d.ctrl[aid[act]] = tgt
    for _ in range(1000):
        mujoco.mj_step(m, d)
    q = jpos(act + "_joint")
    moved = (d.xipos[bid] - p0)[ax] * sgn
    check(abs(q - tgt) < 0.08 and moved > 0.002, f"{act:10s} target {tgt:+.2f} reached {q:+.3f} rad, moved {moved*1000:+.1f} mm the expected way")

# 5. drive straight, then spin
mujoco.mj_resetData(m, d)
for _ in range(500):
    mujoco.mj_step(m, d)
x0 = body_pos("base_link")
d.ctrl[aid["wheel_left"]] = d.ctrl[aid["wheel_right"]] = 5.0   # wheel axis is +Y: positive spin rolls forward
for _ in range(1000):
    mujoco.mj_step(m, d)
dx = body_pos("base_link") - x0
check(dx[0] > 0.3 and abs(dx[1]) < 0.05, f"drive forward 2 s at 5 rad/s: dx={dx[0]:.3f} m, dy={dx[1]:.3f} m (ideal {5*0.065*2:.2f} m)")
yaw0 = d.qpos[3:7].copy()
d.ctrl[aid["wheel_left"]], d.ctrl[aid["wheel_right"]] = -3.0, 3.0
for _ in range(500):
    mujoco.mj_step(m, d)
w, x, y, zq = d.qpos[3:7]
yaw = np.degrees(np.arctan2(2 * (w * zq + x * y), 1 - 2 * (y * y + zq * zq)))
check(yaw > 20, f"left wheel back, right wheel forward turns left: yaw {yaw:.1f} deg")
print(f"sensors after the spin: imu_accel_z={d.sensor('imu_accel').data[2]:.2f} m/s^2")

print("\nALL CHECKS PASSED" if not fails else f"\n{len(fails)} CHECK(S) FAILED")
raise SystemExit(1 if fails else 0)
