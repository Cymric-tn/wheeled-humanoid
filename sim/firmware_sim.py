#!/usr/bin/env python3
"""Firmware in the loop: the Arduino Mega controller drives the MuJoCo digital twin.

The C++ controller in firmware/lib/robot_core is compiled for this computer (make -C firmware lib)
and called every 10 ms, exactly as main.cpp calls it on the Mega. This script plays the hardware:
it turns wheel angles into encoder counts, rangefinder rays into ToF and ultrasonic readings, the IMU
site into a gyro rate, and applies the controller's motor duties and servo angles to the twin.

  python sim/firmware_sim.py                    run DEMO headless, write sim/results/
  python sim/firmware_sim.py --video            the same, plus sim/results/demo.mp4
  mjpython sim/firmware_sim.py --viewer         watch the DEMO live (macOS needs mjpython)
  python sim/firmware_sim.py --tcp 5555         serve the serial protocol on TCP, in real time:
                                                  nc localhost 5555, or the Pi service with
                                                  --port socket://localhost:5555
  python sim/firmware_sim.py --script "FWD 50; TURN -90; WAVE L"
"""
import argparse
import ctypes
import json
import math
import os
import re
import selectors
import socket
import subprocess
import sys
import time

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
FW = os.path.join(ROOT, "firmware")
CORE_SRC = os.path.join(FW, "lib", "robot_core", "src")
LIB = os.path.join(FW, "build", "librobot_core." + ("dylib" if sys.platform == "darwin" else "so"))
RESULTS = os.path.join(HERE, "results")

JOINTS = ["l_sh_pitch", "l_sh_roll", "l_elbow", "r_sh_pitch", "r_sh_roll", "r_elbow", "head_pan", "head_tilt"]
# kit list ratings (spec book): TD-8130MG 30 kg.cm, RCXAZ 18 kg.cm, MG996 13 kg.cm
SERVO_RATED_KGCM = [30, 30, 18, 30, 30, 18, 13, 13]
KGCM = 0.0980665                       # N.m per kg.cm
MOTOR_RATED_NM = 10 * KGCM             # gearmotor, 10 kg.cm (taken as stall torque: conservative)


def read_config():
    """Constants of firmware/lib/robot_core/src/config.h, so the two never disagree."""
    cfg = {}
    with open(os.path.join(CORE_SRC, "config.h")) as f:
        for m in re.finditer(r"const\s+\w+\s+(\w+)\s*=\s*([-0-9.]+)f?\s*;", f.read()):
            cfg[m.group(1)] = float(m.group(2))
    return cfg


CFG = read_config()


# ------------------------------------------------------------------ controller (ctypes)
class CoreIn(ctypes.Structure):
    _fields_ = [("encL", ctypes.c_int32), ("encR", ctypes.c_int32), ("range_mm", ctypes.c_int16 * 4),
                ("range_new", ctypes.c_uint8 * 4), ("gyro_z", ctypes.c_float), ("vbat", ctypes.c_float),
                ("estop", ctypes.c_uint8)]


class CoreOut(ctypes.Structure):
    _fields_ = [("duty", ctypes.c_float * 2), ("drive_enable", ctypes.c_uint8), ("joint_deg", ctypes.c_float * 8),
                ("servo_on", ctypes.c_uint8 * 8)]


class Core:
    def __init__(self):
        subprocess.run(["make", "-s", "-C", FW, "lib"], check=True)
        lib = ctypes.CDLL(LIB)
        lib.core_new.restype = ctypes.c_void_p
        lib.core_free.argtypes = [ctypes.c_void_p]
        lib.core_line.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
        lib.core_step.argtypes = [ctypes.c_void_p, ctypes.POINTER(CoreIn), ctypes.POINTER(CoreOut)]
        lib.core_pop.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
        lib.core_pop.restype = ctypes.c_int
        lib.core_probe.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_float)]
        a, b = ctypes.c_int(), ctypes.c_int()
        lib.core_sizes(ctypes.byref(a), ctypes.byref(b))
        assert (a.value, b.value) == (ctypes.sizeof(CoreIn), ctypes.sizeof(CoreOut)), "struct layout mismatch"
        self.lib, self.h = lib, lib.core_new()
        self.out = CoreOut()
        self._buf = ctypes.create_string_buffer(128)
        self._probe = (ctypes.c_float * 11)()

    def line(self, text):
        self.lib.core_line(self.h, text.encode())

    def step(self, inp):
        self.lib.core_step(self.h, ctypes.byref(inp), ctypes.byref(self.out))
        return self.out

    def messages(self):
        out = []
        while self.lib.core_pop(self.h, self._buf, 128):
            out.append(self._buf.value.decode())
        return out

    def probe(self):
        self.lib.core_probe(self.h, self._probe)
        p = list(self._probe)
        return dict(obst=p[0], x=p[1], y=p[2], th=p[3], gth=p[4], vL=p[5], vR=p[6],
                    drive_busy=p[7] > 0, gest_busy=p[8] > 0, demo=p[9] > 0, faults=int(p[10]))


# ------------------------------------------------------------------ hardware model around the twin
class SimRobot:
    def __init__(self, scene, vbat=11.4, seed=0, tof_noise_mm=5.0, sonar_noise_mm=10.0):
        self.m = mujoco.MjModel.from_xml_path(scene)
        self.d = mujoco.MjData(self.m)
        m = self.m
        self.nsub = max(1, round(CFG["DT"] / m.opt.timestep))
        self.rng = np.random.default_rng(seed)
        self.vbat, self.estop = vbat, False
        self.tof_noise, self.sonar_noise = tof_noise_mm, sonar_noise_mm
        aid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
        self.a_wheel = [aid("wheel_left"), aid("wheel_right")]
        self.a_servo = [aid(n) for n in JOINTS]
        self.q_wheel = [m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]
                        for n in ("wheel_left_joint", "wheel_right_joint")]
        self.q_servo = [m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n + "_joint")] for n in JOINTS]
        sadr = lambda n: m.sensor_adr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, n)]
        self.s_range = [sadr(n) for n in ("tof_left", "tof_right", "sonar_left", "sonar_right")]
        self.s_gyro = sadr("imu_gyro")
        self.site_tof = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "tof_left_link")
        self.base = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        # Gearmotor as a DC motor torque-speed line: torque = T_stall * (duty * V/12 - w / w_noload).
        # A MuJoCo velocity actuator gives force = kv * ctrl - kv * qvel, so kv = T_stall / w_noload.
        kv = MOTOR_RATED_NM / CFG["WHEEL_NOLOAD"]
        # The no-load speed already includes the gearbox losses, so the twin's generic wheel-joint
        # damping (0.05 N.m.s, which would cost 0.3 N.m at cruise) is reduced to bearing friction.
        for n in ("wheel_left_joint", "wheel_right_joint"):
            m.dof_damping[m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]] = 0.002
        for a in self.a_wheel:
            m.actuator_gainprm[a, 0] = kv
            m.actuator_biasprm[a, :3] = [0, 0, -kv]
            m.actuator_forcerange[a] = [-MOTOR_RATED_NM, MOTOR_RATED_NM]
            m.actuator_ctrllimited[a] = 0
        # servo torque limits from the kit ratings; gains kept to make a servo limp when it gets no pulse
        for a, r in zip(self.a_servo, SERVO_RATED_KGCM):
            m.actuator_forcerange[a] = [-r * KGCM, r * KGCM]
        self.servo_gain = [(m.actuator_gainprm[a, 0], m.actuator_biasprm[a, 1], m.actuator_biasprm[a, 2])
                           for a in self.a_servo]
        self.servo_live = [True] * 8
        self.tof_next = [0, 11]  # ms; the two ToF sensors are not in phase
        self.sonar_next, self.sonar_side = 0, 0
        self.t_ms = 0
        mujoco.mj_forward(m, self.d)

    def settle(self, seconds=1.0):
        for _ in range(int(seconds / self.m.opt.timestep)):
            mujoco.mj_step(self.m, self.d)

    def sensors(self):
        d, inp = self.d, CoreIn()
        cpr = CFG["COUNTS_PER_REV"] / (2 * math.pi)
        inp.encL = int(math.floor(d.qpos[self.q_wheel[0]] * cpr))
        inp.encR = int(math.floor(d.qpos[self.q_wheel[1]] * cpr))
        for i in range(2):  # VL53L0X, continuous mode, about one reading per 33 ms
            if self.t_ms >= self.tof_next[i]:
                self.tof_next[i] += 33
                r = d.sensordata[self.s_range[i]] * 1000
                ok = 0 <= r <= CFG["TOF_MAX_MM"]
                inp.range_mm[i] = int(r + self.rng.normal(0, self.tof_noise)) if ok else -1
                inp.range_new[i] = 1
        if self.t_ms >= self.sonar_next:  # HC-SR04, left and right alternate every 60 ms
            self.sonar_next += 60
            i = 2 + self.sonar_side
            self.sonar_side ^= 1
            r = d.sensordata[self.s_range[i]] * 1000
            ok = 0 <= r <= CFG["SONAR_MAX_MM"]
            inp.range_mm[i] = int(r + self.rng.normal(0, self.sonar_noise)) if ok else -1
            inp.range_new[i] = 1
        inp.gyro_z = float(d.sensordata[self.s_gyro + 2])
        inp.vbat = self.vbat
        inp.estop = int(self.estop)
        return inp

    def actuate(self, out):
        m, d = self.m, self.d
        for k, a in enumerate(self.a_wheel):
            d.ctrl[a] = out.duty[k] * CFG["WHEEL_NOLOAD"] * (self.vbat / 12.0) if out.drive_enable else 0.0
            if not out.drive_enable:  # BTS7960 disabled: motor terminals open, wheel free
                m.actuator_gainprm[a, 0] = 0
                m.actuator_biasprm[a, 2] = 0
            else:
                kv = MOTOR_RATED_NM / CFG["WHEEL_NOLOAD"]
                m.actuator_gainprm[a, 0] = kv
                m.actuator_biasprm[a, 2] = -kv
        for k, a in enumerate(self.a_servo):
            on = bool(out.servo_on[k])
            if on != self.servo_live[k]:
                g, b1, b2 = self.servo_gain[k] if on else (0, 0, 0)
                m.actuator_gainprm[a, 0], m.actuator_biasprm[a, 1], m.actuator_biasprm[a, 2] = g, b1, b2
                self.servo_live[k] = on
            d.ctrl[a] = math.radians(out.joint_deg[k])

    def advance(self):
        for _ in range(self.nsub):
            mujoco.mj_step(self.m, self.d)
        self.t_ms += int(CFG["DT_MS"])

    # ---- ground truth, for the logs
    def pose(self):
        p = self.d.xpos[self.base]
        R = self.d.xmat[self.base].reshape(3, 3)
        yaw = math.atan2(R[1, 0], R[0, 0])
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, R[2, 2]))))
        return float(p[0]), float(p[1]), yaw, tilt

    def true_range_mm(self):
        r = [self.d.sensordata[a] for a in self.s_range[:2]]
        r = [x for x in r if x >= 0]
        return min(r) * 1000 if r else float("nan")


def unwrap(prev, ang):
    while ang - prev > math.pi:
        ang -= 2 * math.pi
    while ang - prev < -math.pi:
        ang += 2 * math.pi
    return ang


# ------------------------------------------------------------------ one control step, logged
class Session:
    def __init__(self, robot, core, echo=True):
        self.robot, self.core, self.echo = robot, core, echo
        self.log, self.events = [], []
        self.yaw = 0.0
        self.label = ""

    def step(self):
        r, c = self.robot, self.core
        out = c.step(r.sensors())
        r.actuate(out)
        r.advance()
        t = r.t_ms / 1000
        for msg in c.messages():
            self.events.append((t, msg))
            if msg.startswith("EVT DEMO"):
                self.label = msg.split(" ", 3)[3]
            if self.echo:
                print(f"{t:7.2f}  {msg}")
        x, y, yaw, tilt = r.pose()
        self.yaw = unwrap(self.yaw, yaw)
        p = c.probe()
        vfwd = float(np.dot(r.d.qvel[:2], [math.cos(yaw), math.sin(yaw)])) if r.m.nq >= 7 else 0.0
        self.log.append(dict(
            t=t, x=x, y=y, yaw=math.degrees(self.yaw), tilt=tilt, v=vfwd,
            odo_x=p["x"], odo_y=p["y"], odo_th=math.degrees(p["th"]), gyro_th=math.degrees(p["gth"]),
            obst=p["obst"], true_range=r.true_range_mm(), dutyL=out.duty[0], dutyR=out.duty[1],
            tq_wheelL=float(r.d.actuator_force[r.a_wheel[0]]), tq_wheelR=float(r.d.actuator_force[r.a_wheel[1]]),
            **{f"tq_{n}": float(r.d.actuator_force[a]) for n, a in zip(JOINTS, r.a_servo)},
            **{f"q_{n}": math.degrees(r.d.qpos[q]) for n, q in zip(JOINTS, r.q_servo)},
            label=self.label))
        return p


# ------------------------------------------------------------------ DEMO run and its metrics
def run_demo(args):
    robot = SimRobot(os.path.join(HERE, "demo_scene.xml"), seed=args.seed)
    robot.settle(1.0)
    core = Core()
    s = Session(robot, core, echo=not args.quiet)
    renderer = frames = writer = None
    if args.video or not args.no_frames:
        renderer = mujoco.Renderer(robot.m, 720, 1280)
        frames = {}
    if args.video:
        import imageio.v2 as imageio
        os.makedirs(RESULTS, exist_ok=True)
        writer = imageio.get_writer(os.path.join(RESULTS, "demo.mp4"), fps=100 / 3, codec="libx264", quality=7,
                                    macro_block_size=8)
    for _ in range(50):  # let the range filters fill (0.5 s) before the command arrives
        s.step()
    core.line("DEMO")
    done, k = False, 0
    while not done and robot.t_ms < 90000:
        s.step()
        k += 1
        done = any(m.startswith("DONE DEMO") for _, m in s.events[-3:])
        if renderer is not None:
            want_frame = writer is not None and k % 3 == 0
            key = snapshot_key(s)
            if want_frame or (key and key not in frames):
                img = render(renderer, robot)
                if key and key not in frames:
                    frames[key] = img
                if want_frame:
                    writer.append_data(caption(img, s))
    for _ in range(100):  # one more second at rest
        s.step()
    if writer:
        writer.close()
    metrics = demo_metrics(s)
    write_results(s, metrics, frames)
    return metrics


def snapshot_key(s):
    """Moments for the figure: driving, stopped at the wall, waving, nodding."""
    lab, row = s.log[-1]["label"], s.log[-1]
    if lab == "FWD 100" and abs(row["v"]) > 0.35:
        return "1 driving at 0.40 m/s"
    if lab == "WAVE R" and row["q_r_sh_roll"] < 3:
        return "2 stopped before the wall"
    if lab == "WAVE R" and row["q_r_elbow"] > 90:
        return "3 waving"
    if lab == "NOD" and row["q_head_tilt"] > 18:
        return "4 nodding"
    return None


def render(renderer, robot):
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    x, y, yaw, _ = robot.pose()
    cam.lookat[:] = [x, y, 0.38]
    cam.distance, cam.azimuth, cam.elevation = 2.1, math.degrees(yaw) + 110, -14  # front-right, clear of the wall
    opt = mujoco.MjvOption()
    opt.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = 1
    renderer.update_scene(robot.d, camera=cam, scene_option=opt)
    return renderer.render().copy()


def caption(img, s):
    from PIL import Image, ImageDraw
    row = s.log[-1]
    im = Image.fromarray(img)
    dr = ImageDraw.Draw(im)
    obst = f"{row['obst']:.0f} mm" if row["obst"] >= 0 else "none"
    text = f"t = {row['t']:5.1f} s   step: {row['label'] or '-'}   speed {row['v']:+.2f} m/s   obstacle {obst}"
    dr.rectangle([0, 0, 1280, 44], fill=(20, 20, 30))
    dr.text((16, 12), text + "   (firmware in the loop, MuJoCo)", fill=(255, 255, 255))
    return np.asarray(im)


def demo_metrics(s):
    log, ev = s.log, s.events
    t = np.array([r["t"] for r in log])

    def row_at(tt):
        return log[int(np.clip(np.searchsorted(t, tt), 0, len(log) - 1))]

    seg = {}
    starts = [(tt, m.split(" ", 3)[3]) for tt, m in ev if m.startswith("EVT DEMO")]
    for i, (t0, cmd) in enumerate(starts):
        t1 = starts[i + 1][0] if i + 1 < len(starts) else next(tt for tt, m in ev if m.startswith("DONE DEMO"))
        seg[cmd] = (t0, t1)
    M = {}

    def move(cmd):
        t0, t1 = seg[cmd]
        a, b = row_at(t0), row_at(t1)
        dx, dy = b["x"] - a["x"], b["y"] - a["y"]
        h = math.radians(a["yaw"])
        return dx * math.cos(h) + dy * math.sin(h), -dx * math.sin(h) + dy * math.cos(h), b["yaw"] - a["yaw"], a, b

    along, side, _, a, b = move("FWD 100")
    M["T1 FWD 100"] = dict(true_m=round(along, 4), encoder_m=round(b["odo_x"] - a["odo_x"], 4),
                           error_cm=round((along - 1.0) * 100, 2), drift_cm=round(side * 100, 2))
    along, side, _, _, _ = move("BACK 30")
    M["T1 BACK 30"] = dict(true_m=round(along, 4), error_cm=round((along + 0.30) * 100, 2))
    along, side, dyaw, a, b = move("TURN 90")
    M["T2 TURN 90"] = dict(true_deg=round(dyaw, 2), encoder_deg=round(b["odo_th"] - a["odo_th"], 2),
                           gyro_deg=round(b["gyro_th"] - a["gyro_th"], 2), error_deg=round(dyaw - 90, 2),
                           centre_moved_mm=round(math.hypot(along, side) * 1000, 1))
    t0, t1 = seg["FWD 300"]
    blocked = [(tt, m) for tt, m in ev if t0 <= tt <= t1 and m.startswith("EVT BLOCKED")]
    done = next((m for tt, m in ev if t0 <= tt <= t1 and m.startswith("DONE FWD")), "")
    rows = [r for r in log if t0 <= r["t"] <= t1]
    stop = rows[-1]
    tb = blocked[0][0] if blocked else t1
    rb = row_at(tb)
    M["T3 obstacle stop"] = dict(
        result=done.split()[2] if done else "?", braking_started_at_mm=int(blocked[0][1].split()[2]) if blocked else None,
        stop_distance_mm=round(stop["true_range"], 1), braking_distance_mm=round(
            math.hypot(stop["x"] - rb["x"], stop["y"] - rb["y"]) * 1000, 1),
        speed_at_braking=round(rb["v"], 3), max_tilt_deg=round(max(r["tilt"] for r in rows), 2))
    for name, cmd in (("T4 WAVE", "WAVE R"), ("T5 NOD", "NOD"), ("T5 SHAKE", "SHAKE")):
        t0, t1 = seg[cmd]
        rows = [r for r in log if t0 <= r["t"] <= t1]
        peaks = {}
        for j, rated in zip(JOINTS, SERVO_RATED_KGCM):
            pk = max(abs(r[f"tq_{j}"]) for r in rows) / KGCM
            if pk > 0.05:
                peaks[j] = dict(peak_kgcm=round(pk, 2), share_of_rated=f"{100 * pk / rated:.0f} %")
        M[name] = dict(duration_s=round(t1 - t0, 2), servo_peaks=peaks)
    t0, t1 = seg["SERVO ON"]
    rows = [r for r in log if t0 <= r["t"] <= t1]
    M["power-on snap"] = dict(
        note="servos are limp until SERVO ON, the joints sag under gravity, then each servo returns to zero at full speed",
        peaks={j: f"{max(abs(r[f'tq_{j}']) for r in rows) / KGCM:.1f} kg.cm ({100 * max(abs(r[f'tq_{j}']) for r in rows) / KGCM / rated:.0f} %)"
               for j, rated in zip(JOINTS, SERVO_RATED_KGCM)})
    allr = [r for r in log if r["t"] > t1]  # after the servos are on
    M["whole run"] = dict(
        duration_s=round(seg["REST"][1] - seg["SERVO ON"][0], 1),
        result=next(m for _, m in ev if m.startswith("DONE DEMO")),
        max_tilt_deg=round(max(r["tilt"] for r in allr), 2),
        note="peaks below exclude the power-on snap",
        wheel_torque_peak=f"{100 * max(max(abs(r['tq_wheelL']), abs(r['tq_wheelR'])) for r in allr) / MOTOR_RATED_NM:.0f} % of rated",
        servo_torque_peak={j: f"{100 * max(abs(r[f'tq_{j}']) for r in allr) / KGCM / rated:.0f} %"
                           for j, rated in zip(JOINTS, SERVO_RATED_KGCM)},
        errors=[m for _, m in ev if m.startswith("ERR")])
    return M


def write_results(s, M, frames):
    import csv
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    os.makedirs(RESULTS, exist_ok=True)
    keys = list(s.log[0].keys())
    with open(os.path.join(RESULTS, "demo_log.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, keys)
        w.writeheader()
        w.writerows(s.log)
    with open(os.path.join(RESULTS, "demo_events.txt"), "w") as f:
        f.writelines(f"{t:7.2f}  {m}\n" for t, m in s.events)
    with open(os.path.join(RESULTS, "demo_metrics.json"), "w") as f:
        json.dump(M, f, indent=2)

    L = {k: np.array([r[k] for r in s.log]) for k in keys if k != "label"}
    t = L["t"]
    fig, ax = plt.subplots(4, 1, figsize=(10, 11), sharex=True)
    ax[0].plot(t, L["v"], label="true forward speed")
    ax[0].plot(t, 0.5 * (L["dutyL"] + L["dutyR"]), lw=0.8, alpha=0.7, label="mean motor duty")
    ax[0].axhline(CFG["V_MAX"], ls=":", c="grey")
    ax[0].set_ylabel("m/s  |  duty")
    ax[0].legend(loc="upper right", fontsize=8)
    obst = np.where(L["obst"] >= 0, L["obst"], np.nan)
    ax[1].plot(t, obst, label="firmware obstacle estimate (median, min of 4)")
    ax[1].plot(t, L["true_range"], lw=0.8, alpha=0.7, label="true ToF range")
    ax[1].axhline(CFG["OBST_BRAKE_MM"], ls=":", c="grey", label="braking threshold 550 mm")
    ax[1].axhline(300, ls="--", c="red", label="stop limit 300 mm")
    ax[1].set_ylim(0, 1300)
    ax[1].set_ylabel("mm")
    ax[1].legend(loc="upper right", fontsize=8)
    ax[2].plot(t, L["yaw"], label="true yaw")
    ax[2].plot(t, L["odo_th"], "--", label="encoder heading")
    ax[2].plot(t, L["gyro_th"], ":", label="gyro heading")
    ax[2].set_ylabel("deg")
    ax[2].legend(loc="upper left", fontsize=8)
    for j, rated in zip(JOINTS, SERVO_RATED_KGCM):
        share = 100 * np.abs(L[f"tq_{j}"]) / KGCM / rated
        if share.max() > 0.5:
            ax[3].plot(t, share, lw=0.9, label=j)
    ax[3].axhline(33, ls="--", c="red", label="design limit: a third of rated")
    ax[3].set_ylabel("% of rated torque")
    ax[3].set_xlabel("time (s)")
    ax[3].legend(loc="upper left", fontsize=8, ncol=3)
    # step labels along the top
    starts = [(tt, m.split(" ", 3)[3]) for tt, m in s.events if m.startswith("EVT DEMO")]
    for a in ax:
        for tt, _ in starts:
            a.axvline(tt, c="k", lw=0.3, alpha=0.4)
    for tt, lab in starts:
        ax[0].text(tt + 0.1, ax[0].get_ylim()[1] * 0.92, lab, fontsize=7, rotation=90, va="top")
    fig.suptitle("Firmware in the loop: DEMO on the V2 digital twin")
    fig.tight_layout()
    fig.savefig(os.path.join(RESULTS, "demo_signals.png"), dpi=130)
    plt.close(fig)
    if frames:
        ks = sorted(frames)
        fig, ax = plt.subplots(1, len(ks), figsize=(4.2 * len(ks), 3.0))
        for a, k in zip(np.atleast_1d(ax), ks):
            a.imshow(frames[k][60:660, 240:1040])
            a.set_title(k[2:], fontsize=10)
            a.axis("off")
        fig.tight_layout()
        fig.savefig(os.path.join(RESULTS, "demo_frames.png"), dpi=130)
        plt.close(fig)
    with open(os.path.join(RESULTS, "demo_metrics.md"), "w") as f:
        f.write("# DEMO on the V2 twin, firmware in the loop\n\n")
        f.write(f"Generated by `sim/firmware_sim.py` (seed {s.robot.rng.bit_generator.state['state']['state'] % 1000}).\n\n")
        for k, v in M.items():
            f.write(f"- **{k}**: {json.dumps(v)}\n")


# ------------------------------------------------------------------ interactive modes
def run_script(args):
    robot = SimRobot(os.path.join(HERE, "demo_scene.xml"), seed=args.seed)
    robot.settle(1.0)
    core = Core()
    s = Session(robot, core)
    cmds = [c.strip() for c in args.script.split(";") if c.strip()]
    for _ in range(50):
        s.step()
    for cmd in cmds:
        n0 = len(s.events)
        core.line(cmd)
        name = cmd.split()[0].upper()
        while robot.t_ms < 600000:
            s.step()
            new = [m for _, m in s.events[n0:]]
            if any(m.startswith("ERR") for m in new) or any(m.startswith("DONE") for m in new) or \
                    name in ("SERVO", "STREAM", "STATE?", "PING", "RESET"):
                break
    for _ in range(100):
        s.step()
    x, y, yaw, _ = robot.pose()
    print(f"final pose: x={x:.3f} m, y={y:.3f} m, yaw={math.degrees(yaw):.1f} deg")


def run_live(args):
    """Real-time loop; optional TCP server for the serial protocol and optional viewer."""
    robot = SimRobot(os.path.join(HERE, "demo_scene.xml"), seed=args.seed)
    robot.settle(1.0)
    core = Core()
    s = Session(robot, core, echo=True)
    sel, clients, server = selectors.DefaultSelector(), {}, None
    if args.tcp:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(("127.0.0.1", args.tcp))
        server.listen()
        server.setblocking(False)
        sel.register(server, selectors.EVENT_READ)
        print(f"serial protocol on tcp://127.0.0.1:{args.tcp}  (try: nc localhost {args.tcp}, then HELP)")
    if not args.tcp:
        core.line("DEMO")
    viewer = None
    if args.viewer:
        import mujoco.viewer
        viewer = mujoco.viewer.launch_passive(robot.m, robot.d)
        viewer.cam.trackbodyid = robot.base
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        viewer.cam.distance, viewer.cam.elevation = 2.2, -18
    start = time.time()
    sent = 0
    try:
        while viewer is None or viewer.is_running():
            for key, _ in sel.select(timeout=0):
                if key.fileobj is server:
                    conn, _ = server.accept()
                    conn.setblocking(False)
                    sel.register(conn, selectors.EVENT_READ)
                    clients[conn] = b""
                    conn.sendall(b"READY humanoid-fw 1.0 (simulated)\n")
                else:
                    conn = key.fileobj
                    try:
                        data = conn.recv(1024)
                    except ConnectionError:
                        data = b""
                    if not data:
                        sel.unregister(conn)
                        conn.close()
                        clients.pop(conn, None)
                        continue
                    clients[conn] += data
                    while b"\n" in clients[conn]:
                        ln, clients[conn] = clients[conn].split(b"\n", 1)
                        if ln.strip():
                            core.line(ln.decode(errors="replace").strip())
            s.step()
            for _, msg in s.events[sent:]:
                for conn in list(clients):
                    try:
                        conn.sendall((msg + "\n").encode())
                    except OSError:
                        pass
            sent = len(s.events)
            if len(s.log) > 2000:  # keep memory flat in long sessions
                del s.log[:1000]
            if viewer is not None:
                viewer.sync()
            lag = robot.t_ms / 1000 - (time.time() - start)
            if lag > 0:
                time.sleep(lag)
            if not args.tcp and viewer is None and any(m.startswith("DONE DEMO") for _, m in s.events[-2:]):
                break
    except KeyboardInterrupt:
        pass


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--video", action="store_true", help="also write sim/results/demo.mp4")
    ap.add_argument("--no-frames", action="store_true", help="skip the rendered figure")
    ap.add_argument("--viewer", action="store_true", help="live MuJoCo viewer (real time)")
    ap.add_argument("--tcp", type=int, default=0, help="serve the serial protocol on this TCP port (real time)")
    ap.add_argument("--script", default="", help='commands separated by ";", each waits for its DONE')
    ap.add_argument("--seed", type=int, default=1, help="sensor noise seed")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()
    if args.viewer or args.tcp:
        run_live(args)
    elif args.script:
        run_script(args)
    else:
        M = run_demo(args)
        print(json.dumps(M, indent=2))
        print(f"\nresults in {os.path.relpath(RESULTS, os.getcwd())}/")


if __name__ == "__main__":
    main()
