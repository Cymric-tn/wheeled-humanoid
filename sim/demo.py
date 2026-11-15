#!/usr/bin/env python3
"""Live demo of the digital twin: the robot waves, looks around, and drives a loop.

macOS:          .venv/bin/mjpython sim/demo.py
Linux/Windows:  .venv/bin/python   sim/demo.py
Close the window to stop. Press Space in the viewer to pause.
"""
import os
import time
import numpy as np
import mujoco
import mujoco.viewer

HERE = os.path.dirname(os.path.abspath(__file__))
m = mujoco.MjModel.from_xml_path(os.path.join(HERE, "scene.xml"))
d = mujoco.MjData(m)
act = {m.actuator(i).name: i for i in range(m.nu)}
CYCLE = 16.0  # seconds per loop of the routine


def command(t):
    """Target for every actuator at time t (servos in rad, wheels in rad/s)."""
    u = dict.fromkeys(act, 0.0)
    s = t % CYCLE
    if s < 4:                                   # wave with the right arm, look at it
        u["r_sh_roll"] = 1.2
        u["r_elbow"] = 0.9 + 0.5 * np.sin(2 * np.pi * 1.2 * s)
        u["head_pan"] = -0.6
        u["head_tilt"] = -0.2
    elif s < 7:                                 # look around
        u["head_pan"] = 0.9 * np.sin(2 * np.pi * (s - 4) / 3)
        u["head_tilt"] = 0.25 * np.sin(2 * np.pi * (s - 4) / 1.5)
    elif s < 11:                                # drive forward, arms swinging
        u["wheel_left"] = u["wheel_right"] = 6.0
        swing = 0.4 * np.sin(2 * np.pi * 1.0 * (s - 7))
        u["l_sh_pitch"], u["r_sh_pitch"] = swing, -swing
        u["l_elbow"] = u["r_elbow"] = 0.4
    elif s < 13:                                # turn about 180 deg on the spot, arms out
        u["wheel_left"], u["wheel_right"] = -4.0, 4.0
        u["l_sh_roll"] = u["r_sh_roll"] = 1.3
    else:                                       # both arms up: "ta-da"
        u["l_sh_pitch"] = u["r_sh_pitch"] = 1.5
        u["l_elbow"] = u["r_elbow"] = 0.3
        u["head_tilt"] = -0.3
    return u


with mujoco.viewer.launch_passive(m, d) as viewer:
    viewer.cam.lookat[:] = [0.3, 0, 0.35]
    viewer.cam.distance = 2.2
    viewer.cam.azimuth = 150
    viewer.cam.elevation = -18
    viewer.cam.trackbodyid = m.body("base_link").id
    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    start = time.time()
    while viewer.is_running():
        for name, v in command(d.time).items():
            d.ctrl[act[name]] = v
        mujoco.mj_step(m, d)
        viewer.sync()
        lag = d.time - (time.time() - start)    # keep real time
        if lag > 0:
            time.sleep(lag)
