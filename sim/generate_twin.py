#!/usr/bin/env python3
"""Generate the wheeled_humanoid_v2 digital twin (URDF + MuJoCo MJCF) from the CAD meshes.

Source of truth: ../preview/*.stl. Every mesh there is exported in ONE shared
assembly frame (millimetres, X forward, Y left, Z up, origin on the floor under
the robot centre). This script:

  1. groups the meshes into rigid links (LINKS below),
  2. places each joint on the servo / motor output shaft (JOINTS below; the
     shaft positions were measured from the servo horn geometry in the meshes),
  3. rewrites every mesh into its link frame, in metres (sim/meshes/),
  4. computes mass, centre of mass and inertia per link from the mesh volumes
     (printed / laser-cut parts) or from assigned masses (bought parts),
  5. writes  wheeled_humanoid_v2.urdf  and  wheeled_humanoid_v2.xml (MJCF)
     plus scene.xml (floor + lights + robot) for MuJoCo.

Re-run after changing the CAD:   python3 sim/generate_twin.py
Every number tagged DESIGN VALUE is an estimate: replace it with a measured one.
"""
import os
import struct
import shutil
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SRC = os.path.join(ROOT, "preview")
OUT_MESH = os.path.join(HERE, "meshes")
NAME = "wheeled_humanoid_v2"
MM = 0.001

# ---------------------------------------------------------------- materials
# Effective densities in kg/m^3. DESIGN VALUES: weigh the real parts and fix.
DENSITY_PRINTED = 750.0   # PLA at roughly 30-40 % infill + walls (solid PLA is ~1240)
DENSITY_LASER = 1190.0    # assumes acrylic sheet; plywood would be ~650

# Plates in laser/ (DXF) are laser cut; parts in print/ are 3D printed.
PLATES = {os.path.splitext(f)[0] for f in os.listdir(os.path.join(ROOT, "laser")) if f.endswith(".dxf")}
PRINTED = {os.path.splitext(f)[0] for f in os.listdir(os.path.join(ROOT, "print")) if f.endswith(".stl")}

# Bought parts: mass in kg. DESIGN VALUES from typical datasheets, not measured.
MASS = {
    "ref_wheel_left": 0.120, "ref_wheel_right": 0.120,
    "ref_motor_left": 0.250, "ref_motor_right": 0.250,           # 37 mm gearmotor
    "ref_coupling_left": 0.012, "ref_coupling_right": 0.012,
    "ref_caster_front": 0.030, "ref_caster_rear": 0.030,
    "ref_battery": 0.350,                                         # 3S LiPo
    **{f"ref_servo_{s}": 0.055 for s in (                         # standard-size servo
        "l_sh_pitch", "r_sh_pitch", "l_sh_roll", "r_sh_roll",
        "l_elbow", "r_elbow", "head_pan", "head_tilt")},
    "power_02": 0.070, "power_03": 0.070,   # BTS7960 drivers
    "power_04": 0.040,                      # emergency stop
    "power_05": 0.006,                      # rocker switch
    "power_06": 0.010,                      # fuse holder
    "power_07": 0.050, "power_08": 0.050,   # XL4016 converters
    "power_09": 0.020,                      # XY-3606 5 V converter
    "sensing_10": 0.003,                    # MPU-6050
    "control_11": 0.037,                    # Arduino Mega
    "control_12": 0.100,                    # Raspberry Pi 5 in case
    "control_13": 0.010,                    # PCA9685
    "power_14": 0.020,                      # perfboard servo power strip
    "control_15": 0.005, "control_16": 0.005,  # OLED eyes
    "sensing_17": 0.003,                    # Pi camera
    "sensing_18": 0.002, "sensing_19": 0.002,  # VL53L0X
    "sensing_20": 0.009, "sensing_21": 0.009,  # HC-SR04
}

COLORS = {
    "printed": (0.90, 0.42, 0.10, 1), "plate": (0.13, 0.14, 0.16, 1),
    "light": (0.80, 0.81, 0.83, 1), "servo": (0.12, 0.12, 0.13, 1),
    "rubber": (0.08, 0.08, 0.08, 1), "metal": (0.62, 0.63, 0.66, 1),
    "pcb": (0.10, 0.45, 0.25, 1), "battery": (0.15, 0.30, 0.65, 1),
    "red": (0.80, 0.10, 0.10, 1),
}
LIGHT_PRINTED = {"head_shell", "upper_arm_left", "upper_arm_right"}

# ---------------------------------------------------------------- kinematics
# Links -> list of preview mesh names. Anything not listed goes to base_link.
# A servo BODY belongs to the link before its joint; its horn drives the next link.
LINKS = {
    "wheel_left_link": ["ref_wheel_left", "ref_coupling_left"],
    "wheel_right_link": ["ref_wheel_right", "ref_coupling_right"],
    "shoulder_yoke_left_link": ["shoulder_yoke_left", "ref_servo_l_sh_roll"],
    "upper_arm_left_link": ["upper_arm_left", "ref_servo_l_elbow"],
    "forearm_left_link": ["forearm_left"],
    "shoulder_yoke_right_link": ["shoulder_yoke_right", "ref_servo_r_sh_roll"],
    "upper_arm_right_link": ["upper_arm_right", "ref_servo_r_elbow"],
    "forearm_right_link": ["forearm_right"],
    "head_yoke_link": ["head_yoke", "ref_servo_head_tilt"],
    "head_link": ["head_shell", "control_15", "control_16", "sensing_17"],
}
SKIP = {"power_01"}  # same box as ref_battery (the battery appears twice)

DEG = np.pi / 180
# name: (type, parent, child, origin_mm (on the output shaft), axis, lower, upper, effort Nm, vel rad/s)
# Axis signs chosen so that, for both sides: +sh_pitch raises the arm forward,
# +sh_roll swings it outward, +elbow bends the forearm forward, +head_pan turns
# left, +head_tilt looks down. Limits/effort/velocity are DESIGN VALUES
# (+-90 deg servo travel, ~0.94 Nm / 6 rad/s for an MG996R-class servo).
SERVO = dict(effort=0.94, vel=6.0)
JOINTS = {
    "wheel_left_joint": ("continuous", "base_link", "wheel_left_link", (0, 150, 65), (0, 1, 0), None, None, 1.0, 30.0),
    "wheel_right_joint": ("continuous", "base_link", "wheel_right_link", (0, -150, 65), (0, 1, 0), None, None, 1.0, 30.0),
    "l_sh_pitch_joint": ("revolute", "base_link", "shoulder_yoke_left_link", (0, 131, 550), (0, -1, 0), -90, 90, SERVO["effort"], SERVO["vel"]),
    "l_sh_roll_joint": ("revolute", "shoulder_yoke_left_link", "upper_arm_left_link", (33, 148.6, 550), (1, 0, 0), -90, 90, SERVO["effort"], SERVO["vel"]),
    "l_elbow_joint": ("revolute", "upper_arm_left_link", "forearm_left_link", (19.9, 187.6, 430), (0, -1, 0), -90, 90, SERVO["effort"], SERVO["vel"]),
    "r_sh_pitch_joint": ("revolute", "base_link", "shoulder_yoke_right_link", (0, -131, 550), (0, -1, 0), -90, 90, SERVO["effort"], SERVO["vel"]),
    "r_sh_roll_joint": ("revolute", "shoulder_yoke_right_link", "upper_arm_right_link", (33, -148.6, 550), (-1, 0, 0), -90, 90, SERVO["effort"], SERVO["vel"]),
    "r_elbow_joint": ("revolute", "upper_arm_right_link", "forearm_right_link", (19.9, -187.6, 430), (0, -1, 0), -90, 90, SERVO["effort"], SERVO["vel"]),
    "head_pan_joint": ("revolute", "base_link", "head_yoke_link", (0, 0, 555.2), (0, 0, 1), -90, 90, SERVO["effort"], SERVO["vel"]),
    "head_tilt_joint": ("revolute", "head_yoke_link", "head_link", (0, 51, 575.3), (0, 1, 0), -45, 45, SERVO["effort"], SERVO["vel"]),
}

# Sensor frames (fixed). pos in mm (assembly frame), all look along +X (forward).
SENSORS = {
    "camera_link": ("head_link", "sensing_17"),
    "imu_link": ("base_link", "sensing_10"),
    "tof_left_link": ("base_link", "sensing_18"),
    "tof_right_link": ("base_link", "sensing_19"),
    "sonar_left_link": ("base_link", "sensing_20"),
    "sonar_right_link": ("base_link", "sensing_21"),
}

# Casters: modelled as frictionless spheres (fixed). Centre/radius in mm.
CASTERS = {"caster_front_link": (150, 0, 14.0, 12.5), "caster_rear_link": (-150, 0, 14.0, 12.5)}
WHEEL_R, WHEEL_W = 65.0, 30.0


# ---------------------------------------------------------------- mesh utils
def read_stl(path):
    d = open(path, "rb").read()
    n = struct.unpack("<I", d[80:84])[0]
    if 84 + 50 * n != len(d):
        raise ValueError(f"{path}: not a binary STL")
    rec = np.frombuffer(d[84:], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    return rec["v"].reshape(-1, 3, 3).astype(np.float64)


def write_stl(path, tris):
    tris = tris.astype(np.float32)
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    ln = np.linalg.norm(n, axis=1, keepdims=True)
    n = np.where(ln > 0, n / np.maximum(ln, 1e-30), 0).astype(np.float32)
    rec = np.zeros(len(tris), dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    rec["n"], rec["v"] = n, tris.reshape(-1, 9)
    with open(path, "wb") as f:
        f.write(b"wheeled_humanoid_v2 digital twin mesh (metres, link frame)".ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        f.write(rec.tobytes())


def solid_props(tris):
    """Volume, centroid and inertia (about centroid, unit density) of a closed mesh."""
    a, b, c = tris[:, 0], tris[:, 1], tris[:, 2]
    det = np.einsum("ij,ij->i", a, np.cross(b, c))
    vol = det.sum() / 6.0
    if abs(vol) < 1e-15:
        return None
    com = (det[:, None] * (a + b + c)).sum(0) / (24.0 * vol)
    canon = np.array([[2, 1, 1], [1, 2, 1], [1, 1, 2]]) / 120.0
    A = np.stack([a, b, c], axis=2)                      # columns = vertices
    C = np.einsum("t,tij,jk,tlk->il", det, A, canon, A)  # second moment about origin
    if vol < 0:
        vol, C = -vol, -C
    C = C - vol * np.outer(com, com)                     # shift to centroid
    I = np.trace(C) * np.eye(3) - C
    return vol, com, I


def box_props(tris):
    mn, mx = tris.reshape(-1, 3).min(0), tris.reshape(-1, 3).max(0)
    e = mx - mn
    vol = float(np.prod(e))
    I = vol / 12.0 * np.diag([e[1]**2 + e[2]**2, e[0]**2 + e[2]**2, e[0]**2 + e[1]**2])
    return vol, (mn + mx) / 2, I


def part_props(name, tris_m):
    """mass [kg], com [m, assembly frame], inertia about com [kg m^2]."""
    p = solid_props(tris_m)
    mn, mx = tris_m.reshape(-1, 3).min(0), tris_m.reshape(-1, 3).max(0)
    bbox_vol = float(np.prod(mx - mn))
    ok = p is not None and 0.005 * bbox_vol < p[0] <= 1.01 * bbox_vol and np.all(p[1] >= mn - 1e-6) and np.all(p[1] <= mx + 1e-6)
    vol, com, I = p if ok else box_props(tris_m)
    base = name.rsplit("_copy_", 1)[0]
    if name in MASS:
        m = MASS[name]
        src = "assigned"
    elif base in PRINTED or any(base.startswith(k.rsplit("_x", 1)[0]) for k in PRINTED if "_x" in k):
        m, src = vol * DENSITY_PRINTED, "printed"
    elif base in PLATES:
        m, src = vol * DENSITY_LASER, "laser"
    else:
        raise KeyError(f"no mass rule for {name}")
    return m, com, I * (m / vol), src, ok


def color_of(name, src):
    if name.startswith("ref_servo"):
        return "servo"
    if name.startswith(("ref_wheel", "ref_caster")):
        return "rubber"
    if name.startswith(("ref_motor", "ref_coupling")):
        return "metal"
    if name == "ref_battery":
        return "battery"
    if name == "power_04":
        return "red"
    if name.startswith(("power_", "control_", "sensing_")):
        return "pcb"
    if name in PLATES:
        return "plate"
    return "light" if name in LIGHT_PRINTED else "printed"


# ---------------------------------------------------------------- build
def build():
    names = sorted(os.path.splitext(f)[0] for f in os.listdir(SRC) if f.endswith(".stl"))
    names = [n for n in names if n not in SKIP]
    owner = {m: l for l, ms in LINKS.items() for m in ms}
    for m in owner:
        if m not in names:
            raise FileNotFoundError(f"{m}.stl missing in preview/")
    origin = {"base_link": np.zeros(3)}
    for j in JOINTS.values():
        origin[j[2]] = np.array(j[3], float) * MM

    if os.path.isdir(OUT_MESH):
        shutil.rmtree(OUT_MESH)
    os.makedirs(OUT_MESH)

    links = {l: {"parts": [], "m": 0.0} for l in ["base_link", *LINKS]}
    report = []
    for n in names:
        link = owner.get(n, "base_link")
        tris = read_stl(os.path.join(SRC, n + ".stl")) * MM
        m, com, I, src, ok = part_props(n, tris)
        write_stl(os.path.join(OUT_MESH, n + ".stl"), tris - origin[link])
        links[link]["parts"].append(dict(name=n, m=m, com=com, I=I, color=color_of(n, src)))
        report.append((link, n, src, m, ok))

    # combine per link (about link COM, expressed in link frame = world-aligned)
    for l, L in links.items():
        M = sum(p["m"] for p in L["parts"])
        c = sum(p["m"] * p["com"] for p in L["parts"]) / M
        I = np.zeros((3, 3))
        for p in L["parts"]:
            d = p["com"] - c
            I += p["I"] + p["m"] * (d @ d * np.eye(3) - np.outer(d, d))
        L.update(m=M, com=c - origin[l], I=I)
    return links, origin, report


def fmt(v, k=6):
    out = []
    for x in v:
        s = f"{x:.{k}f}".rstrip("0").rstrip(".")
        out.append("0" if s in ("-0", "") else s)
    return " ".join(out)


# ---------------------------------------------------------------- URDF
def write_urdf(links, origin, path=None, mesh_prefix="meshes/"):
    L = []
    w = L.append
    w('<?xml version="1.0"?>')
    w(f'<!-- Generated by sim/generate_twin.py from preview/*.stl. Do not edit by hand: change the script and re-run. -->')
    w(f'<robot name="{NAME}">')
    for cname, rgba in COLORS.items():
        w(f'  <material name="{cname}"><color rgba="{fmt(rgba, 3)}"/></material>')
    w('  <link name="base_footprint"/>')
    w('  <joint name="base_footprint_joint" type="fixed"><parent link="base_footprint"/><child link="base_link"/><origin xyz="0 0 0"/></joint>')

    coll = collision_shapes(links, origin)
    for ln, Ld in links.items():
        w(f'  <link name="{ln}">')
        I = Ld["I"]
        w(f'    <inertial><origin xyz="{fmt(Ld["com"])}"/><mass value="{Ld["m"]:.5f}"/>'
          f'<inertia ixx="{I[0,0]:.4e}" ixy="{I[0,1]:.4e}" ixz="{I[0,2]:.4e}" iyy="{I[1,1]:.4e}" iyz="{I[1,2]:.4e}" izz="{I[2,2]:.4e}"/></inertial>')
        for p in Ld["parts"]:
            w(f'    <visual name="{p["name"]}"><geometry><mesh filename="{mesh_prefix}{p["name"]}.stl"/></geometry><material name="{p["color"]}"/></visual>')
        for c in coll.get(ln, []):
            w("    " + urdf_collision(c))
        w('  </link>')

    for jn, (jt, par, ch, o, ax, lo, hi, eff, vel) in JOINTS.items():
        xyz = (np.array(o) * MM) - origin[par]
        lim = (f'<limit lower="{lo*DEG:.4f}" upper="{hi*DEG:.4f}" effort="{eff}" velocity="{vel}"/>' if jt == "revolute"
               else f'<limit effort="{eff}" velocity="{vel}"/>')
        damp = 0.05 if jt == "continuous" else 0.1
        w(f'  <joint name="{jn}" type="{jt}"><parent link="{par}"/><child link="{ch}"/>'
          f'<origin xyz="{fmt(xyz)}" rpy="0 0 0"/><axis xyz="{fmt(ax)}"/>{lim}<dynamics damping="{damp}" friction="0.01"/></joint>')

    for cn, (x, y, z, r) in CASTERS.items():
        w(f'  <link name="{cn}"><collision><geometry><sphere radius="{r*MM}"/></geometry></collision></link>')
        w(f'  <joint name="{cn.replace("_link", "_joint")}" type="fixed"><parent link="base_link"/><child link="{cn}"/><origin xyz="{fmt(np.array([x, y, z])*MM)}"/></joint>')
        w(f'  <gazebo reference="{cn}"><mu1>0.0</mu1><mu2>0.0</mu2></gazebo>')
    for sn, (par, mesh) in SENSORS.items():
        pos = sensor_pos(links, par, mesh) - origin[par]
        w(f'  <link name="{sn}"/>')
        w(f'  <joint name="{sn.replace("_link", "_joint")}" type="fixed"><parent link="{par}"/><child link="{sn}"/><origin xyz="{fmt(pos)}" rpy="0 0 0"/></joint>')
    for side in ("left", "right"):
        w(f'  <gazebo reference="wheel_{side}_link"><mu1>1.0</mu1><mu2>1.0</mu2></gazebo>')
    w('</robot>')
    open(path or os.path.join(HERE, NAME + ".urdf"), "w").write("\n".join(L) + "\n")


def sensor_pos(links, link, mesh):
    tris = read_stl(os.path.join(SRC, mesh + ".stl")) * MM
    v = tris.reshape(-1, 3)
    c = (v.min(0) + v.max(0)) / 2
    c[0] = v[:, 0].max()  # front face
    return c


def collision_shapes(links, origin):
    """Primitive collision shapes per link: boxes from mesh bounds, cylinders for wheels."""
    out = {}

    def bbox(meshes, link):
        v = np.concatenate([read_stl(os.path.join(SRC, m + ".stl")).reshape(-1, 3) for m in meshes]) * MM
        mn, mx = v.min(0), v.max(0)
        return ("box", (mn + mx) / 2 - origin[link], mx - mn)

    out["base_link"] = [
        bbox(["lower_deck", "upper_deck", "bumper_pillar_x4", "bumper_pillar_copy_2", "bumper_pillar_copy_3",
              "bumper_pillar_copy_4", "motor_bracket_left", "motor_bracket_right"], "base_link"),
        bbox(["sensor_bar"], "base_link"),
        bbox(["control_panel"], "base_link"),
        bbox(["torso_side_left_mega", "torso_side_right_pi", "chest_plate", "back_plate"], "base_link"),
        bbox(["shoulder_plate"], "base_link"),
    ]
    for side in ("left", "right"):
        out[f"wheel_{side}_link"] = [("cylinder", np.zeros(3), (WHEEL_R * MM, WHEEL_W * MM))]
        out[f"shoulder_yoke_{side}_link"] = [bbox([f"shoulder_yoke_{side}"], f"shoulder_yoke_{side}_link")]
        out[f"upper_arm_{side}_link"] = [bbox([f"upper_arm_{side}"], f"upper_arm_{side}_link")]
        out[f"forearm_{side}_link"] = [bbox([f"forearm_{side}"], f"forearm_{side}_link")]
    out["head_yoke_link"] = [bbox(["head_yoke"], "head_yoke_link")]
    # head: the box above the yoke only (the shell legs straddle the yoke)
    v = read_stl(os.path.join(SRC, "head_shell.stl")).reshape(-1, 3) * MM
    mn, mx = v.min(0), v.max(0)
    mn = mn.copy(); mn[2] = 0.600
    out["head_link"] = [("box", (mn + mx) / 2 - origin["head_link"], mx - mn)]
    return out


def urdf_collision(c):
    kind, pos, size = c
    if kind == "box":
        return f'<collision><origin xyz="{fmt(pos)}"/><geometry><box size="{fmt(size)}"/></geometry></collision>'
    r, l = size
    return f'<collision><origin xyz="{fmt(pos)}" rpy="1.5708 0 0"/><geometry><cylinder radius="{r}" length="{l}"/></geometry></collision>'


# ---------------------------------------------------------------- MJCF
def write_mjcf(links, origin):
    coll = collision_shapes(links, origin)
    children = {}
    for jn, j in JOINTS.items():
        children.setdefault(j[1], []).append((jn, j))
    L = []
    w = L.append
    w(f'<!-- Generated by sim/generate_twin.py. Load scene.xml to simulate. -->')
    w(f'<mujoco model="{NAME}">')
    w('  <compiler angle="radian" meshdir="meshes" inertiafromgeom="false" autolimits="true"/>')
    w('  <option timestep="0.002" integrator="implicitfast"/>')
    w('  <default>')
    w('    <default class="visual"><geom type="mesh" contype="0" conaffinity="0" group="2" density="0"/></default>')
    w('    <default class="collision"><geom group="3" rgba="0.2 0.6 1 0.3" friction="0.8 0.02 0.001"/></default>')
    w('    <default class="servo"><joint damping="0.1" armature="0.005" frictionloss="0.01"/>'
      f'<position kp="8" kv="0.2" forcerange="-{SERVO["effort"]} {SERVO["effort"]}"/></default>')
    w('    <default class="wheel"><joint damping="0.05" armature="0.002"/><velocity kv="0.5" forcerange="-1 1"/></default>')
    w('  </default>')
    w('  <asset>')
    for cname, rgba in COLORS.items():
        w(f'    <material name="{cname}" rgba="{fmt(rgba, 3)}"/>')
    for Ld in links.values():
        for p in Ld["parts"]:
            w(f'    <mesh name="{p["name"]}" file="{p["name"]}.stl" inertia="shell"/>')
    w('  </asset>')
    w('  <worldbody>')

    def body(ln, depth, joint=None):
        ind = "  " * depth
        Ld = links[ln]
        if joint is None:
            w(f'{ind}<body name="{ln}" pos="0 0 0.002">')
            w(f'{ind}  <freejoint name="root"/>')
        else:
            jn, (jt, par, ch, o, ax, lo, hi, eff, vel) = joint
            pos = np.array(o) * MM - origin[par]
            w(f'{ind}<body name="{ln}" pos="{fmt(pos)}">')
            cls = "wheel" if jt == "continuous" else "servo"
            rng = f' range="{lo*DEG:.4f} {hi*DEG:.4f}"' if jt == "revolute" else ""
            w(f'{ind}  <joint name="{jn}" class="{cls}" axis="{fmt(ax)}"{rng}/>')
        I = Ld["I"]
        w(f'{ind}  <inertial pos="{fmt(Ld["com"])}" mass="{Ld["m"]:.5f}" '
          f'fullinertia="{I[0,0]:.4e} {I[1,1]:.4e} {I[2,2]:.4e} {I[0,1]:.4e} {I[0,2]:.4e} {I[1,2]:.4e}"/>')
        for p in Ld["parts"]:
            w(f'{ind}  <geom class="visual" mesh="{p["name"]}" material="{p["color"]}"/>')
        for kind, pos, size in coll.get(ln, []):
            if kind == "box":
                w(f'{ind}  <geom class="collision" type="box" pos="{fmt(pos)}" size="{fmt(np.array(size)/2)}"/>')
            else:
                w(f'{ind}  <geom class="collision" type="cylinder" pos="{fmt(pos)}" quat="0.7071068 -0.7071068 0 0" '
                  f'size="{size[0]} {size[1]/2}" friction="1.0 0.01 0.001"/>')
        if ln == "base_link":
            for cn, (x, y, z, r) in CASTERS.items():
                w(f'{ind}  <geom name="{cn}" class="collision" type="sphere" pos="{fmt(np.array([x, y, z])*MM)}" size="{r*MM}" condim="1" priority="1"/>')
        for sn, (par, mesh) in SENSORS.items():
            if par == ln:
                pos = sensor_pos(links, par, mesh) - origin[par]
                # MuJoCo rangefinders/cameras look along -Z of the site/camera frame; rotate so they look +X
                if sn == "camera_link":
                    w(f'{ind}  <camera name="head_camera" pos="{fmt(pos)}" xyaxes="0 -1 0 0 0 1" fovy="62"/>')
                orient = "" if sn == "imu_link" else ' zaxis="1 0 0"'  # IMU axes = robot axes
                w(f'{ind}  <site name="{sn}" pos="{fmt(pos)}"{orient} size="0.004" rgba="1 0 1 1" group="4"/>')
        for jn, j in children.get(ln, []):
            body(j[2], depth + 1, (jn, j))
        w(f'{ind}</body>')

    body("base_link", 2)
    w('  </worldbody>')
    w('  <actuator>')
    for jn, j in JOINTS.items():
        a = jn.replace("_joint", "")
        if j[0] == "continuous":
            w(f'    <velocity name="{a}" class="wheel" joint="{jn}"/>')
        else:
            w(f'    <position name="{a}" class="servo" joint="{jn}" ctrlrange="{j[5]*DEG:.4f} {j[6]*DEG:.4f}"/>')
    w('  </actuator>')
    w('  <sensor>')
    w('    <accelerometer name="imu_accel" site="imu_link"/>')
    w('    <gyro name="imu_gyro" site="imu_link"/>')
    for s in ("tof_left", "tof_right", "sonar_left", "sonar_right"):
        w(f'    <rangefinder name="{s}" site="{s}_link"/>')
    for jn in JOINTS:
        w(f'    <jointpos name="{jn.replace("_joint", "")}_pos" joint="{jn}"/>')
    w('  </sensor>')
    w('</mujoco>')
    open(os.path.join(HERE, NAME + ".xml"), "w").write("\n".join(L) + "\n")

    scene = f"""<!-- Floor + lights + robot. Open with:  python -m mujoco.viewer --mjcf=sim/scene.xml -->
<mujoco model="{NAME}_scene">
  <include file="{NAME}.xml"/>
  <visual><headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3"/><global azimuth="150" elevation="-20" offwidth="1280" offheight="960"/><rgba rangefinder="1 1 0 0.15"/></visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.9 0.92 0.95" rgb2="0.6 0.65 0.7" width="256" height="256"/>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.82 0.83 0.85" rgb2="0.74 0.75 0.78" width="512" height="512"/>
    <material name="grid" texture="grid" texrepeat="8 8" reflectance="0.05"/>
  </asset>
  <worldbody>
    <light pos="1 -1 2.5" dir="-0.4 0.4 -1" directional="true"/>
    <geom name="floor" type="plane" size="5 5 0.05" material="grid" friction="1.0 0.01 0.001"/>
    <geom name="test_wall" type="box" pos="2.0 0 0.3" size="0.02 1.0 0.3" rgba="0.75 0.7 0.6 1"/>
  </worldbody>
</mujoco>
"""
    open(os.path.join(HERE, "scene.xml"), "w").write(scene)


# ---------------------------------------------------------------- viewer package
PKG = NAME + "_description"
VIEWER_DIR = os.path.join(ROOT, "robot_viewer")


def write_viewer_package(links, origin):
    """Self-contained folder (+ zip) for URDF viewers, laid out as a ROS 2 package.

    robot_viewer/<PKG>/urdf/<NAME>.urdf      meshes as package://<PKG>/meshes/...  (RViz, Foxglove, most web viewers)
    robot_viewer/<PKG>/<NAME>.urdf           meshes as relative meshes/...          (viewers that ignore package://)
    """
    pkg = os.path.join(VIEWER_DIR, PKG)
    if os.path.isdir(pkg):
        shutil.rmtree(pkg)
    for sub in ("urdf", "meshes", "launch"):
        os.makedirs(os.path.join(pkg, sub))
    for f in os.listdir(OUT_MESH):
        shutil.copy2(os.path.join(OUT_MESH, f), os.path.join(pkg, "meshes", f))
    write_urdf(links, origin, os.path.join(pkg, "urdf", NAME + ".urdf"), f"package://{PKG}/meshes/")
    write_urdf(links, origin, os.path.join(pkg, NAME + ".urdf"), "meshes/")
    open(os.path.join(pkg, "package.xml"), "w").write(f"""<?xml version="1.0"?>
<package format="3">
  <name>{PKG}</name>
  <version>0.2.0</version>
  <description>URDF and meshes of the wheeled humanoid v2 (generated by sim/generate_twin.py).</description>
  <maintainer email="ibrahim.fourati@ieee.org">Ibrahim Fourati</maintainer>
  <license>TODO</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher_gui</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <export><build_type>ament_cmake</build_type></export>
</package>
""")
    open(os.path.join(pkg, "CMakeLists.txt"), "w").write(f"""cmake_minimum_required(VERSION 3.8)
project({PKG})
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf meshes launch DESTINATION share/${{PROJECT_NAME}})
ament_package()
""")
    open(os.path.join(pkg, "launch", "display.launch.py"), "w").write(f"""# ros2 launch {PKG} display.launch.py   -> RViz with a slider per joint
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("{PKG}")
    with open(os.path.join(share, "urdf", "{NAME}.urdf")) as f:
        urdf = f.read()
    return LaunchDescription([
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{{"robot_description": urdf}}]),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui"),
        Node(package="rviz2", executable="rviz2"),
    ])
""")
    open(os.path.join(pkg, "README.md"), "w").write(f"""# {PKG}

Generated by `sim/generate_twin.py`; do not edit by hand, re-run the generator instead.

- **Web / desktop URDF viewers:** drag this whole folder in (or the zip next to it).
  If the viewer asks for one file, pick `urdf/{NAME}.urdf`; if meshes do not show,
  use `{NAME}.urdf` at the folder root instead (relative mesh paths).
- **ROS 2:** copy the folder into `~/ros2_ws/src/`, then
  `colcon build --packages-select {PKG} && source install/setup.bash && ros2 launch {PKG} display.launch.py`.
  In RViz set Fixed Frame to `base_footprint` and add a RobotModel display.

Units: metres. Frames: X forward, Y left, Z up. 10 movable joints (2 wheels, 8 servos);
see `sim/README.md` for directions, limits and which values are still estimates.
""")
    zpath = shutil.make_archive(os.path.join(VIEWER_DIR, PKG), "zip", VIEWER_DIR, PKG)
    return pkg, zpath


def main():
    links, origin, report = build()
    write_urdf(links, origin)
    write_mjcf(links, origin)
    write_viewer_package(links, origin)
    total = sum(L["m"] for L in links.values())
    print(f"{'link':28s} {'mass kg':>8s}  com (m, link frame)")
    for ln, L in links.items():
        print(f"{ln:28s} {L['m']:8.4f}  {fmt(L['com'], 4)}")
    print(f"{'TOTAL':28s} {total:8.4f}")
    bad = [r for r in report if not r[4]]
    if bad:
        print("box inertia fallback (mesh not closed or degenerate):", ", ".join(r[1] for r in bad))


if __name__ == "__main__":
    main()
