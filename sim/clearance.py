#!/usr/bin/env python3
"""Clearance check of the V2 digital twin with the exact CAD solids (not the simplified collision shapes).

Every part of a moving group (one arm, or the head) is intersected with every part it could touch, at
each pose of a sweep through the firmware joint ranges, the gesture keyframes and the logged DEMO
trajectory. Parts that already touch at the CAD pose (servo horns screwed to brackets) keep that contact
as a baseline: a pose interferes when the overlap grows by more than VOL_MM3 over the baseline.

  python sim/clearance.py         writes sim/results/clearance.md, clearance.csv, clearance_map.png
Needs: pip install manifold3d   (and mujoco, numpy, matplotlib)
"""
import csv
import itertools
import os
import re
import time

import manifold3d as mf
import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RESULTS = os.path.join(HERE, "results")
VOL_MM3 = 5.0                     # overlap growth that counts as interference
STEP = 15                         # degrees, sweep step

ARM_L = ["shoulder_yoke_left_link", "upper_arm_left_link", "forearm_left_link"]
ARM_R = ["shoulder_yoke_right_link", "upper_arm_right_link", "forearm_right_link"]
HEAD = ["head_yoke_link", "head_link"]
JOINTS = ["l_sh_pitch", "l_sh_roll", "l_elbow", "r_sh_pitch", "r_sh_roll", "r_elbow", "head_pan", "head_tilt"]


def firmware_limits():
    """J_MIN / J_MAX from firmware/lib/robot_core/src/config.h."""
    txt = open(os.path.join(ROOT, "firmware", "lib", "robot_core", "src", "config.h")).read()
    arr = lambda n: [float(v) for v in re.search(n + r"\[NJ\]\s*=\s*\{([^}]*)\}", txt).group(1).split(",")]
    return arr("J_MIN"), arr("J_MAX")


class Twin:
    def __init__(self):
        self.m = mujoco.MjModel.from_xml_path(os.path.join(HERE, "wheeled_humanoid_v2.xml"))
        self.d = mujoco.MjData(self.m)
        m = self.m
        self.qadr = {j: m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j + "_joint")] for j in JOINTS}
        self.parts = []  # (geom id, body name, part name, local manifold, local vertices)
        for g in range(m.ngeom):
            if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
                continue
            mid = m.geom_dataid[g]
            a, n = m.mesh_vertadr[mid], m.mesh_vertnum[mid]
            fa, fn = m.mesh_faceadr[mid], m.mesh_facenum[mid]
            V = m.mesh_vert[a:a + n].astype(np.float64) * 1000.0  # mm
            F = m.mesh_face[fa:fa + fn].astype(np.uint32)
            man = mf.Manifold(mf.Mesh(V.astype(np.float32), F))
            body = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g])
            self.parts.append((g, body, mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_MESH, mid), man, V))
        self.set_pose({})

    def set_pose(self, deg):
        self.d.qpos[:] = self.m.qpos0
        for j, v in deg.items():
            self.d.qpos[self.qadr[j]] = np.radians(v)
        mujoco.mj_kinematics(self.m, self.d)
        self.world = {}
        for g, body, name, man, V in self.parts:
            R = self.d.geom_xmat[g].reshape(3, 3)
            t = self.d.geom_xpos[g] * 1000.0
            W = V @ R.T + t
            T = np.hstack([R, t[:, None]]).astype(np.float32)  # 3 x 4
            self.world[g] = (W.min(0), W.max(0), T)

    def overlap(self, ga, gb):
        amin, amax, Ta = self.world[ga]
        bmin, bmax, Tb = self.world[gb]
        if np.any(amax < bmin) or np.any(bmax < amin):
            return 0.0
        A = self.man[ga].transform(Ta)
        B = self.man[gb].transform(Tb)
        return (A ^ B).volume()

    @property
    def man(self):
        if not hasattr(self, "_man"):
            self._man = {g: man for g, _, _, man, _ in self.parts}
        return self._man


def pairs_for(twin, moving):
    """(moving part, other part) for every part of the moving bodies against every part of other bodies,
    plus the non-adjacent links inside the group (forearm against yoke)."""
    mov = [p for p in twin.parts if p[1] in moving]
    rest = [p for p in twin.parts if p[1] not in moving]
    out = [(a, b) for a in mov for b in rest]
    if len(moving) == 3:
        out += [(a, b) for a in mov if a[1] == moving[2] for b in mov if b[1] == moving[0]]
    return out


def check(twin, pairs, base, pose):
    twin.set_pose(pose)
    hits = []
    for a, b in pairs:
        v = twin.overlap(a[0], b[0]) - base[(a[0], b[0])]
        if v > VOL_MM3:
            hits.append((round(v, 1), a[2], b[2]))
    return sorted(hits, reverse=True)


def baseline(twin, pairs):
    twin.set_pose({})
    return {(a[0], b[0]): twin.overlap(a[0], b[0]) for a, b in pairs}


def rng(lo, hi, step=STEP):
    return [float(v) for v in np.arange(lo, hi + 0.1, step)]


def main():
    t0 = time.time()
    J_MIN, J_MAX = firmware_limits()
    tw = Twin()
    rows = []

    def run(group, label, poses, pairs, base):
        for pose in poses:
            hits = check(tw, pairs, base, pose)
            rows.append(dict(group=group, sweep=label, **{j: pose.get(j, 0.0) for j in JOINTS},
                             overlap_mm3=round(sum(h[0] for h in hits), 1),
                             parts="; ".join(f"{a} / {b} ({v} mm3)" for v, a, b in hits[:3])))

    for side, arm in (("left", ARM_L), ("right", ARM_R)):
        p = "l_" if side == "left" else "r_"
        pairs = pairs_for(tw, arm)
        base = baseline(tw, pairs)
        P, R, E = p + "sh_pitch", p + "sh_roll", p + "elbow"
        lims = {P: (J_MIN[0], J_MAX[0]), R: (J_MIN[1], J_MAX[1]), E: (J_MIN[2], J_MAX[2])}
        # grid over the firmware ranges (pitch x roll x elbow), with roll from -30 deg to show the boundary
        grid = [{P: a, R: b, E: c} for a, b, c in itertools.product(rng(*lims[P]), rng(-30, lims[R][1]), rng(*lims[E], 45))]
        run(f"{side} arm", "grid", grid, pairs, base)
    pairs = pairs_for(tw, HEAD)
    base = baseline(tw, pairs)
    run("head", "grid", [{"head_pan": a, "head_tilt": b} for a in rng(J_MIN[6], J_MAX[6])
                         for b in rng(J_MIN[7], J_MAX[7], 10)], pairs, base)
    # DEMO trajectory (all joints at once), from the firmware-in-the-loop log
    log = os.path.join(RESULTS, "demo_log.csv")
    if os.path.exists(log):
        allpairs = pairs_for(tw, ARM_L) + pairs_for(tw, ARM_R) + pairs_for(tw, HEAD)
        allpairs = list({(a[0], b[0]): (a, b) for a, b in allpairs}.values())
        base = baseline(tw, allpairs)
        with open(log) as f:
            demo = [{j: float(r["q_" + j]) for j in JOINTS} for i, r in enumerate(csv.DictReader(f)) if i % 10 == 0]
        run("whole robot", "DEMO trajectory", demo, allpairs, base)
    write(rows, (J_MIN, J_MAX), boundary(tw), time.time() - t0)


def boundary(tw):
    """Highest interfering inward roll at any pitch, in 1 deg steps (elbow 0 and 135)."""
    out = {}
    for side, arm, p in (("left", ARM_L, "l_"), ("right", ARM_R, "r_")):
        pairs = pairs_for(tw, arm)
        base = baseline(tw, pairs)
        worst = None
        for pitch in rng(-45, 180, 5):
            for elbow in (0.0, 135.0):
                for roll in rng(-30, 0, 1):
                    h = check(tw, pairs, base, {p + "sh_pitch": pitch, p + "sh_roll": roll, p + "elbow": elbow})
                    if h and (worst is None or roll > worst[0]):
                        worst = (roll, pitch, elbow, h[0])
        out[side] = worst
    return out


def allowed(row, side, lim):
    J_MIN, J_MAX = lim
    p = "l_" if side == "left" else "r_"
    return row[p + "sh_roll"] >= J_MIN[1]


def write(rows, lim, bnd, secs):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    os.makedirs(RESULTS, exist_ok=True)
    with open(os.path.join(RESULTS, "clearance.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    J_MIN, J_MAX = lim
    L = ["# Clearance check, V2 twin, exact CAD solids", "",
         f"Generated by `sim/clearance.py` in {secs:.0f} s. A pose interferes when the overlap between two parts "
         f"grows by more than {VOL_MM3:g} mm3 over the CAD pose. Ranges are the firmware joint limits "
         "(`config.h`), in {STEP} deg steps (elbow in 45 deg steps, head tilt in 10 deg steps).".replace("{STEP}", str(STEP)), ""]
    fig, axes = plt.subplots(1, 4, figsize=(17, 4.4), sharey=True)
    for side in ("left", "right"):
        p = "l_" if side == "left" else "r_"
        g = [r for r in rows if r["group"] == f"{side} arm"]
        bad = [r for r in g if r["overlap_mm3"] > 0]
        bad_allowed = [r for r in bad if allowed(r, side, lim)]
        L.append(f"## {side.capitalize()} arm: {len(g)} poses")
        L.append(f"- interfering poses in the full range: {len(bad)}")
        L.append(f"- interfering poses that the firmware allows (roll >= {J_MIN[1]:g} deg): **{len(bad_allowed)}**")
        b = bnd[side]
        if b:
            L.append(f"- 1 deg sweep: the highest interfering inward roll is **{b[0]:g} deg** (pitch {b[1]:g}, elbow "
                     f"{b[2]:g}: {b[3][1]} / {b[3][2]}, {b[3][0]} mm3), so the {J_MIN[1]:g} deg limit leaves "
                     f"{J_MIN[1] - b[0]:g} deg of margin")
        for r in bad_allowed[:8]:
            L.append(f"  - pitch {r[p + 'sh_pitch']:g}, roll {r[p + 'sh_roll']:g}, elbow {r[p + 'elbow']:g}: {r['parts']}")
        worst = {}
        for r in bad:
            k = r["parts"].split(" (")[0]
            worst[k] = worst.get(k, 0) + 1
        if worst:
            L.append("- parts that meet (number of poses): " + "; ".join(f"{k} ({n})" for k, n in
                                                                       sorted(worst.items(), key=lambda x: -x[1])[:5]))
        L.append("")
        if side == "left":  # map: worst overlap over the elbow values, per pitch x roll
            pitches = sorted({r[p + "sh_pitch"] for r in g})
            rolls = sorted({r[p + "sh_roll"] for r in g})
            for k, elbow in enumerate(sorted({r[p + "elbow"] for r in g})):
                M = np.zeros((len(rolls), len(pitches)))
                for r in g:
                    if r[p + "elbow"] == elbow:
                        M[rolls.index(r[p + "sh_roll"]), pitches.index(r[p + "sh_pitch"])] = r["overlap_mm3"]
                ax = axes[k]
                ax.imshow(np.log10(1 + M), origin="lower", aspect="auto", cmap="Reds", vmin=0, vmax=4,
                          extent=[pitches[0] - STEP / 2, pitches[-1] + STEP / 2, rolls[0] - STEP / 2, rolls[-1] + STEP / 2])
                ax.fill_between([pitches[0] - STEP / 2, pitches[-1] + STEP / 2], rolls[0] - STEP / 2,
                                J_MIN[1] - STEP / 2, facecolor="none", edgecolor="k", hatch="//", lw=0)
                ax.set_title(f"left arm, elbow {elbow:g} deg")
                ax.set_xlabel("shoulder pitch (deg)")
                if k == 0:
                    ax.set_ylabel("shoulder roll (deg, + outward)")
    fig.suptitle("Interference (red, log of overlap mm3) over the firmware range; hatched: outside the firmware roll limit")
    fig.tight_layout()
    fig.savefig(os.path.join(RESULTS, "clearance_map.png"), dpi=110)
    plt.close(fig)
    for grp in ("head", "whole robot"):
        g = [r for r in rows if r["group"] == grp]
        if not g:
            continue
        bad = [r for r in g if r["overlap_mm3"] > 0]
        L.append(f"## {grp.capitalize()} ({g[0]['sweep']}): {len(g)} poses, {len(bad)} interfering")
        for r in bad[:8]:
            L.append(f"  - {', '.join(f'{j} {r[j]:.0f}' for j in JOINTS if abs(r[j]) > 0.5)}: {r['parts']}")
        L.append("")
    L.append("![clearance map](clearance_map.png)")
    with open(os.path.join(RESULTS, "clearance.md"), "w") as f:
        f.write("\n".join(L) + "\n")
    print("\n".join(L))


if __name__ == "__main__":
    main()
