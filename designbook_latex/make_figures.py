#!/usr/bin/env python3
"""Figures of the design book that come from the V2 model. Run from the repository root:
    .venv/bin/python designbook_latex/make_figures.py
Renders the twin on a white background, draws the torque and tip-over plots, and copies the figures
produced elsewhere (simulation results, fabrication sheet, wiring diagram, drawings) into figs/.
"""
import math
import os
import shutil

import matplotlib
import mujoco
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
FIGS = os.path.join(HERE, "figs")
KGCM = 0.0980665
plt.rcParams.update({"font.family": "sans-serif", "font.size": 9, "axes.edgecolor": "black",
                     "axes.linewidth": 0.6, "axes.grid": True, "grid.color": "0.85", "grid.linewidth": 0.5})


# ------------------------------------------------------------------ renders
def model():
    spec = mujoco.MjSpec.from_file(os.path.join(ROOT, "sim", "wheeled_humanoid_v2.xml"))
    spec.visual.global_.offwidth, spec.visual.global_.offheight = 1600, 1600
    spec.visual.headlight.ambient = [0.45, 0.45, 0.45]
    spec.visual.headlight.diffuse = [0.55, 0.55, 0.55]
    w = spec.worldbody
    w.add_light(pos=[1.5, 1.0, 2.5], dir=[-0.5, -0.4, -1], type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL,
                diffuse=[0.5, 0.5, 0.5])
    m = spec.compile()
    return m, mujoco.MjData(m)


def shot(m, d, r, azimuth, elevation=-14, distance=1.55, lookat=(0, 0, 0.36), size=(900, 1100)):
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = lookat
    cam.distance, cam.azimuth, cam.elevation = distance, azimuth, elevation
    r.update_scene(d, camera=cam)
    img = r.render().copy()
    r.enable_segmentation_rendering()
    r.update_scene(d, camera=cam)
    seg = r.render()
    r.disable_segmentation_rendering()
    img[seg[:, :, 0] < 0] = 255  # background to white
    return img


def trim(img, pad=20):
    """Crop the white border around a render."""
    mask = (img[:, :, :3] < 250).any(axis=2)
    ys, xs = np.nonzero(mask)
    y0, y1 = max(ys.min() - pad, 0), min(ys.max() + pad, img.shape[0])
    x0, x1 = max(xs.min() - pad, 0), min(xs.max() + pad, img.shape[1])
    return img[y0:y1, x0:x1]


def renders():
    m, d = model()
    q = lambda j: m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j + "_joint")]
    mujoco.mj_forward(m, d)
    r = mujoco.Renderer(m, 1100, 900)
    # assembly, rest pose: front left, rear left, front right (robot faces +x)
    views = [shot(m, d, r, a) for a in (215, 325, 145)]
    fig, ax = plt.subplots(1, 3, figsize=(12, 5.2))
    for a, im, t in zip(ax, views, ("front left", "rear left", "front right")):
        a.imshow(trim(im))
        a.set_title(t, fontsize=10)
        a.axis("off")
    fig.tight_layout()
    fig.savefig(os.path.join(FIGS, "v2_assembly_views.png"), dpi=150)
    plt.close(fig)
    # base close-up from above the front left
    img = shot(m, d, r, 200, elevation=-38, distance=0.85, lookat=(0, 0, 0.12))
    plt.imsave(os.path.join(FIGS, "v2_base_view.png"), trim(img))
    # cover: waving pose
    d.qpos[q("r_sh_roll")] = math.radians(70)
    d.qpos[q("r_elbow")] = math.radians(80)
    d.qpos[q("head_pan")] = math.radians(-25)
    mujoco.mj_forward(m, d)
    img = shot(m, d, r, 205, elevation=-8, distance=1.5)
    plt.imsave(os.path.join(FIGS, "v2_cover.png"), trim(img))


# ------------------------------------------------------------------ plots
def torque_plot():
    m = mujoco.MjModel.from_xml_path(os.path.join(ROOT, "sim", "wheeled_humanoid_v2.xml"))
    d = mujoco.MjData(m)
    j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "l_sh_pitch_joint")
    pitch = np.arange(-45, 181, 2.5)
    tq = []
    for p in pitch:
        d.qpos[:] = m.qpos0
        d.qvel[:] = 0
        d.qpos[m.jnt_qposadr[j]] = math.radians(p)
        mujoco.mj_forward(m, d)
        tq.append(abs(d.qfrc_bias[m.jnt_dofadr[j]]) / KGCM)
    tq = np.array(tq)
    payload = 0.1 * 9.81 * 0.245 * np.abs(np.sin(np.radians(pitch))) / KGCM
    fig, ax = plt.subplots(figsize=(5.2, 3.3))
    ax.plot(pitch, tq, "k-", lw=1.4, label="arm alone")
    ax.plot(pitch, tq + payload, "k--", lw=1.2, label="with 100 g in the hand")
    ax.axhline(10, color="0.45", lw=1, ls=":", label="design limit: a third of 30 kg.cm")
    ax.set_xlabel("shoulder pitch (deg, 0 = arm hanging)")
    ax.set_ylabel("static torque (kg.cm)")
    ax.set_ylim(0, 11)
    ax.legend(fontsize=8, loc="center right", frameon=False)
    fig.tight_layout()
    fig.savefig(os.path.join(FIGS, "v2_shoulder_torque.png"), dpi=200)
    plt.close(fig)
    return tq.max()


def tipover_plot(com_h=0.217, d_back=0.1435, d_front=0.1565):
    h = np.linspace(0.10, 0.50, 200)
    fig, ax = plt.subplots(figsize=(5.2, 3.3))
    ax.plot(h * 1000, 9.81 * d_back / h, "k-", lw=1.4, label="tips backward (over the rear caster)")
    ax.plot(h * 1000, 9.81 * d_front / h, "k--", lw=1.2, label="tips forward (over the front caster)")
    ax.axhline(0.5, color="0.45", ls=":", lw=1, label="firmware acceleration limit 0.5 m/s2")
    a = 9.81 * d_back / com_h
    ax.plot([com_h * 1000], [a], "ko", ms=5)
    ax.annotate(f"design: {com_h * 1000:.0f} mm, {a:.1f} m/s2", (com_h * 1000, a), (com_h * 1000 + 30, a + 2),
                fontsize=8, arrowprops=dict(arrowstyle="-", lw=0.6))
    ax.set_xlabel("centre of mass height (mm)")
    ax.set_ylabel("acceleration that tips the robot (m/s2)")
    ax.set_ylim(0, 15)
    ax.legend(fontsize=8, frameon=False)
    fig.tight_layout()
    fig.savefig(os.path.join(FIGS, "v2_tipover.png"), dpi=200)
    plt.close(fig)


def copies():
    pairs = {
        "sim/results/demo_frames.png": "v2_demo_frames.png",
        "sim/results/demo_signals.png": "v2_demo_signals.png",
        "sim/results/clearance_map.png": "v2_clearance_map.png",
        "fabrication/print_orientations.png": "v2_print_orientations.png",
        "wiring/wheeled_humanoid_wiring.pdf": "v2_wiring.pdf",
    }
    for src, dst in pairs.items():
        shutil.copy(os.path.join(ROOT, src), os.path.join(FIGS, dst))
    masked = os.path.join(FIGS, "v2_drawings.pdf")
    mask_drawings(os.path.join(ROOT, "info", "wheeled_humanoid_v2_drawings_1.pdf"), masked)
    # the sheets used in the book, as 300 dpi images: no hidden text under the white patches
    import pypdfium2 as pdfium
    doc = pdfium.PdfDocument(masked)
    for n in (1, 2, 13, 25, 30):
        doc[n - 1].render(scale=300 / 72).to_pil().convert("L").save(os.path.join(FIGS, f"drawing_sheet{n:02d}.png"), optimize=True)
    doc.close()
    os.remove(masked)


def mask_drawings(src, dst):
    """Copy of the drawings with the " v2" of the title blocks covered in white (the original is untouched)."""
    import re
    import pypdfium2 as pdfium
    from pypdf import PdfWriter, PageObject
    from pypdf.generic import DecodedStreamObject, NameObject
    doc, writer = pdfium.PdfDocument(src), PdfWriter(clone_from=src)
    for i, page in enumerate(writer.pages):
        tp = doc[i].get_textpage()
        text = tp.get_text_range()
        ops = []
        for m in re.finditer(r"(?i)v2", text):
            boxes = [tp.get_charbox(k) for k in range(m.start(), m.end())]
            hgt = max(b[3] for b in boxes) - min(b[1] for b in boxes)  # text size sets the margins
            x0 = min(b[0] for b in boxes) - 0.25 * hgt
            x1 = max(b[2] for b in boxes) + 0.15 * hgt
            y0 = min(b[1] for b in boxes) - 0.2 * hgt
            y1 = max(b[3] for b in boxes) + 0.2 * hgt
            ops.append(f"{x0:.2f} {y0:.2f} {x1 - x0:.2f} {y1 - y0:.2f} re f")
        if ops:
            w, h = float(page.mediabox.width), float(page.mediabox.height)
            over = PageObject.create_blank_page(width=w, height=h)
            stream = DecodedStreamObject()
            stream.set_data(("q 1 1 1 rg " + " ".join(ops) + " Q").encode())
            over[NameObject("/Contents")] = writer._add_object(stream)
            page.merge_page(over)
    with open(dst, "wb") as f:
        writer.write(f)


if __name__ == "__main__":
    os.makedirs(FIGS, exist_ok=True)
    renders()
    print("peak static shoulder torque %.2f kg.cm" % torque_plot())
    tipover_plot()
    copies()
    print("figures written to", FIGS)
