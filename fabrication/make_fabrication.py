#!/usr/bin/env python3
"""Build the fabrication package from print/ and laser/ (run after any CAD change).

For every 3D-printed part it searches the orientations on the bed (6 axis-aligned + the mesh's stable
poses) and keeps the one with the least unsupported overhang, then the largest contact area, then the
lowest height. It writes the oriented STL with a numbered name, and the tables in README.md / parts.csv.
For every laser-cut plate it copies the DXF with a numbered name and measures size and cut length.

Part numbers are the item numbers of the drawings' parts list (info/wheeled_humanoid_v2_drawings_1.pdf).
Needs: pip install trimesh manifold3d ezdxf rtree scipy
"""
import csv
import math
import os
import shutil

import numpy as np
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
BED = (300, 300, 300)               # mm, FabLab printer (team information)
PLA_DENSITY = 1.24                  # g/cm3
LINE_W, LAYER = 0.45, 0.20          # mm, nozzle line width and layer height
OVERHANG_DEG = 45                   # faces steeper than this from vertical need help
SMALL_FEATURE = 10.0                # mm: holes and slots this narrow print without support
MAX_BRIDGE = 45.0                   # mm: flat spans up to this bridge without support in PLA

# (no., file stem, process, qty, infill %, walls, role) ; material PLA for every printed part
PRINTED = [
    (3, "bumper_pillar_x4", 4, 40, 4, "deck spacer, carries the upper deck and torso in compression"),
    (4, "motor_bracket_left", 1, 50, 4, "holds a gearmotor: robot weight and motor torque"),
    (4, "motor_bracket_right", 1, 50, 4, "holds a gearmotor: robot weight and motor torque"),
    (5, "caster_spacer_front", 1, 100, 3, "3.5 mm shim under a caster: solid"),
    (5, "caster_spacer_rear", 1, 100, 3, "3.5 mm shim under a caster: solid"),
    (6, "sensor_bar", 1, 25, 3, "carries the 2 ToF and 2 ultrasonic sensors: light load"),
    (7, "control_panel", 1, 40, 4, "holds the E-stop and the switch: must not flex when the button is hit"),
    (13, "corner_bracket_x8", 8, 60, 4, "joins the torso plates: small and stressed"),
    (15, "shoulder_bracket_left", 1, 40, 4, "holds a shoulder pitch servo (30 kg.cm)"),
    (15, "shoulder_bracket_right", 1, 40, 4, "holds a shoulder pitch servo (30 kg.cm)"),
    (16, "shoulder_yoke_left", 1, 40, 4, "pitch horn to roll servo: carries the whole arm"),
    (16, "shoulder_yoke_right", 1, 40, 4, "pitch horn to roll servo: carries the whole arm"),
    (17, "upper_arm_left", 1, 30, 4, "roll horn to elbow servo, 164 mm lever"),
    (17, "upper_arm_right", 1, 30, 4, "roll horn to elbow servo, 164 mm lever"),
    (19, "head_yoke", 1, 40, 4, "pan horn to tilt servo: carries the head"),
    (20, "head_shell", 1, 15, 3, "cosmetic shell with the camera and the OLED eyes"),
]
# (no., file stem, qty)
LASER = [
    (1, "lower_deck", 1), (2, "upper_deck", 1), (8, "torso_side_left_mega", 1), (9, "torso_side_right_pi", 1),
    (10, "torso_shelf_1", 1), (10, "torso_shelf_2", 1), (11, "chest_plate", 1), (12, "back_plate", 1),
    (14, "shoulder_plate", 1), (18, "forearm_left", 1), (18, "forearm_right", 1),
]


def qty_name(stem, qty):
    base = stem[: -len(f"_x{qty}")] if stem.endswith(f"_x{qty}") else stem
    return f"{base}_x{qty}"


# ------------------------------------------------------------------ print orientation
def candidate_rotations(mesh):
    rots = []
    for axis, ang in [((1, 0, 0), 0), ((1, 0, 0), 180), ((1, 0, 0), 90), ((1, 0, 0), -90), ((0, 1, 0), 90),
                      ((0, 1, 0), -90)]:
        rots.append(trimesh.transformations.rotation_matrix(math.radians(ang), axis))
    try:
        poses, prob = trimesh.poses.compute_stable_poses(mesh, n_samples=1, threshold=0.01)
        for T in poses[:8]:
            T = T.copy()
            T[:3, 3] = 0
            rots.append(T)
    except Exception:
        pass
    return rots


def overhang_report(m):
    """Unsupported downward faces grouped into connected regions, classified by their narrow width."""
    zmin = m.bounds[0, 2]
    n = m.face_normals
    cz = m.triangles_center[:, 2]
    down = (n[:, 2] < -math.cos(math.radians(90 - OVERHANG_DEG))) & (cz > zmin + 0.3)
    idx = np.nonzero(down)[0]
    res = dict(small=0.0, bridge=0.0, support=0.0, max_bridge=0.0, support_regions=[])
    if not len(idx):
        return res
    sel = set(idx.tolist())
    adj = [p for p in m.face_adjacency if p[0] in sel and p[1] in sel]
    graph = trimesh.graph.connected_components(adj, nodes=idx, min_len=1) if adj else [[i] for i in idx]
    areas = m.area_faces
    for comp in graph:
        comp = np.asarray(list(comp))
        pts = m.triangles[comp].reshape(-1, 3)
        ext = np.sort(pts[:, :2].max(0) - pts[:, :2].min(0))
        a = float(areas[comp].sum())
        width = ext[0]
        if width <= SMALL_FEATURE:
            res["small"] += a
        elif width <= MAX_BRIDGE:
            res["bridge"] += a
            res["max_bridge"] = max(res["max_bridge"], float(width))
        else:
            res["support"] += a
            res["support_regions"].append((round(float(ext[0])), round(float(ext[1])),
                                           round(float(pts[:, 2].max() - zmin), 1)))
    return res


def bed_contact(m):
    zmin = m.bounds[0, 2]
    face_down = (m.face_normals[:, 2] < -0.999) & (np.abs(m.triangles[:, :, 2] - zmin).max(axis=1) < 0.05)
    return float(m.area_faces[face_down].sum())


def best_orientation(mesh):
    best = None
    for R in candidate_rotations(mesh):
        m = mesh.copy()
        m.apply_transform(R)
        oh = overhang_report(m)
        contact = bed_contact(m)
        h = m.extents[2]
        key = (round(oh["support"], -1), -round(contact, -1), round(oh["bridge"], -1), h)
        if best is None or key < best[0]:
            best = (key, m, oh, contact)
    _, m, oh, contact = best
    m.apply_translation([-m.bounds[:, 0].mean(), -m.bounds[:, 1].mean(), -m.bounds[0, 2]])
    return m, oh, contact


def grams(mesh, infill, walls):
    """Shell (walls, top and bottom) solid, the rest at the infill ratio."""
    shell = min(1.0, mesh.area * walls * LINE_W / 2 / 1000 / (mesh.volume / 1000) if mesh.volume else 1)
    vol_cm3 = mesh.volume / 1000
    return PLA_DENSITY * vol_cm3 * (shell + infill / 100 * (1 - shell))


def describe(oh, contact):
    if oh["support"] > 0:
        top = max(z for _, _, z in oh["support_regions"])
        if top <= 6:  # only a rounded or chamfered lower edge
            sup = f"build plate only, under the rounded lower edge (first {top:.0f} mm)"
        else:
            regs = ", ".join(f"{a} x {b} mm up to {z:.0f} mm" for a, b, z in oh["support_regions"][:3])
            sup = f"yes, {oh['support'] / 100:.1f} cm2 ({regs})"
    elif oh["bridge"] > 0:
        sup = f"none (bridges up to {oh['max_bridge']:.0f} mm)"
    else:
        sup = "none"
    return sup


# ------------------------------------------------------------------ laser plates
def dxf_report(path):
    import ezdxf
    import ezdxf.path
    doc = ezdxf.readfile(path)
    msp = doc.modelspace()
    length, holes, xs, ys = 0.0, 0, [], []

    def add_pts(pts):
        for x, y in pts:
            xs.append(x)
            ys.append(y)

    for e in msp:
        t = e.dxftype()
        if t == "LINE":
            a, b = (e.dxf.start.x, e.dxf.start.y), (e.dxf.end.x, e.dxf.end.y)
            length += math.dist(a, b)
            add_pts([a, b])
        elif t == "CIRCLE":
            length += 2 * math.pi * e.dxf.radius
            holes += 1
            c, r = e.dxf.center, e.dxf.radius
            add_pts([(c[0] - r, c[1] - r), (c[0] + r, c[1] + r)])
        elif t == "ARC":
            span = (e.dxf.end_angle - e.dxf.start_angle) % 360 or 360
            length += math.radians(span) * e.dxf.radius
            add_pts([(p.x, p.y) for p in e.flattening(0.05)])
        elif t in ("LWPOLYLINE", "POLYLINE", "SPLINE", "ELLIPSE"):
            pts = [(p.x, p.y) for p in ezdxf.path.make_path(e).flattening(0.05)]
            length += sum(math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))
            add_pts(pts)
    units = doc.header.get("$INSUNITS", 0)
    return dict(w=max(xs) - min(xs), h=max(ys) - min(ys), cut_m=length / 1000, circles=holes, units=units)


def plate_section(stem):
    """Cut the flat plate solid (laser_stl/) at mid-thickness: the exact 2D profile of the part."""
    m = trimesh.load(os.path.join(ROOT, "laser_stl", stem + ".stl"))
    ax = int(np.argmin(m.extents))
    n = np.zeros(3)
    n[ax] = 1
    p2, _ = m.section(plane_origin=m.bounds.mean(0), plane_normal=n).to_2D()
    return p2


def dxf_net_area(path):
    import ezdxf
    import ezdxf.path as P
    from shapely.geometry import LineString
    from shapely.ops import polygonize, unary_union
    lines = []
    for e in ezdxf.readfile(path).modelspace():
        if e.dxftype() in ("LINE", "ARC", "CIRCLE", "LWPOLYLINE"):
            pts = [(round(v.x, 4), round(v.y, 4)) for v in P.make_path(e).flattening(0.02)]
            if len(pts) > 1:
                lines.append(LineString(pts))
    polys = list(polygonize(unary_union(lines)))
    if not polys:
        return 0.0, 0
    big = max(polys, key=lambda p: p.area)
    return big.area, len(big.interiors)


def write_dxf_from_section(p2, path):
    """Outline and holes as a DXF in mm; near-perfect circles become CIRCLE entities."""
    import ezdxf
    doc = ezdxf.new("R2010")
    doc.header["$INSUNITS"] = 4  # mm
    msp = doc.modelspace()
    mn = np.array(p2.bounds[0])
    for poly in p2.polygons_full:
        for ring in [poly.exterior, *poly.interiors]:
            pts = np.asarray(ring.coords)[:-1] - mn
            c = pts.mean(0)
            r = np.linalg.norm(pts - c, axis=1)
            if len(pts) >= 8 and r.std() < 0.01 * r.mean():
                msp.add_circle(tuple(c), float(r.mean()))
            else:
                msp.add_lwpolyline([tuple(q) for q in pts], close=True)
    doc.saveas(path)


def main():
    out_p, out_l = os.path.join(HERE, "print"), os.path.join(HERE, "laser")
    for d in (out_p, out_l):
        shutil.rmtree(d, ignore_errors=True)
        os.makedirs(d)
    rows_p, rows_l = [], []
    for no, stem, qty, infill, walls, role in PRINTED:
        mesh = trimesh.load(os.path.join(ROOT, "print", stem + ".stl"))
        assert mesh.is_watertight, stem
        m, oh, contact = best_orientation(mesh)
        orig = mesh.copy()
        orig.apply_translation([0, 0, -orig.bounds[0, 2]])
        c0, oh0 = bed_contact(orig), overhang_report(orig)
        if abs(contact - c0) < 1 and abs(oh["bridge"] - oh0["bridge"]) < 1:
            orient = f"as designed: {contact / 100:.0f} cm2 flat on the bed"
        else:
            orient = (f"turned: flat contact {c0 / 100:.1f} to {contact / 100:.0f} cm2"
                      + (f", bridging {oh0['bridge'] / 100:.0f} to {oh['bridge'] / 100:.0f} cm2" if oh0["bridge"] > oh["bridge"] + 100 else ""))
        name = f"P{no:02d}_{qty_name(stem, qty)}.stl"
        m.export(os.path.join(out_p, name))
        ex = m.extents
        g = grams(mesh, infill, walls)
        rows_p.append(dict(no=f"P{no:02d}", file=name, part=stem.replace("_x4", "").replace("_x8", ""), qty=qty,
                           material="PLA", infill=f"{infill} %", walls=walls, layer=f"{LAYER} mm",
                           size_on_bed=f"{ex[0]:.0f} x {ex[1]:.0f} x {ex[2]:.0f}",
                           bed_contact_cm2=round(contact / 100, 1), supports=describe(oh, contact),
                           grams_each=round(g), grams_total=round(g * qty), orient=orient,
                           fits_bed=all(e <= b for e, b in zip(sorted(ex[:2]) + [ex[2]], sorted(BED[:2]) + [BED[2]])),
                           role=role))
    for no, stem, qty in LASER:
        name = f"L{no:02d}_{stem}_x{qty}.dxf"
        src = os.path.join(ROOT, "laser", stem + ".dxf")
        # check the DXF against the CAD solid; regenerate it from the solid if they disagree
        sec = plate_section(stem)
        a_dxf, holes_dxf = dxf_net_area(src)
        holes_cad = sum(len(p.interiors) for p in sec.polygons_full)
        if abs(a_dxf - sec.area) > 0.005 * sec.area or holes_dxf != holes_cad:
            print(f"  {stem}.dxf disagrees with the CAD solid ({a_dxf:.0f} vs {sec.area:.0f} mm2, "
                  f"{holes_dxf} vs {holes_cad} holes): regenerated from the solid")
            write_dxf_from_section(sec, src)
            a_dxf, holes_dxf = dxf_net_area(src)
        shutil.copy(src, os.path.join(out_l, name))
        r = dxf_report(src)
        rows_l.append(dict(no=f"L{no:02d}", file=name, part=stem, qty=qty, material="5 mm sheet",
                           size=f"{r['w']:.1f} x {r['h']:.1f}", cut_length_m=round(r["cut_m"], 2),
                           circles=r["circles"], area_cm2=round(r["w"] * r["h"] / 100),
                           net_area_cm2=round(a_dxf / 100, 1), holes=holes_dxf,
                           cad_check=f"{100 * (a_dxf - sec.area) / sec.area:+.2f} %"))
    with open(os.path.join(HERE, "parts.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["no", "file", "qty", "process", "material", "infill", "walls", "layer", "size_mm",
                    "supports", "grams_each"])
        for r in rows_p:
            w.writerow([r["no"], r["file"], r["qty"], "3D print", "PLA", r["infill"], r["walls"], r["layer"],
                        r["size_on_bed"], r["supports"], r["grams_each"]])
        for r in rows_l:
            w.writerow([r["no"], r["file"], r["qty"], "laser cut", r["material"], "", "", "", r["size"], "", ""])
    write_readme(rows_p, rows_l)
    render_sheet(out_p)
    return rows_p, rows_l


def render_sheet(folder):
    """One picture of every printed part as it sits on the bed."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    files = sorted(f for f in os.listdir(folder) if f.endswith(".stl"))
    fig = plt.figure(figsize=(16, 16))
    light = np.array([0.3, -0.5, 0.8]) / np.linalg.norm([0.3, -0.5, 0.8])
    for i, f in enumerate(files):
        m = trimesh.load(os.path.join(folder, f))
        ax = fig.add_subplot(4, 4, i + 1, projection="3d")
        sh = 0.35 + 0.65 * np.clip(m.face_normals @ light, 0, 1)
        ax.add_collection3d(Poly3DCollection(m.triangles, facecolors=np.c_[sh * 0.9, sh * 0.55, sh * 0.2],
                                             edgecolor="none"))
        e, c = m.extents.max(), m.bounds.mean(0)
        ax.set_xlim(c[0] - e / 2, c[0] + e / 2)
        ax.set_ylim(c[1] - e / 2, c[1] + e / 2)
        ax.set_zlim(0, e)
        xx, yy = np.meshgrid([c[0] - e / 2, c[0] + e / 2], [c[1] - e / 2, c[1] + e / 2])
        ax.plot_surface(xx, yy, np.zeros_like(xx), color="lightblue", alpha=0.4)
        ax.view_init(22, -60)
        ax.set_title(f[:-4], fontsize=10)
        ax.set_axis_off()
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, "print_orientations.png"), dpi=75)
    plt.close(fig)


def write_readme(rows_p, rows_l):
    gp = sum(r["grams_total"] for r in rows_p)
    n_p = sum(r["qty"] for r in rows_p)
    cut = sum(r["cut_length_m"] * r["qty"] for r in rows_l)
    L = ["# Fabrication package", "",
         "Generated by `make_fabrication.py` from `print/` and `laser/`. Part numbers are the item numbers of the "
         "parts list in the drawings (`info/wheeled_humanoid_v2_drawings_1.pdf`, sheet 1): **P** = 3D printed, "
         "**L** = laser cut, `_x<n>` = quantity to make from that file.", "",
         f"**3D printing:** {len(rows_p)} files, {n_p} pieces, about {gp} g of PLA. Every STL here is already "
         "oriented for printing (placed on the bed at z = 0) and fits a 300 x 300 x 300 mm bed; the largest part is "
         "the head shell, 197 mm tall. Common settings: PLA, 0.2 mm layers, 0.4 mm nozzle, brim on the tall thin "
         "parts (P03, P06, P20).", "",
         "| No. | File | Qty | Infill | Walls | Orientation on the bed | Supports | Size on bed (mm) | g each | Why these settings |",
         "|---|---|---|---|---|---|---|---|---|---|"]
    for r in rows_p:
        L.append(f"| {r['no']} | `{r['file']}` | {r['qty']} | {r['infill']} | {r['walls']} | {r['orient']} | "
                 f"{r['supports']} | {r['size_on_bed']} | {r['grams_each']} | {r['role']} |")
    L += ["", "How the orientation was chosen: the script tries the six flat orientations and the mesh's stable "
          f"poses, and keeps the one with the least unsupported overhang (faces steeper than {OVERHANG_DEG} deg), "
          "then the largest contact with the bed, then the lowest height; on a tie it keeps the designed orientation. Holes and slots up to "
          f"{SMALL_FEATURE:.0f} mm wide print without support; flat spans up to {MAX_BRIDGE:.0f} mm are bridged. "
          "Masses: PLA 1.24 g/cm3, walls and top/bottom solid, the rest at the infill ratio.", "",
          "![every part as it sits on the bed](print_orientations.png)", "",
          f"**Laser cutting:** {len(rows_l)} files, one piece each, 5 mm sheet (acrylic in the digital twin; plywood or "
          f"MDF works with the same files), about {cut:.1f} m of cut. The machine bed must take a 435 mm part "
          "(L08, L09, L11, L12). DXF in mm, 1:1, outline and holes only.", "",
          "| No. | File | Qty | Size (mm) | Holes | Cut length (m) | Check against the CAD solid |",
          "|---|---|---|---|---|---|---|"]
    for r in rows_l:
        L.append(f"| {r['no']} | `{r['file']}` | {r['qty']} | {r['size']} | {r['holes']} | {r['cut_length_m']} | "
                 f"net area {r['cad_check']} |")
    L += ["", "Every DXF is checked against a section of the plate's solid at mid-thickness: net area and number of "
          "holes must agree. The forearm files (L18) failed this check (they held an edge view, 38 x 5 mm) and "
          "were regenerated from the solid; round holes are written as true circles.", "",
          "Hardware not in the kit, to request from the organisers: M3 x 10 and M3 x 16 screws, M3 nuts, M3 "
          "standoffs, M2.5 standoffs for the Pi."]
    with open(os.path.join(HERE, "README.md"), "w") as f:
        f.write("\n".join(L) + "\n")


if __name__ == "__main__":
    import json
    p, l = main()
    for r in p:
        print(json.dumps({k: r[k] for k in ("no", "file", "size_on_bed", "bed_contact_cm2", "supports", "grams_each", "fits_bed")}))
    for r in l:
        print(json.dumps(r))
    print("printed total g:", sum(r["grams_total"] for r in p), " pieces:", sum(r["qty"] for r in p))
    print("laser total cut m:", round(sum(r["cut_length_m"] * r["qty"] for r in l), 2),
          " area cm2:", sum(r["area_cm2"] * r["qty"] for r in l))
