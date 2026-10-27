"""Build every part and write the exports.

python build.py [OUT_DIR]          SLS edition (default ../exports)
python build.py --fdm [OUT_DIR]    FDM edition (default ../exports/fdm)

SLS:
  step/grip_assembly.step       whole arm, home pose, one named solid per part
  step/<part>.step              each printed part
  stl/<part>.stl                each printed part, ready to send to an SLS service
  printed_parts.csv             part, qty, material, volume, bounding box
FDM:
  stl/<part>.stl, step/<part>.step   FDM fits, links solid and already turned to their print
                                orientation, sitting on z = 0
  printed_parts.csv             part, qty, material, grams of filament, smallest printer it fits
"""
import csv
import os
import sys
from pathlib import Path

FDM_MODE = "--fdm" in sys.argv
if FDM_MODE:
    os.environ["GRIP_PROCESS"] = "fdm"     # read by grip_cad.params at import
    sys.argv.remove("--fdm")

import trimesh
from build123d import Compound, export_step, export_stl

from grip_cad import arm, cycloidal as cy
from grip_cad.assembly import assembly_parts
from grip_cad.gripper import gripper_local
from grip_cad.kits import make_stack
from grip_cad.params import J13_DRIVE, J2_DRIVE, WRIST_DRIVE

PA12 = "SLS PA12"
PA12GF = "SLS PA12-GF (glass filled)"
TPU = "SLS TPU 90A-95A"

LINK_MATERIAL = {"base": PA12, "turret": PA12GF, "upper_arm": PA12GF, "forearm": PA12,
                 "wrist_yoke": PA12, "wrist_link": PA12}
DRIVE_SETS = [("j2_drive", J2_DRIVE, 1), ("j1_j3_drive", J13_DRIVE, 2), ("wrist_drive", WRIST_DRIVE, 3)]
# gearbox discs and housings take the contact loads; glass-filled for stiffness and wear
DRIVE_MATERIAL = {"housing": PA12GF, "cap": PA12, "disc_a": PA12GF, "disc_b": PA12GF,
                  "eccentric": PA12GF, "output_flange": PA12GF}


def printed_parts():
    """[(name, shape, qty, material)] in print-ready (local) orientation where it matters."""
    rows = []
    for name, fn in arm.LINKS.items():
        rows.append((name, fn(), 1, LINK_MATERIAL[name]))
    for set_name, c, qty in DRIVE_SETS:
        for k, shape in cy.printed_parts(c).items():
            rows.append((f"{set_name}_{k}", shape, qty, DRIVE_MATERIAL[k]))
    g = gripper_local()
    rows.append(("gripper_palm", g["palm"][0], 1, PA12))
    rows.append(("gripper_frame", g["frame"][0], 1, PA12))
    rows.append(("gripper_pinion", g["pinion"][0], 1, PA12GF))
    rows.append(("gripper_jaw", g["jaw_a"][0], 2, PA12GF))
    rows.append(("gripper_finger", g["finger_a"][0], 2, TPU))
    return rows


def check_mesh(name, path):
    """Print services reject anything that is not one closed, manifold body."""
    m = trimesh.load(path)
    bodies = m.split(only_watertight=False)
    if not m.is_watertight or len(bodies) != 1:
        raise SystemExit(f"{name}: {len(bodies)} bodies, watertight={m.is_watertight} -- fix before exporting")


def kits():
    """[(name, shape, qty, material)] — bureau-friendly stacked kits of the small parts."""
    out = []
    for set_name, c, qty in DRIVE_SETS:
        out.append((f"kit_{set_name}", make_stack(list(cy.printed_parts(c).values())), qty, PA12))
    g = gripper_local()
    out.append(("kit_gripper", make_stack([g["pinion"][0], g["jaw_a"][0], g["jaw_b"][0]]), 1, PA12))
    out.append(("kit_fingers", make_stack([g["finger_a"][0], g["finger_b"][0]]), 1, TPU))
    return out


def main(out):
    out = Path(out)
    (out / "step").mkdir(parents=True, exist_ok=True)
    (out / "stl").mkdir(parents=True, exist_ok=True)
    rows = printed_parts()
    with open(out / "printed_parts.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["part", "qty", "material", "volume_cm3", "bbox_x_mm", "bbox_y_mm", "bbox_z_mm", "stl"])
        for name, shape, qty, mat in rows:
            export_step(shape, str(out / "step" / f"{name}.step"))
            export_stl(shape, str(out / "stl" / f"{name}.stl"), tolerance=0.03, angular_tolerance=0.15)
            check_mesh(name, out / "stl" / f"{name}.stl")
            bb = shape.bounding_box().size
            w.writerow([name, qty, mat, round(shape.volume / 1000, 1), round(bb.X, 1), round(bb.Y, 1), round(bb.Z, 1),
                        f"stl/{name}.stl"])
            print(f"{name:28s} x{qty}  {shape.volume / 1000:7.1f} cm3  {mat}")
    (out / "stl" / "kits").mkdir(exist_ok=True)
    with open(out / "service_kits.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["kit", "qty", "material", "volume_cm3", "bbox_x_mm", "bbox_y_mm", "bbox_z_mm", "stl"])
        for name, shape, qty, mat in kits():
            path = out / "stl" / "kits" / f"{name}.stl"
            export_stl(shape, str(path), tolerance=0.03, angular_tolerance=0.15)
            check_mesh(name, path)
            bb = shape.bounding_box().size
            w.writerow([name, qty, mat, round(shape.volume / 1000, 1), round(bb.X, 1), round(bb.Y, 1), round(bb.Z, 1),
                        f"stl/kits/{name}.stl"])
            print(f"{name:28s} x{qty}  {shape.volume / 1000:7.1f} cm3  {mat}  {bb.X:.0f}x{bb.Y:.0f}x{bb.Z:.0f}")
    named = []
    for name, (shape, kind) in assembly_parts().items():
        shape.label = name
        named.append(shape)
    export_step(Compound(children=named, label="grip_arm"), str(out / "step" / "grip_assembly.step"))
    print("assembly written")


# ---- FDM edition ------------------------------------------------------------------------

PETG = "FDM PETG"
PETG_CF = "FDM PETG-CF (or PA-CF)"
FTPU = "FDM TPU 95A"
DENSITY = {PETG: 1.27, PETG_CF: 1.30, FTPU: 1.21}       # g/cm3
FDM_DRIVE_MATERIAL = {"housing": PETG, "cap": PETG, "disc_a": PETG_CF, "disc_b": PETG_CF,
                      "eccentric": PETG_CF, "output_flange": PETG_CF}
# which way each part faces the bed, in the frame it is modelled in. Links print with
# their socket mouth up, so the bore that takes the gearbox is round and needs no support.
FDM_DOWN = {"base": (0, 0, -1), "turret": (0, 1, 0), "upper_arm": (0, -1, 0), "forearm": (0, 0, -1),
            "wrist_yoke": (0, 1, 0), "wrist_link": (0, 0, -1),
            "gripper_jaw": (1, 0, 0), "gripper_finger": (0, -1, 0)}
FLAT = 0.6           # shaved off a domed underside so it has a footprint on the bed
WALLS, INFILL = 1.6, 0.15    # 4 perimeters / top-bottom shell, 15% gyroid
BEDS = [("Ender-3 class 220x220x250", 220, 220, 250), ("Prusa MK4S 250x210x220", 250, 210, 220),
        ("Bambu A1/P1S/X1 256x256x256", 256, 256, 256)]


def fdm_orient(name, shape):
    from build123d import Align, Box, Pos, Rot
    down = FDM_DOWN.get(name, (0, 0, -1))
    rot = {(0, 0, -1): Rot(0, 0, 0), (0, 1, 0): Rot(-90, 0, 0), (0, -1, 0): Rot(90, 0, 0),
           (1, 0, 0): Rot(0, 90, 0), (-1, 0, 0): Rot(0, -90, 0)}[down]
    s = rot * shape
    bb = s.bounding_box()
    s = Pos(-bb.center().X, -bb.center().Y, -bb.min.Z) * s
    if name in arm.LINKS and name != "base":
        s = s - Pos(0, 0, -50) * Box(1000, 1000, 50 + FLAT, align=(Align.CENTER, Align.CENTER, Align.MIN))
        s = Pos(0, 0, -FLAT) * s
    return s


def fdm_grams(mesh, mat):
    shell = min(mesh.area * WALLS, mesh.volume)
    return DENSITY[mat] * (shell + (mesh.volume - shell) * INFILL) / 1000


def bed_fit(mesh):
    """Smallest printer in BEDS that takes the part, turning it about Z to fit."""
    import numpy as np
    from math import radians
    xy = mesh.convex_hull.vertices[:, :2]
    h = mesh.bounds[1][2] - mesh.bounds[0][2]
    for label, bx, by, bz in BEDS:
        if h > bz:
            continue
        for deg in range(0, 180):
            a = radians(deg)
            r = xy @ np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
            w, d = np.ptp(r[:, 0]), np.ptp(r[:, 1])
            if w <= bx - 4 and d <= by - 4:
                return label
    return "too big for a 256 mm printer"


def fdm_main(out):
    from grip_cad.coupon import coupon
    out = Path(out)
    (out / "step").mkdir(parents=True, exist_ok=True)
    (out / "stl").mkdir(parents=True, exist_ok=True)
    rows = [(name, shape, qty, PETG) for name, shape, qty, _ in printed_parts()]
    rows = [(n, s, q, FDM_DRIVE_MATERIAL.get(n.split("_drive_")[-1], PETG) if "_drive_" in n else m)
            for n, s, q, m in rows]
    rows = [(n, s, q, PETG_CF if n in ("gripper_pinion", "gripper_jaw") else FTPU if n == "gripper_finger" else m)
            for n, s, q, m in rows]
    rows.insert(0, ("fit_coupon", coupon(), 1, PETG))
    total = {}
    with open(out / "printed_parts.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["part", "qty", "material", "grams_each", "solid_cm3", "bbox_x_mm", "bbox_y_mm", "bbox_z_mm",
                    "fits", "stl"])
        for name, shape, qty, mat in rows:
            shape = fdm_orient(name, shape)
            export_step(shape, str(out / "step" / f"{name}.step"))
            path = out / "stl" / f"{name}.stl"
            export_stl(shape, str(path), tolerance=0.03, angular_tolerance=0.15)
            check_mesh(name, path)
            m = trimesh.load(path)
            g = fdm_grams(m, mat)
            total[mat] = total.get(mat, 0.0) + g * qty
            fit = bed_fit(m)
            ext = m.bounds[1] - m.bounds[0]
            w.writerow([name, qty, mat, round(g), round(m.volume / 1000, 1), *[round(v, 1) for v in ext], fit,
                        f"stl/{name}.stl"])
            print(f"{name:28s} x{qty}  {g:6.0f} g  {mat:22s} {fit}")
    for mat, g in total.items():
        print(f"{mat}: {g / 1000:.2f} kg")


if __name__ == "__main__":
    if FDM_MODE:
        fdm_main(sys.argv[1] if len(sys.argv) > 1 else Path(__file__).parent.parent / "exports" / "fdm")
    else:
        main(sys.argv[1] if len(sys.argv) > 1 else Path(__file__).parent.parent / "exports")
