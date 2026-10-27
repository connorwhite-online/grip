"""Build every part and write the exports.

python build.py [OUT_DIR]   (default ../exports)

  step/grip_assembly.step       whole arm, home pose, one named solid per part
  step/<part>.step              each printed part
  stl/<part>.stl                each printed part, ready to send to an SLS service
  printed_parts.csv             part, qty, material, volume, bounding box
"""
import csv
import sys
from pathlib import Path

import trimesh
from build123d import Compound, export_step, export_stl

from grip_cad import arm, cycloidal as cy
from grip_cad.assembly import assembly_parts
from grip_cad.gripper import gripper_local
from grip_cad.kits import make_kit
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
    """[(name, shape, qty, material)] — bureau-friendly sprue kits of the small parts."""
    out = []
    for set_name, c, qty in DRIVE_SETS:
        out.append((f"kit_{set_name}", make_kit(list(cy.printed_parts(c).values())), qty, PA12))
    g = gripper_local()
    out.append(("kit_gripper", make_kit([g["pinion"][0], g["jaw_a"][0], g["jaw_b"][0]]), 1, PA12))
    out.append(("kit_fingers", make_kit([g["finger_a"][0], g["finger_b"][0]]), 1, TPU))
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


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else Path(__file__).parent.parent / "exports")
