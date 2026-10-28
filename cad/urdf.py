"""Write a URDF of the arm, generated from the same CAD as the printed parts.

python urdf.py [OUT_DIR]      (default ../urdf)

  grip.urdf          6 revolute joints + 2 prismatic gripper jaws (jaw_b mimics jaw_a)
  meshes/<link>.stl  one mesh per moving body, in millimetres (scaled 0.001 in the URDF)

Masses and inertias come from the CAD: printed parts at SLS PA12 density, steel pins
and bearings at steel density, motors at their catalogue mass (estimates, marked
below). Every link frame sits on its joint axis in the upright home pose, unrotated, so
the meshes keep their home-pose coordinates and only shift by the frame origin.
"""
import sys
import tempfile
from math import radians
from pathlib import Path
from xml.sax.saxutils import escape

import fast_simplification
import numpy as np
import trimesh
from build123d import Pos, export_stl

from grip_cad import arm, gripper as gr
from grip_cad.params import GEOM as G, NEMA17_34, NEMA17_60, NEMA23_56

# kg per mm^3
PA12, PA12GF, TPU, STEEL = 1.01e-6, 1.22e-6, 1.20e-6, 7.85e-6
BEARING_FILL = 0.65          # a ball bearing weighs about this much of a solid steel ring
# catalogue masses (VERIFY), each with its closed-loop driver board (~40 g)
MOTOR_KG = {NEMA17_60.name: 0.50 + 0.04, NEMA17_34.name: 0.22 + 0.04, NEMA23_56.name: 0.70 + 0.05}
SERVO_KG = 0.055             # STS3215
CAMERA_KG = 0.015            # Arducam UB0234 board, lens and cable stub
GF_LINKS = {"turret", "upper_arm"}
GF_DRIVE = {"housing", "disc_a", "disc_b", "eccentric", "output_flange"}

# joint: (parent, child, origin in the home pose [mm], axis, lower, upper [deg], effort [N m], velocity [rad/s])
# effort = holding torque x ratio x 0.8; velocity = 600 rpm at the motor. Both estimates.
JOINTS = [
    ("j1", "base_link", "turret", (0, 0, 0), (0, 0, 1), -170, 170, 13.0, 2.5),
    ("j2", "turret", "upper_arm", (0, G.y_j2_out, G.z_j2), (0, 1, 0), -60, 60, 30.0, 2.1),
    ("j3", "upper_arm", "forearm", (0, arm.y_j3_out, G.z_j3), (0, 1, 0), -120, 120, 13.0, 2.5),
    ("j4", "forearm", "wrist_yoke", (0, G.y_forearm, G.z_j5 - 40.0), (0, 0, 1), -170, 170, 3.1, 4.2),
    ("j5", "wrist_yoke", "wrist_link", (0, arm.y_j5_out, G.z_j5), (0, 1, 0), -100, 100, 3.1, 4.2),
    ("j6", "wrist_link", "tool", (0, G.y_forearm, G.z_flange), (0, 0, 1), -170, 170, 3.1, 4.2),
]
JAW_FORCE = 30.0             # N, STS3215 at 30 kg cm on a 10 mm pinion is ~290 N; fingers limit it far lower


def bodies():
    """{link: [(shape, kg_per_mm3 or ('kg', mass))]} with every part in home-pose coordinates."""
    out = {k: [] for k in ("base_link", "turret", "upper_arm", "forearm", "wrist_yoke", "wrist_link", "tool",
                           "jaw_a", "jaw_b")}
    link_body = {"base": "base_link", "turret": "turret", "upper_arm": "upper_arm", "forearm": "forearm",
                 "wrist_yoke": "wrist_yoke", "wrist_link": "wrist_link"}
    for name, fn in arm.LINKS.items():
        out[link_body[name]].append((fn(), PA12GF if name in GF_LINKS else PA12))
    stationary_in = {j: parent for j, parent, *_ in JOINTS}
    output_in = {j: child for j, _, child, *_ in JOINTS}
    for j, parts in arm.drive_parts().items():
        c = arm.JOINTS[j][0]
        for k, shape in parts.items():
            body = output_in[j] if k in ("output_flange", "output_pins") else stationary_in[j]
            if k == "motor":
                dens = ("kg", MOTOR_KG[c.motor.name])
            elif k == "bearings":
                dens = STEEL * BEARING_FILL
            elif k in ("ring_pins", "output_pins"):
                dens = STEEL
            else:
                dens = PA12GF if k in GF_DRIVE else PA12
            out[body].append((shape, dens))
    loc = Pos(0, G.y_forearm, G.z_flange)
    for k, (shape, kind) in gr.gripper_local(0.0).items():
        body = "jaw_a" if k.endswith("_a") and not k.startswith("rod") else \
            "jaw_b" if k.endswith("_b") and not k.startswith("rod") else "tool"
        if k == "servo":
            dens = ("kg", SERVO_KG)
        elif k == "camera":
            dens = ("kg", CAMERA_KG)
        elif k.startswith("rod"):
            dens = STEEL
        elif k.startswith("bearings"):
            dens = STEEL * BEARING_FILL
        elif kind == "tpu":
            dens = TPU
        else:
            dens = PA12GF if k in ("pinion", "jaw_a", "jaw_b") else PA12
        out[body].append((loc * shape, dens))
    return out


def _mesh(shape, tmp, tol):
    f = Path(tmp) / "p.stl"
    export_stl(shape, str(f), tolerance=tol, angular_tolerance=0.3)
    return trimesh.load(f)


def mass_props(parts, tmp):
    """(mass kg, centre of mass mm, inertia about it kg mm^2) of a list of (shape, density)."""
    items = []
    for shape, dens in parts:
        vol = shape.volume
        mass = dens[1] if isinstance(dens, tuple) else dens * vol
        m = _mesh(shape, tmp, 0.2)
        m.density = mass / abs(m.volume)
        c = shape.center()
        items.append((mass, np.array([c.X, c.Y, c.Z]), m.moment_inertia))
    M = sum(i[0] for i in items)
    com = sum(i[0] * i[1] for i in items) / M
    I = np.zeros((3, 3))
    for mass, c, Ii in items:
        d = c - com
        I += Ii + mass * (d @ d * np.eye(3) - np.outer(d, d))
    return M, com, I


def write(out):
    out = Path(out)
    (out / "meshes").mkdir(parents=True, exist_ok=True)
    origin = {child: np.array(o, float) for _, _, child, o, *_ in JOINTS}
    origin["base_link"] = np.zeros(3)
    origin["jaw_a"] = origin["jaw_b"] = origin["tool"]
    xml = ['<?xml version="1.0"?>',
           '<!-- Generated by cad/urdf.py from the GRIP CAD. Do not edit by hand. -->',
           '<robot name="grip">']
    with tempfile.TemporaryDirectory() as tmp:
        for link, parts in bodies().items():
            M, com, I = mass_props(parts, tmp)
            o = origin[link]
            merged = trimesh.util.concatenate([_mesh(s, tmp, 0.25) for s, _ in parts])
            merged.merge_vertices()
            if len(merged.faces) > 20000:      # keep the repo light; physics uses the inertials above
                v, f = fast_simplification.simplify(merged.vertices.astype(np.float32),
                                                    merged.faces.astype(np.int32), target_reduction=0.75)
                merged = trimesh.Trimesh(v, f)
            merged.export(out / "meshes" / f"{link}.stl")
            c = (com - o) / 1000
            I = I / 1e6                                    # kg mm^2 -> kg m^2
            vis = " ".join(f"{v / 1000:.6g}" for v in -o)
            mesh = f'<mesh filename="meshes/{link}.stl" scale="0.001 0.001 0.001"/>'
            xml += [f'  <link name="{link}">',
                    f'    <inertial><origin xyz="{c[0]:.6g} {c[1]:.6g} {c[2]:.6g}"/><mass value="{M:.4f}"/>',
                    f'      <inertia ixx="{I[0, 0]:.6g}" ixy="{I[0, 1]:.6g}" ixz="{I[0, 2]:.6g}" '
                    f'iyy="{I[1, 1]:.6g}" iyz="{I[1, 2]:.6g}" izz="{I[2, 2]:.6g}"/></inertial>',
                    f'    <visual><origin xyz="{vis}"/><geometry>{mesh}</geometry></visual>',
                    f'    <collision><origin xyz="{vis}"/><geometry>{mesh}</geometry></collision>',
                    '  </link>']
            print(f"{link:12s} {M:6.3f} kg  com {np.round(com, 1)}")
    for j, parent, child, o, axis, lo, hi, effort, vel in JOINTS:
        rel = (np.array(o, float) - origin[parent]) / 1000
        xml += [f'  <joint name="{j}" type="revolute">',
                f'    <parent link="{parent}"/><child link="{child}"/>',
                f'    <origin xyz="{rel[0]:.6g} {rel[1]:.6g} {rel[2]:.6g}"/><axis xyz="{axis[0]} {axis[1]} {axis[2]}"/>',
                f'    <limit lower="{radians(lo):.5f}" upper="{radians(hi):.5f}" effort="{effort}" velocity="{vel}"/>',
                '  </joint>']
    stroke = gr.STROKE / 1000
    for name, sign, mimic in (("jaw_a", 1, None), ("jaw_b", -1, "jaw_a")):
        xml += [f'  <joint name="{name}" type="prismatic">',
                f'    <parent link="tool"/><child link="{name}"/>',
                f'    <origin xyz="0 0 0"/><axis xyz="{sign} 0 0"/>',
                f'    <limit lower="0" upper="{stroke}" effort="{JAW_FORCE}" velocity="0.05"/>']
        if mimic:
            xml.append(f'    <mimic joint="{escape(mimic)}" multiplier="1" offset="0"/>')
        xml.append('  </joint>')
    # wrist camera: a massless frame on the board front, +Z along the optical axis (the
    # ROS optical convention). Hand-eye calibration refines it; see docs/VISION.md.
    xml += ['  <link name="wrist_camera_optical"/>',
            '  <joint name="wrist_camera" type="fixed">',
            '    <parent link="tool"/><child link="wrist_camera_optical"/>',
            f'    <origin xyz="0 {gr.CAM_Y / 1000:.6g} {gr.CAM_Z / 1000:.6g}" rpy="{radians(-gr.CAM_TILT):.5f} 0 0"/>',
            '  </joint>']
    xml.append('</robot>')
    (out / "grip.urdf").write_text("\n".join(xml) + "\n")


if __name__ == "__main__":
    write(sys.argv[1] if len(sys.argv) > 1 else Path(__file__).parent.parent / "urdf")
