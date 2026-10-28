"""Render the v0.3 hand: generated palm, three TPU fingers and their nails,
open and closed. Writes generative/renders/hand.png."""
import math
import os
import sys

import numpy as np
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from hand import FINGER, ROOTS, ROOT_Z, SERVO_BODY, SHAFT, SPOOL_R  # noqa: E402

OUT = os.path.join(HERE, "hand")


def _rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def curled(mesh, deg):
    """Bend the finger at its three flexures by `deg` each (toward the palm side, +y)."""
    f = FINGER
    t, yp = f["thick"], f["thick"] / 2
    L = f["root"] + sum(f["segments"])
    za = f["root"] + f["segments"][0] * 0.5
    zs = [f["root"], f["root"] + f["segments"][0], f["root"] + f["segments"][0] + f["segments"][1]]
    v0 = mesh.vertices.copy()
    v = v0.copy()
    for zh in reversed(zs):
        back = -yp if zh <= za else -yp + (t - f["tip_thick"]) * (zh - za) / (L - za)
        piv = np.array([0.0, back + f["hinge"] / 2, zh])
        sel = v0[:, 2] > zh
        r = _rot_x(-math.radians(deg))
        v[sel] = (v[sel] - piv) @ r.T + piv
    return trimesh.Trimesh(v, mesh.faces, process=False)


def placed(mesh, x, y, cy):
    m = mesh.copy()
    if cy < 0:   # palm side faces -y: turn the finger round
        m.apply_transform(trimesh.transformations.rotation_matrix(math.pi, [0, 0, 1]))
    m.apply_translation([x, y, ROOT_Z - FINGER["root"]])
    return m


def scene(deg):
    finger = trimesh.load(os.path.join(OUT, "finger.stl"))
    parts = [(trimesh.load(os.path.join(OUT, "palm.stl")), (0.80, 0.80, 0.82))]
    # the gripper servo and its spool, for scale
    lo, hi = np.array(SERVO_BODY["min"]), np.array(SERVO_BODY["max"])
    servo = trimesh.creation.box(extents=hi - lo)
    servo.apply_translation(0.5 * (lo + hi))
    parts.append((servo, (0.20, 0.35, 0.65)))
    spool = trimesh.creation.cylinder(radius=SPOOL_R, height=10.0)
    spool.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [0, 1, 0]))
    spool.apply_translation([SHAFT[0] + 5.0, SHAFT[1], SHAFT[2]])
    parts.append((spool, (0.20, 0.35, 0.65)))
    nail = trimesh.load(os.path.join(OUT, "nail.stl"))
    nail.apply_translation([0, FINGER["thick"] / 2 - 1.2 - FINGER["nail_t"] / 2, 0])
    for x, y, cy in ROOTS:
        parts.append((placed(curled(finger, deg), x, y, cy), (0.30, 0.32, 0.36)))
        parts.append((placed(curled(nail, deg), x, y, cy), (0.85, 0.55, 0.20)))
    return parts


def main():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    poses = [(0, "open"), (26, "closed")]
    views = [(18, -60), (18, 30)]
    fig = plt.figure(figsize=(16, 8), dpi=110)
    k = 0
    for deg, label in poses:
        parts = scene(deg)
        allv = np.vstack([m.vertices for m, _ in parts])
        lo, hi = allv.min(0), allv.max(0)
        mid, span = 0.5 * (lo + hi), (hi - lo).max()
        for elev, azim in views:
            k += 1
            ax = fig.add_subplot(2, 2, k, projection="3d")
            e, a = math.radians(elev), math.radians(azim)
            vd = np.array([math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e)])
            key = vd + np.array([0.3, -0.2, 0.6])
            key /= np.linalg.norm(key)
            for m, base in parts:
                if len(m.faces) > 40000:
                    try:
                        m = m.simplify_quadric_decimation(face_count=40000)
                    except Exception:  # noqa: BLE001
                        pass
                sh = np.clip(m.face_normals @ key, 0, 1)
                col = np.clip(np.array(base)[None] * (0.35 + 0.6 * sh)[:, None], 0, 1)
                ax.add_collection3d(Poly3DCollection(m.triangles, facecolors=col, edgecolor="none"))
            for i, c in enumerate("xyz"):
                getattr(ax, f"set_{c}lim")(mid[i] - span / 2, mid[i] + span / 2)
            ax.set_box_aspect((1, 1, 1), zoom=1.3)
            ax.view_init(elev, azim)
            ax.set_axis_off()
            ax.set_title(label, fontsize=11)
    fig.tight_layout()
    path = os.path.join(HERE, "renders", "hand.png")
    fig.savefig(path, facecolor="white")
    print(path)


if __name__ == "__main__":
    main()
