"""Quick shaded previews of the arm from STL meshes (matplotlib, no GPU needed).

python render.py out.png part1.stl part2.stl ... [--view elev,azim] [--colors c1,c2,...]
"""
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import trimesh
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

LIGHT = np.array([0.4, -0.5, 0.8])
LIGHT = LIGHT / np.linalg.norm(LIGHT)


def render(out, meshes, views=((20, -60),), size=(9, 11), title=None):
    fig = plt.figure(figsize=(size[0] * len(views), size[1]), dpi=110)
    allv = np.vstack([m.vertices for m, _ in meshes])
    lo, hi = allv.min(0), allv.max(0)
    mid, span = (lo + hi) / 2, (hi - lo).max() / 2
    for i, (elev, azim) in enumerate(views):
        ax = fig.add_subplot(1, len(views), i + 1, projection="3d")
        for m, color in meshes:
            tris = m.vertices[m.faces]
            n = m.face_normals
            shade = 0.35 + 0.65 * np.clip(n @ LIGHT, 0, 1)
            base = np.array(matplotlib.colors.to_rgb(color))
            fc = np.clip(base[None] * shade[:, None] + 0.08, 0, 1)
            pc = Poly3DCollection(tris, facecolors=fc, edgecolors="none", linewidths=0)
            ax.add_collection3d(pc)
        ax.set_xlim(mid[0] - span, mid[0] + span)
        ax.set_ylim(mid[1] - span, mid[1] + span)
        ax.set_zlim(mid[2] - span, mid[2] + span)
        ax.set_box_aspect((1, 1, 1))
        ax.view_init(elev=elev, azim=azim)
        ax.set_axis_off()
    if title:
        fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(out, facecolor="white")
    plt.close(fig)


if __name__ == "__main__":
    args = sys.argv[1:]
    views = ((20, -60),)
    colors = None
    if "--view" in args:
        i = args.index("--view")
        views = tuple(tuple(float(x) for x in v.split(",")) for v in args[i + 1].split(";"))
        del args[i:i + 2]
    if "--colors" in args:
        i = args.index("--colors")
        colors = args[i + 1].split(",")
        del args[i:i + 2]
    out, files = args[0], args[1:]
    colors = colors or ["#e8e4dc"] * len(files)
    render(out, [(trimesh.load(f), c) for f, c in zip(files, colors)], views)
