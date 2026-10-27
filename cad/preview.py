"""Render preview images of the assembled arm.

python preview.py OUT_DIR [--pose j1,j2,j3,j4,j5,j6]   (degrees, default home pose)
Writes STL meshes to OUT_DIR/mesh and PNG views to OUT_DIR.
"""
import json
import subprocess
import sys
from pathlib import Path

from build123d import export_stl

from grip_cad.assembly import assembly_parts

HERE = Path(__file__).parent
COLORS = {"shell": "#efe9dd", "drive": "#8d9aa6", "motor": "#2b2b2e", "tpu": "#e07a5f", "vendor": "#b8b8b8"}
VIEWS = {"front": {"azim": -60, "elev": 16, "zoom": 1.05},
         "back": {"azim": 120, "elev": 16, "zoom": 1.05},
         "side": {"azim": 0, "elev": 8, "zoom": 1.05}}


def main(out, pose=None, views=VIEWS, width=900, height=1200):
    out = Path(out)
    (out / "mesh").mkdir(parents=True, exist_ok=True)
    parts = []
    for name, (shape, kind) in assembly_parts(pose).items():
        f = out / "mesh" / f"{name}.stl"
        export_stl(shape, str(f), tolerance=0.08, angular_tolerance=0.15)
        parts.append({"file": str(f.resolve()), "color": COLORS[kind],
                      "metalness": 0.35 if kind == "motor" else 0.0})
    pngs = []
    for vname, v in views.items():
        cfg = out / f"view_{vname}.json"
        cfg.write_text(json.dumps({"width": width, "height": height, "view": v, "parts": parts}))
        png = out / f"{vname}.png"
        subprocess.run(["node", str(HERE / "render3d" / "render.mjs"), str(cfg), str(png)], check=True)
        pngs.append(png)
    return pngs


if __name__ == "__main__":
    args = sys.argv[1:]
    pose = None
    if "--pose" in args:
        i = args.index("--pose")
        pose = [float(x) for x in args[i + 1].split(",")]
        del args[i:i + 2]
    print(main(args[0], pose))
