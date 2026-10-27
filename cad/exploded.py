"""Exploded renders of each gearbox for the build guide, written to docs/img/.

python exploded.py [OUT_DIR]      (needs Node + Playwright, like preview.py)

Parts are pulled apart along the joint axis in assembly order, bottom to top.
"""
import json
import subprocess
import sys
import tempfile
from pathlib import Path

from build123d import Pos, export_stl

from grip_cad import cycloidal as cy
from grip_cad.params import J13_DRIVE, J2_DRIVE, WRIST_DRIVE

HERE = Path(__file__).parent
COLORS = {"printed": "#efe9dd", "steel": "#b8bcc2", "motor": "#2b2b2e"}


def layers(c):
    """[(name, shape, kind)] in the order they go together."""
    L = cy.layout(c)
    eb = c.ecc_bearing
    e1 = cy.bearing_ring(eb, L.disc1[0] + (c.disc_t - eb.w) / 2, c.ecc)
    e2 = cy.bearing_ring(eb, L.disc2[0] + (c.disc_t - eb.w) / 2, -c.ecc)
    return [
        ("motor", cy.motor(c), "motor"),
        ("housing", cy.housing(c), "printed"),
        ("ring_pins", cy.ring_pins(c), "steel"),
        ("eccentric", cy.eccentric(c), "printed"),
        ("disc_b", cy.disc(c, second=True), "printed"),
        ("bearing_b", e2, "steel"),
        ("disc_a", cy.disc(c), "printed"),
        ("bearing_a", e1, "steel"),
        ("output_pins", cy.output_pins(c), "steel"),
        ("output_flange", cy.output_flange(c), "printed"),
        ("output_bearing", cy.bearing_ring(c.out_bearing, L.cap_bot), "steel"),
        ("cap", cy.cap(c), "printed"),
    ]


def render(c, png, gap=None, view=None):
    gap = gap or c.housing_r * 0.45
    view = view or {"azim": -55, "elev": 22, "zoom": 0.68}
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        parts = []
        lift = 0.0
        for i, (name, shape, kind) in enumerate(layers(c)):
            # bearings travel with their disc; everything else gets its own step
            if not name.startswith("bearing_") or name == "output_bearing":
                lift = gap * i if i else -gap * 0.6
            f = tmp / f"{name}.stl"
            export_stl(Pos(0, 0, lift) * shape, str(f), tolerance=0.03, angular_tolerance=0.15)
            parts.append({"file": str(f), "color": COLORS[kind], "metalness": 0.35 if kind != "printed" else 0.0})
        cfg = tmp / "view.json"
        cfg.write_text(json.dumps({"width": 900, "height": 1300, "view": view, "parts": parts}))
        subprocess.run(["node", str(HERE / "render3d" / "render.mjs"), str(cfg), str(png)], check=True)


def main(out):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    for c, slug in ((J13_DRIVE, "j1_j3"), (J2_DRIVE, "j2"), (WRIST_DRIVE, "wrist")):
        render(c, out / f"exploded_{slug}.png")
        print(slug)


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else HERE.parent / "docs" / "img")
