"""Technical drawings for the build guide, written as SVG to docs/drawings/.

python drawings.py [OUT_DIR]

Each gearbox gets a half-section with numbered parts, a parts list and the main
dimensions. The arm and gripper get dimensioned side views. All views are generated
from the same CAD as the exports, so they stay in step with params.py.
"""
import re
import sys
from pathlib import Path

from build123d import (
    Align, Box, Compound, Draft, Edge, ExportSVG, ExtensionLine, LineType, Plane, Pos, Text, Unit,
)

from grip_cad import cycloidal as cy
from grip_cad.params import GEOM as G, J13_DRIVE, J2_DRIVE, WRIST_DRIVE

INK = (30, 30, 30)
FILL = {"printed": (236, 226, 208), "steel": (168, 176, 184), "motor": (206, 208, 212)}
DRAFT = Draft(font_size=3.0, decimal_precision=0, display_units=False, arrow_length=2.0,
              line_width=0.15, extension_gap=1.5, pad_around_text=1.0)


class Sheet:
    """Collects 2D shapes by layer and writes one SVG on a white background."""

    def __init__(self, draft=DRAFT):
        self.draft = draft
        self.layers = {k: [] for k in ("printed", "steel", "motor", "visible", "hidden", "thin", "ink")}

    def add(self, layer, *shapes):
        self.layers[layer].extend(shapes)

    def text(self, s, x, y, size=3.0, align=(Align.MIN, Align.CENTER)):
        self.add("ink", Pos(x, y) * Text(s, size, align=align))

    def line(self, p, q, layer="thin"):
        self.add(layer, Edge.make_line((*p, 0), (*q, 0)))

    def balloon(self, n, target, at, r=3.0):
        self.add("thin", Edge.make_circle(r).moved(Pos(*at)))
        self.text(str(n), at[0], at[1], 2.8, (Align.CENTER, Align.CENTER))
        dx, dy = target[0] - at[0], target[1] - at[1]
        d = (dx * dx + dy * dy) ** 0.5
        if d > r:
            self.line((at[0] + dx / d * r, at[1] + dy / d * r), target)
        self.add("ink", Pos(*target) * Text("•", 3.0, align=(Align.CENTER, Align.CENTER)))

    def dim(self, p, q, offset, label=None):
        self.add("ink", ExtensionLine(border=[(*p, 0), (*q, 0)], offset=offset, draft=self.draft, label=label))

    def write(self, path, scale=1.0):
        svg = ExportSVG(unit=Unit.MM, scale=scale, margin=6, line_weight=0.25)
        styles = {
            "printed": dict(fill_color=FILL["printed"], line_color=None),
            "steel": dict(fill_color=FILL["steel"], line_color=None),
            "motor": dict(fill_color=FILL["motor"], line_color=None),
            "visible": dict(line_color=INK, line_weight=0.35),
            "hidden": dict(line_color=(150, 150, 150), line_weight=0.15, line_type=LineType.HIDDEN),
            "thin": dict(line_color=INK, line_weight=0.15),
            "ink": dict(fill_color=INK, line_color=None),
        }
        for name, style in styles.items():
            svg.add_layer(name, **style)
            if self.layers[name]:
                svg.add_shape(self.layers[name], layer=name)
        svg.write(str(path))
        # white page so the drawing reads on dark backgrounds too
        s = Path(path).read_text()
        s = re.sub(r"(<svg[^>]*>)", r'\1<rect x="-10000" y="-10000" width="20000" height="20000" fill="white"/>', s, count=1)
        Path(path).write_text(s)


def _section(parts):
    """Cut every part at y = 0, keeping the far half. Returns (cut parts, {name: [2D section faces]})."""
    keep = Box(1000, 500, 2000, align=(Align.CENTER, Align.MIN, Align.CENTER))
    cut = {k: v & keep for k, v in parts.items()}
    faces = {k: [Plane.XZ.to_local_coords(f) for f in v.faces()
                 if abs(f.center().Y) < 1e-4 and abs(abs(f.normal_at().Y) - 1) < 1e-6]
             for k, v in cut.items()}
    return cut, faces


def _project(shapes, origin=(0, -2000, 0), up=(0, 0, 1)):
    vis, hid = Compound(children=list(shapes)).project_to_viewport(origin, up, (0, 0, 0))
    return vis, hid


def _target(faces, side=1):
    """A point inside the largest section face of a part on one side, for its leader."""
    right = [f for f in faces if f.center().X * side > 0] or faces
    f = max(right, key=lambda f: f.area)
    c = f.center()
    if f.is_inside(c):
        return (c.X, c.Y)
    bb = f.bounding_box()
    best = None
    for i in range(1, 20):
        for j in range(1, 20):
            p = (bb.min.X + bb.size.X * i / 20, bb.min.Y + bb.size.Y * j / 20, 0)
            if f.is_inside(p):
                d = (p[0] - c.X) ** 2 + (p[1] - c.Y) ** 2
                if best is None or d < best[0]:
                    best = (d, p)
    return best[1][:2]


def _spread(ys, gap, ymax):
    """Move sorted (descending) label heights apart by at least `gap`, staying below ymax."""
    out = []
    for y in ys:
        y = min(y, ymax if not out else out[-1] - gap)
        out.append(y)
    return out


def gearbox_sheet(c, title, path, scale):
    L = cy.layout(c)
    m = c.motor
    pp, vp = cy.printed_parts(c), cy.vendor_parts(c)
    parts = {**pp, **vp}
    cut, faces = _section(parts)
    sh = Sheet()
    for k, fs in faces.items():
        layer = "printed" if k in pp else ("motor" if k == "motor" else "steel")
        sh.add(layer, *fs)
        sh.add("visible", *[e for f in fs for e in f.edges()])
    vis, _ = _project(cut.values())
    sh.add("visible", *vis)

    hr = c.housing_r
    top, bot = L.cap_top, L.motor_bot
    rows = [
        ("housing", "Housing", 1, "printed, PA12-GF"),
        ("cap", "Bearing cap", 1, "printed, PA12"),
        ("disc_a", "Cycloidal disc A", 1, "printed, PA12-GF"),
        ("disc_b", "Cycloidal disc B (rotated half a lobe)", 1, "printed, PA12-GF"),
        ("eccentric", "Eccentric", 1, "printed, PA12-GF"),
        ("output_flange", "Output flange", 1, "printed, PA12-GF"),
        ("ring_pins", f"Ring pin, dowel Ø{2 * c.pin_r:g} x {c.ring_pin_len:g}", c.n_pins, "steel"),
        ("output_pins", f"Output pin, dowel Ø{c.out_pin_d:g} x {c.out_pin_len:g}", c.n_out, "steel"),
        ("bearings", f"Bearings: {c.out_bearing.name} output, 2x {c.ecc_bearing.name} eccentric", 3, "steel"),
        ("motor", m.name, 1, "bought"),
    ]
    # balloons in a column on the right, ordered top to bottom like their targets
    # balloons alternate between columns left and right of the view, top to bottom
    order = sorted(range(len(rows)), key=lambda i: -_target(faces[rows[i][0]])[1])
    for side, xb in ((1, hr + 14), (-1, -hr - 26)):
        col = [i for n, i in enumerate(order) if (n % 2 == 0) == (side > 0)]
        tg = [_target(faces[rows[i][0]], side) for i in col]
        col, tg = zip(*sorted(zip(col, tg), key=lambda t: -t[1][1]))
        for y, i, t in zip(_spread([t[1] for t in tg], 8.0, top + 2), col, tg):
            sh.balloon(i + 1, t, (xb, y))

    # ExtensionLine offsets to the right of the border direction
    sh.dim((-hr, top), (hr, top), -7, f"Ø{2 * hr:.0f}")
    sh.dim((-hr, L.housing_bot), (-hr, top), -8, f"{top - L.housing_bot:.1f}")
    sh.dim((-hr, bot), (-hr, L.housing_bot), -8, f"{m.length:g}")
    sh.dim((-m.frame / 2, bot), (m.frame / 2, bot), 7, f"{m.frame:g}")

    # parts list and title under the view
    y = bot - 22
    x0 = -hr - 30
    sh.text(title, x0, y, 4.0)
    sh.text(f"Half section, scale {scale:g}:1, mm. Ratio {c.ratio:g}:1, eccentricity {c.ecc:g} mm, "
            f"{c.n_pins} ring pins on Ø{2 * c.pin_circle_r:g}.", x0, y - 6, 2.6)
    y -= 14
    for i, (_, name, qty, mat) in enumerate(rows):
        sh.text(str(i + 1), x0, y - i * 4.5, 2.6)
        sh.text(name, x0 + 6, y - i * 4.5, 2.6)
        sh.text(f"x{qty}", hr - 6, y - i * 4.5, 2.6)
        sh.text(mat, hr + 4, y - i * 4.5, 2.6)
    sh.write(path, scale)


def arm_sheet(path):
    from grip_cad.assembly import assembly_parts
    shapes = [s for k, (s, kind) in assembly_parts().items() if kind in ("shell", "tpu", "drive", "motor")
              and not k.endswith("_cap")]
    vis, _ = _project(shapes)
    sh = Sheet(Draft(font_size=7.0, decimal_precision=0, display_units=False, arrow_length=5.0,
                     line_width=0.4, extension_gap=3.0, pad_around_text=2.0))
    sh.add("visible", *vis)
    bb = Compound(children=vis).bounding_box()
    x0, x1 = bb.min.X, bb.max.X
    zt = G.z_flange
    ztip = bb.max.Y
    # joint heights on the left, link lengths on the right
    levels = [("J2", G.z_j2), ("J3", G.z_j3), ("J5", G.z_j5), ("flange", zt)]
    for i, (name, z) in enumerate(levels):
        sh.dim((x0, 0), (x0, z), -(14 + 16 * i), f"{z:g}")
        sh.line((x0 - 2, z), (x1 + 4, z))
        sh.text(name, x1 + 6, z + 5, 7.0)
    for (za, zb, label) in ((G.z_j2, G.z_j3, "upper arm"), (G.z_j3, G.z_j5, "forearm"), (G.z_j5, zt, "wrist")):
        sh.dim((x1 + 30, za), (x1 + 30, zb), 4, f"{zb - za:g}")
        sh.text(label, x1 + 44, (za + zb) / 2, 7.0)
    sh.dim((-G.base_r, 0), (G.base_r, 0), 14, f"Ø{2 * G.base_r:g}")
    sh.text("GRIP version B: home pose, side view, mm", x0 - 80, -50, 12.0)
    sh.text(f"Reach J2 to flange {G.reach:g} mm. Joint axes: J1, J4, J6 roll; J2, J3, J5 pitch. "
            f"Tool tip at {ztip:.0f} with fingers.", x0 - 80, -68, 7.0)
    sh.write(path, 0.5)


def gripper_sheet(path):
    from grip_cad import gripper as gr
    g = gr.gripper_local()
    shapes = [s for k, (s, kind) in g.items()]
    vis, _ = _project(shapes)
    sh = Sheet()
    sh.add("visible", *vis)
    bb = Compound(children=vis).bounding_box()
    zf0 = gr.Z_DECK + gr.JAW_TOP
    sh.dim((-gr.OPEN, bb.max.Y), (gr.OPEN, bb.max.Y), 8, f"{2 * gr.OPEN:g} shown, 0 to {2 * gr.STROKE:g}")
    sh.dim((bb.max.X, zf0), (bb.max.X, zf0 + gr.FINGER_H), 8, f"{gr.FINGER_H:g}")
    sh.dim((bb.min.X, 0), (bb.min.X, bb.max.Y), -8, f"{bb.max.Y:.0f}")
    sh.text("Gripper: front view, scale 1:1, mm", bb.min.X - 10, bb.min.Y - 14, 4.0)
    sh.text("STS3215 servo turns a 20-tooth module 1 pinion between two racks; jaws ride two Ø6 rods on LM6UU bearings.",
            bb.min.X - 10, bb.min.Y - 20, 2.6)
    sh.text("TPU fin-ray fingers wrap around the object instead of pinching it.", bb.min.X - 10, bb.min.Y - 25, 2.6)
    sh.write(path, 1.0)


def main(out):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    for c, slug, title, scale in ((J13_DRIVE, "j1_j3", "J1/J3 drive, 25:1 cycloidal", 2.0),
                                  (J2_DRIVE, "j2", "J2 drive, 30:1 cycloidal", 1.6),
                                  (WRIST_DRIVE, "wrist", "J4-J6 wrist drive, 15:1 cycloidal", 2.5)):
        gearbox_sheet(c, title, out / f"drive_{slug}.svg", scale)
        print("drive", slug)
    gripper_sheet(out / "gripper.svg")
    print("gripper")
    arm_sheet(out / "arm.svg")
    print("arm")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else Path(__file__).parent.parent / "docs" / "drawings")
