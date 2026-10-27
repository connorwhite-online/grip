"""Sprue kits: small parts joined into one printable body.

Print bureaus charge a minimum (~$11) per part however small it is, so the 30-odd
gearbox and gripper parts are joined by thin snap-off tabs into a few kits, each
quoted as one part. Snip the 1.8 mm tabs and file the nubs flush. Tabs land on
top faces near the outer edge, away from bores, lobes and bearing seats.

Printing in-house? Use the individual part files instead.
"""
from build123d import Align, Box, Cylinder, Part, Pos, Rot

from .shapes import solids_only

GAP = 5.0          # between neighbouring parts
TAB_R = 0.9
BAR = 3.0          # sprue bar cross-section
BAR_LIFT = 2.5     # bar clearance above the tallest part


def _lay_flat(shape: Part) -> Part:
    bb = shape.bounding_box()
    return Pos(-bb.min.X, -(bb.min.Y + bb.max.Y) / 2, -bb.min.Z) * shape


def _tab_point(shape: Part):
    """A point on the part's top face, 1.5 mm in from its outer edge, near y = 0."""
    top_z = shape.bounding_box().max.Z
    faces = [f for f in shape.faces() if abs(f.center().Z - top_z) < 1e-3 and f.normal_at().Z > 0.99]
    best = None
    for f in faces:
        outer = f.outer_wire()
        for i in range(400):
            p = outer.position_at(i / 400)
            c = f.center()
            v = (c - p)
            if v.length < 1e-6:
                continue
            q = p + v.normalized() * 1.5
            if not f.is_inside(q):
                continue
            score = abs(q.Y) - 0.01 * q.X
            if best is None or score < best[0]:
                best = (score, q)
    if best is None:
        raise ValueError("no flat top face to attach a sprue tab")
    return best[1]


MAX_ROW = 260.0    # keep kits inside a typical bureau SLS bed


def _row(parts):
    """Lay parts along X and tie them to one bar; returns (body, bar_z, x_start)."""
    placed, x = [], 0.0
    for p in parts:
        p = Pos(x, 0, 0) * _lay_flat(p)
        x = p.bounding_box().max.X + GAP
        placed.append(p)
    bar_z = max(p.bounding_box().max.Z for p in placed) + BAR_LIFT
    pts = [_tab_point(p) for p in placed]
    body = placed[0]
    for p in placed[1:]:
        body = body + p
    x0, x1 = min(q.X for q in pts), max(q.X for q in pts)
    body = body + Pos(x0 - BAR / 2, -BAR / 2, bar_z) * Box(x1 - x0 + BAR, BAR, BAR, align=(Align.MIN, Align.MIN, Align.MIN))
    for q in pts:
        # tab from the part's top face up to the bar, with a short leg along Y to the bar
        body = body + Pos(q.X, q.Y, q.Z - 0.2) * Cylinder(TAB_R, bar_z - q.Z + 0.2 + BAR / 2,
                                                          align=(Align.CENTER, Align.CENTER, Align.MIN))
        if abs(q.Y) > BAR / 2 - TAB_R:
            y0, y1 = sorted((q.Y, 0.0))
            body = body + Pos(q.X - TAB_R, y0 - TAB_R, bar_z) * Box(2 * TAB_R, y1 - y0 + 2 * TAB_R, BAR,
                                                                     align=(Align.MIN, Align.MIN, Align.MIN))
    return body, bar_z, x0


def make_kit(parts) -> Part:
    """Rows of parts, each on its own bar, rows joined by a spine bar at the left end."""
    rows, cur, length = [], [], 0.0
    for p in parts:
        w = p.bounding_box().size.X
        if cur and length + w > MAX_ROW:
            rows.append(cur)
            cur, length = [], 0.0
        cur.append(p)
        length += w + GAP
    rows.append(cur)
    body, y, anchors = None, 0.0, []
    for r in rows:
        width = max(p.bounding_box().size.Y for p in r)
        row, bar_z, x0 = _row(r)
        row = Pos(0, y + width / 2, 0) * row
        anchors.append((x0, y + width / 2, bar_z))
        body = row if body is None else body + row
        y += width + GAP
    if len(anchors) > 1:
        top = max(a[2] for a in anchors)
        xs = min(a[0] for a in anchors)
        ys = [a[1] for a in anchors]
        body = body + Pos(xs - BAR / 2, min(ys) - BAR / 2, top) * Box(BAR, max(ys) - min(ys) + BAR, BAR,
                                                                     align=(Align.MIN, Align.MIN, Align.MIN))
        for x0, yc, bz in anchors:
            # run each row bar back to the spine, then up to it
            body = body + Pos(xs - BAR / 2, yc - BAR / 2, bz) * Box(x0 - xs + BAR, BAR, BAR,
                                                                    align=(Align.MIN, Align.MIN, Align.MIN))
            body = body + Pos(xs - BAR / 2, yc - BAR / 2, bz) * Box(BAR, BAR, top - bz + BAR,
                                                                    align=(Align.MIN, Align.MIN, Align.MIN))
    return solids_only(body)
