"""Soft, organic primitives for the link shells.

Every primitive takes a `grow` offset so the same call builds the outer skin (grow=0)
and, with grow=-wall, the hollow core that gets subtracted. That keeps shells hollow
for SLS without relying on fragile solid-offset operations.
"""
from math import sqrt

from build123d import (
    Axis, Cone, Cylinder, Ellipse, GeomType, Location, Part, Plane, Pos, Sketch, Vector, loft, make_face,
)


def _orient(axis):
    """Plane whose z_dir is `axis` (one of 'X', 'Y', 'Z', '-X', '-Y', '-Z')."""
    d = {"X": (1, 0, 0), "Y": (0, 1, 0), "Z": (0, 0, 1),
         "-X": (-1, 0, 0), "-Y": (0, -1, 0), "-Z": (0, 0, -1)}[axis]
    x = (1, 0, 0) if axis in ("Y", "-Y", "Z", "-Z") else (0, 1, 0)
    return Plane(origin=(0, 0, 0), x_dir=x, z_dir=d)


def puck(center, axis, r, h, soft, grow=0.0) -> Part:
    """Cylinder centred on `center` along `axis`, both rims rounded by `soft`."""
    r, h = r + grow, h + 2 * grow
    soft = max(0.4, min(soft + grow, r - 0.2, h / 2 - 0.2))
    body = Cylinder(r, h)
    body = body.fillet(soft, body.edges().filter_by(GeomType.CIRCLE))
    return Pos(*center) * (_orient(axis).location * body)


def pill(center, axis, r0, r1, h, soft, grow=0.0) -> Part:
    """Tapered puck: radius r0 at the -axis end, r1 at the +axis end."""
    r0, r1, h = r0 + grow, r1 + grow, h + 2 * grow
    soft = max(0.4, min(soft + grow, min(r0, r1) - 0.2, h / 2 - 0.2))
    body = Cylinder(r0, h) if abs(r0 - r1) < 1e-6 else Cone(r0, r1, h)
    body = body.fillet(soft, body.edges().filter_by(GeomType.CIRCLE))
    return Pos(*center) * (_orient(axis).location * body)


def bone(stations, grow=0.0, x_dir=(1, 0, 0)) -> Part:
    """Smooth loft through elliptical sections.

    stations: list of ((x, y, z), (a, b)) — section centre and semi-axes; `a` lies along
    x_dir, `b` across it. Section normals follow the path between neighbours, so the
    bone bends smoothly through lateral offsets.
    """
    pts = [Vector(*p) for p, _ in stations]
    sections = []
    for i, (p, (a, b)) in enumerate(stations):
        prev_pt = pts[max(i - 1, 0)]
        next_pt = pts[min(i + 1, len(pts) - 1)]
        tangent = (next_pt - prev_pt).normalized()
        xd = Vector(*x_dir)
        xd = (xd - tangent * xd.dot(tangent)).normalized()
        plane = Plane(origin=pts[i], x_dir=xd, z_dir=tangent)
        sections.append(plane * Ellipse(max(a + grow, 0.5), max(b + grow, 0.5)))
    return loft(sections)


def solids_only(shape) -> Part:
    """Drop stray faces/edges that booleans sometimes leave in a compound."""
    from build123d import Compound
    solids = shape.solids()
    if len(solids) == 1:
        return solids[0]
    return Compound(children=list(solids))


def union(*parts) -> Part:
    out = parts[0]
    for p in parts[1:]:
        out = out + p
    return out


def revolve_profile(points, axis, base, grow=0.0, end_tangent=None, start_tangent=None) -> Part:
    """Revolve a smooth (r, s) profile around `axis` through `base`.

    points: [(r, s), ...] with s measured along the axis; the region between the spline
    and the axis is revolved, so every pod/dome is one smooth surface of revolution.
    grow < 0 builds the hollow core: a true 2D offset of the profile, so the wall stays
    uniform even where the surface is nearly flat.
    """
    from build123d import Axis as A, Kind, Polyline, Rectangle, Spline, offset, revolve
    # a profile that closes on the axis leaves a pole in the surface, which meshes as a
    # non-manifold fan; end on a tiny flat disc instead (tangent-continuous, invisible)
    if points[-1][0] < 2.0:
        points = [*points[:-1], (2.0, points[-1][1])]
    tangents = None
    if start_tangent or end_tangent:
        tangents = [(*(start_tangent or (points[1][0] - points[0][0], points[1][1] - points[0][1])), 0),
                    (*(end_tangent or (points[-1][0] - points[-2][0], points[-1][1] - points[-2][1])), 0)]
    curve = Spline(*[(r, s, 0) for r, s in points], tangents=tangents)
    ring = [points[-1], (0, points[-1][1]), (0, points[0][1]), points[0]]
    dedup = [ring[0]]
    for q in ring[1:]:
        if abs(q[0] - dedup[-1][0]) + abs(q[1] - dedup[-1][1]) > 1e-6:
            dedup.append(q)
    face = make_face([curve, Polyline(*[(r, s, 0) for r, s in dedup])])
    if grow < 0:
        inner = offset(face, grow, kind=Kind.INTERSECTION)
        bb = inner.bounding_box()
        strip = Pos(0, (bb.min.Y + bb.max.Y) / 2, 0) * Rectangle(2 * (-grow + 0.5), bb.max.Y - bb.min.Y)
        face = (inner + strip) & face
        face = face.faces()[0] if len(face.faces()) == 1 else face
    elif grow > 0:
        face = offset(face, grow, kind=Kind.INTERSECTION)
    solid = revolve(face, A.Y)
    return Pos(*base) * (_orient_y_to(axis).location * solid)


def _orient_y_to(axis):
    """Plane mapping local +Y (the revolve axis) onto `axis`."""
    d = {"X": (1, 0, 0), "Y": (0, 1, 0), "Z": (0, 0, 1),
         "-X": (-1, 0, 0), "-Y": (0, -1, 0), "-Z": (0, 0, -1)}[axis]
    # choose x_dir perpendicular to d, z_dir = x × y
    from build123d import Vector as V
    y = V(*d)
    x = V(1, 0, 0) if abs(y.X) < 0.9 else V(0, 1, 0)
    x = (x - y * x.dot(y)).normalized()
    z = x.cross(y)
    return Plane(origin=(0, 0, 0), x_dir=x, z_dir=z)


def egg(base, axis, length, r_open, r_max, bulge_at=0.35, tip=0.55, grow=0.0) -> Part:
    """Pod open (flat) at `base`, swelling to r_max, closing in a soft dome `length` along `axis`."""
    pts = [(r_open, 0.0),
           (r_max, bulge_at * length),
           (r_max * 0.92, 0.72 * length),
           (r_max * tip, 0.93 * length),
           (0.0, length)]
    return revolve_profile(pts, axis, base, grow, start_tangent=(0.6, 1.0), end_tangent=(-1.0, 0.0))


def soft_fuse(parts, radius):
    """Fuse parts and try to round the new seam edges; fall back to a plain fuse."""
    fused = union(*parts)
    if radius <= 0:
        return fused
    import numpy as np
    old = []
    for p in parts:
        old += [e.center() for e in p.edges()]
    old = np.array([[v.X, v.Y, v.Z] for v in old])
    seams = [e for e in fused.edges()
             if np.min(np.linalg.norm(old - np.array([e.center().X, e.center().Y, e.center().Z]), axis=1)) > 0.05]
    if not seams:
        return fused
    for r in (radius, radius * 0.6, radius * 0.35):
        try:
            out = fused.fillet(r, seams)
            if out.is_valid:
                return out
        except Exception:
            pass
    return fused
