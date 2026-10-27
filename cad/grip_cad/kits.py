"""Service kits: small parts joined into one printable body.

Print bureaus charge a minimum (~$11) per part however small it is, and price a
part by its bounding box, so the 30-odd gearbox and gripper parts are stacked into
a few compact towers, each quoted as one part. Neighbours are joined by 1.6 mm
pillars on flat faces near the outer edge; snip them and file the nubs flush.

Printing in-house? Use the individual part files instead.
"""
from build123d import Align, Cylinder, Part, Pos, Rot

from .shapes import solids_only

STACK_GAP = 2.0
PILLAR_R = 0.8


def _mesh(part: Part):
    """Tessellate once for fast vectorised point-in-solid tests."""
    import numpy as np
    import trimesh
    verts, tris = part.tessellate(0.1, 0.3)
    return trimesh.Trimesh(np.array([[v.X, v.Y, v.Z] for v in verts]), np.array(tris))


def _pillar_sites(lower: Part, upper: Part, n=3):
    import numpy as np
    zl = lower.bounding_box().max.Z - 0.3
    zu = upper.bounding_box().min.Z + 0.3
    bb = upper.bounding_box()
    cx, cy = (bb.min.X + bb.max.X) / 2, (bb.min.Y + bb.max.Y) / 2
    rmax = max(bb.size.X, bb.size.Y) / 2
    r = np.repeat(np.linspace(rmax / 24, rmax, 24), 48)
    a = np.tile(np.linspace(0, 2 * np.pi, 48, endpoint=False), 24)
    x, y = cx + r * np.cos(a), cy + r * np.sin(a)
    ok = np.ones_like(x, dtype=bool)
    ml, mu = _mesh(lower), _mesh(upper)
    for dx, dy in ((0, 0), (PILLAR_R, 0), (-PILLAR_R, 0), (0, PILLAR_R), (0, -PILLAR_R)):
        ok &= ml.contains(np.c_[x + dx, y + dy, np.full_like(x, zl)])
        ok &= mu.contains(np.c_[x + dx, y + dy, np.full_like(x, zu)])
    idx = np.flatnonzero(ok)
    if idx.size == 0:
        return []
    idx = idx[np.argsort(-r[idx])]
    chosen = [idx[0]]
    gap = lambda i, j: abs((a[i] - a[j] + np.pi) % (2 * np.pi) - np.pi)
    while len(chosen) < n:
        best = max(idx, key=lambda i: min(gap(i, c) for c in chosen))
        if min(gap(best, c) for c in chosen) < 0.5:
            break
        chosen.append(best)
    return [(float(x[i]), float(y[i])) for i in chosen]


def _centered(p: Part) -> Part:
    bb = p.bounding_box()
    return Pos(-(bb.min.X + bb.max.X) / 2, -(bb.min.Y + bb.max.Y) / 2, -bb.min.Z) * p


def make_stack(parts) -> Part:
    """Stack parts into one tower joined by snap-off pillars. Largest footprint at the bottom."""
    parts = sorted(parts, key=lambda p: -(p.bounding_box().size.X * p.bounding_box().size.Y))
    placed = []
    for p in parts:
        if not placed:
            placed.append(_centered(p))
            continue
        below = placed[-1]
        z = below.bounding_box().max.Z + STACK_GAP
        # try the part as modelled, then upside down, sliding it sideways if needed
        for cand in (Pos(dx, 0, z) * _centered(q) for q in (p, Rot(180, 0, 0) * p)
                     for dx in (0, 8, 14, 20, -14, 26)):
            sites = _pillar_sites(below, cand)
            if len(sites) >= 2:
                break
        else:
            raise ValueError("no pillar sites between stacked parts")
        cand.pillars = sites
        placed.append(cand)
    body = placed[0]
    for lower, upper in zip(placed, placed[1:]):
        z0 = lower.bounding_box().max.Z - 0.3
        z1 = upper.bounding_box().min.Z + 0.3
        body = body + upper
        for x, y in upper.pillars:
            body = body + Pos(x, y, z0) * Cylinder(PILLAR_R, z1 - z0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    return solids_only(body)
