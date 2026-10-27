"""Fit-test coupon for FDM: print it before the real parts and tune hole compensation.

Each row is one fit the arm uses, at -0.1 / nominal / +0.1 mm on diameter (the middle
hole is what the parts are drawn at, and has a notch beside it). Push the real dowel,
rod or bearing into each: if only the +0.1 hole takes it, raise the slicer's hole
(XY) compensation by about 0.05 mm, and so on.
"""
from build123d import Align, Box, Cylinder, Pos

from .params import B6801, PROCESS

T = 5.0
ROWS = [  # (label, nominal diameter, radial fit offset)
    ("dowel 3", 3.0, PROCESS.pin_hole),
    ("dowel 4", 4.0, PROCESS.pin_hole),
    ("dowel 5", 5.0, PROCESS.pin_hole),
    ("rod 6", 6.0, PROCESS.pin_hole),
    ("LM6UU 12", 12.0, PROCESS.bearing_seat),
    ("6801 21", B6801.od, PROCESS.bearing_seat),
]
STEPS = (-0.1, 0.0, 0.1)


def coupon():
    gap = 3.0
    y = 0.0
    strips, holes = [], []
    for label, d, fit in ROWS:
        dd = d + 2 * fit
        pitch = d + 0.1 + gap
        strips.append(Pos(0, y, 0) * Box(3 * pitch + gap, pitch + gap, T, align=(Align.MIN, Align.MIN, Align.MIN)))
        for k, step in enumerate(STEPS):
            cx, cy = gap + pitch * k + (d + 0.1) / 2, y + gap + (d + 0.1) / 2
            holes.append(Pos(cx, cy, -1) * Cylinder((dd + step) / 2, T + 2, align=(Align.CENTER, Align.CENTER, Align.MIN)))
            if step == 0.0:   # notch on the strip's top face marks the nominal column
                holes.append(Pos(cx, y + 1.0, T) * Box(1.6, 1.6, 1.6))
        y += pitch
    plate = strips[0]
    for s_ in strips[1:]:
        plate = plate + s_
    for h in holes:
        plate = plate - h
    return plate
