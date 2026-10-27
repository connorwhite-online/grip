"""Parallel-jaw gripper: Feetech STS3215 driving a pinion between two racks, jaws on two
Ø6 steel rods (LM6UU bearings), TPU fin-ray fingers.

Local frame: origin at the J6 output face centre, Z along the tool axis, jaws open
along X. `gripper_parts()` returns the parts placed on the arm's home-pose flange.
"""
from math import acos, atan2, cos, pi, radians, sin, sqrt, tan

from build123d import (
    Align, Axis, Box, Cylinder, Location, Part, Polygon, Pos, Rot, extrude, make_face, Polyline,
)

from .params import GEOM, PROCESS, WRIST_DRIVE
from .shapes import revolve_profile, soft_fuse, solids_only, union

# Feetech STS3215 (VERIFY against the servo you buy)
SERVO = dict(l=45.2, w=24.7, h=35.0, horn_dx=11.4, horn_h=4.0, ear_h=27.0)
MODULE = 1.0
PINION_T = 20
PITCH_R = MODULE * PINION_T / 2
STROKE = 40.0          # per jaw: 0..80 mm opening
FINGER_H = 72.0
FINGER_W = 22.0
FINGER_BASE = 30.0

Z_DECK = 46.0          # top of the palm
Z_RACK = Z_DECK + 3.0  # rack pitch-plane bottom
OPEN = 30.0            # jaw position used for the preview (half-opening, mm)

# guide rods and jaws
ROD_D = 6.0            # hardened Ø6 linear shaft, 150 mm
ROD_HALF = 75.0        # rods run x = -75..75, ends flush with the frame posts
ROD_Y = 24.0
ROD_Z = Z_DECK + 19.5
LM6UU = dict(od=12.0, l=19.0)
JAW_L = 24.0           # jaw length along X; jaws span OPEN+0.5 .. OPEN+24.5
JAW_Y = 32.5           # jaw and frame half-width
JAW_TOP = 28.0         # jaw top (finger seat) above the deck
POST_X0 = 69.0         # frame end posts x = 69..75; a fully open jaw stops 4.5 mm short
FRAME_Y0 = 17.0        # frame rails |y| >= 17, clear of the racks
FRAME_BOLTS = [(x, y) for x in (-12.0, 12.0) for y in (-22.0, 22.0)]


def _cyl(r, z0, z1, x=0.0, y=0.0):
    return Pos(x, y, z0) * Cylinder(r, z1 - z0, align=(Align.CENTER, Align.CENTER, Align.MIN))


def _box(x0, x1, y0, y1, z0, z1):
    return Pos(x0, y0, z0) * Box(x1 - x0, y1 - y0, z1 - z0, align=(Align.MIN, Align.MIN, Align.MIN))


def _soften(body, r_vert, r_top, z_min):
    """Round the vertical edges, then every edge above z_min. Two passes: one pass over
    all edges leaves corner blends that tessellate into a leaky STL."""
    body = body.fillet(r_vert, body.edges().filter_by(Axis.Z))
    top = body.edges().filter_by(lambda e: e.center().Z > z_min and abs(e.tangent_at(0.5).Z) < 0.99)
    for r in (r_top, 0.75 * r_top, 0.5 * r_top):
        try:
            return body.fillet(r, top)
        except ValueError:
            continue
    return body


def involute_gear(teeth, module, thickness, pressure=20.0, clearance=0.1):
    """Spur gear outline built from involute flanks, extruded along Z."""
    rp = module * teeth / 2
    rb = rp * cos(radians(pressure))
    ra = rp + module
    rf = rp - 1.25 * module
    inv = lambda a: tan(a) - a
    half = pi / (2 * teeth) - clearance / (2 * rp) + inv(radians(pressure))
    pts = []
    for k in range(teeth):
        c = 2 * pi * k / teeth
        flank = []
        for i in range(9):
            r = max(rb, rf) + (ra - max(rb, rf)) * i / 8
            a = acos(min(1.0, rb / r))
            flank.append((r, half - inv(a)))
        right = [(r * cos(c - t), r * sin(c - t)) for r, t in flank]
        left = [(r * cos(c + t), r * sin(c + t)) for r, t in reversed(flank)]
        root_a = c - half - 0.5 * (2 * pi / teeth - 2 * half)
        pts += [(rf * cos(c - half), rf * sin(c - half))] + right + left + [(rf * cos(c + half), rf * sin(c + half))]
    return extrude(make_face(Polygon(*pts, align=None)), thickness)


def rack(length, module, width, base_h, pressure=20.0):
    """Rack along X, teeth pointing -Y, extruded `width` along Z."""
    p = pi * module
    h_a, h_f = module, 1.25 * module
    t = tan(radians(pressure))
    pts = [(0, base_h + h_f)]
    x = 0.0
    while x + p <= length + 1e-6:
        # one pitch: bottom land, rising flank, top land, falling flank (pitch line at y = h_f)
        pts += [(x + p / 4 - h_f * t, h_f), (x + p / 4 - h_f * t, 0)]  # placeholder replaced below
        x += p
    # simpler explicit trapezoid teeth
    pts = [(0, base_h + h_a + h_f)]
    x = 0.0
    tooth_top = p / 2 - 2 * h_a * t
    while x + p <= length + 1e-6:
        x0 = x + (p - (p / 2 + 2 * h_f * t)) / 2
        pts += [(x0, h_a + h_f), (x0 + h_f * t + h_a * t, 0), (x0 + h_f * t + h_a * t + tooth_top, 0),
                (x0 + 2 * (h_f + h_a) * t + tooth_top, h_a + h_f)]
        x += p
    pts += [(length, h_a + h_f), (length, base_h + h_a + h_f)]
    return extrude(make_face(Polygon(*pts, align=None)), width)


def fin_ray(height=FINGER_H, base=FINGER_BASE, width=FINGER_W, skin=1.8, ribs=6):
    """Fin-ray finger in the XZ plane (contact face along x=0, back flank slanting),
    extruded along Y. Print in TPU 90A-95A."""
    tri = [(0, 0), (base, 0), (3.0, height), (0, height)]
    outer = make_face(Polygon(*tri, align=None))
    inner_pts = [(skin, 2.5), (base - skin * 1.6, 2.5), (3.0 - 0.4, height - 6), (skin, height - 6)]
    inner = make_face(Polygon(*inner_pts, align=None))
    profile = outer - inner
    for i in range(1, ribs + 1):
        z = 2.5 + i * (height - 10) / (ribs + 1)
        # ribs tilt down toward the contact face, which is what makes fin-ray wrap
        rib = Pos(base / 2, z, 0) * Rot(0, 0, -12) * make_face(Polygon((-base, -0.8), (base, -0.8), (base, 0.8), (-base, 0.8), align=None))
        profile = profile + (rib & outer)
    solid = extrude(profile, width)               # in XY, extruded +Z
    # map local (x, y=height, z=width) -> gripper (x, z, y)
    return Pos(0, width / 2, 0) * Rot(90, 0, 0) * solid


def palm():
    """Organic palm shell; holds the servo and carries the rod frame on its deck."""
    shell = lambda g: revolve_profile([(31, 0), (34, 10), (37, 28), (36, Z_DECK - 6), (30, Z_DECK)],
                                      "Z", (0, 0, 0), g, start_tangent=(0.4, 1), end_tangent=(-0.6, 0.4))
    outer = shell(0)
    core = shell(-2.6)
    s = SERVO
    servo_pocket = _box(-s["l"] / 2 - s["horn_dx"] - 0.4, s["l"] / 2 - s["horn_dx"] + 0.4,
                        -s["w"] / 2 - 0.4, s["w"] / 2 + 0.4, Z_DECK - s["h"] - 6, Z_DECK + 1)
    ears = _box(-s["l"] / 2 - s["horn_dx"] - 4, s["l"] / 2 - s["horn_dx"] + 4, -s["w"] / 2 - 3, s["w"] / 2 + 3,
                Z_DECK - 8, Z_DECK)
    pads = union(*[_cyl(4.5, Z_DECK - 8, Z_DECK, x, y) for x, y in FRAME_BOLTS])
    body = solids_only(outer - solids_only(core - ears - pads))
    body = solids_only(body - servo_pocket)
    # J6 flange bolts (4x M3 on the flange insert circle)
    ob = WRIST_DRIVE.out_bearing
    br = (ob.id / 2 + WRIST_DRIVE.motor.shaft_d / 2 + 2.0) / 2
    for i in range(4):
        a = 2 * pi * i / 4 + pi / 4
        body = body - _cyl(1.7, -1, 8, br * cos(a), br * sin(a)) - _cyl(3.0, 3.0, 12, br * cos(a), br * sin(a))
    # M3 heat-set inserts for the rod frame
    for x, y in FRAME_BOLTS:
        body = body - _cyl(2.0, Z_DECK - 6, Z_DECK + 1, x, y)
    return Pos(0, 0, 0.5) * body


def frame():
    """Rod frame: two rails on the palm deck joined by end posts that hold the guide rods.

    The racks run in the slot between the rails; the jaws ride above them on LM6UU bearings.
    """
    zb = Z_DECK + 0.5
    rails = [_box(-ROD_HALF, ROD_HALF, y0, y1, zb, Z_DECK + 9.5) for y0, y1 in ((FRAME_Y0, JAW_Y), (-JAW_Y, -FRAME_Y0))]
    posts = [_box(x0, x1, -JAW_Y, JAW_Y, zb, Z_DECK + JAW_TOP) for x0, x1 in ((POST_X0, ROD_HALF), (-ROD_HALF, -POST_X0))]
    body = union(*rails, *posts)
    body = _soften(body, 2.9, 2.5, zb + 0.1)
    for y in (-ROD_Y, ROD_Y):
        rod = Pos(0, y, ROD_Z) * Rot(0, 90, 0) * Cylinder(ROD_D / 2 + PROCESS.pin_hole, 2 * ROD_HALF + 2)
        body = body - rod
        for x in (-(ROD_HALF + POST_X0) / 2, (ROD_HALF + POST_X0) / 2):
            body = body - _cyl(1.25, ROD_Z, Z_DECK + JAW_TOP + 1, x, y)      # M3 set screw, self-tapping
    for x, y in FRAME_BOLTS:
        body = body - _cyl(1.7, zb - 1, Z_DECK + 10, x, y) - _cyl(3.0, Z_DECK + 6.5, Z_DECK + 10, x, y)
    return body


def jaw(side):
    """Jaw block: two LM6UU bearings on the guide rods, the rack below, a finger on top."""
    sgn = 1 if side > 0 else -1
    x0, x1 = sorted((sgn * (OPEN + 0.5), sgn * (OPEN + JAW_L + 0.5)))
    body = _box(x0, x1, -JAW_Y, JAW_Y, Z_DECK + 11, Z_DECK + JAW_TOP)
    body = _soften(body, 4.0, 3.0, Z_DECK + 12)
    # pocket underneath for the finger screw heads; the wall on the rack side carries the rack
    y_lo, y_hi = (-15.0, 11.6) if sgn > 0 else (-11.6, 15.0)
    body = body - _box(x0 - 1, x1 + 1, y_lo, y_hi, Z_DECK + 10, Z_DECK + JAW_TOP - 5)
    tab = _box(x0, x1, 11.6, 15.0, Z_DECK + 9.5, Z_DECK + 11.5) if sgn > 0 else \
        _box(x0, x1, -15.0, -11.6, Z_DECK + 9.5, Z_DECK + 11.5)
    body = body + tab
    for y in (-ROD_Y, ROD_Y):
        body = body - Pos((x0 + x1) / 2, y, ROD_Z) * Rot(0, 90, 0) * Cylinder(LM6UU["od"] / 2 + PROCESS.bearing_seat, 60)
    return body


def gripper_local(opening=None):
    """Parts in the gripper frame; `opening` is the half-opening (0..STROKE), default OPEN."""
    global OPEN
    keep, OPEN = OPEN, OPEN if opening is None else opening
    try:
        return _gripper_local()
    finally:
        OPEN = keep


def _gripper_local():
    s = SERVO
    parts = {}
    parts["palm"] = (palm(), "shell")
    parts["frame"] = (frame(), "shell")
    parts["servo"] = (_box(-s["l"] / 2 - s["horn_dx"], s["l"] / 2 - s["horn_dx"], -s["w"] / 2, s["w"] / 2,
                           Z_DECK - s["h"] - 5, Z_DECK - 5) + _cyl(3, Z_DECK - 5, Z_DECK + 2), "motor")
    # the pinion turns with the jaws: the racks are phased to it at OPEN = 30
    turn = 180 / PINION_T - (OPEN - 30.0) / PITCH_R * 180 / pi
    parts["pinion"] = (Pos(0, 0, Z_DECK + 2) * Rot(0, 0, turn) * involute_gear(PINION_T, MODULE, 8.0) - _cyl(2.6, Z_DECK, Z_DECK + 12), "drive")
    # racks: one each side of the pinion, moving opposite ways
    rk = rack(56, MODULE, 8.0, 4.0)
    # rack teeth reach the pinion over the whole 0..STROKE travel
    parts["rack_a"] = (Pos(OPEN - STROKE - 4, PITCH_R - MODULE, Z_DECK + 2) * rk, "drive")
    parts["rack_b"] = (Pos(-(OPEN - STROKE - 4), -(PITCH_R - MODULE), Z_DECK + 2) * Rot(0, 0, 180) * rk, "drive")
    for y in (-ROD_Y, ROD_Y):
        parts[f"rod_{'a' if y > 0 else 'b'}"] = (Pos(0, y, ROD_Z) * Rot(0, 90, 0) * Cylinder(ROD_D / 2, 2 * ROD_HALF), "vendor")
    for side, name in ((1, "a"), (-1, "b")):
        rk_part = parts.pop(f"rack_{name}")[0]
        j = jaw(side) + rk_part                                       # rack printed as part of the jaw
        xc = side * (OPEN + 0.5 + JAW_L / 2)
        lm = None
        for y in (-ROD_Y, ROD_Y):
            b = Pos(xc, y, ROD_Z) * Rot(0, 90, 0) * (Cylinder(LM6UU["od"] / 2, LM6UU["l"]) - Cylinder(ROD_D / 2, LM6UU["l"]))
            lm = b if lm is None else lm + b
        parts[f"bearings_{name}"] = (lm, "vendor")
        f = fin_ray()          # contact face on x = 0, toward the centre
        f = Pos(OPEN, 0, Z_DECK + JAW_TOP) * f if side > 0 else Pos(-OPEN, 0, Z_DECK + JAW_TOP) * Rot(0, 0, 180) * f
        # finger: 2x M3 up through the jaw top and the finger's bottom skin, nut in the first cell
        for d in (6.0, 16.0):
            hole = _cyl(1.65, Z_DECK + JAW_TOP - 6, Z_DECK + JAW_TOP + 4, side * (OPEN + d), 0.0)
            j, f = j - hole, f - hole
        parts[f"jaw_{name}"] = (j, "drive")
        parts[f"finger_{name}"] = (f, "tpu")
    return parts


def gripper_parts():
    loc = Pos(0, GEOM.y_forearm, GEOM.z_flange)
    return {k: (loc * shape, kind) for k, (shape, kind) in gripper_local().items()}
