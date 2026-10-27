"""Single-stage, two-disc cycloidal reducer built from a Cyclo spec.

Local frame: drive axis = +Z, output flange face at z = 0, gearbox below it and the
motor below the gearbox. Printed parts: housing, cap, two discs, eccentric, output
flange. Bought parts: ring pins and output pins (steel dowels), bearings, motor.
"""
from dataclasses import dataclass
from math import cos, sin, pi, atan2, sqrt

from build123d import (
    Align, Axis, Box, Cylinder, Location, Part, Pos, Rot, Spline, Wire, make_face, extrude,
)

from .params import Cyclo

SCREW_D = 3.4        # M3 clearance
N_SCREWS = 6


def cycloid_points(n_pins, R, rp, e, samples_per_lobe=14):
    n = (n_pins - 1) * samples_per_lobe
    pts = []
    for i in range(n):
        t = 2 * pi * i / n
        psi = atan2(sin((1 - n_pins) * t), R / (e * n_pins) - cos((1 - n_pins) * t))
        x = R * cos(t) - rp * cos(t + psi) - e * cos(n_pins * t)
        y = -R * sin(t) + rp * sin(t + psi) + e * sin(n_pins * t)
        pts.append((x, y))
    return pts


def _cyl(r, z0, z1, x=0.0, y=0.0):
    return Pos(x, y, z0) * Cylinder(r, z1 - z0, align=(Align.CENTER, Align.CENTER, Align.MIN))


@dataclass
class CycloLayout:
    """Z levels of every layer in the stack (all <= 0)."""
    cap_top: float
    cap_bot: float
    plate_bot: float
    disc1: tuple
    disc2: tuple
    bore_bot: float
    housing_bot: float
    motor_bot: float


def layout(c: Cyclo) -> CycloLayout:
    ob = c.out_bearing
    cap_top = -1.0
    cap_bot = cap_top - (ob.w + 1.5)
    plate_bot = cap_bot - 4.0
    d1 = (plate_bot - 0.5 - c.disc_t, plate_bot - 0.5)
    d2 = (d1[0] - 0.5 - c.disc_t, d1[0] - 0.5)
    bore_bot = d2[0] - 1.0
    housing_bot = bore_bot - 5.0
    return CycloLayout(cap_top, cap_bot, plate_bot, d1, d2, bore_bot, housing_bot,
                       housing_bot - c.motor.length)


def screw_circle_r(c: Cyclo) -> float:
    return c.pin_circle_r + c.pin_r + c.wall / 2 + 0.5


def screw_angles(c: Cyclo):
    # sit midway between ring pins so the holes never cut into a pin
    step = 360 / c.n_pins
    return [((360 / N_SCREWS) * i // step) * step + step / 2 for i in range(N_SCREWS)]


def disc(c: Cyclo, second=False) -> Part:
    L = layout(c)
    z0, z1 = L.disc2 if second else L.disc1
    pts = cycloid_points(c.n_pins, c.pin_circle_r, c.pin_r + c.clearance, c.ecc)
    profile = make_face(Spline(*pts, periodic=True))
    body = extrude(profile, z1 - z0)
    if second:
        # half a lobe pitch out of phase so it meshes when offset by -e
        body = body.rotate(Axis.Z, 180 / (c.n_pins - 1))
    body -= Cylinder(c.ecc_bearing.od / 2, 50)                      # eccentric bearing seat
    hole_r = c.out_pin_d / 2 + c.ecc + c.clearance
    for i in range(c.n_out):
        a = 2 * pi * i / c.n_out
        body -= Pos(c.out_circle_r * cos(a), c.out_circle_r * sin(a), 0) * Cylinder(hole_r, 50)
    if second:
        return Pos(-c.ecc, 0, z0) * body
    return Pos(c.ecc, 0, z0) * body


def eccentric(c: Cyclo) -> Part:
    L = layout(c)
    r = c.ecc_bearing.id / 2
    body = _cyl(r, L.disc1[0], L.disc1[1], x=c.ecc) + _cyl(r, L.disc2[0], L.disc2[1], x=-c.ecc)
    body += _cyl(r - 0.5, L.bore_bot + 0.3, L.disc2[0])            # spacer collar
    body -= _cyl(c.motor.shaft_d / 2 + 0.1, L.bore_bot, L.disc1[1] + 1)
    # D-flat key on the shaft
    flat = Pos(c.motor.shaft_d / 2 - 0.5 + 2, 0, (L.bore_bot + L.disc1[1]) / 2) * Box(4, 10, L.disc1[1] - L.bore_bot)
    body += flat & _cyl(c.motor.shaft_d / 2 + 0.2, L.bore_bot, L.disc1[1])
    return body


def output_flange(c: Cyclo) -> Part:
    L = layout(c)
    ob = c.out_bearing
    plate_r = c.out_circle_r + c.out_pin_d / 2 + 3.0
    body = _cyl(ob.id / 2, L.cap_bot, 0.0) + _cyl(plate_r, L.plate_bot, L.cap_bot)
    body -= _cyl(c.motor.shaft_d / 2 + 2.0, L.plate_bot, 0.0)       # clearance for shaft tip
    for i in range(c.n_out):                                          # blind pin sockets
        a = 2 * pi * i / c.n_out
        body -= _cyl(c.out_pin_d / 2 - 0.05, L.plate_bot, L.plate_bot + 3.5,
                     c.out_circle_r * cos(a), c.out_circle_r * sin(a))
    # child-link bolt pattern: M3 heat-set inserts in the hub face
    br = (ob.id / 2 + c.motor.shaft_d / 2 + 2.0) / 2
    for i in range(4):
        a = 2 * pi * i / 4 + pi / 4
        body -= _cyl(2.0, -8.0, 0.0, br * cos(a), br * sin(a))
    return body


def housing(c: Cyclo) -> Part:
    L = layout(c)
    m = c.motor
    body = _cyl(c.housing_r, L.housing_bot, L.cap_bot)
    body -= _cyl(c.pin_circle_r, L.bore_bot, L.cap_bot)
    for i in range(c.n_pins):
        a = 2 * pi * i / c.n_pins
        body -= _cyl(c.pin_r - 0.03, L.bore_bot, L.cap_bot, c.pin_circle_r * cos(a), c.pin_circle_r * sin(a))
    body -= _cyl(m.pilot_d / 2 + 0.2, L.housing_bot, L.bore_bot)
    s = m.bolt_spacing / 2
    for x, y in [(s, s), (-s, s), (s, -s), (-s, -s)]:
        body -= _cyl(m.bolt_d / 2, L.housing_bot, L.bore_bot, x, y)
    for a in screw_angles(c):
        r = screw_circle_r(c)
        body -= _cyl(SCREW_D / 2, L.housing_bot, L.cap_bot, r * cos(a * pi / 180), r * sin(a * pi / 180))
    return body


def cap(c: Cyclo) -> Part:
    L = layout(c)
    ob = c.out_bearing
    body = _cyl(c.housing_r, L.cap_bot, L.cap_top)
    body -= _cyl(ob.od / 2 - 0.05, L.cap_bot, L.cap_top - 1.5)       # press-fit seat
    body -= _cyl(ob.od / 2 - 2.0, L.cap_bot, L.cap_top)              # retaining lip
    for a in screw_angles(c):
        r = screw_circle_r(c)
        body -= _cyl(SCREW_D / 2, L.cap_bot, L.cap_top, r * cos(a * pi / 180), r * sin(a * pi / 180))
        body -= _cyl(3.1, L.cap_top - 3.0, L.cap_top, r * cos(a * pi / 180), r * sin(a * pi / 180))
    return body


# ---- bought parts, for the assembly only -------------------------------------------------

def ring_pins(c: Cyclo) -> Part:
    L = layout(c)
    pins = None
    for i in range(c.n_pins):
        a = 2 * pi * i / c.n_pins
        p = _cyl(c.pin_r, L.bore_bot, L.cap_bot, c.pin_circle_r * cos(a), c.pin_circle_r * sin(a))
        pins = p if pins is None else pins + p
    return pins


def output_pins(c: Cyclo) -> Part:
    L = layout(c)
    pins = None
    for i in range(c.n_out):
        a = 2 * pi * i / c.n_out
        p = _cyl(c.out_pin_d / 2, L.disc2[0], L.plate_bot + 3.5, c.out_circle_r * cos(a), c.out_circle_r * sin(a))
        pins = p if pins is None else pins + p
    return pins


def bearing_ring(b, z0, x=0.0):
    return _cyl(b.od / 2, z0, z0 + b.w, x) - _cyl(b.id / 2, z0, z0 + b.w, x)


def bearings(c: Cyclo) -> Part:
    L = layout(c)
    out = bearing_ring(c.out_bearing, L.cap_bot)
    eb = c.ecc_bearing
    e1 = bearing_ring(eb, L.disc1[0] + (c.disc_t - eb.w) / 2, c.ecc)
    e2 = bearing_ring(eb, L.disc2[0] + (c.disc_t - eb.w) / 2, -c.ecc)
    return out + e1 + e2


def motor(c: Cyclo) -> Part:
    L = layout(c)
    m = c.motor
    body = Pos(0, 0, L.housing_bot - m.length) * Box(m.frame, m.frame, m.length, align=(Align.CENTER, Align.CENTER, Align.MIN))
    body = body.fillet(4.0, body.edges().filter_by(Axis.Z))
    body += _cyl(m.pilot_d / 2, L.housing_bot, L.housing_bot + m.pilot_h)
    body += _cyl(m.shaft_d / 2, L.housing_bot, L.housing_bot + m.shaft_len)
    return body


def printed_parts(c: Cyclo) -> dict:
    return {
        "housing": housing(c),
        "cap": cap(c),
        "disc_a": disc(c),
        "disc_b": disc(c, second=True),
        "eccentric": eccentric(c),
        "output_flange": output_flange(c),
    }


def vendor_parts(c: Cyclo) -> dict:
    return {
        "ring_pins": ring_pins(c),
        "output_pins": output_pins(c),
        "bearings": bearings(c),
        "motor": motor(c),
    }
