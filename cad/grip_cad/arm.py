"""Link shells and joint placement for the arm in its upright home pose.

Frame: Z up, base on the table at z = 0, the arm's working side is +X. Pitch joints
(J2, J3, J5) turn about Y; J1, J4, J6 turn about Z. Each joint is a cycloidal drive
(see cycloidal.py) whose stationary housing sits in a socket in the parent link and whose
output flange bolts to the child link's hub.
"""
from math import cos, sin, pi, sqrt

from build123d import Align, Box, Cylinder, Location, Part, Pos, Rot

from . import cycloidal as cy
from .params import GEOM, J13_DRIVE, J2_DRIVE, WRIST_DRIVE, Cyclo
from .shapes import bone, pill, puck, revolve_profile, soft_fuse, solids_only, union

G = GEOM
W = G.shell_wall
FIT = 0.3            # radial clearance between a housing and its socket
INSERT_D = 4.0       # M3 heat-set insert hole
CABLE_R = 6.0

UP = Rot(0, 0, 0)
TO_NEG_Y = Rot(90, 0, 0)     # drive output faces -Y
TO_POS_Y = Rot(-90, 0, 0)    # drive output faces +Y

# where each drive's output face sits, and which way it points
y_j3_out = -42.0
y_j5_out = -43.0
JOINTS = {
    "j1": (J13_DRIVE, Pos(0, 0, G.z_j1_out) * UP),
    "j2": (J2_DRIVE, Pos(0, G.y_j2_out, G.z_j2) * TO_NEG_Y),
    "j3": (J13_DRIVE, Pos(0, y_j3_out, G.z_j3) * TO_POS_Y),
    "j4": (WRIST_DRIVE, Pos(0, G.y_forearm, G.z_j5 - 40.0) * UP),
    "j5": (WRIST_DRIVE, Pos(0, y_j5_out, G.z_j5) * TO_NEG_Y),
    "j6": (WRIST_DRIVE, Pos(0, G.y_forearm, G.z_flange) * UP),
}


def _local_cyl(r, z0, z1, x=0.0, y=0.0):
    return Pos(x, y, z0) * Cylinder(r, z1 - z0, align=(Align.CENTER, Align.CENTER, Align.MIN))


def socket(c: Cyclo, loc: Location):
    """(material to add, material to cut) for a drive housing + motor seated in a link.

    The housing drops into a bore and its bottom face lands on a ledge carrying six
    M3 heat-set inserts; the motor hangs inside the pod beneath it.
    """
    L = cy.layout(c)
    m = c.motor
    # an 8 mm sleeve round the housing plus a ledge web out to the skin; only the part
    # inside the skin survives (it is carved out of the hollow core, see _shell)
    sleeve = _local_cyl(c.housing_r + FIT + 8.0, L.housing_bot - 6.0, L.cap_bot)
    web = _local_cyl(c.housing_r + 40.0, L.housing_bot - 6.0, L.housing_bot)
    # drain holes so the chamber above the web isn't a sealed powder trap
    rd = c.housing_r + FIT + 8.0 + 4.0
    for k in range(6):
        a = 2 * pi * (k + 0.5) / 6
        web -= _local_cyl(3.0, L.housing_bot - 7.0, L.housing_bot + 1.0, rd * cos(a), rd * sin(a))
    add = sleeve + web
    cut = _local_cyl(c.housing_r + FIT, L.housing_bot, L.cap_top + 2.0)
    half = m.frame / 2 + 0.6
    motor_pocket = Pos(0, 0, L.motor_bot - 1.0) * Box(2 * half, 2 * half, L.housing_bot - L.motor_bot + 1.0,
                                                      align=(Align.CENTER, Align.CENTER, Align.MIN))
    cut += motor_pocket
    r = cy.screw_circle_r(c)
    for a in cy.screw_angles(c):
        cut += _local_cyl(INSERT_D / 2, L.housing_bot - 7.0, L.housing_bot, r * cos(a * pi / 180), r * sin(a * pi / 180))
    return loc * add, loc * cut


def hub_bolts(c: Cyclo, loc: Location, thickness: float, reach: float = 5.0):
    """Counterbored M3 through holes in a child hub matching the output flange inserts.

    On the side hubs the counterbores run on out through the link (`reach`), so a
    driver goes straight in from outside; elsewhere see hub_access.
    """
    ob = c.out_bearing
    br = (ob.id / 2 + c.motor.shaft_d / 2 + 2.0) / 2
    cut = None
    for i in range(4):
        a = 2 * pi * i / 4 + pi / 4
        h = _local_cyl(1.7, 0.5, 0.5 + thickness, br * cos(a), br * sin(a))
        h += _local_cyl(3.0, 0.5 + 3.0, 0.5 + thickness + reach, br * cos(a), br * sin(a))
        cut = h if cut is None else cut + h
    return loc * cut


# Pod profiles: (r, s) stations along each pod's axis, measured from its open face.
# They wrap the drive housing (plus socket sleeve) first, then the motor and the
# closed-loop driver board on its back (DRIVER_T), then close off.
POD_J2 = [(56, 0), (58.5, 22), (56, 50), (51, 82), (47, 104), (42, 116), (0, 124)]
POD_J3 = [(47, 0), (49, 18), (45, 42), (38, 64), (36, 92), (35, 106), (29, 115), (0, 119)]
POD_WRIST = [(34, 0), (36, 15), (35.5, 40), (35, 62), (34.5, 78), (31, 84), (0, 86)]
DRIVER_T = {42.3: 16.0, 56.4: 18.0}   # MKS SERVO42D / 57D on the motor's back, by motor frame


HUB_HEAD_Z = 4.5      # a hub bolt head above its output face
TOOL_TILT = 26.0      # most a ball-end hex driver can lean off the bolt axis


def hub_access(c: Cyclo, loc: Location, c_next: Cyclo, loc_next: Location, r=3.5):
    """Tool channels from each hub bolt head out through the link's next (still empty) socket.

    The hub bolts are driven from inside the link, during assembly, before the next
    gearbox goes in. Each channel runs straight from a bolt head to the point on that
    socket's mouth that keeps the driver closest to the bolt axis.
    """
    from math import acos, degrees
    from build123d import Plane, Vector
    L = cy.layout(c_next)
    ob = c.out_bearing
    br = (ob.id / 2 + c.motor.shaft_d / 2 + 2.0) / 2
    axis = (loc * Location((0, 0, 1))).position - loc.position
    mouth = [loc_next * Location((rr * cos(t), rr * sin(t), L.cap_top + 2.0))
             for rr in (0.0, 0.4 * c_next.housing_r, 0.75 * c_next.housing_r)
             for t in [k * pi / 6 for k in range(12)]]
    out = None
    for i in range(4):
        a = 2 * pi * i / 4 + pi / 4
        head = (loc * Location((br * cos(a), br * sin(a), HUB_HEAD_Z))).position
        best = min((m.position for m in mouth), key=lambda m: acos(max(-1, min(1, (m - head).normalized().dot(axis)))))
        d = best - head
        if degrees(acos(d.normalized().dot(axis))) > TOOL_TILT:
            raise ValueError(f"{c.name}: no tool path to hub bolt {i}")
        ch = Plane(origin=head, z_dir=d.normalized()).location * Cylinder(r, d.length, align=(Align.CENTER, Align.CENTER, Align.MIN))
        out = ch if out is None else out + ch
    return out


def cable_port(start, direction=(-1, 0, 0), r=5.0, length=70.0):
    """Hole from inside a link out through its back wall. Doubles as an SLS powder escape."""
    from build123d import Axis as A, Plane
    plane = Plane(origin=start, z_dir=direction)
    return plane.location * Cylinder(r, length, align=(Align.CENTER, Align.CENTER, Align.MIN))


def pod(base, axis, profile):
    return lambda g: revolve_profile(profile, axis, base, g, start_tangent=(0.35, 1.0), end_tangent=(-1.0, 0.0))


def _shell(outer_fns, extra_add=(), cuts=(), seam=8.0, channels=()):
    """outer_fns: callables grow -> Part. Hollow with wall W, keep socket material, then cut.

    channels are cut only inside the hollow core, so they open up internal walls and
    sleeves without ever piercing the outer skin.
    """
    outer = soft_fuse([f(0.0) for f in outer_fns], seam)
    core = union(*[f(-W) for f in outer_fns])
    core0 = core
    # keep socket material by carving it out of the core rather than fusing it back on;
    # fusing near-coincident skins is where OCC booleans go wrong
    for a in extra_add:
        core = solids_only(core - a)
    body = solids_only(outer - core)
    for c in cuts:
        body = solids_only(body - c)
    for ch in channels:
        body = solids_only(body - (ch & core0))
    # booleans can leave slivers; anything under 1% of the main body is debris
    solids = sorted(body.solids(), key=lambda x: -x.volume)
    keep = [x for x in solids if x.volume > 0.01 * solids[0].volume]
    if len(keep) > 1:
        raise ValueError(f"link splits into {len(keep)} pieces: {[round(x.volume) for x in keep]}")
    return keep[0]


# --------------------------------------------------------------------------------------
def base() -> Part:
    c, loc = JOINTS["j1"]
    add, cut = socket(c, loc)
    top = G.z_j1_out - 1.0                      # flush with the J1 cap
    bell = lambda g: revolve_profile([(80, 0), (74, 12), (64, 30), (55, 62), (50, 95), (c.housing_r + 5, top)],
                                     "Z", (0, 0, 0), g, start_tangent=(-0.25, 1.0), end_tangent=(-0.05, 1.0))
    bosses = []
    cuts = [cut,
            _local_cyl(40.0, -1.0, 3.0),                                          # service opening
            Pos(0, 60, 22) * Rot(90, 0, 0) * Cylinder(CABLE_R, 40)]               # cable exit, back
    for k in range(3):
        a = pi / 2 + k * 2 * pi / 3
        x, y = 66 * cos(a), 66 * sin(a)
        bosses.append(_local_cyl(8.0, 0.0, 16.0, x, y))
        cuts.append(_local_cyl(3.2, -1.0, 10.0, x, y))                           # M5 heat-set insert, from below
    return _shell([bell], extra_add=[add, *bosses], cuts=cuts)


def turret() -> Part:
    c1, l1 = JOINTS["j1"]
    c2, l2 = JOINTS["j2"]
    add, cut = socket(c2, l2)
    hub_t = 8.0
    z_hub = G.z_j1_out + 0.5
    hub = lambda g: puck((0, 0, z_hub + hub_t / 2), "Z", c1.housing_r - 1.0, hub_t, 3.5, g)
    neck = lambda g: bone([((0, 0, z_hub + 3), (c1.housing_r - 5, c1.housing_r - 5)),
                          ((0, 12, z_hub + 26), (36, 33)),
                          ((0, 20, G.z_j2 - 25), (40, 36))], g)
    shell = pod((0, G.y_j2_out + 1.0, G.z_j2), "Y", POD_J2)
    cuts = [cut, hub_bolts(c1, l1, hub_t),
            Pos(0, 36, z_hub + 28) * Rot(-60, 0, 0) * Cylinder(CABLE_R, 40),   # cable port, back
            cable_port((0, 45, G.z_j2))]
    return _shell([shell, hub, neck], extra_add=[add], cuts=cuts, channels=[hub_access(c1, l1, c2, l2)])


def upper_arm() -> Part:
    c2, l2 = JOINTS["j2"]
    c3, l3 = JOINTS["j3"]
    add, cut = socket(c3, l3)
    hub_t = 9.0
    y_hub = G.y_j2_out - 0.5 - hub_t / 2
    hub = lambda g: puck((0, y_hub, G.z_j2), "Y", c2.housing_r - 3.0, hub_t, 4.0, g)
    arm = lambda g: bone([((0, y_hub - 6, G.z_j2 + 8), (36, 14)),
                         ((0, -52, G.z_j2 + 60), (27, 16)),
                         ((0, -60, G.z_j2 + 130), (26, 17)),
                         ((0, -67, G.z_j3 - 40), (32, 19))], g)
    shell = pod((0, y_j3_out - 1.0, G.z_j3), "-Y", POD_J3)
    cuts = [cut, hub_bolts(c2, l2, hub_t, reach=45.0),
            cable_port((0, -52, G.z_j2 + 45)), cable_port((0, -110, G.z_j3))]
    return _shell([hub, arm, shell], extra_add=[add], cuts=cuts)


def forearm() -> Part:
    c3, l3 = JOINTS["j3"]
    c4, l4 = JOINTS["j4"]
    add, cut = socket(c4, l4)
    hub_t = 8.5
    y_hub = y_j3_out + 0.5 + hub_t / 2
    z_out4 = G.z_j5 - 40.0
    hub = lambda g: puck((0, y_hub, G.z_j3), "Y", c3.housing_r - 3.0, hub_t, 3.5, g)
    arm = lambda g: bone([((0, y_hub + 4, G.z_j3 + 6), (32, 12)),
                         ((0, G.y_forearm - 2, G.z_j3 + 55), (23, 15)),
                         ((0, G.y_forearm, z_out4 - 75), (27, 24))], g)
    shell = pod((0, G.y_forearm, z_out4 - 1.0), "-Z", POD_WRIST)
    cuts = [cut, hub_bolts(c3, l3, hub_t, reach=45.0),
            cable_port((0, G.y_forearm - 3, G.z_j3 + 30)), cable_port((0, G.y_forearm, G.z_j5 - 95))]
    return _shell([hub, arm, shell], extra_add=[add], cuts=cuts)


def wrist_yoke() -> Part:
    """J4 output -> J5 drive."""
    c4, l4 = JOINTS["j4"]
    c5, l5 = JOINTS["j5"]
    add, cut = socket(c5, l5)
    hub_t = 8.0
    z_hub = G.z_j5 - 40.0 + 0.5
    hub = lambda g: puck((0, G.y_forearm, z_hub + hub_t / 2), "Z", c4.housing_r - 1.0, hub_t, 3.0, g)
    neck = lambda g: bone([((0, G.y_forearm, z_hub + 3), (24, 24)),
                          ((0, G.y_forearm + 12, G.z_j5 - 14), (25, 25))], g)
    shell = pod((0, y_j5_out + 1.0, G.z_j5), "Y", POD_WRIST)
    cuts = [cut, hub_bolts(c4, l4, hub_t),
            cable_port((0, 12, G.z_j5), r=4.0), cable_port((0, G.y_forearm + 6, G.z_j5 - 22), r=4.0)]
    return _shell([hub, neck, shell], extra_add=[add], cuts=cuts, seam=5.0,
                  channels=[hub_access(c4, l4, c5, l5)])


def wrist_link() -> Part:
    """J5 output -> J6 drive."""
    c5, l5 = JOINTS["j5"]
    c6, l6 = JOINTS["j6"]
    add, cut = socket(c6, l6)
    hub_t = 8.5
    y_hub = y_j5_out - 0.5 - hub_t / 2
    hub = lambda g: puck((0, y_hub, G.z_j5), "Y", c5.housing_r - 2.0, hub_t, 3.0, g)
    # stay outboard of the yoke's J5 pod (radius ~36) before swinging in under the J6 pod
    arm = lambda g: bone([((0, y_hub - 5, G.z_j5 + 12), (25, 9)),
                         ((0, y_hub - 5, G.z_j5 + 30), (23, 9)),
                         ((0, y_hub - 5, G.z_j5 + 44), (23, 9)),
                         ((0, G.y_forearm - 14, G.z_flange - 58), (26, 15))], g)
    shell = pod((0, G.y_forearm, G.z_flange - 1.0), "-Z", POD_WRIST)
    cuts = [cut, hub_bolts(c5, l5, hub_t),
            cable_port((0, y_hub - 5, G.z_j5 + 25), r=4.0), cable_port((0, G.y_forearm, G.z_flange - 50), r=4.0)]
    return _shell([hub, arm, shell], extra_add=[add], cuts=cuts, seam=5.0)


LINKS = {
    "base": base,
    "turret": turret,
    "upper_arm": upper_arm,
    "forearm": forearm,
    "wrist_yoke": wrist_yoke,
    "wrist_link": wrist_link,
}


def drive_parts():
    """{joint: {part_name: Part}} for every joint's gearbox, motor and bought hardware, placed."""
    out = {}
    for j, (c, loc) in JOINTS.items():
        parts = {**cy.printed_parts(c), **cy.vendor_parts(c)}
        out[j] = {k: loc * v for k, v in parts.items()}
    return out
