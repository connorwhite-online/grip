"""
GRIP v0.3 hand: three soft fingers on one servo (concept).

Replaces the parallel-jaw gripper (two Ø6 rods, four LM6UU bearings, a rack
and pinion, set screws) with:

- **Three monolithic TPU fingers.** Each is one print: three phalanges joined
  by flexure hinges (thin strips on the back of the finger, V-notches on the
  palm side), a tendon channel down the palm side, ridged grip pads, and a
  slot for a rigid fingernail. Tendon tension curls the finger; the TPU springs
  it straight again when the tendon lets go.
- **One servo winds one tendon.** It pulls a floating three-way balance plate
  (a whiffletree), and the plate pulls the three finger tendons. A finger that
  touches first stops; the others keep closing until all three bear, so the
  hand wraps odd shapes with no extra motors or sensors.
- **Fingernails.** A 1.5 mm PA12 nail at each fingertip, flush with the pad, so
  the hand can still get under a flat part on a print bed (GRIP's first job).
- **2 + 1 layout.** Two fingers face a thumb. Closing on a flat part makes a
  line pinch between nails; on a round one, a three-point wrap.
- **The palm is generated** (loadpath) around the servo, the spool, the
  balance plate's travel and the tendon runs.

Grip force comes from the servo's current limit. The STS3215 reports its
load, so the arm knows when it has a grip and how hard, without a sensor.

    python generative/hand.py fingers      # finger + nail STLs, force estimate
    python generative/hand.py studies      # palm study JSON
    python generative/hand.py run          # optimize the palm

Everything here is a first concept: VERIFY the servo, the flexure thickness
(print a coupon first) and the tendon routing on a real print.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
OUT = os.path.join(HERE, "hand")

# --- finger -----------------------------------------------------------------
# Local frame: root face on z = 0, length along +z, palm side +y, width along x.
FINGER = dict(
    segments=(38.0, 26.0, 22.0),   # proximal, middle, distal (mm)
    root=10.0,                     # block that bolts to the palm, below segment 1
    width=18.0,
    thick=13.0,                    # y, palm face to back, at the root
    tip_thick=9.0,                 # ... and at the tip
    edge_r=3.5,                    # long edges rounded
    hinge=2.2,                     # flexure strip left at the back (TPU 95A)
    notch=6.0,                     # V-notch opening on the palm side at each hinge
    channel_r=1.1,                 # tendon: 1 mm braided Dyneema
    channel_y=3.2,                 # channel centre, in from the palm face
    ridge_pitch=3.0,
    ridge_h=0.7,
    nail_t=1.5,
    nail_out=2.5,                  # nail stands past the fingertip
)
TPU_E = 26.0          # MPa, TPU 95A (datasheet range 20-40; VERIFY with a coupon)


def finger():
    """One finger as a build123d Part (TPU)."""
    from build123d import Align, Axis, Box, Cylinder, Pos, Rot

    f = FINGER
    L = f["root"] + sum(f["segments"])
    w, t = f["width"], f["thick"]
    # palm side at y = +t/2
    yp = t / 2
    # The back tapers from full thickness at the root to `tip_thick` at the
    # tip, and every long edge is rounded: a finger, not a bar. The palm face
    # stays flat so the pads and nail sit in one plane.
    side = [(yp, 0.0), (yp, L), (yp - f["tip_thick"], L), (-yp, f["root"] + f["segments"][0] * 0.5), (-yp, 0.0)]
    body = _yz_prism(side, w)
    body = body.fillet(f["edge_r"], body.edges().filter_by(Axis.Z))
    body = body.fillet(min(f["edge_r"], 3.0), body.edges().filter_by(lambda e: e.center().Z > L - 0.5))
    z = f["root"]
    hinges = []
    for seg in f["segments"][:-1]:
        z += seg
        hinges.append(z)
    for zh in [f["root"]] + hinges:
        # V-notch from the palm face down to the flexure strip at the back
        # down to the flexure strip, measured from the tapered back
        za = f["root"] + f["segments"][0] * 0.5
        back = -yp if zh <= za else -yp + (t - f["tip_thick"]) * (zh - za) / (L - za)
        depth = (yp - back) - f["hinge"]
        tri = [(yp + 0.1, zh - f["notch"] / 2), (yp - depth, zh), (yp + 0.1, zh + f["notch"] / 2)]
        body = body - _yz_prism(tri, w + 2)
    # tendon channel down the palm side, open at the root, ending in a knot pocket
    ch_y = yp - f["channel_y"]
    body = body - Pos(0, ch_y, -1) * Cylinder(f["channel_r"], L - 3.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    body = body - Pos(0, ch_y, L - 7.0) * Cylinder(2.2, 4.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    # grip ridges across the palm face of every phalanx
    z0 = f["root"]
    for seg in f["segments"]:
        n = int((seg - f["notch"] - 2) // f["ridge_pitch"])
        for i in range(n):
            zr = z0 + f["notch"] / 2 + 1.5 + i * f["ridge_pitch"]
            body = body + Pos(0, yp, zr) * Rot(0, 90, 0) * Cylinder(f["ridge_h"], w - 2.0)
        z0 += seg
    # nail pocket on the palm side: a narrow slot for the tongue, a wider
    # recess at the tip for the blade (its shoulder takes the scraping push),
    # and an M2 cross pin through finger and tongue (takes pull-out and the prying)
    ny = yp - 1.2 - f["nail_t"] / 2
    body = body - Pos(0, ny, L - NAIL["tongue"] - 0.2) * Box(NAIL["tongue_w"] + 0.2, f["nail_t"] + 0.2, NAIL["tongue"] + 1.0,
                                                               align=(Align.CENTER, Align.CENTER, Align.MIN))
    body = body - Pos(0, ny, L - NAIL["seat"]) * Box(NAIL["blade_w"] + 0.2, f["nail_t"] + 0.2, NAIL["seat"] + 1.0,
                                                       align=(Align.CENTER, Align.CENTER, Align.MIN))
    # (the pin runs through the thickness, from the back of the finger: across
    # the 1.5 mm tongue it would cut it in two)
    body = body - Pos(0, 0, L - NAIL["pin_z"]) * Rot(90, 0, 0) * Cylinder(NAIL["pin_r"], t + 2)
    # two M3 holes through the root block (along y) to bolt it to the palm
    for x in (-5.0, 5.0):
        body = body - Pos(x, 0, f["root"] / 2) * Rot(90, 0, 0) * Cylinder(1.6, t + 2)
    return body


def _yz_prism(pts, length):
    """Extrude a polygon given as (y, z) points along x, centred on x = 0."""
    from build123d import Plane, Polyline, extrude, make_face

    face = Plane.YZ * make_face(Polyline(*pts, close=True))
    return extrude(face, length / 2, both=True)


# Fingernail: a blade standing past the tip and a tongue running down into the
# distal phalanx. Scraping a part off a bed pushes the nail back into the
# finger (the blade's shoulder bears on the recess floor: 20 N over ~6 mm2 is
# ~3 MPa on TPU) and pries it away from the palm face (resisted by the tongue
# bearing along its 15 mm and an M2 screw through the back of the finger and the tongue, not by the slot's lip). VERIFY with
# a scrape test on a real bed.
NAIL = dict(blade_w=FINGER["width"] - 3.0, tongue_w=FINGER["width"] - 8.0, seat=2.5, tongue=15.0,
            pin_z=9.0, pin_r=1.1)


def nail():
    """Rigid fingernail (PA12-GF): blade + tongue + cross-pin hole, chamfered edge."""
    from build123d import Align, Box, Cylinder, Pos, Rot, chamfer

    f = FINGER
    L = f["root"] + sum(f["segments"])
    blade = Pos(0, 0, L - NAIL["seat"]) * Box(NAIL["blade_w"], f["nail_t"], NAIL["seat"] + f["nail_out"],
                                              align=(Align.CENTER, Align.CENTER, Align.MIN))
    tongue = Pos(0, 0, L - NAIL["tongue"]) * Box(NAIL["tongue_w"], f["nail_t"], NAIL["tongue"],
                                                 align=(Align.CENTER, Align.CENTER, Align.MIN))
    n = blade + tongue
    n = n - Pos(0, 0, L - NAIL["pin_z"]) * Rot(90, 0, 0) * Cylinder(NAIL["pin_r"], 4.0)
    # bevel the leading edge on the back side so it slides under a part
    top = n.edges().filter_by(lambda e: e.center().Z > L + f["nail_out"] - 0.01 and e.center().Y < 0)
    n = chamfer(top, f["nail_t"] * 0.6)
    return n


def force_estimate(tendon_n=40.0, close_deg=70.0):
    """Closing and grip force from small-deflection beam theory.

    Each hinge is a short TPU strip in bending (k = E I / l). The tendon runs
    `e` from the strip's neutral axis and puts T * e on every hinge. Closing
    each hinge `close_deg` takes T = k * angle / e; whatever tendon force is
    left over after that becomes grip. A pad pushes with (T - T_close) * e
    over its distance from the hinge it turns on: the distal pad (short lever)
    at the high end, a contact at the base of the proximal phalanx at the low
    end."""
    f = FINGER
    I = f["width"] * f["hinge"] ** 3 / 12.0
    lh = f["notch"] * (f["thick"] - f["hinge"]) / f["thick"] / 2 + 1.0   # effective flexure length
    k = TPU_E * I / lh                                                    # N mm / rad
    e = f["thick"] / 2 - f["channel_y"] + (f["thick"] / 2 - f["hinge"] / 2)
    t_close = k * math.radians(close_deg) / e
    m = max(tendon_n - t_close, 0.0) * e
    return {"tendonN": tendon_n, "servoTorqueNm": round(3 * tendon_n * SPOOL_R / 1000.0, 2),
            "hingeStiffnessNmmPerRad": round(k, 1), "tendonArmMm": round(e, 2),
            "tendonToCloseN": round(t_close, 1),
            "padForceN": [round(m / sum(f["segments"]), 1), round(m / (f["segments"][-1] * 0.6), 1)]}


# --- palm -------------------------------------------------------------------
SPOOL_R = 8.0
TENDON_MAX_N = 2.9 / (SPOOL_R / 1000.0) / 2   # half of the STS3215's stall torque on the spool (~180 N)
# finger roots: (x, y, curl direction in y). Two fingers face the thumb.
ROOTS = [(-20.0, 30.0, -1), (20.0, 30.0, -1), (0.0, -30.0, 1)]   # 47 mm between pads, open
ROOT_Z = 66.0          # palm top: the fingers stand out of their cups here
SERVO_BODY = {"min": [-38.0, -22.6, 14.0], "max": [-3.0, 22.6, 38.7]}   # STS3215, shaft along +x
SHAFT = (-3.0, 11.1, 26.35)
from chain import MATERIAL  # noqa: E402  (SLS PA12-GF, same as the links)


def _box(lo, hi):
    return {"type": "box", "min": [round(v, 2) for v in lo], "max": [round(v, 2) for v in hi]}


def _cyl(p0, p1, r):
    return {"type": "cylinder", "p0": [round(v, 2) for v in p0], "p1": [round(v, 2) for v in p1], "r": r}


def palm_study(max_elements=90_000):
    pres, keep, sup = [], [], []
    # J6 horn hub at the base, on the tool axis (the J6 servo turns the hand)
    pres.append({"id": "j6-hub", "shape": {"type": "cylinder", "p0": [0, 0, 0], "p1": [0, 0, 5], "r": 15.0}})
    keep.append({"id": "j6-horn", "shape": _cyl((0, 0, -6), (0, 0, 0.6), 12.0)})
    for i in range(4):
        a = math.pi / 4 + i * math.pi / 2
        q = (7.0 * math.cos(a), 7.0 * math.sin(a))
        keep.append({"id": f"j6-bolt-{i}", "shape": _cyl((q[0], q[1], -1), (q[0], q[1], 6), 1.3)})
        keep.append({"id": f"j6-driver-{i}", "shape": _cyl((q[0], q[1], 6), (q[0], q[1], 40), 2.6)})
    sup.append({"region": "j6-hub"})
    # gripper servo: body pocket open on top for assembly, four screw bosses on its far face
    lo, hi = SERVO_BODY["min"], SERVO_BODY["max"]
    keep.append({"id": "servo", "shape": _box((lo[0] - 0.4, lo[1] - 0.4, lo[2] - 0.4), (hi[0] + 0.4, hi[1] + 0.4, hi[2] + 14))})
    bosses = []
    for y in (lo[1] + 5.0, hi[1] - 5.0):
        for z in (lo[2] + 5.0, hi[2] - 5.0):
            bosses.append(_cyl((lo[0] + 0.4, y, z), (lo[0] - 4.0, y, z), 3.5))
            keep.append({"id": f"servo-screw-{len(bosses)}", "shape": _cyl((lo[0] + 6, y, z), (lo[0] - 4.5, y, z), 1.1)})
            keep.append({"id": f"servo-driver-{len(bosses)}", "shape": _cyl((lo[0] - 4.5, y, z), (lo[0] - 30, y, z), 2.6)})
    pres.append({"id": "servo-mount", "shape": {"type": "union", "shapes": bosses}})
    # spool on the horn, and the balance plate's travel above it
    sx, sy, sz = SHAFT
    keep.append({"id": "spool", "shape": _cyl((sx - 0.5, sy, sz), (sx + 12.0, sy, sz), SPOOL_R + 4.0)})
    keep.append({"id": "lead", "shape": _cyl((sx + 6.0, sy, sz), (sx + 6.0, 0.0, 50.0), 3.0)})
    keep.append({"id": "balance-plate", "shape": _box((-14.0, -16.0, 46.0), (14.0, 16.0, 56.0))})
    # finger roots: only the interfaces. The root block drops between two
    # pairs of bosses (two M3 through bosses and block, along y) and stands on
    # a seat pad that takes the tendon's pull. The clevis between them is the
    # optimizer's (a preserved cup came out as a box).
    fw, ft, fr = FINGER["width"], FINGER["thick"], FINGER["root"]
    for i, (x, y, cy) in enumerate(ROOTS):
        zb = ROOT_Z - fr / 2
        pieces = [_box((x - fw / 2, y - ft / 2, ROOT_Z - fr - 2.5), (x + fw / 2, y + ft / 2, ROOT_Z - fr + 0.01))]
        for dx in (-5.0, 5.0):
            for side in (-1, 1):
                y0 = y + side * (ft / 2 + 0.2)
                pieces.append(_cyl((x + dx, y0, zb), (x + dx, y0 + side * 3.5, zb), 3.6))
            keep.append({"id": f"root-bolt-{i}{'ab'[dx > 0]}", "shape": _cyl((x + dx, y - 16, zb), (x + dx, y + 16, zb), 1.6)})
        pres.append({"id": f"root-{i}", "shape": {"type": "union", "shapes": pieces}})
        keep.append({"id": f"socket-{i}", "shape": _box((x - fw / 2 - 0.2, y - ft / 2 - 0.2, ROOT_Z - fr),
                                                        (x + fw / 2 + 0.2, y + ft / 2 + 0.2, ROOT_Z + 1))})
        keep.append({"id": f"finger-{i}", "shape": _box((x - fw / 2 - 1, y - ft / 2 - 1, ROOT_Z), (x + fw / 2 + 1, y + ft / 2 + 1, ROOT_Z + 100))})
        keep.append({"id": f"tendon-{i}", "shape": _cyl((0.0, 0.0, 51.0), (x, y + cy * (ft / 2 - FINGER["channel_y"]), ROOT_Z - fr + 0.5), 2.0)})
    # loads: grasp reaction, a 1 kg part pulled out along the tool axis or sideways (x1.5)
    w = 1.0 * 9.81 * 1.5
    grip = 20.0
    cases = [
        {"name": "squeeze", "loads": [{"region": f"root-{i}", "force": [0.0, -cy * grip, 0.0],
                                       "moment": [-cy * grip * 45.0, 0.0, 0.0]} for i, (x, y, cy) in enumerate(ROOTS)]},
        # the tendon at full servo torque: the spool pulls the servo toward
        # the balance plate, and the plate's three tendons pull each finger
        # root back into the palm. Without this case nothing ties the servo
        # mount to the rest of the palm (the first run left its bosses floating).
        {"name": "tendon", "loads": [{"region": "servo-mount", "force": [0.0, 0.0, TENDON_MAX_N]}] +
                                    [{"region": f"root-{i}", "force": [0.0, 0.0, -TENDON_MAX_N / 3]} for i in range(3)]},
        {"name": "pull-out", "loads": [{"region": f"root-{i}", "force": [0.0, 0.0, w / 3]} for i in range(3)]},
        {"name": "side", "loads": [{"region": f"root-{i}", "force": [w / 3, 0.0, 0.0],
                                    "moment": [0.0, w / 3 * 50.0, 0.0]} for i in range(3)]},
    ]
    space = {"type": "box", "min": [-48.0, -44.0, 0.0], "max": [36.0, 44.0, ROOT_Z]}  # (cups preserved past it)
    return {
        "name": "grip-hand-palm",
        "description": "GRIP v0.3 three-finger hand: palm between the J6 horn, the gripper servo and three finger roots",
        "designSpace": [space], "preserve": pres, "keepOut": keep, "supports": sup,
        "loadCases": cases, "material": MATERIAL,
        "goal": {"safetyFactor": 2.0, "volumeFraction": 0.12},
        "minMember": 3.0, "resolution": {"maxElements": max_elements},
    }


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["fingers", "studies", "run"])
    ap.add_argument("--max-elements", type=int, default=90_000)
    args = ap.parse_args(argv)
    os.makedirs(OUT, exist_ok=True)
    if args.cmd == "fingers":
        from build123d import export_stl
        export_stl(finger(), os.path.join(OUT, "finger.stl"))
        export_stl(nail(), os.path.join(OUT, "nail.stl"))
        est = {f"{t:g}N": force_estimate(t) for t in (30.0, 60.0, 100.0)}
        json.dump(est, open(os.path.join(OUT, "finger-forces.json"), "w"), indent=1)
        print(json.dumps(est, indent=1))
        return 0
    study = palm_study(args.max_elements)
    with open(os.path.join(OUT, "palm.json"), "w") as fh:
        json.dump(study, fh, indent=1)
    if args.cmd == "studies":
        return 0
    from loadpath.spec import study_from_dict
    from loadpath.study import run_study
    r = run_study(study_from_dict(study),
                  on_progress=lambda ev, _x: ev.get("stage") in ("outcome", "confirmed") and print(json.dumps(
                      {k: ev.get(k) for k in ("stage", "volumeFraction", "mass", "safetyFactor", "margin")}), flush=True))
    if r.best is None:
        print("no outcome met the goal")
        return 1
    o = r.outcomes[r.best]
    o.mesh.export(os.path.join(OUT, "palm.stl"))
    json.dump({"g": o.report["mass"], "sf": o.report["safetyFactor"],
               "defl": {c["name"]: c["deflection"] for c in o.report["cases"]}},
              open(os.path.join(OUT, "palm-summary.json"), "w"), indent=1)
    print("palm", round(o.report["mass"], 1), "g  SF", round(o.report["safetyFactor"], 2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
