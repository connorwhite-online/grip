"""
GRIP v0.3, the architecture generative/architecture.py points at, with every
structural link regrown by loadpath around it:

- J1, J2, J3: one drive design, the NEMA 17 60 mm + 25:1 printed cycloidal.
  J2 used to need the NEMA 23 + 30:1; with the lighter wrist and the spring
  below it needs 8.4 of the 13 N·m this drive is rated for.
- J2 counterbalance: a zero-free-length spring (an extension spring on a
  cable over a pulley) between a pulley 62 mm above J2 on the turret and a
  pin 80 mm up the upper arm. Its torque k·a·b·sin(theta) follows gravity's,
  so it carries the arm's own weight in every pose; the motor only carries
  the payload and accelerations. k = 1.0 N/mm, 101 N at full reach.
- J4, J5, J6: Feetech bus servos on the same bus as the gripper servo:
  STS3215 on the two rolls, STS3250 on the pitch (J5), which carries the
  payload. The servo's own bearings carry the rolls; the J5 child is a yoke
  that also takes the servo's rear (idle) shaft in a bearing.

    python generative/v3.py studies
    python generative/v3.py run [--jobs 3]

Servo horn and shaft positions are not published by Feetech; the numbers in
SERVOS are estimates marked VERIFY. Measure a servo before printing.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import chain as c2  # noqa: E402
from chain import JOINTS as J, at, cyl, socket, hub, urdf_masses, distal  # noqa: E402

from grip_cad.params import J13_DRIVE  # noqa: E402

# J2 moves to the J1/J3 drive (chain.socket/hub read this table)
J["j2"] = (J13_DRIVE,) + tuple(J["j2"][1:])

G_ACC, DYN, LATERAL_N = c2.G_ACC, c2.DYNAMIC, c2.LATERAL_N
MATERIAL = c2.MATERIAL

# Feetech servos, mm. Body L along the link, W across, H along the output
# shaft (horn side up). Shaft centre `shaft` mm from the body's horn-end.
SERVOS = {
    "STS3215": {"L": 45.2, "W": 24.7, "H": 35.0, "shaft": 11.5, "kg": 0.055, "stall_nm": 2.9,   # shaft VERIFY
                "horn_r": 11.0, "horn_bolt_r": 7.0, "usd": 29.0},
    "STS3250": {"L": 45.2, "W": 24.7, "H": 35.0, "shaft": 11.5, "kg": 0.075, "stall_nm": 4.9,   # shaft VERIFY
                "horn_r": 11.0, "horn_bolt_r": 7.0, "usd": 74.0},
}
WRIST = {"j4": "STS3215", "j5": "STS3250", "j6": "STS3215"}
# For each servo joint: the direction from the shaft along the servo's long
# side. It must be perpendicular to the shaft: the rolls (J4, J6, shaft along
# the arm) lie across the arm; the pitch (J5, shaft across) hangs down it.
BODY_DIR = {"j4": (0, 1, 0), "j5": (0, 0, -1), "j6": (0, 1, 0)}
STEPPER_JOINT_KG = 0.37          # what each wrist joint weighs today: stepper + driver + printed cycloidal

# spring counterbalance at J2 (home-pose coordinates, mm)
SPRING_K = 1.0                   # N/mm
PULLEY_A = (0.0, -20.0, 262.0)   # on the turret, 62 mm above J2
ANCHOR_B = (0.0, -46.0, 280.0)   # on the upper arm, 80 mm up from J2
SPRING_PATH_X = 58.0             # the spring runs down the turret's side, clear of the J2 socket
SPRING_MAX_N = SPRING_K * math.hypot(62.0, 80.0)


# Where the servo case takes screws, as (along e from the body centre, along
# w, along d from the horn face) for the far face and the two side faces.
# VERIFY against the STS3215/3250 drawing: the far-face pattern and the side
# holes near the horn end are typical of the series, not measured.
BOSS_R, BOSS_T, SCREW_R, DRIVER_R = 3.5, 4.0, 1.1, 2.6
HEAD_R = 3.4                       # M2.5 socket head (4.5 mm) + a washer, plus clearance


def _mount_points(s):
    pts = []
    for se in (-1, 1):
        for sw in (-1, 1):
            pts.append(((se * (s["L"] / 2 - 5.0), sw * (s["W"] / 2 - 4.0), -s["H"]), "d"))
        for sw in (-1, 1):
            pts.append(((se * (s["L"] / 2 - 6.0), sw * s["W"] / 2, -6.0), "w"))
    return pts


def servo_parent(joint):
    """Pocket for a servo body in the link that carries it, and the screw
    bosses it is held by. Only the bosses are preserved: the structure that
    joins them is the optimizer's."""
    s = SERVOS[WRIST[joint]]
    _, p, d = J[joint]
    e = BODY_DIR[joint]
    assert sum(a * b for a, b in zip(d, e)) == 0, f"{joint}: servo body must lie across its shaft"
    w = [d[1] * e[2] - d[2] * e[1], d[2] * e[0] - d[0] * e[2], d[0] * e[1] - d[1] * e[0]]
    ce = s["L"] / 2 - s["shaft"]          # body centre, along e from the shaft
    lo, hi = [], []
    for k in range(3):
        a = p[k] - e[k] * s["shaft"] - d[k] * s["H"] - abs(w[k]) * s["W"] / 2
        b = p[k] + e[k] * (s["L"] - s["shaft"]) + abs(w[k]) * s["W"] / 2
        lo.append(min(a, b, p[k] - abs(w[k]) * s["W"] / 2))
        hi.append(max(a, b, p[k] + abs(w[k]) * s["W"] / 2))
    for k in range(3):
        if d[k]:
            lo[k], hi[k] = (p[k] - s["H"], p[k]) if d[k] > 0 else (p[k], p[k] + s["H"])
    body = {"type": "box", "min": [round(v - 0.4, 2) for v in lo], "max": [round(v + 0.4, 2) for v in hi]}
    # open the horn side: the keep-out runs on past the horn face
    open_lo, open_hi = list(body["min"]), list(body["max"])
    for k in range(3):
        if d[k] > 0:
            open_hi[k] += 12.0
        elif d[k] < 0:
            open_lo[k] -= 12.0
    bosses, holes = [], []
    for i, ((oe, ow, od), face) in enumerate(_mount_points(s)):
        q = [p[k] + (ce + oe) * e[k] + ow * w[k] + od * d[k] for k in range(3)]
        n = [-x for x in d] if face == "d" else [(1 if ow > 0 else -1) * x for x in w]
        bosses.append(cyl(q, n, -0.4, BOSS_T, BOSS_R))
        holes.append({"id": f"{joint}-screw-{i}", "shape": cyl(q, n, -6.0, BOSS_T + 0.5, SCREW_R)})
        # the screw head and its driver: clear from the boss face outward, so
        # nothing grows over the face the head seats on
        holes.append({"id": f"{joint}-driver-{i}", "shape": cyl(q, n, BOSS_T, BOSS_T + 25.0, DRIVER_R)})
    preserve = [{"id": f"{joint}-mount", "shape": {"type": "union", "shapes": bosses}}]
    keep = [{"id": f"{joint}-servo", "shape": {"type": "box", "min": open_lo, "max": open_hi}},
            # the horn and the child's hub sweep a disc above the horn face
            {"id": f"{joint}-horn-sweep", "shape": cyl(p, d, 0.0, 9.0, s["horn_r"] + 6.0)}] + holes
    return preserve, keep


def servo_child(joint, yoke=False):
    """Hub on a servo's horn (and, for a yoke, on its idle shaft opposite)."""
    s = SERVOS[WRIST[joint]]
    _, p, d = J[joint]
    preserve = [{"id": f"{joint}-horn-hub", "shape": cyl(p, d, 4.0, 8.0, s["horn_r"] + 3.0)}]
    # The horn is bolted and clamped to the whole hub face, so hold the hub
    # rather than the four bolt holes: at a coarse level of the search a
    # 2.6 mm hole can have no cell of its own beside it.
    keep, supports = [], [{"region": f"{joint}-horn-hub"}]
    u = (1, 0, 0) if d[0] == 0 else (0, 1, 0)
    v = [d[1] * u[2] - d[2] * u[1], d[2] * u[0] - d[0] * u[2], d[0] * u[1] - d[1] * u[0]]
    for i in range(4):
        a = math.pi / 4 + i * math.pi / 2
        q = [p[k] + s["horn_bolt_r"] * (math.cos(a) * u[k] + math.sin(a) * v[k]) for k in range(3)]
        keep.append({"id": f"{joint}-horn-bolt-{i}", "shape": cyl(q, d, 3.5, 9.0, 1.3)})
        # head + washer seat on the hub face, then the driver (QA 10/28: the
        # organic body grew over the hub's bolt faces at r = 2.5)
        keep.append({"id": f"{joint}-horn-driver-{i}", "shape": cyl(q, d, 8.0, 40.0, HEAD_R)})
    # everything the parent carries stays clear: the servo body and its cradle
    pp, pk = servo_parent(joint)
    keep += [{"id": f"{joint}-parent-{r['id']}", "shape": r["shape"]} for r in pp]
    if yoke:
        # the rear shaft runs in a 6801 bearing on the far side of the body
        far = at(p, d, -s["H"] - 4.0)
        preserve.append({"id": f"{joint}-idler-hub", "shape": cyl(far, d, -4.0, 0.0, 13.0)})
        keep.append({"id": f"{joint}-idler-bearing", "shape": cyl(far, d, -4.5, 0.5, 10.6)})
        supports.append({"region": f"{joint}-idler-bearing", "fix": "pin"})
    return preserve, keep, supports


def spring_parts(link):
    if link == "turret":
        preserve = [{"id": "spring-pulley", "shape": cyl(PULLEY_A, (1, 0, 0), -9.0, 9.0, 8.0)},
                    {"id": "spring-anchor", "shape": cyl((SPRING_PATH_X, -20.0, 140.0), (1, 0, 0), -6.0, 6.0, 7.0)}]
        keep = [{"id": "spring-pulley-pin", "shape": cyl(PULLEY_A, (1, 0, 0), -12.0, 12.0, 2.6)},
                {"id": "spring-run", "shape": cyl((SPRING_PATH_X, -20.0, 150.0), (0, 0, 1), 0.0, 108.0, 9.0)},
                {"id": "spring-anchor-pin", "shape": cyl((SPRING_PATH_X, -20.0, 140.0), (1, 0, 0), -9.0, 9.0, 2.6)}]
        loads = [{"region": "spring-pulley", "force": [-SPRING_MAX_N * 0.62, 0.0, -SPRING_MAX_N * (1 + 0.0)]},
                 {"region": "spring-anchor", "force": [0.0, 0.0, SPRING_MAX_N]}]
        return preserve, keep, loads
    if link == "upper_arm":
        preserve = [{"id": "spring-pin-boss", "shape": cyl(ANCHOR_B, (1, 0, 0), -8.0, 8.0, 7.0)}]
        keep = [{"id": "spring-pin", "shape": cyl(ANCHOR_B, (1, 0, 0), -11.0, 11.0, 2.6)},
                {"id": "spring-cable", "shape": cyl(ANCHOR_B, (0, 0.28, -0.96), 0.0, 25.0, 4.0)}]
        # arm reaching out: the cable pulls the pin back toward the turret, against gravity's moment
        loads = [{"region": "spring-pin-boss", "force": [round(SPRING_MAX_N * 0.62, 1), 0.0, round(-SPRING_MAX_N * 0.78, 1)]}]
        return preserve, keep, loads
    return [], [], []


LINKS = {
    # link: (below joint, above joint)
    "turret": ("j1", "j2"),
    "upper_arm": ("j2", "j3"),
    "forearm": ("j3", "j4"),
    "wrist_yoke": ("j4", "j5"),
    "wrist_link": ("j5", "j6"),
}
HUB_T = {"turret": 8.0, "upper_arm": 9.0, "forearm": 8.5}


def v3_masses():
    """URDF masses with the v0.3 changes: wrist joints become servos."""
    m = urdf_masses()
    for link, joint in (("forearm", "j4"), ("wrist_yoke", "j5"), ("wrist_link", "j6")):
        kg, com = m[link]
        m[link] = (kg - STEPPER_JOINT_KG + SERVOS[WRIST[joint]]["kg"], com)
    return m


def efforts():
    e = c2.urdf_efforts()
    e["j2"] = 13.0                                   # now the NEMA 17 25:1 drive
    for j, name in WRIST.items():
        e[j] = SERVOS[name]["stall_nm"]
    return e


def study(link, masses, eff, max_elements=60000):
    below, above = LINKS[link]
    if below in WRIST:
        pres, keep, supports = servo_child(below, yoke=(below == "j5"))
    else:
        pres, keep, supports = hub(below, HUB_T[link])
    if above in WRIST:
        p2, k2 = servo_parent(above)
        region = f"{above}-mount"
    else:
        p2, k2 = socket(above)
        region = f"{above}-sleeve"
        c, p, d = J[above]
        k2.append({"id": f"{above}-child-sweep", "shape": cyl(p, d, 0.0, 30.0, c.housing_r + 12.0)})
    sp, sk, sl = spring_parts(link)
    preserve, keep = pres + p2 + sp, keep + k2 + sk

    m, com = distal(masses, link)
    _, p, d = J[above]
    w = m * G_ACC * DYN
    force = [-w, 0.0, 0.0]
    r = [com[0] * 1000 - p[0], com[1] * 1000 - p[1], max(com[2] * 1000 - p[2], 0.0)]
    mom = [r[1] * force[2] - r[2] * force[1], r[2] * force[0] - r[0] * force[2], r[0] * force[1] - r[1] * force[0]]
    axis = [float(x) for x in d]
    cases = [
        {"name": "reach", "loads": [{"region": region, "force": [round(v, 2) for v in force],
                                     "moment": [round(v, 1) for v in mom]}] + sl},
        {"name": f"{above}-rated-torque", "loads": [{"region": region,
                                                     "moment": [round(eff[above] * 1000 * a, 1) for a in axis]}]},
        {"name": "lateral", "loads": [{"region": region, "force": [round(LATERAL_N * a, 2) for a in axis]}]},
    ]
    space = c2.design_space(preserve, keep)
    return {
        "name": f"grip-v3-{link.replace('_', '-')}",
        "description": f"GRIP v0.3 {link.replace('_', ' ')} (generative/v3.py): {below} -> {above}"
                       + (" with the J2 spring counterbalance" if sl else ""),
        "designSpace": [space], "preserve": preserve, "keepOut": keep, "supports": supports,
        "loadCases": cases, "material": MATERIAL,
        "goal": {"safetyFactor": 2.0, "volumeFraction": 0.12},
        "minMember": 3.0, "symmetry": ["x"] if not sl or link != "turret" else ["x"],
        "resolution": {"maxElements": max_elements},
        "chain": {"distalKg": m, "staticTorqueNm": abs(mom[1] if axis[1] else mom[0]) / 1000 / DYN},
    }


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["studies", "run", "remesh"])
    ap.add_argument("--links", default=",".join(LINKS))
    ap.add_argument("--jobs", type=int, default=3)
    ap.add_argument("--out", default=os.path.join(HERE, "v3", "out"))
    ap.add_argument("--max-elements", type=int, default=60000,
                    help="optimization cells; finer resolves the small fillets where peaks sit")
    args = ap.parse_args(argv)
    links = args.links.split(",")
    masses, eff = v3_masses(), efforts()
    sdir = os.path.join(HERE, "v3", "studies")
    os.makedirs(sdir, exist_ok=True)
    studies = {l: study(l, masses, eff, args.max_elements) for l in links}
    for l, s in studies.items():
        with open(os.path.join(sdir, f"{l}.json"), "w") as fh:
            json.dump(s, fh, indent=1)
    if args.cmd == "studies":
        for l, s in studies.items():
            print(l, s["chain"])
        return 0
    if args.cmd == "remesh":
        # redraw parts from their saved density fields (after a change to
        # loadpath's reconstruction), verify, and update the summary
        import numpy as np
        import trimesh  # noqa: F401
        from loadpath.domain import build_domain
        from loadpath.reconstruct import reconstruct
        from loadpath.spec import study_from_dict
        from loadpath.verify import verify_part
        sfile = os.path.join(args.out, "summary.json")
        summary = json.load(open(sfile)) if os.path.exists(sfile) else {}
        for l in links:
            f = np.load(os.path.join(args.out, f"{l}.field.npz"))
            st = study_from_dict(studies[l])
            d = build_domain(st, pitch=float(f["pitch"]))
            mesh, _ = reconstruct(st, d, f["xphys"])
            rep = verify_part(st, mesh, max_cells=150_000, min_pitch=0.3 * d.pitch, contact_band=d.pitch)
            mesh.export(os.path.join(args.out, f"{l}.stl"))
            summary[l] = {"g": rep["mass"], "sf": rep["safetyFactor"], "boundaryCutArea": rep["boundaryCutArea"],
                          "defl": {c["name"]: c["deflection"] for c in rep["cases"]}}
            print(json.dumps({l: summary[l]}), flush=True)
        json.dump(summary, open(sfile, "w"), indent=1)
        return 0
    import multiprocessing as mp
    from concurrent.futures import ProcessPoolExecutor
    os.makedirs(args.out, exist_ok=True)
    with ProcessPoolExecutor(args.jobs, mp_context=mp.get_context("spawn")) as ex:
        results = dict(zip(links, ex.map(c2._solve, [studies[l] for l in links])))
    sfile = os.path.join(args.out, "summary.json")
    summary = json.load(open(sfile)) if os.path.exists(sfile) else {}
    for l in links:
        r = results[l]
        if r.best is None:
            summary[l] = {"error": "no outcome met the goal",
                          "tried": [o.summary()["safetyFactor"] for o in r.outcomes]}
            continue
        o = r.outcomes[r.best]
        o.mesh.export(os.path.join(args.out, f"{l}.stl"))
        if o.xphys is not None:
            # the density field, so `remesh` can redraw the part without re-optimizing
            import numpy as np
            np.savez_compressed(os.path.join(args.out, f"{l}.field.npz"), xphys=o.xphys,
                                pitch=r.domain["pitch"])
        summary[l] = {"g": o.report["mass"], "sf": o.report["safetyFactor"],
                      "defl": {c["name"]: c["deflection"] for c in o.report["cases"]}}
        print(json.dumps({l: summary[l]}), flush=True)
    with open(sfile, "w") as fh:
        json.dump(summary, fh, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
