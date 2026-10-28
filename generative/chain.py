"""
GRIP, redesigned generatively: every structural link grown by loadpath from
the joint layout in cad/grip_cad/params.py, and the whole chain sized
together.

A link is a hub on the joint below it (a bolt ring on that drive's output
flange) and a socket for the drive of the joint above it (the housing's
bore, a sleeve and the ledge its inserts sit in, with the motor and its
driver board hanging below). Everything else is the optimizer's: where the
material goes between them. Keep-outs carry what the assembly needs: the
drives, motors and driver boards, the hex-driver access to every hub bolt,
the cable run, and room for the neighbouring links to swing.

Loads come from statics over the arm: for each link, the weight of
everything past it (links, drives, gripper, payload) in the worst pose,
with a dynamic factor, the torque of the joint above at its rating, and a
lateral push. After the links are optimized their new masses go back in,
the joint torques are recomputed, and the links are re-optimized, until
the masses settle. Then each joint's drive can be checked against the
torque it now needs.

    python generative/chain.py studies        # write studies/*.json for the current masses
    python generative/chain.py run [--iterations 2] [--links upper_arm,forearm]
    python generative/chain.py report         # compare against the shipped parts

Needs loadpath (pip install ./loadpath from the Materialize repo, or on
PYTHONPATH) and trimesh.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "cad"))

from grip_cad.params import GEOM as G, J13_DRIVE, J2_DRIVE, PROCESS, WRIST_DRIVE  # noqa: E402

FIT = PROCESS.socket_fit
SLEEVE = 5.0          # radial wall round a drive housing (the shipped socket uses 8)
LEDGE = 6.0           # the ledge under the housing that carries its six M3 inserts
DRIVER_T = {42.3: 16.0, 56.4: 18.0}
G_ACC = 9.81
DYNAMIC = 1.5         # accelerations on top of gravity
PAYLOAD_KG = 1.0      # peak payload
LATERAL_N = 20.0      # a bump along the joint axis at the link's far joint

y_j3_out, y_j5_out = -42.0, -43.0
JOINTS = {
    # name: (drive, output-face point, unit output direction)
    "j1": (J13_DRIVE, (0.0, 0.0, G.z_j1_out), (0, 0, 1)),
    "j2": (J2_DRIVE, (0.0, G.y_j2_out, G.z_j2), (0, -1, 0)),
    "j3": (J13_DRIVE, (0.0, y_j3_out, G.z_j3), (0, 1, 0)),
    "j4": (WRIST_DRIVE, (0.0, G.y_forearm, G.z_j5 - 40.0), (0, 0, 1)),
    "j5": (WRIST_DRIVE, (0.0, y_j5_out, G.z_j5), (0, -1, 0)),
    "j6": (WRIST_DRIVE, (0.0, G.y_forearm, G.z_flange), (0, 0, 1)),
}
# link: (joint below whose output it bolts to, joint above whose drive it carries, hub thickness)
LINKS = {
    "turret": ("j1", "j2", 8.0),
    "upper_arm": ("j2", "j3", 9.0),
    "forearm": ("j3", "j4", 8.5),
    "wrist_yoke": ("j4", "j5", 8.0),
    "wrist_link": ("j5", "j6", 8.5),
}
SHIPPED_STL = os.path.join(ROOT, "exports", "stl", "{}.stl")
MATERIAL = {"name": "pa12-gf", "E": 3200, "nu": 0.37, "strength": 48, "density": 1.22,
            "note": "SLS glass-filled PA12 (EOS PA 3200 GF datasheet)"}


def layout(c):
    """Stack levels of a cycloidal drive along its axis (cad/grip_cad/cycloidal.py)."""
    ob = c.out_bearing
    cap_top = -1.0
    cap_bot = cap_top - (ob.w + 1.5)
    plate_bot = cap_bot - 4.0
    d1_lo = plate_bot - 0.5 - c.disc_t
    d2_lo = d1_lo - 0.5 - c.disc_t
    bore_bot = d2_lo - 1.0
    housing_bot = bore_bot - max(5.0, c.motor.head_h + 3.0)
    return {"cap_top": cap_top, "cap_bot": cap_bot, "housing_bot": housing_bot,
            "motor_bot": housing_bot - c.motor.length}


def at(p, d, s):
    return [round(p[i] + d[i] * s, 3) for i in range(3)]


def cyl(p, d, s0, s1, r, r_in=None):
    shape = {"type": "cylinder", "p0": at(p, d, s0), "p1": at(p, d, s1), "r": round(r, 3)}
    if r_in:
        shape["rInner"] = round(r_in, 3)
    return shape


def axis_box(p, d, s0, s1, half):
    """A square-section box along an axis-aligned direction d."""
    a, b = at(p, d, s0), at(p, d, s1)
    lo = [min(a[i], b[i]) if d[i] else p[i] - half for i in range(3)]
    hi = [max(a[i], b[i]) if d[i] else p[i] + half for i in range(3)]
    return {"type": "box", "min": [round(v, 3) for v in lo], "max": [round(v, 3) for v in hi]}


def socket(joint):
    c, p, d = JOINTS[joint]
    L = layout(c)
    bore = c.housing_r + FIT
    half = c.motor.frame / 2 + 0.6
    t = DRIVER_T[c.motor.frame]
    preserve = [
        {"id": f"{joint}-sleeve", "shape": cyl(p, d, L["housing_bot"], L["cap_bot"], bore + SLEEVE, bore)},
        {"id": f"{joint}-ledge", "shape": cyl(p, d, L["housing_bot"] - LEDGE, L["housing_bot"], bore + SLEEVE,
                                              half * math.sqrt(2) + 1.0)},
    ]
    keep = [
        {"id": f"{joint}-drive", "shape": cyl(p, d, L["housing_bot"], L["cap_top"] + 14.0, bore)},
        {"id": f"{joint}-motor", "shape": axis_box(p, d, L["motor_bot"] - 1.0, L["housing_bot"], half)},
        {"id": f"{joint}-driver", "shape": axis_box(p, d, L["motor_bot"] - t - 2.0, L["motor_bot"] - 1.0, half)},
    ]
    return preserve, keep


def hub(joint, thickness):
    c, p, d = JOINTS[joint]
    ob = c.out_bearing
    br = (ob.id / 2 + c.motor.shaft_d / 2 + 2.0) / 2
    preserve = [{"id": f"{joint}-hub", "shape": cyl(p, d, 0.5, 0.5 + min(thickness, 5.0), br + 6.0, max(br - 6.0, 2.0))}]
    keep, supports = [], []
    # the bolt pattern in the plane normal to d
    u = (1, 0, 0) if d[0] == 0 else (0, 1, 0)
    v = [d[1] * u[2] - d[2] * u[1], d[2] * u[0] - d[0] * u[2], d[0] * u[1] - d[1] * u[0]]
    for i in range(4):
        a = math.pi / 4 + i * math.pi / 2
        q = [p[k] + br * (math.cos(a) * u[k] + math.sin(a) * v[k]) for k in range(3)]
        keep.append({"id": f"{joint}-bolt-{i}", "shape": cyl(q, d, -0.5, 0.5 + thickness + 1.0, 1.7)})
        keep.append({"id": f"{joint}-driver-access-{i}", "shape": cyl(q, d, 3.5, 0.5 + thickness + 45.0, 3.0)})
        supports.append({"region": f"{joint}-bolt-{i}"})
    # the drive below (and its cap) stays clear of the hub's back
    keep.append({"id": f"{joint}-below", "shape": cyl(p, d, -70.0, 0.0, c.housing_r + 12.0)})
    return preserve, keep, supports


# --------------------------------------------------------------------------- statics

def urdf_masses():
    """{link: (kg, com in metres, world)} from urdf/grip.urdf, frames placed by the joint origins."""
    root = ET.parse(os.path.join(ROOT, "urdf", "grip.urdf")).getroot()
    origin = {"base_link": (0.0, 0.0, 0.0)}
    joints = []
    for j in root.findall("joint"):
        xyz = tuple(float(v) for v in j.find("origin").get("xyz").split())
        joints.append((j.find("parent").get("link"), j.find("child").get("link"), xyz))
    for _ in range(len(joints)):
        for parent, child, xyz in joints:
            if parent in origin and child not in origin:
                origin[child] = tuple(origin[parent][i] + xyz[i] for i in range(3))
    out = {}
    for link in root.findall("link"):
        i = link.find("inertial")
        if i is None:
            continue
        m = float(i.find("mass").get("value"))
        com = tuple(float(v) for v in i.find("origin").get("xyz").split())
        o = origin.get(link.get("name"), (0.0, 0.0, 0.0))
        out[link.get("name")] = (m, tuple(o[k] + com[k] for k in range(3)))
    return out


CHAIN = ["base_link", "turret", "upper_arm", "forearm", "wrist_yoke", "wrist_link", "tool", "jaw_a", "jaw_b"]


def distal(masses, link):
    """Mass and centre of mass (m, world, home pose) of everything carried by `link`'s far joint."""
    i = CHAIN.index(link)
    bodies = [masses[n] for n in CHAIN[i + 1:] if n in masses]
    tool = masses.get("tool", (0, (0, 0, 0)))[1]
    bodies.append((PAYLOAD_KG, (tool[0], tool[1], tool[2] + 0.07)))
    m = sum(b[0] for b in bodies)
    com = tuple(sum(b[0] * b[1][k] for b in bodies) / m for k in range(3))
    return m, com


def link_loads(masses, link, efforts):
    """Load cases at the socket of the joint above `link`, in N and N·mm."""
    below, above, _ = LINKS[link]
    c, p, d = JOINTS[above]
    m, com = distal(masses, link)
    w = m * G_ACC * DYNAMIC
    # arm stretched out horizontally: the chain's up direction (+z at home) becomes horizontal,
    # gravity acts along -x in the link's home frame; lever from the socket to the distal COM
    lever_mm = max(com[2] * 1000.0 - p[2], 0.0)
    force = [-w, 0.0, 0.0]
    # moment of the distal weight about the socket, reacted by the drive into this link
    r = [com[0] * 1000.0 - p[0], com[1] * 1000.0 - p[1], lever_mm]
    mom = [r[1] * force[2] - r[2] * force[1], r[2] * force[0] - r[0] * force[2], r[0] * force[1] - r[1] * force[0]]
    effort = efforts[above] * 1000.0
    axis = [float(x) for x in d]
    lateral = [LATERAL_N * a for a in axis]
    region = f"{above}-sleeve"
    return [
        {"name": "reach", "loads": [{"region": region, "force": [round(v, 2) for v in force],
                                     "moment": [round(v, 1) for v in mom]}]},
        {"name": f"{above}-rated-torque", "loads": [{"region": region, "moment": [round(effort * a, 1) for a in axis]}]},
        {"name": "lateral", "loads": [{"region": region, "force": [round(v, 2) for v in lateral]}]},
    ], {"distalKg": m, "leverMm": lever_mm, "staticTorqueNm": abs(mom[1] if axis[1] else mom[0]) / 1000.0 / DYNAMIC}


def urdf_efforts():
    root = ET.parse(os.path.join(ROOT, "urdf", "grip.urdf")).getroot()
    out = {}
    for j in root.findall("joint"):
        lim = j.find("limit")
        if lim is not None and j.get("name").startswith("j"):
            out[j.get("name")] = float(lim.get("effort"))
    return out


# --------------------------------------------------------------------------- studies

def design_space(preserve, keep, margin=14.0):
    """Box hull of the hub and the socket, plus a margin, as the space to grow in."""
    pts = []
    for r in preserve:
        s = r["shape"]
        rr = s["r"]
        along = [abs(s["p1"][i] - s["p0"][i]) > 1e-9 for i in range(3)]
        for e in (s["p0"], s["p1"]):
            off = [0.0 if along[i] else rr for i in range(3)]
            pts.append([e[i] - off[i] for i in range(3)])
            pts.append([e[i] + off[i] for i in range(3)])
    lo = [min(p[i] for p in pts) - margin for i in range(3)]
    hi = [max(p[i] for p in pts) + margin for i in range(3)]
    return {"type": "box", "min": [round(v, 2) for v in lo], "max": [round(v, 2) for v in hi]}


def study(link, masses, efforts, max_elements=60000):
    below, above, hub_t = LINKS[link]
    h_pres, h_keep, supports = hub(below, hub_t)
    s_pres, s_keep = socket(above)
    preserve = h_pres + s_pres
    keep = h_keep + s_keep
    # the child link swings round the joint above: keep its side clear
    c, p, d = JOINTS[above]
    keep.append({"id": f"{above}-child-sweep", "shape": cyl(p, d, 0.0, 30.0, c.housing_r + 12.0)})
    cases, info = link_loads(masses, link, efforts)
    space = design_space(preserve, keep)
    return {
        "name": f"grip-{link.replace('_', '-')}",
        "description": f"GRIP {link.replace('_', ' ')}, generated by generative/chain.py: hub on {below}, "
                       f"socket for {above}; carries {info['distalKg']:.2f} kg past {above} "
                       f"({info['staticTorqueNm']:.1f} N·m static reaching out).",
        "designSpace": [space],
        "preserve": preserve,
        "keepOut": keep,
        "supports": supports,
        "loadCases": cases,
        "material": MATERIAL,
        "goal": {"safetyFactor": 2.0, "volumeFraction": 0.12},
        "minMember": 3.0,
        "symmetry": ["x"],
        "resolution": {"maxElements": max_elements},
        "chain": info,
    }


def _solve(s):
    os.environ.setdefault("LOADPATH_WORKERS", "1")
    from loadpath.spec import study_from_dict
    from loadpath.study import run_study
    return run_study(study_from_dict(s))


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["studies", "run", "report"])
    ap.add_argument("--links", default=",".join(LINKS))
    ap.add_argument("--iterations", type=int, default=2)
    ap.add_argument("--out", default=os.path.join(HERE, "out"))
    ap.add_argument("--jobs", type=int, default=3)
    args = ap.parse_args(argv)
    links = args.links.split(",")
    masses = urdf_masses()
    efforts = urdf_efforts()
    os.makedirs(os.path.join(HERE, "studies"), exist_ok=True)
    if args.cmd == "studies":
        for link in links:
            s = study(link, masses, efforts)
            with open(os.path.join(HERE, "studies", f"{link}.json"), "w") as fh:
                json.dump(s, fh, indent=1)
            print(link, s["chain"])
        return 0
    if args.cmd == "run":
        from loadpath.spec import study_from_dict
        from loadpath.study import run_study
        os.makedirs(args.out, exist_ok=True)
        history = []
        shipped_part_kg = {}
        import trimesh
        for link in links:
            mesh = trimesh.load(SHIPPED_STL.format(link))
            shipped_part_kg[link] = mesh.volume * MATERIAL["density"] / 1e6
        import multiprocessing as mp
        from concurrent.futures import ProcessPoolExecutor
        for it in range(args.iterations):
            row = {"iteration": it, "links": {}}
            studies = {}
            for link in links:
                studies[link] = study(link, masses, efforts)
                with open(os.path.join(HERE, "studies", f"{link}.json"), "w") as fh:
                    json.dump(studies[link], fh, indent=1)
            # the links are independent within an iteration (their loads use last
            # iteration's masses), so solve them side by side
            with ProcessPoolExecutor(args.jobs, mp_context=mp.get_context("spawn")) as ex:
                results = dict(zip(links, ex.map(_solve, [studies[l] for l in links])))
            for link in links:
                s = studies[link]
                result = results[link]
                if result.best is None:
                    row["links"][link] = {"error": "no outcome met the goal", "chain": s["chain"]}
                    continue
                o = result.outcomes[result.best]
                o.mesh.export(os.path.join(args.out, f"{link}.stl"))
                new_kg = o.report["mass"] / 1000.0
                m, com = masses[link]
                # swap the printed link's mass, keep the drives and hardware it carries
                masses[link] = (m - shipped_part_kg[link] + new_kg, com)
                row["links"][link] = {"printedG": o.report["mass"], "shippedG": shipped_part_kg[link] * 1000,
                                      "safetyFactor": o.report["safetyFactor"],
                                      "deflection": max(c["deflection"] for c in o.report["cases"]),
                                      "chain": s["chain"]}
                print(json.dumps({link: row["links"][link]}), flush=True)
            history.append(row)
            with open(os.path.join(args.out, "chain.json"), "w") as fh:
                json.dump(history, fh, indent=1)
        return 0
    if args.cmd == "report":
        return report(links, args.out)
    return 0


def report(links, out):
    """Shipped part vs generated part, each verified under the same study, plus
    the joint torques the arm needs with each set of link masses."""
    import trimesh
    from loadpath.spec import study_from_dict
    from loadpath.verify import verify_part

    masses0 = urdf_masses()
    masses1 = dict(masses0)
    rows = []
    for link in links:
        path = os.path.join(HERE, "studies", f"{link}.json")
        if not os.path.exists(path) or not os.path.exists(os.path.join(out, f"{link}.stl")):
            continue
        s = study_from_dict(json.load(open(path)))
        shipped = trimesh.load(SHIPPED_STL.format(link))
        ours = trimesh.load(os.path.join(out, f"{link}.stl"))
        r0 = verify_part(s, shipped, max_cells=120_000, contact_band=2.0)
        r1 = verify_part(s, ours, max_cells=120_000, contact_band=2.0)
        m, com = masses0[link]
        masses1[link] = (m - r0["mass"] / 1000 + r1["mass"] / 1000, com)
        rows.append({
            "link": link,
            "shipped": {"g": r0["mass"], "sf": r0["safetyFactor"],
                        "defl": {c["name"]: c["deflection"] for c in r0["cases"]}},
            "generated": {"g": r1["mass"], "sf": r1["safetyFactor"],
                          "defl": {c["name"]: c["deflection"] for c in r1["cases"]}},
        })
    torques = {}
    for name, masses in (("shipped", masses0), ("generated", masses1)):
        torques[name] = {}
        for link, joint in (("turret", "j2"), ("upper_arm", "j3")):
            m, com = distal(masses, link)
            p = JOINTS[joint][1]
            torques[name][joint] = m * G_ACC * max(com[2] * 1000 - p[2], 0) / 1000.0
    result = {"links": rows, "staticTorqueNm": torques}
    with open(os.path.join(out, "report.json"), "w") as fh:
        json.dump(result, fh, indent=1)
    print(json.dumps(result, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
