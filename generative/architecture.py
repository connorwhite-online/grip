"""
Architecture trade study for GRIP: what moving the wrist motors inboard, belt
wrist reductions and a shoulder counterbalance do to the torque the shoulder
(J2) and elbow (J3) need, and so to which drives they can use.

Static torques with the arm reaching out horizontally, from the URDF masses
(link frames placed by the joint origins), times a 1.5 dynamic factor, against
each drive's rating (holding torque x ratio x 0.8, as in urdf.py).

    python generative/architecture.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from chain import JOINTS, urdf_masses  # noqa: E402

G = 9.81
DYN = 1.5
PAYLOAD = (1.0, 0.50)             # kg, distance of the payload from J2 along the arm (m), reach 500 mm
RATING = {"NEMA 23 + 30:1 cycloidal": 30.0, "NEMA 17 60 mm + 25:1 cycloidal": 13.0}
WRIST_MOTOR_KG = 0.26             # NEMA 17 34 mm + its driver board
WRIST_GEARBOX_KG = 0.11           # printed 15:1 cycloidal: housing, cap, discs, flange, bearings, pins
BELT_STAGE_KG = 0.035             # GT2 pulleys, idler, belt per wrist joint at the joint end
GENERATED_SAVING_KG = {"upper_arm": 0.041, "forearm": 0.010, "wrist_yoke": 0.039, "wrist_link": -0.003}


def torques(items):
    """items: [(kg, height above J2 along the arm in m)] -> (J2, J3) static N·m."""
    z3 = (JOINTS["j3"][1][2] - JOINTS["j2"][1][2]) / 1000.0
    t2 = sum(m * G * max(h, 0.0) for m, h in items)
    t3 = sum(m * G * max(h - z3, 0.0) for m, h in items)
    return t2, t3


def baseline_items():
    masses = urdf_masses()
    z2 = JOINTS["j2"][1][2] / 1000.0
    items = []
    for link in ("upper_arm", "forearm", "wrist_yoke", "wrist_link", "tool", "jaw_a", "jaw_b"):
        m, com = masses[link]
        items.append((m, com[2] - z2))
    items.append(PAYLOAD)
    return items, masses


def main():
    items, masses = baseline_items()
    z2 = JOINTS["j2"][1][2] / 1000.0
    z3 = JOINTS["j3"][1][2] / 1000.0
    wrist_z = {j: JOINTS[j][1][2] / 1000.0 - z2 for j in ("j4", "j5", "j6")}
    options = {}
    options["A. as built"] = list(items)
    gen = list(items) + [(-kg, masses[l][1][2] - z2) for l, kg in GENERATED_SAVING_KG.items()]
    options["B. generated links"] = gen
    # C: wrist motors moved to just past the elbow, belts to the joints; the
    # printed wrist gearboxes become belt stages at the joints.
    moved = list(gen)
    for j, h in wrist_z.items():
        moved.append((-(WRIST_MOTOR_KG + WRIST_GEARBOX_KG), h))
        moved.append((BELT_STAGE_KG, h))
        moved.append((WRIST_MOTOR_KG, (z3 - z2) + 0.045))
    options["C. B + wrist motors at the elbow, belt wrist"] = moved
    rows = []
    for name, it in options.items():
        t2, t3 = torques(it)
        rows.append((name, t2, t3, sum(m for m, _ in it)))
    t2c, t3c = rows[-1][1], rows[-1][2]
    payload_t2 = PAYLOAD[0] * G * PAYLOAD[1]
    # D: a spring at J2 carrying 85% of the arm's own weight torque (not the payload's)
    t2d = payload_t2 + 0.15 * (t2c - payload_t2)
    rows.append(("D. C + spring counterbalance at J2", t2d, t3c, rows[-1][3]))
    print(f"{'option':48s} {'J2 static':>9s} {'J2 x1.5':>8s} {'J3 static':>9s} {'J3 x1.5':>8s}  J2 drive that fits")
    for name, t2, t3, _ in rows:
        fits = [d for d, r in sorted(RATING.items(), key=lambda kv: kv[1]) if DYN * t2 <= r]
        print(f"{name:48s} {t2:9.2f} {DYN * t2:8.2f} {t3:9.2f} {DYN * t3:8.2f}  {fits[0] if fits else 'none'}")


if __name__ == "__main__":
    main()
