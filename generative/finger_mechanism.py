"""
The GRIP finger, designed as a compliant mechanism (loadpath.mechanism).

The hand-drawn finger in hand.py is three blocks on flexure hinges. Here the
optimizer decides where the finger bends: the tendon pulls the tip toward
the root along the palm side, and the finger should curl its distal pad
toward the object as far as it can, against a spring that stands in for the
object (so the result also squeezes, not only moves). The profile is
designed in the finger's side plane and extruded across its width, which is
how a fin-ray finger is built and what prints well in TPU.

Fixed: the root block (bolted into the palm) and the distal phalanx (pad,
nail seat, tendon knot). The tendon runs down the palm side, 2-4.5 mm in.

    python generative/finger_mechanism.py         # writes hand/finger-mech.stl + .json
"""
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from hand import FINGER, OUT  # noqa: E402

W = FINGER["width"]
L = FINGER["root"] + sum(FINGER["segments"])      # 96 mm
Y_PALM, Y_BACK = 6.5, -10.0                       # palm face, and the deepest the back may go
SKIN = 1.6
WRAP_BACK = -22.0
CONTACTS = (20.0, 36.0, 52.0)     # where an object may press, mm up the finger
TENDON_N = 30.0
TPU = {"name": "tpu-95a", "E": 26.0, "nu": 0.45, "strength": 25.0, "density": 1.2}


def box(lo, hi):
    return {"type": "box", "min": lo, "max": hi}


def study():
    x0, x1 = -W / 2, W / 2
    return {
        "name": "grip-finger-mechanism",
        "designSpace": [box([x0, Y_BACK, 0], [x1, Y_PALM, L])],
        "preserve": [
            {"id": "root", "shape": box([x0, Y_BACK, 0], [x1, Y_PALM, FINGER["root"]])},
            # the distal phalanx: pad, nail seat and the tendon's knot. Fixed,
            # so the optimizer designs everything that makes it curl. (A
            # preserved palm-side skin the full length put the structure's
            # neutral axis right on the tendon: it compressed, it never curled.)
            {"id": "tip", "shape": box([x0, Y_PALM - 7.0, L - 24.0], [x1, Y_PALM, L])},
        ],
        "keepOut": [],
        "supports": [{"region": "root"}],
        "loadCases": [
            # the tendon's knot sits in the tip block, on the palm side, pulling toward the root
            {"name": "tendon", "loads": [{"shape": box([x0, Y_PALM - 4.5, L - 3.0], [x1, Y_PALM - 2.0, L + 0.1]),
                                          "force": [0.0, 0.0, -TENDON_N]}]},
            # the distal pad, pushed toward the object
            {"name": "pad", "loads": [{"shape": box([x0, Y_PALM - 0.5, L - 22.0], [x1, Y_PALM + 0.1, L - 4.0]),
                                       "force": [0.0, 1.0, 0.0]}]},
        ],
        "material": TPU,
        "goal": {"volumeFraction": 0.3},
        "minMember": 1.6,
        "resolution": {"pitch": 1.0},
    }


def study_wrap():
    """The fin-ray question, asked of the optimizer: when an object presses
    the middle of the palm face, the fingertip should swing IN toward it
    (wrap), not away. Root held (the tendon has already closed the finger at
    its root flexure); a palm-side skin the pads sit on; a small tip block."""
    x0, x1 = -W / 2, W / 2
    s = study()
    s["name"] = "grip-finger-wrap"
    # a fin ray needs depth at its root to lever with (commercial ones are
    # ~30 mm deep): the 16 mm of the tendon finger gave the optimizer nothing
    s["designSpace"] = [box([x0, WRAP_BACK, 0], [x1, Y_PALM, L])]
    s["preserve"] = [
        {"id": "root", "shape": box([x0, WRAP_BACK, 0], [x1, Y_PALM, FINGER["root"]])},
        {"id": "skin", "shape": box([x0, Y_PALM - SKIN, FINGER["root"]], [x1, Y_PALM, L])},
        {"id": "tip", "shape": box([x0, Y_PALM - 5.0, L - 6.0], [x1, Y_PALM, L])},
    ]
    # contact anywhere along the finger: three places, each paired with the
    # tip's inward travel (a fin ray wraps wherever it is pressed)
    s["loadCases"] = [
        {"name": f"contact-{i}", "loads": [{"shape": box([x0, Y_PALM - 0.5, z0], [x1, Y_PALM + 0.1, z0 + 14.0]),
                                            "force": [0.0, -10.0, 0.0]}]}
        for i, z0 in enumerate(CONTACTS)] + [
        {"name": "tip", "loads": [{"shape": box([x0, Y_PALM - 0.5, L - 14.0], [x1, Y_PALM + 0.1, L]),
                                   "force": [0.0, 1.0, 0.0]}]},
    ]
    return s


def main_wrap():
    from loadpath.domain import build_domain
    from loadpath.mechanism import MechanismOptions, optimize_mechanism
    from loadpath.reconstruct import ReconstructOptions, reconstruct
    from loadpath.spec import study_from_dict

    st = study_from_dict(study_wrap())
    d = build_domain(st)
    opts = MechanismOptions(volume_fraction=0.3, filter_radius=1.8, modulus=TPU["E"], k_in=5.0, k_out=5.0,
                            extrusion=0, max_iterations=160,
                            pairs=[(i, len(CONTACTS)) for i in range(len(CONTACTS))])
    t0 = time.time()
    r = optimize_mechanism(d, TPU["nu"], opts,
                           on_iter=lambda rec, _x: rec["iteration"] % 20 == 0 and print(json.dumps(rec), flush=True))
    mesh, info = reconstruct(st, d, r.xphys, ReconstructOptions())
    mesh.export(os.path.join(OUT, "finger-wrap.stl"))
    out = {"tipInwardMm": r.history[-1]["outputs"], "contactN": 10.0, "grams": mesh.volume * TPU["density"] / 1000,
           "iterations": r.iterations, "seconds": time.time() - t0, "watertight": info["watertight"]}
    json.dump(out, open(os.path.join(OUT, "finger-wrap.json"), "w"), indent=1)
    print(json.dumps(out))


def main():
    from loadpath.domain import build_domain
    from loadpath.mechanism import MechanismOptions, optimize_mechanism
    from loadpath.reconstruct import ReconstructOptions, reconstruct
    from loadpath.spec import study_from_dict

    st = study_from_dict(study())
    d = build_domain(st)
    opts = MechanismOptions(volume_fraction=0.3, filter_radius=1.8, modulus=TPU["E"], k_in=50.0, k_out=2.0,
                            extrusion=0, max_iterations=160)
    t0 = time.time()
    r = optimize_mechanism(d, TPU["nu"], opts,
                           on_iter=lambda rec, _x: rec["iteration"] % 20 == 0 and print(json.dumps(rec), flush=True))
    mesh, info = reconstruct(st, d, r.xphys, ReconstructOptions())
    os.makedirs(OUT, exist_ok=True)
    mesh.export(os.path.join(OUT, "finger-mech.stl"))
    out = {"padDisplacementMm": r.output_displacement, "tendonN": TENDON_N, "grams": mesh.volume * TPU["density"] / 1000,
           "iterations": r.iterations, "seconds": time.time() - t0, "watertight": info["watertight"]}
    json.dump(out, open(os.path.join(OUT, "finger-mech.json"), "w"), indent=1)
    print(json.dumps(out))


if __name__ == "__main__":
    main_wrap() if sys.argv[1:] == ["wrap"] else main()
