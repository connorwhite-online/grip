"""Stress maps of the v0.3 links: re-verify each delivered STL under its
study's load cases and render worst-case von Mises over strength, on the
exaggerated deflected shape. Writes generative/renders/sim-<link>.png."""
import json, os, sys

import trimesh
from loadpath.render import render_stress
from loadpath.spec import study_from_dict
from loadpath.verify import verify_part

HERE = os.path.dirname(os.path.abspath(__file__))


def main(links):
    out = {}
    for l in links:
        study = study_from_dict(json.load(open(os.path.join(HERE, "v3", "studies", f"{l}.json"))), base_dir=os.path.join(HERE, "v3", "studies"))
        mesh = trimesh.load(os.path.join(HERE, "v3", "out", f"{l}.stl"))
        rep = verify_part(study, mesh, max_cells=120_000, fields=True)
        f = rep.pop("fields")
        render_stress(mesh, f, os.path.join(HERE, "renders", f"sim-{l}.png"), study.material.strength, safety_factor=study.goal.safety_factor or 1.0,
                      title=f"{l.replace('_', ' ')}: {rep['mass']:.0f} g, SF {rep['safetyFactor']:.2f}")
        out[l] = {"g": rep["mass"], "sf": rep["safetyFactor"],
                  "cases": {c["name"]: {"peakMPa": round(c["peakStress"], 1), "deflMm": round(c["deflection"], 3)}
                            for c in rep["cases"]}}
        print(l, json.dumps(out[l]), flush=True)
    json.dump(out, open(os.path.join(HERE, "v3", "out", "sims.json"), "w"), indent=1)


if __name__ == "__main__":
    main(sys.argv[1].split(",") if len(sys.argv) > 1 else
         ["turret", "upper_arm", "forearm", "wrist_yoke", "wrist_link"])
