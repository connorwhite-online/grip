"""Place every part in a given joint pose.

Joint angles are in degrees, zero = upright home pose. Positive J2/J3/J5 tip the arm
toward +X.
"""
from build123d import Axis, Location, Pos, Rot

from . import arm

# (link, parent joint) chain; each joint rotates everything after it
CHAIN = ["base", "turret", "upper_arm", "forearm", "wrist_yoke", "wrist_link"]
JOINT_AXES = {
    "j1": ((0, 0, 0), (0, 0, 1)),
    "j2": ((0, 0, arm.G.z_j2), (0, 1, 0)),
    "j3": ((0, 0, arm.G.z_j3), (0, 1, 0)),
    "j4": ((0, arm.G.y_forearm, 0), (0, 0, 1)),
    "j5": ((0, 0, arm.G.z_j5), (0, 1, 0)),
    "j6": ((0, arm.G.y_forearm, 0), (0, 0, 1)),
}
ORDER = ["j1", "j2", "j3", "j4", "j5", "j6"]


def joint_transforms(pose):
    """Location applied to the parts moved by each joint (cumulative, in home-pose frame)."""
    pose = pose or [0] * 6
    locs = {}
    acc = Location()
    for j, q in zip(ORDER, pose):
        origin, direction = JOINT_AXES[j]
        # axis expressed in the home frame, then carried by the accumulated transform
        ax = Axis(origin, direction)
        acc = acc * Location(ax.position) * Location((0, 0, 0), ax.direction, q) * Location(-ax.position)
        locs[j] = acc
    return locs


def assembly_parts(pose=None, include_gripper=True):
    """{name: (shape, kind)} for the whole arm in `pose`."""
    locs = joint_transforms(pose)
    moved_by = {"base": None, "turret": "j1", "upper_arm": "j2", "forearm": "j3",
                "wrist_yoke": "j4", "wrist_link": "j5"}
    out = {}
    for name, fn in arm.LINKS.items():
        j = moved_by[name]
        shape = fn()
        out[name] = ((locs[j] * shape) if j else shape, "shell")
    drives = arm.drive_parts()
    # a drive's stationary half moves with its parent link, its output with the child
    parent_of = {"j1": None, "j2": "j1", "j3": "j2", "j4": "j3", "j5": "j4", "j6": "j5"}
    for j, parts in drives.items():
        for k, shape in parts.items():
            moving = k in ("output_flange", "output_pins")
            jj = j if moving else parent_of[j]
            kind = "motor" if k == "motor" else ("vendor" if k in ("ring_pins", "output_pins", "bearings") else "drive")
            if k in ("disc_a", "disc_b", "eccentric", "ring_pins", "bearings", "output_pins"):
                continue   # hidden inside; skip for previews
            out[f"{j}_{k}"] = ((locs[jj] * shape) if jj else shape, kind)
    if include_gripper:
        try:
            from .gripper import gripper_parts
            for k, (shape, kind) in gripper_parts().items():
                out[f"gripper_{k}"] = (locs["j6"] * shape, kind)
        except ImportError:
            pass
    return out
