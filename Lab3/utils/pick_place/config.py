"""
Robot-dependent configuration selector.

All numbers live in utils/pick_place/robots/<robot>.py (CONFIG dict).
`use("heal")` / `use("franka")` rebinds the module-level names below, which the
scene builder, planner and logger read at run time:

    TABLE, TRAY, CUBE, SPAWN, SPAWN_PROFILES, PLAN, CTRL, CHECK, HOME_TCP,
    ROBOT (name), ROBOT_MOD (the robot module)

Spawn profiles:
  'nominal' = designed spawn region; 'stress' = whole table top, including corners
  outside the reliable top-down workspace (used to provoke and study failures).
"""
import importlib
import os

import numpy as np

ROBOTS = ("heal", "franka")


def use(name):
    if name not in ROBOTS:
        raise ValueError(f"unknown robot {name!r}, choose from {ROBOTS}")
    mod = importlib.import_module(f"{__package__}.robots.{name}")
    g = globals()
    g.pop("CAMERA_POS", None)
    for k, v in mod.CONFIG.items():
        g[k] = v
    T = mod.CONFIG["TABLE"]
    g["SPAWN_PROFILES"] = {
        "nominal": mod.CONFIG["SPAWN"],
        "stress": dict(
            x_range=(T["center_xy"][0] - T["length"] / 2 + 0.03,
                     T["center_xy"][0] + T["length"] / 2 - 0.03),
            y_range=(T["center_xy"][1] - T["width"] / 2 + 0.03,
                     T["center_xy"][1] + T["width"] / 2 - 0.03),
            r_range=(0.0, 10.0), yaw_range=(-np.pi, np.pi), tray_keepout=0.0),
    }
    g["ROBOT"], g["ROBOT_MOD"] = name, mod
    return mod


use(os.environ.get("PICK_PLACE_ROBOT", "heal"))
