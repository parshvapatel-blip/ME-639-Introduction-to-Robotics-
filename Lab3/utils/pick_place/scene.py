"""
Builds the full pick-and-place scene programmatically with MjSpec:

    robot + gripper + 'tcp' site       (utils/pick_place/robots/<robot>.py)
  + table, free-floating cube, open-top tray, floor, lights, cameras

`python -m utils.pick_place.scene --robot franka --save` also writes the compiled scene to
robot_descriptions/<robot>_pick_place_scene.xml for inspection.
"""
import os

import mujoco
import numpy as np

from . import config as C

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
DESC_DIR = os.path.join(REPO_ROOT, "robot_descriptions")


def _box(body, name, size, pos, rgba, **kw):
    g = body.add_geom()
    g.name = name
    g.type = mujoco.mjtGeom.mjGEOM_BOX
    g.size = list(size)
    g.pos = list(pos)
    g.rgba = list(rgba)
    for k, v in kw.items():
        setattr(g, k, v)
    return g


def build_spec(table=None, tray=None, cube=None):
    table = table or C.TABLE
    tray = tray or C.TRAY
    cube = cube or C.CUBE

    arm = C.ROBOT_MOD.build_arm_spec(C)          # robot + gripper + 'tcp' site
    arm.option.timestep = C.CTRL["sim_timestep"]
    arm.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    arm.option.impratio = 10.0
    arm.option.noslip_iterations = 10

    wb = arm.worldbody

    # --- table ----------------------------------------------------------- #
    tx, ty = table["center_xy"]
    th = table["thickness"]
    tb = wb.add_body()
    tb.name = "table"
    tb.pos = [tx, ty, table["height"] - th / 2]
    _box(tb, "table_top", (table["length"] / 2, table["width"] / 2, th / 2), (0, 0, 0),
         table["rgba"], friction=[1.0, 0.005, 0.0001], contype=1 | 8, conaffinity=1 | 8)
    # legs (visual only) down to the floor plane at z = 0 (base plane)
    leg_h = max(table["height"] - th, 0.0)
    if leg_h > 1e-3:
        for i, (sx, sy) in enumerate([(1, 1), (1, -1), (-1, 1), (-1, -1)]):
            _box(tb, f"table_leg{i}", (0.015, 0.015, leg_h / 2),
                 (sx * (table["length"] / 2 - 0.03), sy * (table["width"] / 2 - 0.03),
                  -th / 2 - leg_h / 2), table["rgba"], contype=0, conaffinity=0)

    # --- tray (open top box) ---------------------------------------------- #
    ix, iy = tray["inner_size"]
    wt, wh, ft = tray["wall_thickness"], tray["wall_height"], tray["floor_thickness"]
    trb = wb.add_body()
    trb.name = "tray"
    trb.pos = [tray["center_xy"][0], tray["center_xy"][1], table["height"]]
    rgba = tray["rgba"]
    tk = dict(contype=1 | 8, conaffinity=1 | 8)
    _box(trb, "tray_floor", (ix / 2 + wt, iy / 2 + wt, ft / 2), (0, 0, ft / 2), rgba, **tk)
    zc = ft + wh / 2
    _box(trb, "tray_wall_px", (wt / 2, iy / 2 + wt, wh / 2), (ix / 2 + wt / 2, 0, zc), rgba, **tk)
    _box(trb, "tray_wall_nx", (wt / 2, iy / 2 + wt, wh / 2), (-ix / 2 - wt / 2, 0, zc), rgba, **tk)
    _box(trb, "tray_wall_py", (ix / 2, wt / 2, wh / 2), (0, iy / 2 + wt / 2, zc), rgba, **tk)
    _box(trb, "tray_wall_ny", (ix / 2, wt / 2, wh / 2), (0, -iy / 2 - wt / 2, zc), rgba, **tk)

    # --- cube --------------------------------------------------------------- #
    s = cube["half_size"]
    cb = wb.add_body()
    cb.name = "cube"
    cb.pos = [tx, ty, table["height"] + s + 1e-3]
    fj = cb.add_freejoint()
    fj.name = "cube_free"
    g = _box(cb, "cube_geom", (s, s, s), (0, 0, 0), cube["rgba"],
             friction=list(cube["friction"]), condim=cube.get("condim", 6), priority=1,
             solref=[0.004, 1], solimp=[0.95, 0.99, 0.001, 0.5, 2])
    g.mass = cube["mass"]
    # arm XML uses inertiafromgeom="false" -> give the cube explicit inertia
    cb.mass = cube["mass"]
    I = cube["mass"] * (2 * s) ** 2 / 6.0
    cb.inertia = [I, I, I]
    cb.ipos = [0, 0, 0]
    cb.iquat = [1, 0, 0, 0]
    cb.explicitinertial = True

    # --- cosmetics ----------------------------------------------------------- #
    if not any(g.name == "floor" for g in arm.geoms):
        fl = wb.add_geom()
        fl.name, fl.type, fl.size = "floor", mujoco.mjtGeom.mjGEOM_PLANE, [3, 3, 0.05]
        fl.rgba, fl.contype, fl.conaffinity = [0.25, 0.3, 0.35, 1], 0, 0
    li = wb.add_light()
    li.pos = [0.4, 0, 2.0]
    li.dir = [0, 0, -1]
    li.diffuse = [0.8, 0.8, 0.8]
    cam = wb.add_camera()
    cam.name = "overview"
    cam.pos = list(C.CAMERA_POS) if hasattr(C, "CAMERA_POS") else [1.45, 0.75, 0.95]
    cam.mode = mujoco.mjtCamLight.mjCAMLIGHT_TARGETBODY
    cam.targetbody = "table"
    arm.visual.global_.offwidth = 1280
    arm.visual.global_.offheight = 720
    return arm


def build_model(**kw):
    spec = build_spec(**kw)
    model = spec.compile()
    return model, spec


def arm_indices(model):
    R = C.ROBOT_MOD
    qidx = np.array([model.jnt_qposadr[model.joint(j).id] for j in R.ARM_JOINTS])
    vidx = np.array([model.jnt_dofadr[model.joint(j).id] for j in R.ARM_JOINTS])
    aidx = np.array([model.actuator(n).id for n in R.ARM_ACTUATORS])
    return qidx, vidx, aidx


if __name__ == "__main__":
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("--robot", choices=C.ROBOTS, default="heal")
    p.add_argument("--save", action="store_true")
    p.add_argument("--view", action="store_true")
    a = p.parse_args()
    C.use(a.robot)
    m, spec = build_model()
    print(f"compiled: nq={m.nq} nv={m.nv} nu={m.nu} ngeom={m.ngeom}")
    if a.save:
        out = os.path.join(DESC_DIR, f"{a.robot}_pick_place_scene.xml")
        open(out, "w").write(spec.to_xml())
        print("saved", out)
    if a.view:
        import mujoco.viewer
        mujoco.viewer.launch(m)
