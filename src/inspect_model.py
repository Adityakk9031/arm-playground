"""Day 1: inspect a MuJoCo model without simulating anything.

Concepts used here
- MjModel: the *static* description of the world (bodies, joints, actuators, masses,
  timestep...). Loaded once from XML, never changes while simulating.
- MjData: the *dynamic* state (positions, velocities, controls, forces). Every
  mj_step() changes it. One MjModel can back many MjData objects.
- qpos / ctrl: qpos is where the joints ARE (state). ctrl is what we COMMAND
  (input to the actuators). Actuators turn ctrl into forces on joints.
- timestep: seconds of simulated time per mj_step().
- keyframe: a named snapshot of (qpos, ctrl, ...) stored in the XML, e.g. "home".

Run: python src/inspect_model.py
"""

from pathlib import Path

import mujoco

SCENE = Path(__file__).resolve().parent.parent / "mujoco_menagerie/franka_emika_panda/scene.xml"

# mjtJoint is an enum; map the integer in model.jnt_type to a readable name.
JOINT_TYPES = {
    mujoco.mjtJoint.mjJNT_FREE: "free",    # 6 DoF floating body (7 qpos: xyz + quaternion)
    mujoco.mjtJoint.mjJNT_BALL: "ball",    # 3 DoF rotation (4 qpos: quaternion)
    mujoco.mjtJoint.mjJNT_SLIDE: "slide",  # 1 DoF prismatic (linear)
    mujoco.mjtJoint.mjJNT_HINGE: "hinge",  # 1 DoF revolute (rotary)
}


def name_of(model, objtype, idx):
    """Look up an object's name by index; unnamed objects return '<unnamed>'."""
    return mujoco.mj_id2name(model, objtype, idx) or "<unnamed>"


def main():
    model = mujoco.MjModel.from_xml_path(str(SCENE))

    print(f"Model: {SCENE.name}   timestep = {model.opt.timestep} s")
    # nq: length of qpos = number of position coordinates. Can exceed nv because a
    #     free/ball joint stores orientation as a quaternion (4 numbers, 3 DoF).
    # nv: length of qvel = number of degrees of freedom (velocity coordinates).
    # nu: length of ctrl = number of actuators, i.e. things we can command.
    print(f"nq = {model.nq} (position coords)   nv = {model.nv} (degrees of freedom)   "
          f"nu = {model.nu} (actuators)\n")

    print(f"BODIES ({model.nbody})")
    for i in range(model.nbody):
        parent = name_of(model, mujoco.mjtObj.mjOBJ_BODY, model.body_parentid[i])
        print(f"  {i:2d}  {name_of(model, mujoco.mjtObj.mjOBJ_BODY, i):<12} parent = {parent}")

    print(f"\nJOINTS ({model.njnt})")
    print(f"  {'id':>2}  {'name':<14} {'type':<6} {'qpos idx':>8}  {'range (rad or m)':<20}")
    for i in range(model.njnt):
        lo, hi = model.jnt_range[i]
        rng = f"[{lo:+.3f}, {hi:+.3f}]" if model.jnt_limited[i] else "unlimited"
        # jnt_qposadr says where this joint's value lives inside the qpos vector.
        print(f"  {i:2d}  {name_of(model, mujoco.mjtObj.mjOBJ_JOINT, i):<14} "
              f"{JOINT_TYPES[mujoco.mjtJoint(model.jnt_type[i])]:<6}{model.jnt_qposadr[i]:>8}  {rng}")

    print(f"\nACTUATORS ({model.nu})")
    print(f"  {'id':>2}  {'name':<10} {'ctrl range':<20} drives")
    for i in range(model.nu):
        lo, hi = model.actuator_ctrlrange[i]
        # actuator_trnid[i, 0] is the id of the thing the actuator pushes on.
        # actuator_trntype says whether that's a joint or a tendon (the gripper uses one).
        target_id = model.actuator_trnid[i, 0]
        if model.actuator_trntype[i] == mujoco.mjtTrn.mjTRN_JOINT:
            target = "joint " + name_of(model, mujoco.mjtObj.mjOBJ_JOINT, target_id)
        else:
            target = "tendon " + name_of(model, mujoco.mjtObj.mjOBJ_TENDON, target_id)
        print(f"  {i:2d}  {name_of(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i):<10} "
              f"[{lo:+9.3f}, {hi:+9.3f}]  {target}")

    print(f"\nKEYFRAMES ({model.nkey})")
    for i in range(model.nkey):
        name = name_of(model, mujoco.mjtObj.mjOBJ_KEY, i)
        # key_qpos holds one full qpos vector per keyframe.
        print(f"  {i}  {name}: qpos = {[round(float(x), 3) for x in model.key_qpos[i]]}")


if __name__ == "__main__":
    main()
