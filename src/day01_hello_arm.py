"""Day 1: make the Franka Panda arm move with sine-wave commands.

Concepts used here
- MjModel vs MjData: `model` is the fixed description of the robot; `data` is the
  live state we mutate (data.qpos, data.ctrl, data.time). mj_step(model, data)
  advances `data` by one timestep.
- qpos vs ctrl: qpos = where the joints are right now (read it). ctrl = what we ask
  the actuators to do (write it). We never set qpos during the run; physics does.
- Actuators: the Panda's 7 arm actuators are *position* actuators, so ctrl[i] is a
  TARGET ANGLE in radians. An internal spring-damper (PD controller) pushes the joint
  toward it. ctrl[7] is the gripper (0..255), which we leave alone.
- timestep: model.opt.timestep seconds pass per mj_step. Real time needs us to
  take 1/timestep steps per wall-clock second, so we sleep to match.
- Keyframe: "home" is a named pose stored in the XML. Resetting to it gives us a
  known starting qpos AND matching ctrl, so the arm doesn't sag at t=0.

Two viewer modes:
  default   our loop (launch_passive) drives the sine waves. Reset / Load key go home.
  --manual  the viewer's own loop (launch) runs physics and we never touch ctrl:
            Control sliders set targets (ctrl); Space pauses; while paused the Joint
            sliders set positions (qpos) directly; "Load key" returns to home.

Run: mjpython src/day01_hello_arm.py            (sine waves; launch_passive needs mjpython)
     python   src/day01_hello_arm.py --manual   (drive it yourself; plain python here)
     python   src/day01_hello_arm.py --headless (no window, prints joint positions)
"""

import argparse
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

SCENE = Path(__file__).resolve().parent.parent / "mujoco_menagerie/franka_emika_panda/scene.xml"

# Actuator indices to wiggle: 1 = shoulder (joint2), 3 = elbow (joint4).
# Each gets (frequency in Hz, desired amplitude in rad).
WIGGLE = {1: (0.25, 0.5), 3: (0.40, 0.6)}
FRAME_DT = 1 / 60  # viewer refresh period: physics runs in batches between redraws


def make_controller(model, home_ctrl):
    """Return ctrl_at(t): a full ctrl vector of sine waves centered on the home pose."""
    lo, hi = model.actuator_ctrlrange[:, 0], model.actuator_ctrlrange[:, 1]
    # Shrink each amplitude so center +/- amp stays inside the actuator's ctrl range,
    # with 10% margin. The clamp only activates when the requested amplitude exceeds
    # 90% of the room to the nearer joint limit. Elbow: home -1.57, range -3.07..-0.07,
    # room 1.5, cap 1.35, so 0.6 passes unchanged but requesting 3.0 gets capped to ~1.35.
    amps = {}
    for i, (_, amp) in WIGGLE.items():
        room = min(hi[i] - home_ctrl[i], home_ctrl[i] - lo[i])
        amps[i] = min(amp, 0.9 * room)
    print("amplitudes:", amps)

    def ctrl_at(t):
        ctrl = home_ctrl.copy()  # joints we don't wiggle hold their home target
        for i, (freq, _) in WIGGLE.items():
            # Sine is smooth (continuous velocity), so the arm never gets jerked.
            ctrl[i] = home_ctrl[i] + amps[i] * np.sin(2 * np.pi * freq * t)
        return np.clip(ctrl, lo, hi)  # belt and braces: never exceed ctrlrange

    return ctrl_at


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true", help="no viewer; print joints")
    parser.add_argument("--steps", type=int, default=2000, help="headless step count")
    parser.add_argument("--manual", action="store_true",
                        help="no sine waves; the viewer's Control sliders set the targets")
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)

    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    mujoco.mj_resetDataKeyframe(model, data, key_id)  # sets qpos, qvel, ctrl, time
    mujoco.mj_forward(model, data)  # recompute derived quantities (body poses, etc.)
    ctrl_at = make_controller(model, data.ctrl.copy())

    if args.headless:
        for step in range(1, args.steps + 1):
            data.ctrl[:] = ctrl_at(data.time)
            mujoco.mj_step(model, data)
            if step % 500 == 0:
                # qpos[:7] = the 7 arm joints (qpos[7:9] are the two gripper fingers).
                print(f"step {step:5d}  t={data.time:5.2f}s  qpos={np.round(data.qpos[:7], 3)}")
        return

    if args.manual:
        # Manual mode: the script must NOT write data.ctrl, because the Control sliders
        # are now the only source of actuator targets. Any write from our code would
        # overwrite your slider value on the next step and the arm would ignore you.
        # With no ctrl to write we have no reason to own the loop, so hand it to the
        # managed viewer: it runs physics itself, and its Space (pause), Joint sliders
        # (set qpos directly, only while paused) and Control sliders all work as built.
        # It blocks until you close the window. Run it with plain `python`: launch()
        # draws from the main thread, which mjpython reserves for itself.
        mujoco.viewer.launch(model, data)
        return

    # launch_passive: the viewer only draws and handles clicks; WE own the physics
    # loop. The catch: in passive mode the viewer ignores Space (pause) and keeps its
    # Joint sliders greyed out, so pausing and posing by hand need --manual.
    with mujoco.viewer.launch_passive(model, data) as viewer:
        last_time = data.time
        wall_start = time.perf_counter()
        while viewer.is_running():
            frame_start = time.perf_counter()
            # sync() is two-way (applies UI edits to `data`, then redraws it) and holds
            # a lock the window's UI thread needs to handle your clicks. Call it ONCE
            # per frame; calling it every 2 ms step starves the UI and buttons freeze.
            viewer.sync()

            if data.time < last_time:  # Reset/Load key: back to the home pose
                mujoco.mj_resetDataKeyframe(model, data, key_id)
                mujoco.mj_forward(model, data)
                wall_start = time.perf_counter()
                print("reset -> home pose")

            # Real-time pacing: step physics until sim time catches up with wall time
            # (~8 steps of 2 ms per 60 Hz frame). Tracking total elapsed time, not a
            # fixed dt per frame, keeps the error from accumulating.
            wall_now = time.perf_counter() - wall_start
            if wall_now - data.time > 0.2:  # far behind (window dragged): don't sprint
                wall_start = time.perf_counter() - data.time
                wall_now = data.time + FRAME_DT
            while data.time < wall_now:
                data.ctrl[:] = ctrl_at(data.time)
                mujoco.mj_step(model, data)
            last_time = data.time

            # Sleep out the rest of the frame: this gap is when the UI gets the lock.
            time.sleep(max(0.0, FRAME_DT - (time.perf_counter() - frame_start)))


if __name__ == "__main__":
    main()
