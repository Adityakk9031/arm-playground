# arm-playground

Project 1 of a 60-day Physical AI learning challenge: a robot arm in MuJoCo.

## Progress

| Day | Topic | Status |
|-----|-------|--------|
| 1 | Load the Franka Panda, inspect it, make it move | ✅ Completed |

## Day 1: Hello, Franka Panda ✅

Loads the Franka Emika Panda from [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie),
prints its structure, and moves two joints (shoulder and elbow) with smooth sine waves.

- `src/inspect_model.py`: prints bodies, joints (type, range), actuators (ctrl range, target) and keyframes, plus `nq`/`nv`/`nu`.
- `src/day01_hello_arm.py`: resets to the `home` keyframe and drives the arm. Opens the viewer, or runs headless with `--headless`.

### What I learned

- **`MjModel` vs `MjData`:** the model is the fixed robot description; the data is the live state that `mj_step` advances.
- **`qpos` vs `ctrl`:** `qpos` is where the joints *are*; `ctrl` is where you *ask* them to go. Dragging a Control
  slider moves the arm smoothly to a target. Dragging a Joint slider (while paused) teleports it, and on unpause
  physics pulls it back to the `ctrl` target.
- **Position actuators:** `ctrl` is a target angle, and a built-in PD controller produces the force to reach it.
  Each actuator's `ctrlrange` matches its joint limits.
- **Timestep and real time:** each `mj_step` is 2 ms of sim time; the viewer loop runs physics in batches and
  redraws at 60 fps so the UI stays responsive.
- **Keyframes:** `home` stores a matching `qpos` and `ctrl`, so the arm starts in a stable pose.
- **Passive vs managed viewer:** with `launch_passive` your code owns the loop (and the viewer's pause and
  Joint sliders don't work); with `launch` the viewer runs physics itself and all its controls work.

### Setup

Requires [uv](https://docs.astral.sh/uv/) and git.

```bash
uv sync                      # creates .venv (Python 3.12) with mujoco + numpy
bash scripts/setup_assets.sh # shallow-clones mujoco_menagerie/ (~2.4 GB; takes a few minutes)
```

### Run

```bash
.venv/bin/python src/inspect_model.py                      # inspect the model
.venv/bin/python src/day01_hello_arm.py --headless         # no window; prints joints every 500 steps
.venv/bin/mjpython src/day01_hello_arm.py                  # viewer, sine-wave motion (macOS needs mjpython)
.venv/bin/python src/day01_hello_arm.py --manual           # viewer, you drive the arm (plain python, not mjpython)
```

Headless options: `--steps N` (default 2000).

**Sine-wave viewer** (`mjpython`, no flag): the script owns the physics loop and rewrites the
actuator targets every step, so the Control sliders, Space and the Joint sliders do nothing.
Reset / Load key return to the home pose.

**Manual viewer** (`--manual`): the viewer runs physics itself and the script never writes `ctrl`.
- **Control** sliders (right panel) set actuator *targets* (`ctrl`); the arm moves there smoothly.
- **Pause** (button in the left panel, or Space after clicking on the robot scene); while paused,
  **Joint** sliders set joint *positions* (`qpos`) directly. Unpause and the arm springs back to its `ctrl` target.
- **Load key** returns to the home pose (plain Reset goes to the all-zero pose).
