# Day 1: Panda arm moving in MuJoCo

## What I built
- src/inspect_model.py: prints the Panda's bodies, joints, actuators and keyframes.
- src/day01_hello_arm.py: loads the Panda, resets to the home pose, and swings the shoulder and elbow with sine-wave targets. Modes: viewer (mjpython), --manual (drive it with sliders), --headless (prints joint angles).

## Comprehension check

1. What's the difference between MjModel and MjData?
MjModel is the blueprint: bodies, joints, masses, limits, timestep. It never changes while the simulation runs. MjData is the live state: joint positions (qpos), speeds (qvel), commands (ctrl) and time. It changes every step.

2. On the Panda, what does d.ctrl[3] mean, and how is that different from the bat's ctrl?
d.ctrl[3] is the command for actuator 4, which drives joint 4 (the elbow). The Panda uses position actuators, so ctrl is a target angle in radians: "go to this angle and hold it." The bat uses a motor actuator, so its ctrl is a torque (force): "push this hard." Same variable, different meaning depending on the actuator type.

3. What does one call to mj_step do?
It moves the whole simulation forward by one timestep, 0.002 s (2 milliseconds). It computes all the forces (gravity, actuators, contacts) and then updates every joint's position, velocity, and the simulation time.

4. Why does the code reset to the "home" keyframe at the start?
To start from a known, sensible pose. The home keyframe sets both the joint positions and the actuator targets to matching values, so the motors are already aiming where the arm is. Without it, everything starts at zero: the arm is stretched out and jerks toward the first targets.

5. Why are there 8 actuators but 9 joints?
9 joints = 7 arm joints + 2 finger joints. But one actuator drives both fingers together through a tendon, so there are 7 arm actuators + 1 gripper actuator = 8.

6. Why does viewer.sync() run only once per frame and not every step?
sync() holds a lock that the viewer window needs to handle clicks and sliders. Calling it after every 2 ms step (500 times a second) kept the lock busy almost all the time, so the UI froze. Calling it once per frame at 60 fps, running about 8 physics steps per frame and sleeping for the rest, leaves gaps for the UI.

7. Why did the fast shoulder fall short of its target?
A position actuator works like a spring with limited force pulling the joint toward the target. When the target moves too fast, the joint lags behind and the target turns back before the joint arrives, so the swing gets smaller. Gravity takes part of the force budget on one side, so it falls short unevenly. The gap between target and actual is called tracking error.

8. Why did the arm crash even though every joint stayed inside its limits?
The safety cap only protects each joint against its own limits, one joint at a time. It knows nothing about the other links of the arm or the floor, so a combination of legal joint angles can still make the arm hit itself or the ground. Real robots need collision checking and motion planning on top of joint limits.

Weak spots to reread: #3 (timestep), #5 (tendon), #6 (the viewer lock).

## Experiments

1. Faster shoulder (0.25 to 1.0 Hz): the shoulder moved faster but fell short of its target, more on one side. Position actuators can't follow fast targets: tracking error, plus gravity.

2. Bigger elbow (requested 3.0 rad): the code capped it at 1.35 rad (90% of the 1.5 rad room to the joint limit). The original comment claiming the elbow was clamped at 0.6 was wrong. At 1.35 the arm hit its own body and the floor: joint limits are not collision safety.

3. Added base rotation: three independent sine waves produced a complex 3D hand path. The base moves everything after it, and it tracked easily because it rotates around the vertical axis, so gravity doesn't load it. Joint angles in, hand position out = forward kinematics.

## Lessons
- ctrl means different things for different actuator types (target angle vs torque).
- A robot goes where its motors can get it, not where you tell it.
- Staying inside joint limits does not mean staying out of collisions.
- AI-written comments can be wrong; check the numbers.

## Next
Day 2: control each joint from the keyboard.
