# Demos

Standalone scripts, not a ROS package. Source the workspace first; then run
them from anywhere:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Documents/UR5_INTEGRATION/install/setup.bash
python3 demos/follow_hand.py --dry-run
```

They import `ur5_control.geometry` (so the constants match the servers) and
`demo_common` (which Python finds because it sits next to the script).

| Demo | Needs a robot? | Needs a camera? | What it shows |
| --- | --- | --- | --- |
| `check_stack.sh` | no | no | What's up, what's missing, and the command to fix each gap |
| `follow_hand.py` | mock is fine | mock is fine | Continuous tracking: standoff, clamp, leash, deadband; plan vs servo control |
| `handover.py` | yes (gripper too) | mock is fine | Pick, wait for a steady hand, offer, release |
| `pick_and_place.sh` | yes | no | The action interfaces from the CLI |

Every Python demo takes `--dry-run`, which computes and logs every command
without sending it. Use it whenever you change a number.

## Recommended first run (nothing physical involved)

```bash
# terminal 1 -- arm with mocked joints, RViz
ros2 launch ur5_bringup robot.launch.py use_mock_hardware:=true

# terminal 2 -- MoveIt
ros2 launch ur5_bringup moveit.launch.py

# terminal 3 -- our servers + a synthetic hand
ros2 launch ur5_bringup control.launch.py hand_source:=mock gripper_mock:=true

# terminal 4
./demos/check_stack.sh
python3 demos/follow_hand.py
```

In RViz, add a **Marker** display on `/perception/hand_target_marker` to see
the target the arm is chasing.

What you should see: one hop every ~2.5 s (plan + execute), the gripper trailing
the circling target by ~0.18 m, and log lines pairing each `hand=` position with
the `command=` derived from it. The lag is plan mode working as designed — see
the mode notes in `follow_hand.py`.

## On the real rig

Follow `docs/BRINGUP.md` first (driver, pendant, gripper bridge, socat, MoveIt).
Then:

```bash
ros2 launch ur5_bringup control.launch.py hand_source:=apriltag tag_frame:=tag_1
python3 demos/follow_hand.py --standoff 0.20 --max-step 0.05 --velocity-scaling 0.15
```

Tape tag_1 (the 33 mm one) to a wristband. There is no hand *detector* on this
rig yet — the tag stands in for one, and everything downstream is identical.
See `docs/CAMERA_STACK.md`.

Before the first run with a person nearby, check in this order:

1. `--dry-run` and read the commanded poses.
2. `use_mock_hardware:=true` and watch it in RViz.
3. Real arm, `--velocity-scaling 0.15`, hand well clear, finger on the
   emergency stop.

`--max-y` (default 0.65) is what stops the arm short of where someone stands.
Raising it moves the arm into the standing zone.

## Writing another demo

Start from `follow_hand.py` for anything continuous, `handover.py` for anything
sequential. Both are single files on purpose: the control logic should be
readable start to finish without chasing it through a package.

Keep constants that describe the *rig* (table height, workspace bounds, grasp
quaternions) in `ur5_control/geometry.py`, and constants that describe the
*demo* (standoff, gains, timeouts) as argparse defaults.
