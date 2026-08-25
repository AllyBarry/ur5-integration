# UR5_INTEGRATION

A ROS 2 (Jazzy) workspace for the RAIL UR5 + Robotiq Hand-E table rig: motion
primitives, a gripper server, a perception seam, and a `demos/` folder — with
the human-facing demo being **the arm following a hand**.

The scope stops at skills: a MoveIt wrapper, the measured grasp orientations,
the Modbus gripper server, the URDF and a perception seam. There is no task
planner, behaviour tree or execution logging here — anything of that kind sits
above these interfaces, not inside them.

## Layout

```
UR5_INTEGRATION/
├── demos/                    demo scripts (start here)
│   ├── follow_hand.py        follow a tracked hand — plan or servo control
│   ├── handover.py           pick an object, wait for a steady hand, release
│   ├── pick_and_place.sh     the actions from the CLI, no perception
│   ├── check_stack.sh        what's running, what's missing
│   └── demo_common.py        shared client plumbing
├── docs/
│   ├── BRINGUP.md            real-rig startup, terminal by terminal
│   ├── CAMERA_STACK.md       what perception exists, what's missing
│   └── SIMULATION.md         what runs without hardware, what to add
└── src/
    ├── ur5_interfaces/       MoveToPose, MoveToNamedPose, Gripper, TrackedTarget
    ├── ur5_control/          the nodes
    └── ur5_bringup/          URDF + launch files
```

## Build

```bash
cd ~/Documents/UR5_INTEGRATION
vcs import src < ur5_integration.repos       # robotiq_hande_description
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

`robotiq_hande_description` is not on the ROS index; the `.repos` file pulls it.
The driver package is only needed if you switch the gripper to the ros2_control
path, which this stack does not use (see the note in the xacro).

## Run it with no hardware

```bash
ros2 launch ur5_bringup robot.launch.py use_mock_hardware:=true
ros2 launch ur5_bringup moveit.launch.py
ros2 launch ur5_bringup control.launch.py hand_source:=mock gripper_mock:=true
python3 demos/follow_hand.py
```

On the real rig, follow [docs/BRINGUP.md](docs/BRINGUP.md).

## Architecture

```
  perception                    motion                        hardware
  ----------                    ------                        --------
  mock_hand_publisher  ─┐
  hand_target_bridge   ─┴─►  /perception/hand_target
    (apriltag | detections)          │
                                     ▼
                              demos/follow_hand.py
                              demos/handover.py
                                     │
                       ┌─────────────┼──────────────┐
                       ▼             ▼              ▼
                 /move_to_pose  /move_to_named_pose  /gripper
                       │             │              │
                       └──── move_group (MoveIt) ───┤
                                     │              │
                        scaled_joint_trajectory     │
                          controller (ros2_control) │
                                     │              │
                                UR5 via RTDE   Hand-E via
                                               /tmp/ttyUR
```

Two rules hold the thing together:

- **Perception talks to motion through exactly one topic.** Swap the detector,
  nothing downstream changes.
- **All Cartesian motion goes through `/move_to_pose`.** Workspace limits,
  planner selection and grasp orientations live in one place.

## The nodes

| Node | Interface | What it does |
| --- | --- | --- |
| `move_to_pose` | `/move_to_pose` | The one Cartesian primitive. Picks Pilz LIN / PTP / OMPL by distance and constraints; rejects out-of-workspace goals. |
| `move_to_named_pose` | `/move_to_named_pose` | Joint-space move to a pose from `config/named_poses.yaml` (`ready`, `watch`, `stow`). The recovery action. |
| `gripper_server` | `/gripper`, `/gripper/open`, `/gripper/close` | Modbus RTU to the Hand-E over `/tmp/ttyUR`. `use_mock:=true` for no hardware. |
| `hand_target_bridge` | → `/perception/hand_target` | AprilTag TF or `Detection3DArray` → smoothed world-frame target. |
| `mock_hand_publisher` | → `/perception/hand_target` | Synthetic hand with jitter and dropout. |
| `planning_scene_publisher` | → `/apply_planning_scene` | Pushes the table/keep-out boxes in as CollisionObjects. **Off by default** — move_group normally gets them from `/robot_description` already. |

## Where this is incomplete

Two seams are deliberately marked in the source. Grep for them:

```bash
grep -rn "CAMERA-STACK" src/ docs/     # needs perception that doesn't exist yet
grep -rn "SIM-HOOK"     src/ demos/    # where a simulator plugs in
```

**Camera stack** ([docs/CAMERA_STACK.md](docs/CAMERA_STACK.md)) — the Jetson
runs AprilTag detection and nothing else. There is no hand detector, and the
Brio is monocular, so 3D hand positions need either a depth camera or a second
view. Until then, an AprilTag on a wristband stands in and every interface
downstream is identical.

**Simulation** ([docs/SIMULATION.md](docs/SIMULATION.md)) — no simulator is
included. `use_mock_hardware`, `gripper_mock`, `hand_source:=mock` and
`--dry-run` cover the plumbing; `ur_simulation_gz` is the natural addition for
physics, and the URDF's `ros2_control` block is where it would attach.

Also worth knowing:

- **`move_to_pose` goals aren't preemptible.** Cancel is accepted but not
  forwarded to move_group, so `follow_hand --mode plan` always finishes its
  current hop before reacting. That's why servo mode exists.
- **The SRDF is the stock UR one.** move_group *does* get this workspace's URDF
  (it reads `/robot_description`), so the table is in the collision model — but
  the SRDF's `disable_collisions` list names only the six arm links. Nothing
  covers the Hand-E links or `grasped_object`, so those can read as permanent
  self-collisions. Generating a MoveIt config from this URDF is the fix.
- **`watch` and `stow` poses are unverified.** Preview them in RViz before
  sending them to the real arm.

## Conventions worth knowing before you write against this

- **Goal z is plain world z**, targeting `tool0`. The table surface is at
  z=0.80, not z=0. If you want to think in "fingertips this far above the
  table", call `geometry.tip_above_table_to_world_z()` explicitly.
- **One Cartesian action, not one per motion phase.** Approach, descend, lift
  and carry are all `MoveToPose` with different arguments; the sequencing
  belongs in the caller.
- **Named poses are config, not code** — `ur5_control/config/named_poses.yaml`.
- **Rig constants live in exactly one place**, `ur5_control/geometry.py`. Table
  height, workspace bounds and the grasp quaternions are all there, and the
  demos import them rather than restating them.
