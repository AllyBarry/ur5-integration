# Simulation — what's wired now, and what to add

There is no simulator in this workspace. What there *is*: every place a
simulator would plug in is a launch argument, and each one is marked
`SIM-HOOK` in the source. Grep for it:

```bash
grep -rn "SIM-HOOK" src/ demos/
```

## What you can run today without hardware

| Layer | Hook | What it gives you | What it does NOT give you |
| --- | --- | --- | --- |
| Arm | `robot.launch.py use_mock_hardware:=true` | Full ros2_control stack against mocked joints. MoveIt plans and "executes"; RViz shows the arm move. | Dynamics, controller tuning, execution timing, singular/near-limit behaviour. Mocked joints report the commanded position instantly and perfectly. |
| Gripper | `control.launch.py gripper_mock:=true` | `/gripper` action and `/gripper/open|close` answer normally. | Contact. `object_detected` is always `False`, so `handover.py` will refuse to proceed past the grasp check unless you also skip it. |
| Hand | `control.launch.py hand_source:=mock` | A moving target with configurable jitter and dropout. | Occlusion, detector latency, false positives, and the fact that a real hand belongs to a person who moves unpredictably. |
| Demos | `--dry-run` | Every command computed and logged, nothing sent. | Everything. It is a calculator, and that's the point — it's the fastest way to check a geometry change. |

Combined, that's the full stack minus physics:

```bash
ros2 launch ur5_bringup robot.launch.py use_mock_hardware:=true
ros2 launch ur5_bringup moveit.launch.py
ros2 launch ur5_bringup control.launch.py hand_source:=mock gripper_mock:=true
python3 demos/follow_hand.py
```

Good enough to catch frame errors, sign errors, unit errors, workspace-bound
mistakes and action plumbing bugs — which is most of what goes wrong.

## The gap worth closing first: the SRDF

Not a simulator problem, but it bites in both sim and reality.

The good news, verified on this machine: `ur_moveit_config` builds **no URDF of
its own**. `ur_moveit.launch.py` starts `wait_for_robot_description` and
move_group reads the model off the `/robot_description` topic — so it gets
`ur5_hand_e.urdf.xacro` from our `rsp.launch.py`, table and keep-out slabs and
Hand-E included. Collision checking against the table works.

(Hence `publish_scene` defaults to **false**. `planning_scene_publisher` exists
for the case where move_group *doesn't* have the boxes; running it when it does
duplicates geometry the robot is attached to and every plan fails. Check with
`ros2 param get /robot_state_publisher robot_description | grep -c workspace_obstacle`
— ask robot_state_publisher rather than echoing `/robot_description`: the URDF
is ~1 MB and `ros2 topic echo --full-length` on it never returns in practice.)

The bad news: the **SRDF** is still stock `ur_moveit_config/srdf/ur.srdf.xacro`,
whose 18 `disable_collisions` entries name only the six arm links. Nothing
disables:

- Hand-E finger/body links against `wrist_3_link` and `tool0`
- `grasped_object` against the gripper links (both hang off `tool0` as
  siblings, so they overlap and are *not* adjacent — this pair alone will read
  as a permanent self-collision, which is why `enable_grasped_object` defaults
  to false)
- `table` / `ceiling` / obstacle slabs against anything they legitimately touch

MoveIt auto-allows collisions between directly jointed links, which covers some
of this by luck. The rest is unverified. Generate a real config:

```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
# load src/ur5_bringup/urdf/ur5_hand_e.urdf.xacro
# let it recompute the self-collision matrix, then save as src/ur5_moveit_config/
```

Then point `moveit.launch.py` at that package instead of `ur_moveit_config`.
Until then, if a plan fails for no visible reason, check for a self-collision
before blaming the planner:

```bash
ros2 run moveit_ros_planning moveit_print_planning_model_info  # if available
# or watch move_group's log for "collision between ..."
```

## If you want a physics simulator

**Gazebo (`ur_simulation_gz`)** is the path of least resistance — it's the
official UR simulation package, uses `gz_ros2_control`, and exposes the same
controller names, so `move_to_pose` and everything above it are unchanged.

```bash
sudo apt install ros-jazzy-ur-simulation-gz     # not currently installed
```

To use this workspace's world (table, obstacles) rather than the stock UR one,
the URDF needs a `gz_ros2_control` hardware plugin alongside the existing
`ur_ros2_control` one. That's a third branch in the xacro's `ros2_control`
block, selected by an `use_sim` argument.

Then:
- `robot.launch.py` gains `use_sim:=true`, which starts the Gazebo bringup
  instead of `ur_control.launch.py`.
- Everything downstream sets `use_sim_time:=true` — `moveit.launch.py` already
  takes that argument and passes it through.
- `gripper_server` stays in `use_mock:=true` mode, or grows a Gazebo branch if
  you need simulated grasping (which needs a grasp-fixing plugin; simulating
  friction well enough to hold an object is its own project).

**Isaac Sim** is the other option, and the one that would actually help the
hand-following work: it can render a synthetic person, which is the piece a
physics simulator alone doesn't give you. It's much heavier to set up, and the
ROS 2 bridge is version-sensitive. Only worth it if you need vision-in-the-loop
training data.

## Rule of thumb

Simulate the layer you're changing, mock the rest:

- Changing geometry, frames or demo logic → `--dry-run`, then mock hardware.
- Changing planner or collision behaviour → mock hardware plus a real MoveIt
  config from this URDF.
- Changing controller tuning, speed, or anything about contact → no simulator
  available here will tell you the truth. Use the real arm, slowly.
