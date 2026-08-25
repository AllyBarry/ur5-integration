# Bring-up

Full startup for the real rig. For the no-hardware path, skip to
[docs/SIMULATION.md](SIMULATION.md).

Rig facts (verify before trusting — rigs change):

| | |
| --- | --- |
| Robot IP | `10.10.187.168` (UR5, not UR5e → `ur_type:=ur5`) |
| SSH | `root@10.10.187.168`, password `easybot` |
| Gripper bridge port | TCP `54322` (**not** 54321) |
| ROS distro | Jazzy, `ROS_DOMAIN_ID=42` |
| Arm calibration | `~/my_robot_calibration.yaml` — `robot.launch.py` picks it up automatically if present |
| Camera stack | On the Jetson (`jetson@tegra-ubuntu`), same domain ID |

## Architecture

```
+------------------+        +----------+       +------------------+
| PC                |        | Network  |       | UR5 controller   |
|                   |        |          |       |                  |
| ur_robot_driver  <----------> RTDE <----------> External Control |
| MoveIt            |        | 30001..  |       |   URCap  (play)  |
| ur5_control nodes |        |          |       |                  |
|                   |        | TCP      |       | gripper_bridge.py|
| gripper_server    |        | 54322    |       |       |          |
|   /tmp/ttyUR  <--------------------------------+       v          |
|      ^            |        |          |       |  RS-485 -> Hand-E|
|    socat          |        |          |       |                  |
+------------------+        +----------+       +------------------+
```

The gripper does **not** use the UR Tool Communication URCap on this rig.
`gripper_bridge.py` on the controller relays RS-485 over TCP; `socat` turns
that into `/tmp/ttyUR`; `gripper_server` speaks Modbus RTU to it.

Do **not**:
- pass `use_tool_communication:=true` to the driver,
- run `robotiq_hande_driver gripper_controller_preview.launch.py`,
- use port 54321.

All three fight `gripper_bridge.py` and fail with Modbus errors.

## Pendant prerequisites (per power cycle)

- Top right: **Remote** mode.
- **Installation → URCaps**: `External Control` present as `external.urp`;
  Robotiq Hand-E URCap (v1.8.13.22852) installed; **RS-485 daemon URCap
  installed but DISABLED**.
- **Installation → General → Tool I/O**: voltage **24 V**.
- Load `external.urp`, but don't press play until the driver is up.

## Terminal 1 — driver

```bash
cd ~/Documents/UR5_INTEGRATION
source install/setup.bash
ros2 launch ur5_bringup robot.launch.py
```

Then press **play** on `external.urp`. Look for `Robot connected to reverse
interface` and `Successfully switched controllers!`. A "Controller already
active / Aborting" pair right after is harmless.

If controllers won't switch, kill leftovers and retry:

```bash
pkill -f ur_robot_driver; pkill -f ros2_control_node
```

## Terminal 2 — MoveIt

```bash
ros2 launch ur5_bringup moveit.launch.py
```

Add `launch_servo:=true` if you're going to run `follow_hand.py --mode servo`.

## Terminal 3 — gripper bridge (on the robot)

```bash
ssh root@10.10.187.168      # password: easybot
python gripper_bridge.py
```

Leave running. Check from the PC: `nc -zv 10.10.187.168 54322`.

## Terminal 4 — socat

```bash
socat -d -d pty,link=/tmp/ttyUR,raw,echo=0 tcp:10.10.187.168:54322
```

Leave running. Verify: `ls -la /tmp/ttyUR`.

## Terminal 5 — this workspace's nodes

```bash
ros2 launch ur5_bringup control.launch.py hand_source:=apriltag tag_frame:=tag_1
```

`gripper_server` should log `Connected to /tmp/ttyUR` then `Gripper ready.`
(a ~4 s activation sweep).

## Terminal 6 — check, then demo

```bash
./demos/check_stack.sh
python3 demos/follow_hand.py --dry-run
python3 demos/follow_hand.py --velocity-scaling 0.15
```

## Vision (Jetson)

Only needed for `hand_source:=apriltag` or `detections`. See
[docs/CAMERA_STACK.md](CAMERA_STACK.md) and `~/ur5_test/UR5_SETUP.md` for the
four Jetson terminals (usb_cam → rectify → apriltag_node →
extrinsics_publisher) and the calibration procedures.

## Troubleshooting

| Symptom | Cause | Fix |
| --- | --- | --- |
| `xacro: No such file or directory` | Relative `description_file` resolved against the wrong cwd | `rsp.launch.py` uses an absolute share path; if you overrode `description_file`, make it absolute |
| Arm appears in RViz without table/gripper | Something bypassed `rsp.launch.py` | Launch via `ur5_bringup robot.launch.py`, not `ur_control.launch.py` directly |
| move_group never starts | It waits for `/robot_description` | Start `robot.launch.py` before `moveit.launch.py` |
| Plans go through the table | move_group got a description without it | `ros2 param get /robot_state_publisher robot_description \| grep -c workspace_obstacle`; if 0, launch via `ur5_bringup robot.launch.py`, or as a stopgap `control.launch.py publish_scene:=true` |
| Every plan fails on self-collision | Stock SRDF has no `disable_collisions` for the Hand-E or `grasped_object` | Keep `enable_grasped_object:=false`; longer term generate a MoveIt config from this URDF (docs/SIMULATION.md) |
| `Connection refused` on port 54322 | `gripper_bridge.py` not running | SSH in and start it |
| Modbus failure despite TCP connecting | RS-485 daemon URCap enabled, wrong port, or tool voltage 0 V | Disable the URCap, use 54322, set 24 V |
| `/tmp/ttyUR` missing | socat died | Restart terminal 4 |
| `scaled_joint_trajectory_controller` inactive | `external.urp` stopped | Press play again |
| Goals rejected with "outside workspace" | Target outside the box in `ur5_control/geometry.py` | Check the coordinate is world-frame (table top is z=0.8, not z=0) |
| `follow_hand --mode servo` does nothing | `forward_position_controller` inactive | `ros2 control switch_controllers --deactivate scaled_joint_trajectory_controller --activate forward_position_controller` |
| No `/perception/hand_target` | No source running, or the tag isn't visible | `./demos/check_stack.sh` |
