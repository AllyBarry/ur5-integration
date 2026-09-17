# Bring-up

Full startup for the real rig. For the no-hardware path, skip to
[docs/SIMULATION.md](SIMULATION.md).

Rig facts (verify before trusting — rigs change):

| | |
| --- | --- |
| Robot IP | `192.168.50.21` (UR5, not UR5e → `ur_type:=ur5`) |
| SSH | `root@192.168.50.21`, password `easybot` |
| Gripper bridge port | TCP `54322` (**not** 54321) |
| ROS distro | Jazzy, `ROS_DOMAIN_ID=42` |
| Arm calibration | `~/my_robot_calibration.yaml` — `robot.launch.py` picks it up automatically if present |
| PC network | `enp3s0`: DHCP on `10.10.187.0/24` for internet, static `192.168.50.3/24` for the robot ([below](#network-prerequisites-once-per-machine)) |
| Camera stack | On the Jetson (`jetson@tegra-ubuntu`), same domain ID |

### Where the robot IP is set

There is no single source of truth. It is written in four independent places,
and none of the bring-up commands below pass it explicitly:

| Place | Used by |
| --- | --- |
| `DEFAULT_ROBOT_IP` in `src/ur5_bringup/launch/robot.launch.py` | The arm (RTDE). It is the default for the `robot_ip` launch argument, forwarded to `ur_control.launch.py` and on into the URDF via `rsp.launch.py`. This is why terminal 1 needs no `robot_ip:=`. Override per run with `robot_ip:=<ip>`. |
| The `ssh root@...` in terminal 3 | Starting `gripper_bridge.py` on the controller |
| The `tcp:<ip>:54322` in terminal 4's `socat` | The gripper serial link |
| Two hint strings in `demos/check_stack.sh` | Printed advice only, not a connection |

The PC's *own* address is separate again — that lives in `/etc/netplan/01-network.yaml`, and it has to be on the robot's subnet for any of the above to connect. See [Network prerequisites](#network-prerequisites-once-per-machine).

The gripper side never sees the launch argument: `control.launch.py` only knows
the local pty (`gripper_port`, default `/tmp/ttyUR`), and `gripper_server` just
opens that device. So changing `DEFAULT_ROBOT_IP` alone moves the arm to a new
rig but leaves the gripper pointing at the old one.

The `robot_ip` defaults of `0.0.0.0` in `rsp.launch.py` and
`ur5_hand_e.urdf.xacro` are placeholders for launching those files standalone;
`robot.launch.py` always overrides them.

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

## Network prerequisites (once per machine)

Both links to the robot ride the same Ethernet cable: RTDE/External Control to
the arm (ports 30001-30004 out, 50001-50004 back) and the gripper bridge (TCP
54322). Neither is routed anywhere clever, so:

> **The PC needs an address in the robot's `/24`.** The robot is
> `192.168.50.21`, so the PC must hold a `192.168.50.x/24` address. Without one
> there is no route to the robot and *both* the driver and `socat` fail with
> timeouts or `Connection refused` — a failure that looks like a dead
> `gripper_bridge.py` or a stopped `external.urp`, but isn't.

The lab's general network is a different subnet (`10.10.187.0/24`, DHCP) and
that is where internet comes from. So the PC needs to be on both at once, which
is what the netplan config below sets up. Wi-Fi does not substitute for the
cable; it puts the PC on neither.

### The netplan file

Ubuntu 24.04, netplan with the NetworkManager renderer. The live file is
`/etc/netplan/01-network.yaml` (root-owned, mode `600`); a copy of what this PC
runs is checked in at [`01-network.yaml`](../01-network.yaml) in the repo root:

```yaml
network:
  version: 2
  renderer: NetworkManager

  ethernets:
    enp3s0:                     # NIC name is machine-specific: ip -brief link
      dhcp4: true               # internet + default route, from the lab DHCP server
      addresses:
        - 192.168.50.3/24       # fixed address on the robot's subnet, same NIC
```

Two jobs, one interface — this is the part worth understanding:

| Line | Gives you | Result on this PC |
| --- | --- | --- |
| `dhcp4: true` | An address, default gateway and DNS from the lab DHCP server — internet for `apt`, `pip`, `vcs import` | `10.10.187.161/24`, default via `10.10.187.1` |
| `addresses:` | A static address that does **not** depend on DHCP, on the robot's subnet | `192.168.50.3/24`, reaches `192.168.50.21` directly |

`addresses:` **adds** to the DHCP address rather than replacing it. A NIC can
hold several IPv4 addresses at once, so the PC sits on the robot's subnet at a
known, permanent address *and* keeps DHCP-provided internet. The kernel then
picks the source address per destination, with no configuration from you:

```bash
ip route get 192.168.50.21    # -> dev enp3s0 src 192.168.50.3    (robot: static)
ip route get 8.8.8.8          # -> via 10.10.187.1  src 10.10.187.161  (internet: DHCP)
```

Why not one or the other:

- **DHCP only** — no address on `192.168.50.0/24`, so the robot is unreachable
  no matter what `robot_ip` says.
- **Static only** — the robot works, but there is no default route or DNS, so
  the machine is offline. If you ever do go static-only, you must add `routes:`
  and `nameservers:` by hand.

Whatever you do, do **not** put a `gateway4`/default `routes:` entry on the
static block. A second default route competing with the DHCP one is the usual
way this machine loses internet.

### Changing subnets

If the robot moves, the PC's static address has to move with it — the two are
only linked by convention, and nothing warns you when they disagree. Update the
`addresses:` entry here *and* every place from
[Where the robot IP is set](#where-the-robot-ip-is-set). Pick a host number
outside the DHCP pool, and confirm it is free (`ping -c1 <candidate>` gets no
reply) before claiming it.

### Applying and verifying

```bash
sudo chmod 600 /etc/netplan/01-network.yaml   # netplan warns on readable files
sudo netplan try                              # auto-reverts after 120 s if the link drops
sudo netplan apply
```

Then, before starting anything in this workspace:

```bash
ip -brief addr show enp3s0      # expect BOTH: 192.168.50.3/24 and a 10.10.187.x/24
ping -c3 192.168.50.21          # arm reachable
ping -c1 8.8.8.8                # internet still works
nc -zv 192.168.50.21 54322      # gripper bridge -- only once gripper_bridge.py runs
```

`Connection refused` from that last one means the network is fine and
`gripper_bridge.py` isn't running (terminal 3). A *timeout* is the network
problem this section is about.

If the arm answers but External Control connects and immediately drops, the
driver may be advertising the wrong local address for the robot's return
connection — with two addresses on one NIC, check `ip route get 192.168.50.21`
and pass `reverse_ip:=<that source address>`. Note that `robot.launch.py` does
not forward `reverse_ip`, so it has to be added to the `launch_arguments` dict
there.

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
ssh root@192.168.50.21      # password: easybot
python gripper_bridge.py
```

Leave running. Check from the PC: `nc -zv 192.168.50.21 54322`.

## Terminal 4 — socat

```bash
socat -d -d pty,link=/tmp/ttyUR,raw,echo=0 tcp:192.168.50.21:54322
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
| Driver times out AND socat can't connect | PC has no address in the robot's `/24` (lost DHCP lease, cable in the wrong port, on Wi-Fi) | `ip route get 192.168.50.21`; see [Network prerequisites](#network-prerequisites-once-per-machine) |
| `Connection refused` on port 54322 | `gripper_bridge.py` not running | SSH in and start it |
| Modbus failure despite TCP connecting | RS-485 daemon URCap enabled, wrong port, or tool voltage 0 V | Disable the URCap, use 54322, set 24 V |
| `/tmp/ttyUR` missing | socat died | Restart terminal 4 |
| `scaled_joint_trajectory_controller` inactive | `external.urp` stopped | Press play again |
| Goals rejected with "outside workspace" | Target outside the box in `ur5_control/geometry.py` | Check the coordinate is world-frame (table top is z=0.8, not z=0) |
| `follow_hand --mode servo` does nothing | `forward_position_controller` inactive | `ros2 control switch_controllers --deactivate scaled_joint_trajectory_controller --activate forward_position_controller` |
| No `/perception/hand_target` | No source running, or the tag isn't visible | `./demos/check_stack.sh` |
