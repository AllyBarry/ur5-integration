#!/usr/bin/env bash
#
# check_stack.sh -- is everything this workspace needs actually up?
#
# Read-only: lists topics/actions/controllers and reports what's missing. It
# never commands the arm. Run it before any demo, and again when a demo does
# nothing and you can't tell why.
#
#   ./demos/check_stack.sh
set -uo pipefail

ok()   { printf '  \033[32mOK\033[0m    %s\n' "$1"; }
bad()  { printf '  \033[31mMISS\033[0m  %s\n' "$1"; }
note() { printf '        %s\n' "$1"; }

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset>}  (this rig uses 42; the Jetson must match)"
echo

echo "Actions:"
actions=$(ros2 action list 2>/dev/null)
for a in /move_to_pose /move_to_named_pose /gripper; do
  if grep -qx "$a" <<<"$actions"; then ok "$a"; else
    bad "$a"
    note "start it: ros2 launch ur5_bringup control.launch.py"
  fi
done
if grep -qx "/move_action" <<<"$actions"; then ok "/move_action (MoveIt)"; else
  bad "/move_action (MoveIt)"
  note "start it: ros2 launch ur5_bringup moveit.launch.py"
fi
echo

echo "Perception:"
topics=$(ros2 topic list 2>/dev/null)
if grep -qx "/perception/hand_target" <<<"$topics"; then
  ok "/perception/hand_target advertised"
  if timeout 3 ros2 topic echo /perception/hand_target --once >/dev/null 2>&1; then
    ok "/perception/hand_target is publishing"
  else
    bad "/perception/hand_target has no data"
    note "apriltag source: is the tag in view? ros2 run tf2_ros tf2_echo world tag_1"
    note "no camera:      ros2 run ur5_control mock_hand_publisher"
  fi
else
  bad "/perception/hand_target"
  note "ros2 launch ur5_bringup control.launch.py hand_source:=mock"
fi
echo

echo "Gripper serial:"
if [ -e /tmp/ttyUR ]; then ok "/tmp/ttyUR exists"; else
  bad "/tmp/ttyUR"
  note "socat -d -d pty,link=/tmp/ttyUR,raw,echo=0 tcp:10.10.187.168:54322"
  note "(needs gripper_bridge.py running on the robot: ssh root@10.10.187.168)"
  note "or run the stack with gripper_mock:=true"
fi
echo

echo "Controllers:"
controllers=$(ros2 control list_controllers 2>/dev/null)
if [ -z "$controllers" ]; then
  bad "controller_manager not reachable"
  note "ros2 launch ur5_bringup robot.launch.py"
else
  echo "$controllers" | sed 's/^/  /'
  note ""
  note "plan-mode demos need scaled_joint_trajectory_controller ACTIVE."
  note "follow_hand --mode servo needs forward_position_controller ACTIVE instead."
fi
echo

echo "Robot model (does MoveIt know about the table?):"
# move_group reads the URDF off /robot_description, so the boxes are in the
# collision model iff the description came from ur5_bringup.
# Ask robot_state_publisher what it published rather than echoing the topic:
# the URDF is ~1 MB and `ros2 topic echo --full-length` on it is unusably slow.
# move_group takes the model from this same description via /robot_description.
boxes=$(timeout 20 ros2 param get /robot_state_publisher robot_description 2>/dev/null \
        | grep -c workspace_obstacle || true)
if [ "${boxes:-0}" -gt 0 ]; then
  ok "/robot_description includes the table and keep-out slabs"
else
  bad "/robot_description has no keep-out geometry"
  note "launch via 'ros2 launch ur5_bringup robot.launch.py', not ur_control directly"
  note "stopgap: ros2 launch ur5_bringup control.launch.py publish_scene:=true"
fi
if ros2 node list 2>/dev/null | grep -q planning_scene_publisher; then
  note "planning_scene_publisher IS running -- if the description above already"
  note "has the boxes, this duplicates them and plans will fail."
fi
