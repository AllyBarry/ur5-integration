#!/usr/bin/env bash
#
# pick_and_place.sh -- the no-perception demo: move a known object from A to B.
#
# Everything here is driven from the CLI, so it doubles as a reference for the
# goal syntax of each action. If a Python demo misbehaves, run this first to
# work out whether the problem is the demo or the stack underneath it.
#
#   ./demos/pick_and_place.sh
#   PICK_X=-0.5 PICK_Y=0.26 ./demos/pick_and_place.sh
#
# Coordinates are WORLD frame, metres, targeting tool0. The table surface is at
# z=0.80 and the fingertips sit 0.18 below tool0, so "fingertips 0.33 above the
# table" is z = 0.33 + 0.80 - 0.18 = 0.95. Nothing applies that offset for
# you: pass the 0.95.
set -euo pipefail

PICK_X=${PICK_X:--0.30}
PICK_Y=${PICK_Y:-0.25}
PICK_Z=${PICK_Z:-0.95}

PLACE_X=${PLACE_X:-0.30}
PLACE_Y=${PLACE_Y:-0.27}
PLACE_Z=${PLACE_Z:-0.95}

APPROACH=${APPROACH:-0.10}      # metres above pick/place for the approach pose
GRASP_TYPE=${GRASP_TYPE:-top}
GRASP_ORIENT=${GRASP_ORIENT:-grasp1}
VEL=${VEL:-0.25}

add() {  # a b -> a+b   (awk, not bc: bc isn't installed by default)
  awk -v a="$1" -v b="$2" 'BEGIN { printf "%.4f", a + b }'
}

move_to() {   # x y z force_linear label
  echo "=== $5 ==="
  ros2 action send_goal /move_to_pose ur5_interfaces/action/MoveToPose \
    "{x: $1, y: $2, z: $3, grasp_type: '${GRASP_TYPE}', \
      grasp_orientation: '${GRASP_ORIENT}', force_linear: $4, \
      velocity_scaling: ${VEL}}" --feedback
}

echo "=== Reset to ready ==="
ros2 action send_goal /move_to_named_pose ur5_interfaces/action/MoveToNamedPose \
  "{pose_name: 'ready'}" --feedback

move_to "${PICK_X}" "${PICK_Y}" "$(add "${PICK_Z}" "${APPROACH}")" false "Approach pick"

echo "=== Open gripper ==="
ros2 service call /gripper/open std_srvs/srv/Trigger

move_to "${PICK_X}" "${PICK_Y}" "${PICK_Z}" true "Descend to grasp"

echo "=== Close gripper ==="
# The action reports object_detected; the /gripper/close service does not.
ros2 action send_goal /gripper ur5_interfaces/action/Gripper \
  "{target_position: 255, force: 100, speed: 255}"

move_to "${PICK_X}" "${PICK_Y}" "$(add "${PICK_Z}" "${APPROACH}")" true "Lift"
move_to "${PLACE_X}" "${PLACE_Y}" "$(add "${PLACE_Z}" "${APPROACH}")" false "Carry to place"
move_to "${PLACE_X}" "${PLACE_Y}" "${PLACE_Z}" true "Descend to place"

echo "=== Release ==="
ros2 service call /gripper/open std_srvs/srv/Trigger

move_to "${PLACE_X}" "${PLACE_Y}" "$(add "${PLACE_Z}" "${APPROACH}")" true "Retreat"

echo "=== Back to ready ==="
ros2 action send_goal /move_to_named_pose ur5_interfaces/action/MoveToNamedPose \
  "{pose_name: 'ready'}" --feedback
