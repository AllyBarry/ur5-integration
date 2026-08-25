#!/usr/bin/env python3
"""
Shared geometry for the UR5 + Hand-E rig.

Everything that needs to agree on "where is the table", "which quaternion is a
top grasp" or "is this point inside the workspace" imports it from here, so
there is exactly one place to change when the rig moves.

Frames
------
world       Fixed frame at the base of the table. The URDF parents the table to
            it and the arm to the table top, so world z = 0 is the FLOOR and the
            table surface is at z = TABLE_HEIGHT.
tool0       UR tool flange. All Cartesian goals in this stack target tool0.
tip         Not a TF frame -- the point between the Hand-E fingertips, roughly
            TOOL0_TO_TIP below tool0 along the approach axis.
"""
import math

from geometry_msgs.msg import Quaternion

# ---- Rig geometry (must match ur5_bringup/urdf/ur5_hand_e.urdf.xacro) ----
TABLE_HEIGHT = 0.8          # world z of the table surface, metres
TOOL0_TO_TIP = 0.18         # distance from tool0 down to the fingertips

# ---- Workspace box, world frame ----
# Goals outside this are rejected before they ever reach MoveIt. Keep this
# CONSERVATIVE: it is the last cheap guard before the arm moves.
X_MIN, X_MAX = -0.70, 0.70
Y_MIN, Y_MAX = 0.00, 1.00
Z_MIN, Z_MAX = TABLE_HEIGHT + 0.02, TABLE_HEIGHT + 0.85

# ---- Static scene geometry, world frame ----
#
# Mirrors the collision boxes in ur5_bringup/urdf/ur5_hand_e.urdf.xacro.
#
# NOT normally needed: move_group reads the URDF off /robot_description, so it
# already has these as robot links. They live here for planning_scene_publisher,
# which is for the case where move_group is running against a description that
# lacks them -- and which is off by default precisely because duplicating them
# would put the robot permanently in collision. See docs/SIMULATION.md.
#
# Each entry: (id, (size_x, size_y, size_z), (centre_x, centre_y, centre_z)).
#
# The table is a 0.1 m slab whose top sits 5 mm BELOW the real surface rather
# than a full-height box: a box that touches base_link at exactly z=0 reads as
# a collision with the robot's own base and every plan fails.
SCENE_BOXES = [
    ("table",              (2.60, 1.60, 0.10), (0.0,  0.00, TABLE_HEIGHT - 0.005 - 0.05)),
    ("ceiling",            (2.60, 1.60, 0.05), (0.0,  0.00, TABLE_HEIGHT + 1.025)),
    # Keep-out slab in front of the table (people, cables, the rest of the lab).
    ("workspace_obstacle", (2.00, 0.20, 1.00), (0.0, -0.50, TABLE_HEIGHT + 0.50)),
    # Keep-out slab behind the table, where the operator stands.
    ("standing_area",      (2.00, 0.10, 1.00), (0.0,  0.75, TABLE_HEIGHT + 0.50)),
]

# ---- Grasp type / orientation enumerations ----
GRASP_TOP = "top"
GRASP_SIDE = "side"
GRASP_DIAGONAL = "diagonal"
GRASP_NONE = "none"          # position-only goal; no orientation constraint
VALID_GRASP_TYPES = {GRASP_TOP, GRASP_SIDE, GRASP_DIAGONAL, GRASP_NONE}

ORIENT_1 = "grasp1"
ORIENT_2 = "grasp2"
ORIENT_NONE = "none"         # approach axis pinned, wrist roll free
VALID_ORIENTATIONS = {ORIENT_1, ORIENT_2, ORIENT_NONE}


def _quat(x, y, z, w):
    q = Quaternion()
    q.x, q.y, q.z, q.w = x, y, z, w
    return q


# Precomputed quaternions per (grasp_type, grasp_orientation).
#
# Convention: tool0's local +Z is the gripper approach axis (the direction the
# gripper points out from the wrist).
#
#   top:       approach axis = (0, 0, -1)            [straight down]
#   side:      approach axis = (0, +1, 0)            [wrist horizontal, into +Y]
#   diagonal:  approach axis = normalize(0, +1, -1)  [45 deg down toward +Y]
#
# grasp2 = grasp1 rotated +90 deg about the local approach axis. That is the
# "twist the wrist" wrist_3_joint realises, but expressed as a target
# orientation so IK can pick whatever wrist_3 value reaches it from the current
# arm configuration.
#
# There is no entry for grasp_type "none" (no orientation is used on that path).
# For grasp_orientation "none" the grasp1 quaternion is used as the reference
# and the roll tolerance is opened to PI.
#
# These are measured against the real rig. Do not "tidy" them (round them,
# regenerate them from RPY, rename the keys) without re-testing on hardware.
GRASP_QUATERNIONS = {
    (GRASP_TOP, ORIENT_1):      _quat( 1.0,                 0.0,                 0.0,                 0.0),
    (GRASP_TOP, ORIENT_2):      _quat( 0.7071067811865476, -0.7071067811865475,  0.0,                 0.0),
    (GRASP_SIDE, ORIENT_1):     _quat(-0.7071067811865475,  0.0,                 0.0,                 0.7071067811865476),
    (GRASP_SIDE, ORIENT_2):     _quat(-0.5,                 0.5,                 0.5,                 0.5),
    (GRASP_DIAGONAL, ORIENT_1): _quat(-0.9238795325112867,  0.0,                 0.0,                 0.3826834323650898),
    (GRASP_DIAGONAL, ORIENT_2): _quat(-0.6532814824381883,  0.6532814824381882,  0.2705980500730985,  0.27059805007309856),
}

# Unit approach vector in world coordinates for each grasp type. Used to back
# a tool0 target off from a target point along the direction the gripper is
# pointing (see standoff_pose).
APPROACH_AXES = {
    GRASP_TOP:      (0.0, 0.0, -1.0),
    GRASP_SIDE:     (0.0, 1.0, 0.0),
    GRASP_DIAGONAL: (0.0, 1.0 / math.sqrt(2.0), -1.0 / math.sqrt(2.0)),
    GRASP_NONE:     (0.0, 0.0, -1.0),   # assume top-down when unconstrained
}


def tip_above_table_to_world_z(height_above_table):
    """Convert 'fingertips this far above the table' to a world z for tool0.

    Explicit and opt-in. Nothing in the motion path applies this for you: an
    action goal's z is world z, full stop. A server that quietly adds a table
    offset to every goal is the kind of thing that reads fine until someone
    passes a coordinate that already had it applied.
    """
    return height_above_table + TABLE_HEIGHT - TOOL0_TO_TIP


def world_z_to_tip_above_table(world_z):
    """Inverse of tip_above_table_to_world_z."""
    return world_z - TABLE_HEIGHT + TOOL0_TO_TIP


def standoff_pose(x, y, z, grasp_type, standoff):
    """Back a point off along the approach axis by `standoff` metres.

    Given a point you want the FINGERTIPS at, returns the tool0 target that
    hovers `standoff` short of it, approaching along `grasp_type`'s axis.
    For a top grasp that is simply "straight above".
    """
    ax, ay, az = APPROACH_AXES.get(grasp_type, APPROACH_AXES[GRASP_NONE])
    return (x - ax * standoff, y - ay * standoff, z - az * standoff)


def in_workspace(x, y, z):
    """True if (x, y, z) is inside the workspace box."""
    return (X_MIN <= x <= X_MAX) and (Y_MIN <= y <= Y_MAX) and (Z_MIN <= z <= Z_MAX)


def clamp_to_workspace(x, y, z):
    """Clamp a point into the workspace box.

    Returns (x, y, z, was_clamped). Callers that would rather refuse than
    silently retarget should use in_workspace() instead -- follow-style demos
    clamp (so the arm tracks to the edge of its reach and waits), whereas
    pick/place should reject.
    """
    cx = min(max(x, X_MIN), X_MAX)
    cy = min(max(y, Y_MIN), Y_MAX)
    cz = min(max(z, Z_MIN), Z_MAX)
    clamped = (cx, cy, cz) != (x, y, z)
    return cx, cy, cz, clamped


def distance(a, b):
    """Euclidean distance between two (x, y, z) tuples."""
    return math.sqrt(
        (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2
    )


def step_towards(current, target, max_step):
    """Move from `current` towards `target` by at most `max_step` metres.

    The per-cycle leash for follow-style demos: however far the tracked target
    jumps, the commanded pose only ever moves this much. Combined with the
    command rate this is a hard cap on end-effector speed, and it is what makes
    a dropped/teleporting detection safe rather than a lunge.
    """
    d = distance(current, target)
    if d <= max_step or d == 0.0:
        return tuple(target)
    scale = max_step / d
    return (
        current[0] + (target[0] - current[0]) * scale,
        current[1] + (target[1] - current[1]) * scale,
        current[2] + (target[2] - current[2]) * scale,
    )
