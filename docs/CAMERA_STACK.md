# Camera stack — what exists, and what's missing

Short version: **this workspace has no perception in it, and the rig's camera
stack cannot currently detect a hand.** Everything vision-shaped here is either
a bridge that adapts an existing detector, or a stand-in.

## The one contract

Motion code never talks to a camera. It subscribes to:

```
/perception/hand_target      ur5_interfaces/TrackedTarget
    header.stamp       when the target was OBSERVED (not republished)
    header.frame_id    "world"
    label              "hand"
    position           x, y, z in world
    confidence         0.0 - 1.0
```

Anything that fills that in drives the demos. Three producers exist:

| Producer | Status | Notes |
| --- | --- | --- |
| `ur5_control/mock_hand_publisher` | works | Synthetic. No camera involved. |
| `ur5_control/hand_target_bridge source:=apriltag` | works on the rig today | Reads a tag's TF frame. |
| `ur5_control/hand_target_bridge source:=detections` | **needs a detector** | Nothing publishes a `hand` class yet. |

## What the Jetson runs today

Per `~/ur5_test/UR5_SETUP.md`, the vision stack lives on the Jetson
(`ssh jetson@tegra-ubuntu`), on the same `ROS_DOMAIN_ID=42`, so its topics and
TF appear on the PC automatically:

| Node | Publishes |
| --- | --- |
| `usb_cam_node_exe` (Logitech Brio, 1280x720 @30) | `/image_raw`, `/camera_info` |
| `image_proc rectify_node` | `/image_rect` |
| `apriltag_ros apriltag_node` | `/detections`, TF frames `tag_1`, `tag_3`, `tag_12` under `default_cam` |
| `ur_camera extrinsics_publisher` | static TF `world -> default_cam` |

Configured tags: `tag_1` (33 mm), `tag_3` (108 mm), `tag_12` (108 mm).

That chain is what makes the AprilTag source work with no extra code: tag poses
land in `world` already, which is the frame this stack plans in.

### Interim hand tracking that works now

Tape **tag_1** (33 mm — the only one small enough to wear) to a wristband:

```bash
ros2 launch ur5_bringup control.launch.py hand_source:=apriltag tag_frame:=tag_1
python3 demos/follow_hand.py
```

Limits worth knowing before you demo it to anyone:

- **It tracks a tag, not a hand.** The offset from the tag to the palm is
  whatever you taped; `--standoff` absorbs it, roughly.
- **Occlusion is total.** Turn the wrist and the tag vanishes; the demos treat
  that as "target lost" (correct, but it happens often).
- **A single camera means depth comes from tag geometry.** At 33 mm the range
  estimate is the noisy axis. Expect a few cm of jitter along the camera's view
  direction — the bridge's smoothing is doing real work here.
- **Extrinsics decide accuracy.** If tags appear in the wrong place in RViz,
  re-run `ur_camera calibrate`. Every downstream number inherits that error.

## To complete it: a hand detector

The gap is one node on the Jetson publishing hand positions. Requirements:

1. **Detect a hand** in `/image_rect` — MediaPipe Hands is the usual pick and
   runs on the Jetson; any keypoint or bbox detector works.
2. **Give it a depth.** The Brio is monocular. Either:
   - add an RGB-D camera (RealSense) and read depth at the keypoint, or
   - keep the mono camera and infer range from apparent hand size (crude,
     ±5 cm at best), or
   - mount a second camera and triangulate.
   This is the real work; 2D detection alone is not enough to reach for
   something.
3. **Publish** `vision_msgs/Detection3DArray` with `class_id: "hand"` and the
   3D centre in a frame TF can resolve to `world` (`default_cam` is fine —
   `extrinsics_publisher` already links it).
4. Then, on the PC:
   ```bash
   ros2 launch ur5_bringup control.launch.py hand_source:=detections
   ```
   No motion code changes. `hand_target_bridge` does the label filter, the TF
   transform, the smoothing and the staleness handling.

If your detector publishes something other than `Detection3DArray` (a
`MarkerArray` pair like the old `vlm_detector_node`, or `PoseStamped`), add a
source branch to `ur5_control/hand_target_bridge.py` rather than teaching the
demos a second format.

## Camera-dependent things the demos currently fake

| Demo does this | With perception it would be |
| --- | --- |
| `handover.py` calls the grasp successful when the gripper's OBJ status says the fingers stopped early | Confirm visually that the object is in the gripper. Failing that, ask a human — an operator-confirmation service is cheap and honest |
| `handover.py` releases after the hand holds still for `--hold-sec` | Detect an open, upward-facing palm, and that it is under the object |
| `follow_hand.py` clamps at `--max-y` to stay out of the standing zone | Track the person's body, not just their hand, and keep a real separation distance |
| `pick_and_place.sh` takes hardcoded pick/place coordinates | Object detection and pose estimation feeding the coordinates in |

## If object-level perception comes back into scope

Following a hand needs one labelled point. Picking named objects off a table
needs more, and the extra pieces are worth naming now so they don't get
rediscovered later:

- **Detection → world-frame position**, per object rather than per hand. This is
  `hand_target_bridge`'s job generalised: filter by label, transform via TF,
  smooth, age out. One topic carrying many targets, or one topic per label.
- **Zone classification** — point-in-polygon tests against named regions
  (staging, dropoff, rejected) so the planner can ask "where is this" rather
  than "what are its coordinates". Overlapping zones need a documented priority
  order.
- **Per-object grasp metadata** — approach type, orientation, force, and the
  offset from the detected centroid to the actual grasp point. Config, not code.
- **Tag staleness** — a detection that stopped updating is worse than no
  detection, because it looks valid. Whatever consumes detections needs the
  same age check `hand_target_bridge` applies.
