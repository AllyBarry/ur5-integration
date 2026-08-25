# A single tracked point of interest (a hand, a tag, an object centroid).
#
# This is the contract between the perception side and the motion side. Anything
# that can produce a labelled 3D point at a known time can drive the demos:
#   - ur5_control/hand_target_bridge.py  (AprilTag TF, or Detection3DArray)
#   - demos/mock_hand_publisher.py       (no camera at all)
#   - a future hand-keypoint detector on the Jetson  <- see docs/CAMERA_STACK.md

# header.stamp    = when the target was observed (NOT when it was republished)
# header.frame_id = frame `position` is expressed in; consumers expect "world"
std_msgs/Header header

# "hand", "tag_1", "bottle", ...
string label

geometry_msgs/Point position

# 0.0-1.0. Sources with no notion of confidence should publish 1.0.
float64 confidence
