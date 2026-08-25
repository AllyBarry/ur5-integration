#!/usr/bin/env python3
"""
hand_target_bridge: perception -> motion seam.

Turns whatever the camera stack happens to publish into one stable contract:

    /perception/hand_target   ur5_interfaces/TrackedTarget   (world frame)

Everything downstream (demos/follow_hand.py, demos/handover.py) consumes only
that topic, so swapping the detector never touches the motion code.

Sources (`source` parameter)
----------------------------
apriltag    Look up world -> <tag_frame> in TF and republish it as a target.
            WORKS TODAY: the Jetson already runs apriltag_ros plus
            ur_camera/extrinsics_publisher, which parents `default_cam` to
            `world`, so tag frames land in world coordinates with no extra
            work. Tape tag_1 (33 mm) to a wristband and the follow demo runs
            on the rig as it stands.

detections  Subscribe to a vision_msgs/Detection3DArray, keep the highest-score
            hypothesis whose class_id matches `detection_label`, transform its
            centre into `world`.
            CAMERA-STACK: needs a detector publishing a *hand* class. The Jetson
            currently runs AprilTag only -- no hand/keypoint detector exists yet.
            See docs/CAMERA_STACK.md for what to add.

Smoothing
---------
Raw detections jitter, and jitter becomes arm motion. An exponential moving
average (`smoothing_alpha`, 0 = frozen, 1 = no smoothing) runs here rather than
in the demos so every consumer gets the same conditioned signal. Targets older
than `timeout_sec` stop being published at all -- the demos treat "no message"
as "no target", which is the behaviour you want when someone walks away.
"""
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

import tf2_ros
from tf2_ros import TransformException
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped transforms)

from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker

from ur5_interfaces.msg import TrackedTarget

SOURCE_APRILTAG = 'apriltag'
SOURCE_DETECTIONS = 'detections'
VALID_SOURCES = (SOURCE_APRILTAG, SOURCE_DETECTIONS)


class HandTargetBridge(Node):
    def __init__(self):
        super().__init__('hand_target_bridge')

        self.declare_parameter('source', SOURCE_APRILTAG)
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('label', 'hand')
        # apriltag source
        self.declare_parameter('tag_frame', 'tag_1')
        # detections source
        self.declare_parameter('detection_topic', '/vlm_detector_node/detections')
        self.declare_parameter('detection_label', 'hand')
        self.declare_parameter('min_confidence', 0.4)
        # common
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('smoothing_alpha', 0.35)
        self.declare_parameter('timeout_sec', 1.0)
        self.declare_parameter('publish_marker', True)

        self._source = self.get_parameter('source').value
        self._world_frame = self.get_parameter('world_frame').value
        self._label = self.get_parameter('label').value
        self._tag_frame = self.get_parameter('tag_frame').value
        self._min_confidence = float(self.get_parameter('min_confidence').value)
        self._alpha = float(self.get_parameter('smoothing_alpha').value)
        self._timeout = float(self.get_parameter('timeout_sec').value)
        self._publish_marker = bool(self.get_parameter('publish_marker').value)

        if self._source not in VALID_SOURCES:
            raise ValueError(
                f"source '{self._source}' not in {VALID_SOURCES}"
            )

        self._smoothed = None       # (x, y, z)
        self._confidence = 0.0
        self._last_seen = None      # rclpy.time.Time

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        self._pub = self.create_publisher(
            TrackedTarget, '/perception/hand_target', qos
        )
        self._marker_pub = (
            self.create_publisher(Marker, '/perception/hand_target_marker', qos)
            if self._publish_marker else None
        )

        if self._source == SOURCE_DETECTIONS:
            # Imported lazily: vision_msgs is only needed on this path, and it
            # is not installed on every machine that runs the demos.
            from vision_msgs.msg import Detection3DArray
            topic = self.get_parameter('detection_topic').value
            self._det_label = self.get_parameter('detection_label').value
            self.create_subscription(
                Detection3DArray, topic, self._on_detections, 10
            )
            self.get_logger().info(
                f"source=detections, topic={topic}, label='{self._det_label}'"
            )
        else:
            self.get_logger().info(
                f"source=apriltag, tracking TF {self._world_frame} -> {self._tag_frame}"
            )

        rate = float(self.get_parameter('publish_rate_hz').value)
        self.create_timer(1.0 / rate, self._on_timer)

        self.get_logger().info(
            f"hand_target_bridge ready -> /perception/hand_target "
            f"(alpha={self._alpha}, timeout={self._timeout}s)"
        )

    # ---- Sources ----

    def _poll_apriltag(self):
        """Read the tag's current pose out of TF. Returns (x, y, z) or None."""
        try:
            tf = self._tf_buffer.lookup_transform(
                self._world_frame, self._tag_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as e:
            self.get_logger().debug(f"TF {self._tag_frame} unavailable: {e}")
            return None

        # A tag that has dropped out of view leaves a stale transform in the
        # buffer rather than disappearing, so age it ourselves.
        stamp = rclpy.time.Time.from_msg(tf.header.stamp)
        if stamp.nanoseconds > 0:
            age = (self.get_clock().now() - stamp).nanoseconds / 1e9
            if age > self._timeout:
                self.get_logger().debug(f"TF {self._tag_frame} is {age:.2f}s stale.")
                return None

        t = tf.transform.translation
        return (t.x, t.y, t.z)

    def _on_detections(self, msg):
        for det in msg.detections:
            if not det.results:
                continue
            best = max(det.results, key=lambda h: h.hypothesis.score)
            if best.hypothesis.class_id != self._det_label:
                continue
            if best.hypothesis.score < self._min_confidence:
                continue

            src_frame = det.header.frame_id or msg.header.frame_id
            c = det.bbox.center.position
            world = self._to_world(src_frame, c.x, c.y, c.z, msg.header.stamp)
            if world is None:
                continue
            self._update(world, float(best.hypothesis.score))
            return

    def _to_world(self, src_frame, x, y, z, stamp):
        if not src_frame or src_frame == self._world_frame:
            return (x, y, z)
        pt = PointStamped()
        pt.header.frame_id = src_frame
        pt.header.stamp = stamp
        pt.point.x, pt.point.y, pt.point.z = x, y, z
        try:
            out = self._tf_buffer.transform(
                pt, self._world_frame, timeout=Duration(seconds=0.1)
            )
        except TransformException as e:
            self.get_logger().warn(
                f"Could not transform {src_frame} -> {self._world_frame}: {e}",
                throttle_duration_sec=2.0,
            )
            return None
        return (out.point.x, out.point.y, out.point.z)

    # ---- Filtering + publishing ----

    def _update(self, position, confidence):
        if self._smoothed is None:
            self._smoothed = position
        else:
            a = self._alpha
            self._smoothed = tuple(
                a * new + (1.0 - a) * old
                for new, old in zip(position, self._smoothed)
            )
        self._confidence = confidence
        self._last_seen = self.get_clock().now()

    def _on_timer(self):
        if self._source == SOURCE_APRILTAG:
            observed = self._poll_apriltag()
            if observed is not None:
                self._update(observed, 1.0)

        if self._smoothed is None or self._last_seen is None:
            return

        age = (self.get_clock().now() - self._last_seen).nanoseconds / 1e9
        if age > self._timeout:
            # Deliberately silent: consumers treat absence as "no target".
            if self._smoothed is not None:
                self.get_logger().info(
                    f"Target lost ({age:.1f}s since last observation).",
                    throttle_duration_sec=5.0,
                )
            self._smoothed = None
            return

        msg = TrackedTarget()
        msg.header.stamp = self._last_seen.to_msg()
        msg.header.frame_id = self._world_frame
        msg.label = self._label
        msg.position.x, msg.position.y, msg.position.z = self._smoothed
        msg.confidence = self._confidence
        self._pub.publish(msg)

        if self._marker_pub is not None:
            self._marker_pub.publish(self._make_marker(msg))

    def _make_marker(self, target):
        m = Marker()
        m.header = target.header
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'hand_target'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position = target.position
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.06
        m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.9, 0.4, 0.8
        m.lifetime = Duration(seconds=0.5).to_msg()
        return m


def main(args=None):
    rclpy.init(args=args)
    node = HandTargetBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
