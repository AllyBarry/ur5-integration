#!/usr/bin/env python3
"""
mock_hand_publisher: a hand that doesn't need a camera.

SIM-HOOK. Publishes ur5_interfaces/TrackedTarget on /perception/hand_target --
the same topic and contract hand_target_bridge produces -- so the follow and
handover demos run with no Jetson, no camera, and (with the driver in
use_mock_hardware mode) no robot.

Use it to shake out the demo logic, the workspace clamp and the step leash
before pointing anything at a real person.

Motion patterns (`pattern` parameter)
-------------------------------------
static      Sit at (centre_x, centre_y, centre_z).
circle      Horizontal circle of `radius` about the centre, one lap per `period`.
lissajous   x and y on a 2:3 frequency ratio -- sweeps more of the workspace and
            never repeats the same approach twice in a row.

`jitter_stddev` adds per-sample Gaussian noise so the consumer's smoothing and
deadband get exercised rather than fed a mathematically perfect target.
`dropout_probability` randomly withholds samples to imitate a detector losing
the hand, which is what the demos' staleness timeout exists to survive.
"""
import math
import random

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from visualization_msgs.msg import Marker

from ur5_interfaces.msg import TrackedTarget

PATTERN_STATIC = 'static'
PATTERN_CIRCLE = 'circle'
PATTERN_LISSAJOUS = 'lissajous'
VALID_PATTERNS = (PATTERN_STATIC, PATTERN_CIRCLE, PATTERN_LISSAJOUS)


class MockHandPublisher(Node):
    def __init__(self):
        super().__init__('mock_hand_publisher')

        self.declare_parameter('pattern', PATTERN_CIRCLE)
        # Centre of the pattern, world frame. Defaults sit over the middle of
        # the table, roughly where a person would hold their hand out.
        self.declare_parameter('centre_x', 0.0)
        self.declare_parameter('centre_y', 0.45)
        self.declare_parameter('centre_z', 1.15)
        self.declare_parameter('radius', 0.15)
        self.declare_parameter('period_sec', 12.0)
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('label', 'hand')
        self.declare_parameter('jitter_stddev', 0.004)
        self.declare_parameter('dropout_probability', 0.0)
        self.declare_parameter('publish_marker', True)

        self._pattern = self.get_parameter('pattern').value
        if self._pattern not in VALID_PATTERNS:
            raise ValueError(f"pattern '{self._pattern}' not in {VALID_PATTERNS}")

        self._centre = (
            float(self.get_parameter('centre_x').value),
            float(self.get_parameter('centre_y').value),
            float(self.get_parameter('centre_z').value),
        )
        self._radius = float(self.get_parameter('radius').value)
        self._period = float(self.get_parameter('period_sec').value)
        self._label = self.get_parameter('label').value
        self._jitter = float(self.get_parameter('jitter_stddev').value)
        self._dropout = float(self.get_parameter('dropout_probability').value)

        self._pub = self.create_publisher(
            TrackedTarget, '/perception/hand_target', 10
        )
        self._marker_pub = (
            self.create_publisher(Marker, '/perception/hand_target_marker', 10)
            if bool(self.get_parameter('publish_marker').value) else None
        )

        self._t = 0.0
        self._dt = 1.0 / float(self.get_parameter('rate_hz').value)
        self.create_timer(self._dt, self._on_timer)

        self.get_logger().info(
            f"mock_hand_publisher: pattern={self._pattern}, centre={self._centre}, "
            f"radius={self._radius}m, period={self._period}s -> "
            f"/perception/hand_target"
        )

    def _position(self):
        cx, cy, cz = self._centre
        if self._pattern == PATTERN_STATIC:
            return cx, cy, cz

        phase = 2.0 * math.pi * (self._t / self._period)
        if self._pattern == PATTERN_CIRCLE:
            return (
                cx + self._radius * math.cos(phase),
                cy + self._radius * math.sin(phase),
                cz,
            )
        # lissajous
        return (
            cx + self._radius * math.sin(2.0 * phase),
            cy + self._radius * math.sin(3.0 * phase),
            cz + 0.5 * self._radius * math.sin(phase),
        )

    def _on_timer(self):
        self._t += self._dt

        if self._dropout > 0.0 and random.random() < self._dropout:
            return

        x, y, z = self._position()
        if self._jitter > 0.0:
            x += random.gauss(0.0, self._jitter)
            y += random.gauss(0.0, self._jitter)
            z += random.gauss(0.0, self._jitter)

        msg = TrackedTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.label = self._label
        msg.position.x, msg.position.y, msg.position.z = x, y, z
        msg.confidence = 1.0
        self._pub.publish(msg)

        if self._marker_pub is not None:
            m = Marker()
            m.header = msg.header
            m.ns = 'hand_target'
            m.id = 0
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position = msg.position
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.06
            m.color.r, m.color.g, m.color.b, m.color.a = 0.9, 0.6, 0.1, 0.8
            m.lifetime = Duration(seconds=0.5).to_msg()
            self._marker_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = MockHandPublisher()
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
