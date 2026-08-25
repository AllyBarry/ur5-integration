#!/usr/bin/env python3
"""
Robotiq Hand-E gripper server (Modbus RTU over a virtual serial port).

Exposes:
  /gripper                 ur5_interfaces/action/Gripper   (position + force + speed)
  /gripper/open            std_srvs/srv/Trigger            (convenience)
  /gripper/close           std_srvs/srv/Trigger            (convenience)

Hardware path on this rig (see docs/BRINGUP or ~/ur5_test/UR5_SETUP.md):

    Hand-E --RS485--> UR controller (gripper_bridge.py) --TCP:54322--> socat
        --> /tmp/ttyUR --> this node

so /tmp/ttyUR only exists while socat is running. This rig does NOT use the UR
Tool Communication URCap; do not launch the driver with
use_tool_communication:=true and do not run robotiq_hande_driver's
gripper_controller_preview -- both fight gripper_bridge.py.

Parameters
----------
serial_port (str, default /tmp/ttyUR)
baudrate    (int, default 115200)
use_mock    (bool, default False)
    SIM-HOOK. Skips the serial port entirely and answers goals from a simulated
    gripper state, so the demos run end-to-end with no gripper attached (and
    with MoveIt in mock-hardware mode, with no robot at all). It does NOT
    simulate contact: object_detected is always False.
"""
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger

from ur5_interfaces.action import Gripper as GripperAction

DEFAULT_FORCE = 100
DEFAULT_SPEED = 255
POLL_INTERVAL_S = 0.1
POLL_TIMEOUT_S = 3.0

POSITION_OPEN = 0x00
POSITION_CLOSED = 0xFF

# Robotiq gripper status fields.
OBJ_MOVING = 0x00
OBJ_STOPPED_OPENING = 0x01
OBJ_STOPPED_CLOSING = 0x02
OBJ_AT_TARGET = 0x03
STA_ACTIVATED = 0x03


class GripperServer(Node):
    def __init__(self):
        super().__init__('gripper_server')

        self.declare_parameter('serial_port', '/tmp/ttyUR')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('use_mock', False)

        self._port = self.get_parameter('serial_port').value
        self._baudrate = int(self.get_parameter('baudrate').value)
        self._mock = bool(self.get_parameter('use_mock').value)

        self._serial_lock = threading.Lock()
        self.ser = None
        self._mock_position = POSITION_OPEN

        if self._mock:
            self.get_logger().warn(
                "use_mock:=true -- no serial port, no real gripper. "
                "object_detected will always be False."
            )
        else:
            self._connect_serial()

        self._cb_group = ReentrantCallbackGroup()

        self.create_service(Trigger, '/gripper/open', self._handle_open_service)
        self.create_service(Trigger, '/gripper/close', self._handle_close_service)
        self.get_logger().info("Services ready: /gripper/open, /gripper/close")

        self._action_server = ActionServer(
            self, GripperAction, 'gripper',
            execute_callback=self._execute_action,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._cb_group,
        )
        self.get_logger().info("Action ready: /gripper")

    # ---- Serial / Modbus ----

    def _connect_serial(self):
        try:
            import serial  # imported lazily so use_mock works without pyserial
            self.ser = serial.Serial(self._port, self._baudrate, timeout=1)
            self.get_logger().info(f"Connected to {self._port}")
            self._initialize_gripper()
        except Exception as e:  # noqa: BLE001 - report and stay up, don't crash
            self.get_logger().error(
                f"Failed to open {self._port}: {e}. "
                f"Is socat running? "
                f"socat -d -d pty,link={self._port},raw,echo=0 tcp:<robot_ip>:54322"
            )

    def _crc16(self, data):
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 1:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc.to_bytes(2, byteorder='little')

    def _send_raw(self, frame):
        if self.ser is None:
            return b''
        with self._serial_lock:
            self.ser.reset_input_buffer()
            self.ser.write(frame)
            time.sleep(0.1)
            return self.ser.read(self.ser.in_waiting)

    def _initialize_gripper(self):
        """Reset then activate. The activation sweep takes a few seconds."""
        self.get_logger().info("Resetting and activating gripper...")
        self._send_raw(b'\x09\x10\x03\xE8\x00\x03\x06\x00\x00\x00\x00\x00\x00\x73\x30')
        time.sleep(0.5)
        self._send_raw(b'\x09\x10\x03\xE8\x00\x03\x06\x01\x00\x00\x00\x00\x00\x72\xE1')
        time.sleep(4.0)
        self.get_logger().info("Gripper ready.")

    def _send_position(self, position, speed, force):
        if self._mock:
            self._mock_position = position & 0xFF
            return
        payload = bytearray([
            0x09, 0x10, 0x03, 0xE8, 0x00, 0x03, 0x06,
            0x09, 0x00, 0x00,
            position & 0xFF, speed & 0xFF, force & 0xFF,
        ])
        self._send_raw(payload + self._crc16(payload))

    def _read_status(self):
        if self._mock:
            return {
                'obj': OBJ_AT_TARGET,
                'sta': STA_ACTIVATED,
                'fault': 0,
                'position': self._mock_position,
            }
        payload = bytearray([0x09, 0x03, 0x07, 0xD0, 0x00, 0x03])
        reply = self._send_raw(payload + self._crc16(payload))
        if len(reply) < 11 or reply[1] != 0x03:
            return None
        data = reply[3:9]
        status_byte = data[0]
        self.get_logger().debug(f"raw status bytes: {data.hex()}")
        return {
            'obj': (status_byte >> 6) & 0x03,
            'sta': (status_byte >> 4) & 0x03,
            'fault': data[2],
            'position': data[5],
        }

    # ---- Trigger services ----

    def _handle_open_service(self, request, response):
        self._send_position(POSITION_OPEN, DEFAULT_SPEED, DEFAULT_SPEED)
        time.sleep(0.1)
        response.success = True
        response.message = "Gripper opening"
        return response

    def _handle_close_service(self, request, response):
        self._send_position(POSITION_CLOSED, DEFAULT_SPEED, DEFAULT_SPEED)
        time.sleep(0.1)
        response.success = True
        response.message = "Gripper closing"
        return response

    # ---- Action ----

    def _goal_callback(self, goal_request):
        if self.ser is None and not self._mock:
            self.get_logger().error("Rejecting goal: no serial connection.")
            return GoalResponse.REJECT
        self.get_logger().info(
            f"Gripper goal: target={goal_request.target_position}, "
            f"force={goal_request.force}, speed={goal_request.speed}"
        )
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def _execute_action(self, goal_handle):
        req = goal_handle.request
        target = req.target_position
        force = req.force or DEFAULT_FORCE
        speed = req.speed or DEFAULT_SPEED

        result = GripperAction.Result()

        status = self._read_status()
        if status is None:
            goal_handle.abort()
            result.success = False
            result.message = "Failed to read gripper status."
            return result
        if status['sta'] != STA_ACTIVATED:
            goal_handle.abort()
            result.success = False
            result.message = f"Gripper not activated (sta={status['sta']})."
            return result

        self._send_position(target, speed, force)

        # Poll until the gripper reports it stopped moving, or we time out.
        start = time.monotonic()
        last_status = status
        while True:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = "Cancelled."
                return result
            if time.monotonic() - start > POLL_TIMEOUT_S:
                self.get_logger().warn(
                    f"Gripper still moving after {POLL_TIMEOUT_S}s; reporting last status."
                )
                break

            status = self._read_status()
            if status is None:
                time.sleep(POLL_INTERVAL_S)
                continue
            last_status = status

            fb = GripperAction.Feedback()
            fb.current_position = status['position']
            goal_handle.publish_feedback(fb)

            if status['obj'] != OBJ_MOVING:
                break
            time.sleep(POLL_INTERVAL_S)

        result.final_position = last_status['position']
        result.obj_status = last_status['obj']
        result.object_detected = last_status['obj'] in (
            OBJ_STOPPED_OPENING, OBJ_STOPPED_CLOSING
        )

        if last_status['fault'] != 0:
            goal_handle.abort()
            result.success = False
            result.message = f"Gripper fault {last_status['fault']:#x}."
            self.get_logger().error(result.message)
            return result

        goal_handle.succeed()
        result.success = True
        result.message = (
            f"object_detected={result.object_detected} "
            f"at position {result.final_position}"
        )
        self.get_logger().info(result.message)
        return result


def main(args=None):
    rclpy.init(args=args)
    node = GripperServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser is not None:
            node.ser.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
