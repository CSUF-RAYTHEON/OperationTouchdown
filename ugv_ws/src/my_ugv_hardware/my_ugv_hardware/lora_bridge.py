#!/usr/bin/env python3
"""LoRa USB bridge node for UGV.

Receives mission commands from the UAV via SB Components 915 MHz USB
LoRa dongle and publishes goal poses / mission commands on ROS2 topics.
"""

import json
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


class LoraBridge(Node):
    """Transparent serial bridge to SB Components USB LoRa module."""

    def __init__(self):
        super().__init__('lora_bridge')

        self.declare_parameter('lora_port', '/dev/ttyUSB2')
        self.declare_parameter('lora_baud', 9600)

        self._port_name = self.get_parameter('lora_port').value
        self._baud      = self.get_parameter('lora_baud').value

        # Publishers
        self._goal_pub    = self.create_publisher(PoseStamped, '/lora/goal_pose',    10)
        self._raw_pub     = self.create_publisher(String,      '/lora/raw',          10)
        self._mission_pub = self.create_publisher(String,      '/lora/mission_cmd',  10)

        # Subscriber for outbound messages
        self._tx_sub = self.create_subscription(String, '/lora/tx', self._tx_cb, 10)

        self._ser: Optional[object] = None
        self._ser_lock = threading.Lock()
        self._rx_thread: Optional[threading.Thread] = None
        self._running = True

        self._connect()
        self.get_logger().info(
            f'LoraBridge started — port={self._port_name} baud={self._baud}'
        )

    # ──────────────────────────────────────────────────────────────────────────
    # Serial helpers
    # ──────────────────────────────────────────────────────────────────────────

    def _connect(self) -> bool:
        fallbacks = [self._port_name, '/dev/ttyUSB1', '/dev/ttyUSB3']
        for port in fallbacks:
            try:
                ser = serial.Serial(port, self._baud, timeout=1.0)
                time.sleep(0.2)
                ser.reset_input_buffer()
                self._ser = ser
                self._port_name = port
                self.get_logger().info(f'LoRa connected on {port}')
                self._start_rx_thread()
                return True
            except Exception as e:
                self.get_logger().debug(f'LoRa port {port} not available: {e}')
        self.get_logger().warn('LoRa serial not available — running in offline mode')
        return False

    def _start_rx_thread(self):
        if self._rx_thread is not None and self._rx_thread.is_alive():
            return
        self._rx_thread = threading.Thread(
            target=self._rx_loop, daemon=True, name='lora_rx'
        )
        self._rx_thread.start()

    def _rx_loop(self):
        """Background thread: read newline-delimited messages from LoRa."""
        while self._running:
            with self._ser_lock:
                ser = self._ser
            if ser is None or not ser.is_open:
                time.sleep(1.0)
                self.get_logger().warn('LoRa serial lost, retrying connect…')
                self._connect()
                continue
            try:
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode('ascii', errors='replace').strip()
                if line:
                    self._process_rx(line)
            except Exception as e:
                self.get_logger().error(f'LoRa RX error: {e}')
                with self._ser_lock:
                    self._ser = None
                time.sleep(1.0)

    # ──────────────────────────────────────────────────────────────────────────
    # Message processing
    # ──────────────────────────────────────────────────────────────────────────

    def _process_rx(self, line: str):
        """Parse incoming LoRa message and publish to appropriate topic."""
        # Always publish raw
        raw_msg = String()
        raw_msg.data = line
        self._raw_pub.publish(raw_msg)
        self.get_logger().info(f'LoRa RX: {line}')

        # ── GOAL:x,y,theta ───────────────────────────────────────────────────
        if line.startswith('GOAL:'):
            try:
                parts = line[5:].split(',')
                x     = float(parts[0])
                y     = float(parts[1])
                theta = float(parts[2]) if len(parts) > 2 else 0.0
                self._publish_goal(x, y, theta)
                self._send_ack('ACK:received')
                return
            except (ValueError, IndexError) as e:
                self.get_logger().error(f'Malformed GOAL message "{line}": {e}')
                return

        # ── JSON format ───────────────────────────────────────────────────────
        if line.startswith('{'):
            try:
                data  = json.loads(line)
                x     = float(data['x'])
                y     = float(data['y'])
                theta = float(data.get('theta', 0.0))
                self._publish_goal(x, y, theta)
                self._send_ack('ACK:received')
                return
            except (json.JSONDecodeError, KeyError, ValueError) as e:
                self.get_logger().error(f'Malformed JSON message "{line}": {e}')
                return

        # ── Mission commands ──────────────────────────────────────────────────
        cmd_map = {'STRAIGHT', 'STOP', 'START', 'PAUSE', 'RESUME'}
        upper = line.strip().upper()
        if upper in cmd_map:
            cmd_msg = String()
            cmd_msg.data = upper
            self._mission_pub.publish(cmd_msg)
            self.get_logger().info(f'Mission command received: {upper}')
            self._send_ack(f'ACK:{upper}')
            return

        self.get_logger().debug(f'Unknown LoRa message: {line}')

    def _publish_goal(self, x: float, y: float, theta: float):
        """Build and publish a PoseStamped in the map frame."""
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        # Yaw → quaternion
        import math
        msg.pose.orientation.z = math.sin(theta / 2.0)
        msg.pose.orientation.w = math.cos(theta / 2.0)
        self._goal_pub.publish(msg)
        self.get_logger().info(f'Published goal: x={x:.3f} y={y:.3f} theta={theta:.3f}')

    def _send_ack(self, ack: str):
        """Transmit ACK string back to UAV over LoRa."""
        with self._ser_lock:
            if self._ser is None or not self._ser.is_open:
                return
            try:
                self._ser.write((ack + '\n').encode('ascii'))
            except Exception as e:
                self.get_logger().error(f'LoRa TX error: {e}')

    # ──────────────────────────────────────────────────────────────────────────
    # Subscriber callback — transmit to UAV
    # ──────────────────────────────────────────────────────────────────────────

    def _tx_cb(self, msg: String):
        """Write a message out over LoRa serial."""
        text = msg.data.strip()
        if not text.endswith('\n'):
            text += '\n'
        with self._ser_lock:
            if self._ser is None or not self._ser.is_open:
                self.get_logger().warn('LoRa TX: serial not available')
                return
            try:
                self._ser.write(text.encode('ascii'))
                self.get_logger().debug(f'LoRa TX: {text.strip()}')
            except Exception as e:
                self.get_logger().error(f'LoRa TX error: {e}')

    def destroy_node(self):
        self._running = False
        with self._ser_lock:
            if self._ser and self._ser.is_open:
                self._ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LoraBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
