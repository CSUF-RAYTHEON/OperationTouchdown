#!/usr/bin/env python3
"""Roboteq HDC2460 ROS2 driver node.

Communicates with the Roboteq HDC2460 dual-channel brushed DC motor
controller via ASCII serial protocol.  Publishes odometry and TF from
encoder counts and accepts cmd_vel / e_stop commands.
"""

import math
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32MultiArray
from tf2_ros import TransformBroadcaster

try:
    import serial
    import serial.serialutil as serialutil
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


class RoboteqBridge(Node):
    """Full Roboteq HDC2460 driver with odometry and e-stop support."""

    def __init__(self):
        super().__init__('roboteq_bridge')

        # ── parameters ──────────────────────────────────────────────────────
        self.declare_parameter('serial_port',    '/dev/ttyACM0')
        self.declare_parameter('baud_rate',      115200)
        self.declare_parameter('wheel_radius',   0.15875)   # m
        self.declare_parameter('track_width',    0.5874)    # m
        self.declare_parameter('gear_ratio',     40.0 / 9.0)  # motor turns per wheel turn
        self.declare_parameter('encoder_cpr',    2048)       # counts per motor rev (4× quadrature)
        self.declare_parameter('max_motor_rpm',  1000)       # clamp limit (motor shaft)
        self.declare_parameter('left_channel',   1)          # Roboteq channel for left motor
        self.declare_parameter('right_channel',  2)          # Roboteq channel for right motor
        self.declare_parameter('use_speed_mode', False)      # True = !S (closed-loop), False = !G (open-loop power)
        self.declare_parameter('odom_rate',      20.0)       # Hz
        self.declare_parameter('cmd_timeout',    1.0)        # s — stop if no cmd_vel received
        self.declare_parameter('watchdog_ms',    500)        # Roboteq hardware watchdog ms

        self._port_name   = self.get_parameter('serial_port').value
        self._baud        = self.get_parameter('baud_rate').value
        self._wheel_r     = self.get_parameter('wheel_radius').value
        self._track_w     = self.get_parameter('track_width').value
        self._gear_ratio  = self.get_parameter('gear_ratio').value
        self._enc_cpr     = self.get_parameter('encoder_cpr').value
        self._max_rpm     = self.get_parameter('max_motor_rpm').value
        self._left_ch     = self.get_parameter('left_channel').value
        self._right_ch    = self.get_parameter('right_channel').value
        self._speed_mode  = self.get_parameter('use_speed_mode').value
        self._odom_rate   = self.get_parameter('odom_rate').value
        self._cmd_timeout = self.get_parameter('cmd_timeout').value
        self._watchdog_ms = self.get_parameter('watchdog_ms').value

        # Derived kinematics
        self._counts_per_wheel_rev = self._enc_cpr * self._gear_ratio  # ≈ 9102.2
        self._dist_per_count = (2.0 * math.pi * self._wheel_r) / self._counts_per_wheel_rev

        # ── odometry state ──────────────────────────────────────────────────
        self._odom_x   = 0.0
        self._odom_y   = 0.0
        self._odom_th  = 0.0
        self._prev_cnt_left  = None
        self._prev_cnt_right = None
        self._odom_lock = threading.Lock()

        # ── e-stop / cmd watchdog ────────────────────────────────────────────
        self._estopped = False
        self._last_cmd_time: Optional[float] = None

        # ── serial handle ────────────────────────────────────────────────────
        self._ser: Optional[object] = None
        self._ser_lock = threading.Lock()

        # ── ROS interfaces ───────────────────────────────────────────────────
        self._cmd_sub   = self.create_subscription(Twist, '/cmd_vel',   self._cmd_vel_cb,   10)
        self._estop_sub = self.create_subscription(Bool,  '/e_stop',    self._estop_cb,     10)

        self._odom_pub  = self.create_publisher(Odometry,           '/odom',         10)
        self._spd_pub   = self.create_publisher(Float32MultiArray,  '/wheel_speeds', 10)
        self._tf_br     = TransformBroadcaster(self)

        period = 1.0 / self._odom_rate
        self._odom_timer = self.create_timer(period, self._odom_loop)

        # ── connect to hardware ──────────────────────────────────────────────
        self._connect()
        self.get_logger().info(
            f'RoboteqBridge started — port={self._port_name} baud={self._baud} '
            f'speed_mode={self._speed_mode}'
        )

    # ──────────────────────────────────────────────────────────────────────────
    # Serial helpers
    # ──────────────────────────────────────────────────────────────────────────

    def _connect(self) -> bool:
        """Open serial port and send initialisation commands."""
        if not SERIAL_AVAILABLE:
            self.get_logger().warn('pyserial not installed — running in simulation/log-only mode')
            return False

        fallbacks = [self._port_name, '/dev/ttyUSB1', '/dev/ttyACM1']
        for port in fallbacks:
            try:
                ser = serial.Serial(port, self._baud, timeout=0.1)
                time.sleep(0.2)
                ser.reset_input_buffer()
                self._ser = ser
                self._port_name = port
                self.get_logger().info(f'Connected to Roboteq on {port}')
                self._init_roboteq()
                return True
            except Exception as e:
                self.get_logger().debug(f'Could not open {port}: {e}')
        self.get_logger().error('Failed to connect to Roboteq on any port — odometry will be simulated')
        return False

    def _init_roboteq(self):
        """Send startup commands to the controller."""
        # Hardware watchdog: auto-stop if no command within watchdog_ms
        self._send_cmd(f'# {self._watchdog_ms}')
        time.sleep(0.05)
        # Clear fault flags
        self._send_cmd('!MG')
        time.sleep(0.05)
        # Set both motors to 0
        self._send_cmd(f'!G {self._left_ch} 0')
        self._send_cmd(f'!G {self._right_ch} 0')

    def _send_cmd(self, cmd: str):
        """Write an ASCII command (adds \\r)."""
        with self._ser_lock:
            if self._ser is None or not self._ser.is_open:
                return
            try:
                self._ser.write((cmd + '\r').encode('ascii'))
            except Exception as e:
                self.get_logger().error(f'Serial write error: {e}')
                self._ser = None

    def _query(self, cmd: str) -> Optional[str]:
        """Send a query command and return the response line."""
        with self._ser_lock:
            if self._ser is None or not self._ser.is_open:
                return None
            try:
                self._ser.reset_input_buffer()
                self._ser.write((cmd + '\r').encode('ascii'))
                line = self._ser.readline().decode('ascii', errors='replace').strip()
                return line if line else None
            except Exception as e:
                self.get_logger().error(f'Serial query error: {e}')
                self._ser = None
                return None

    def _read_encoder_counts(self):
        """Return (count_left, count_right) or (None, None) on failure."""
        # Query both channels; Roboteq responds with e.g. "C=12345"
        resp_l = self._query(f'?C {self._left_ch}')
        resp_r = self._query(f'?C {self._right_ch}')
        try:
            cnt_l = int(resp_l.split('=')[1]) if resp_l and '=' in resp_l else None
            cnt_r = int(resp_r.split('=')[1]) if resp_r and '=' in resp_r else None
            return cnt_l, cnt_r
        except (ValueError, IndexError):
            return None, None

    def _read_motor_speeds_rpm(self):
        """Return (rpm_left, rpm_right) or (None, None) on failure."""
        resp_l = self._query(f'?S {self._left_ch}')
        resp_r = self._query(f'?S {self._right_ch}')
        try:
            rpm_l = float(resp_l.split('=')[1]) if resp_l and '=' in resp_l else None
            rpm_r = float(resp_r.split('=')[1]) if resp_r and '=' in resp_r else None
            return rpm_l, rpm_r
        except (ValueError, IndexError):
            return None, None

    def _reconnect_if_needed(self):
        """Attempt reconnect if serial is gone."""
        with self._ser_lock:
            if self._ser is not None and self._ser.is_open:
                return
        self.get_logger().warn('Serial disconnected, attempting reconnect…')
        self._connect()

    # ──────────────────────────────────────────────────────────────────────────
    # Motor command helpers
    # ──────────────────────────────────────────────────────────────────────────

    def _cmd_to_motors(self, linear_x: float, angular_z: float):
        """Convert Twist to motor commands and write to hardware."""
        if self._estopped:
            return

        half_track = self._track_w / 2.0
        v_left  = linear_x - angular_z * half_track
        v_right = linear_x + angular_z * half_track

        # m/s → motor-shaft RPM
        wheel_rps_left  = v_left  / (2.0 * math.pi * self._wheel_r)
        wheel_rps_right = v_right / (2.0 * math.pi * self._wheel_r)
        rpm_left  = wheel_rps_left  * 60.0 * self._gear_ratio
        rpm_right = wheel_rps_right * 60.0 * self._gear_ratio

        # Clamp
        rpm_left  = max(-self._max_rpm, min(self._max_rpm, rpm_left))
        rpm_right = max(-self._max_rpm, min(self._max_rpm, rpm_right))

        if self._speed_mode:
            self._send_cmd(f'!S {self._left_ch} {int(rpm_left)}')
            self._send_cmd(f'!S {self._right_ch} {int(rpm_right)}')
        else:
            # Open-loop power: scale RPM to [-1000, 1000]
            pwr_left  = int(rpm_left  / self._max_rpm * 1000.0)
            pwr_right = int(rpm_right / self._max_rpm * 1000.0)
            self._send_cmd(f'!G {self._left_ch} {pwr_left}')
            self._send_cmd(f'!G {self._right_ch} {pwr_right}')

        # Publish wheel speeds for verification
        spd_msg = Float32MultiArray()
        spd_msg.data = [float(rpm_left), float(rpm_right)]
        self._spd_pub.publish(spd_msg)

    # ──────────────────────────────────────────────────────────────────────────
    # Callbacks
    # ──────────────────────────────────────────────────────────────────────────

    def _cmd_vel_cb(self, msg: Twist):
        if self._estopped:
            return
        self._last_cmd_time = time.monotonic()
        self._cmd_to_motors(msg.linear.x, msg.angular.z)

    def _estop_cb(self, msg: Bool):
        if msg.data:
            self._estopped = True
            # Hardware emergency stop
            self._send_cmd('!EX')
            self.get_logger().warn('E-STOP ACTIVATED')
        else:
            self._estopped = False
            # Release motor guard
            self._send_cmd('!MG')
            self.get_logger().info('E-STOP released')

    # ──────────────────────────────────────────────────────────────────────────
    # Odometry loop (runs at odom_rate Hz)
    # ──────────────────────────────────────────────────────────────────────────

    def _odom_loop(self):
        """Main periodic loop: read encoders, update odometry, publish."""
        # Reconnect if needed
        if SERIAL_AVAILABLE:
            self._reconnect_if_needed()

        # Watchdog: if no cmd_vel received recently, stop motors
        if (not self._estopped and
                self._last_cmd_time is not None and
                (time.monotonic() - self._last_cmd_time) > self._cmd_timeout):
            self._send_cmd(f'!G {self._left_ch} 0')
            self._send_cmd(f'!G {self._right_ch} 0')
            self._last_cmd_time = None  # suppress repeated stops

        now = self.get_clock().now()

        # Read encoder counts
        cnt_l, cnt_r = self._read_encoder_counts()

        # Fallback: use RPM integration if count query failed
        if cnt_l is None or cnt_r is None:
            rpm_l, rpm_r = self._read_motor_speeds_rpm()
            if rpm_l is None or rpm_r is None:
                self._publish_odom(now, 0.0, 0.0)
                return
            dt = 1.0 / self._odom_rate
            wheel_rps_l = rpm_l / 60.0 / self._gear_ratio
            wheel_rps_r = rpm_r / 60.0 / self._gear_ratio
            v_left  = wheel_rps_l * 2.0 * math.pi * self._wheel_r
            v_right = wheel_rps_r * 2.0 * math.pi * self._wheel_r
            delta_left  = v_left  * dt
            delta_right = v_right * dt
        else:
            if self._prev_cnt_left is None:
                self._prev_cnt_left  = cnt_l
                self._prev_cnt_right = cnt_r
                self._publish_odom(now, 0.0, 0.0)
                return
            delta_left  = (cnt_l - self._prev_cnt_left)  * self._dist_per_count
            delta_right = (cnt_r - self._prev_cnt_right) * self._dist_per_count
            self._prev_cnt_left  = cnt_l
            self._prev_cnt_right = cnt_r

        # Differential-drive kinematics
        delta_s  = (delta_right + delta_left) / 2.0
        delta_th = (delta_right - delta_left) / self._track_w

        with self._odom_lock:
            self._odom_x  += delta_s * math.cos(self._odom_th + delta_th / 2.0)
            self._odom_y  += delta_s * math.sin(self._odom_th + delta_th / 2.0)
            self._odom_th += delta_th

        # Linear and angular velocity for this cycle
        dt = 1.0 / self._odom_rate
        vx = delta_s  / dt
        vth = delta_th / dt

        self._publish_odom(now, vx, vth)

    def _publish_odom(self, stamp: Time, vx: float, vth: float):
        """Publish Odometry message and broadcast TF odom → base_link."""
        with self._odom_lock:
            x  = self._odom_x
            y  = self._odom_y
            th = self._odom_th

        # Quaternion from yaw
        qz = math.sin(th / 2.0)
        qw = math.cos(th / 2.0)

        # TF broadcast
        tf_msg = TransformStamped()
        tf_msg.header.stamp    = stamp.to_msg()
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id  = 'base_link'
        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        self._tf_br.sendTransform(tf_msg)

        # Odometry message
        odom = Odometry()
        odom.header.stamp    = stamp.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        # Covariance (diagonal)
        odom.pose.covariance[0]  = 0.01   # x
        odom.pose.covariance[7]  = 0.01   # y
        odom.pose.covariance[35] = 0.05   # yaw
        odom.twist.twist.linear.x  = vx
        odom.twist.twist.angular.z = vth
        odom.twist.covariance[0]  = 0.01
        odom.twist.covariance[35] = 0.05
        self._odom_pub.publish(odom)

    def destroy_node(self):
        """Clean shutdown: stop motors and close serial."""
        try:
            self._send_cmd(f'!G {self._left_ch} 0')
            self._send_cmd(f'!G {self._right_ch} 0')
        except Exception:
            pass
        with self._ser_lock:
            if self._ser and self._ser.is_open:
                self._ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RoboteqBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
