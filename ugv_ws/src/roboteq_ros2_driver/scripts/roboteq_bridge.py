#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import serial
import math
import time

class RoboteqBridge(Node):
    def __init__(self):
        super().__init__('roboteq_bridge')
        
        # --- PHYSICAL CONSTANTS ---
        self.WHEEL_RADIUS = 0.165      
        self.WHEEL_BASE = 0.635        
        self.TICKS_PER_REV = 8000      
        self.LEFT_TRIM = 1.0
        self.declare_parameter('drive_invert_linear', False)
        self.declare_parameter('odom_invert_linear', 1.0)
        self._drive_invert = (
            -1.0 if self.get_parameter('drive_invert_linear').value else 1.0
        )
        self._odom_invert = float(self.get_parameter('odom_invert_linear').value)

        self.ser = None
        try:
            # Use AMA0 for the Pi 5 GPIO serial pins
            self.ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=0.01)
            self.ser.write(b"!C 1 0\r!C 2 0\r")
            time.sleep(0.1)
            self.get_logger().info('Roboteq Bridge: Final Alignment Active')
        except Exception as e:
            self.get_logger().error(f'Supply Chain Break — no serial, odom will not publish: {e}')

        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.velocity_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        self.linear_x = 0.0
        self.angular_z = 0.0
        self.first_run = True
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_left_ticks = 0
        self.last_right_ticks = 0
        self.last_time = self.get_clock().now()

        self.timer = self.create_timer(0.1, self.update_loop)

    def velocity_callback(self, msg):
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z

    def update_loop(self):
        if self.ser is None:
            return

        try:
            self.ser.write(b"?C\r")
            time.sleep(0.01)

            if self.ser.in_waiting > 0:
                raw_data = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                lines = raw_data.split('\r')
                for line in lines:
                    clean_line = line.strip()
                    if "C=" in clean_line:
                        try:
                            content = clean_line.split('=')[1]
                            parts = content.split(':')
                            # Right is Ch 1, Left is Ch 2
                            right_ticks = int(parts[0])
                            left_ticks = int(parts[1])
                            self.calculate_odometry(left_ticks, right_ticks)
                            break
                        except (IndexError, ValueError):
                            continue

            # ROS uses +linear.x = forward; drive_invert_linear maps that to motor wiring.
            linear = self._drive_invert * self.linear_x * 350
            angular = self.angular_z * 200

            ideal_left = linear - angular
            ideal_right = linear + angular

            right_motor_cmd = int(ideal_right)
            left_motor_cmd = int(ideal_left)

            command = f"!G 1 {right_motor_cmd}\r!G 2 {left_motor_cmd}\r"
            self.ser.write(command.encode())
        except serial.SerialException as e:
            self.get_logger().error(f'Serial error in update_loop: {e}', throttle_duration_sec=5.0)

    def calculate_odometry(self, left_ticks, right_ticks):
        current_time = self.get_clock().now()
        if self.first_run:
            self.last_left_ticks = left_ticks
            self.last_right_ticks = right_ticks
            self.last_time = current_time
            self.first_run = False
            return

        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0: return 

        d_left_ticks = left_ticks - self.last_left_ticks
        d_right_ticks = right_ticks - self.last_right_ticks

        tick_to_m = (2.0 * math.pi * self.WHEEL_RADIUS) / self.TICKS_PER_REV
        dist_right = d_right_ticks * tick_to_m
        dist_left = -(d_left_ticks * tick_to_m)  # left encoder counts opposite

        d_center = self._odom_invert * (dist_left + dist_right) / 2.0
        d_th = (dist_right - dist_left) / self.WHEEL_BASE  # left turn = +angular.z

        self.th += d_th
        self.x += d_center * math.cos(self.th)
        self.y += d_center * math.sin(self.th)

        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.th / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.th / 2.0)

        odom.twist.twist.linear.x = d_center / dt
        odom.twist.twist.angular.z = d_th / dt

        self.odom_pub.publish(odom)

        self.last_left_ticks = left_ticks
        self.last_right_ticks = right_ticks
        self.last_time = current_time

    def destroy_node(self):
        if self.ser is not None:
            try:
                self.ser.write(b"!G 1 0\r!G 2 0\r")
                self.ser.close()
            except Exception:
                pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    bridge = RoboteqBridge()
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            bridge.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()