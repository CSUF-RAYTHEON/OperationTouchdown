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

        try:
            # Use AMA0 for the Pi 5 GPIO serial pins
            self.ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=0.01)
            self.get_logger().info('Roboteq Bridge: Final Alignment Active')
        except Exception as e:
            self.get_logger().error(f'Supply Chain Break: {e}')
        
        self.ser.write(b"!C 1 0\r!C 2 0\r")
        time.sleep(0.1) 

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

        # --- MOTOR COMMANDS: THE FLIP ---
        linear = self.linear_x * 350
        angular = self.angular_z * 100
        
        # 1. Calculate "Ideal" Differential Logic
        ideal_left = linear - angular
        ideal_right = linear + angular

        # 2. APPLY THE "TACOMA" INVERSION
        # Ch 1 is Right, Ch 2 is Left.
        # Since Ch 2 is flipped, we invert its ideal command.
        right_motor_cmd = int(ideal_right)
        left_motor_cmd = int(ideal_left) # This is the "Pivot Fix"

        command = f"!G 1 {right_motor_cmd}\r!G 2 {left_motor_cmd}\r"
        self.ser.write(command.encode())

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
        
        # Try adding a slight multiplier to "correct" the physical bias
# If the rover thinks it's turning left, we need to scale down the right wheel's impact
# Ensure these lines are perfectly aligned with the block above them
        dist_right = (d_right_ticks / self.TICKS_PER_REV) * (2 * math.pi * self.WHEEL_RADIUS)
        dist_left = -(d_left_ticks / self.TICKS_PER_REV) * (2 * math.pi * self.WHEEL_RADIUS) 
        
        # --- FIX THIS LINE ---
        # Make sure there is NO extra space before 'd_center'
        d_center = (dist_left + dist_right) / 2.0
        d_th = (dist_right - dist_left) / self.WHEEL_BASE

        # Integrate pose
        self.x += d_center * math.cos(self.th)
        self.y += d_center * math.sin(self.th)
        self.th += d_th

        # Prepare Message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint" 
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.th / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.th / 2.0)

        # Diagonal pose covariance: x=0.1, y=0.1, yaw=0.2
        odom.pose.covariance[0] = 0.1   # x
        odom.pose.covariance[7] = 0.1   # y
        odom.pose.covariance[35] = 0.2  # yaw

        odom.twist.twist.linear.x = d_center / dt
        odom.twist.twist.angular.z = d_th / dt

        # Diagonal twist covariance
        odom.twist.covariance[0] = 0.1   # vx
        odom.twist.covariance[35] = 0.2  # vyaw
        
        self.odom_pub.publish(odom)

        self.last_left_ticks = left_ticks
        self.last_right_ticks = right_ticks
        self.last_time = current_time

    def destroy_node(self):
        self.ser.write(b"!G 1 0\r!G 2 0\r")
        self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    bridge = RoboteqBridge()
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()