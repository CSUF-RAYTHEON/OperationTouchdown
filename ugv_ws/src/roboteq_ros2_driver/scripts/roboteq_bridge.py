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
        
        # --- CONSTANTS ---
        self.WHEEL_RADIUS = 0.165      
        self.WHEEL_BASE = 0.635        
        self.TICKS_PER_REV = 2000      
        
        # --- CALIBRATION ---
        # 1.05 = 5% extra power to the left motor to fix the drift
        self.LEFT_TRIM = 1.05  

        # 1. Serial Setup (USB Port)
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.01)
        self.get_logger().info('Roboteq Bridge: Connected via USB')
        
        # Reset encoder counts to 0
        self.ser.write(b"!C 1 0\r!C 2 0\r")
        time.sleep(0.1) 

        # 2. ROS 2 Pubs/Subs
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.velocity_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # 3. State Variables
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.first_run = True
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_left_ticks = 0
        self.last_right_ticks = 0
        self.last_time = self.get_clock().now()

        # 4. Timer (20Hz)
        self.timer = self.create_timer(0.05, self.update_loop)

    def velocity_callback(self, msg):
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z

    def update_loop(self):
        # Request encoder counts
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
                        # Logic for motor orientations
                        right_ticks = int(parts[0])
                        left_ticks = int(parts[1])
                        self.calculate_odometry(left_ticks, right_ticks)
                        break 
                    except (IndexError, ValueError):
                        continue

        # --- SEND MOTOR COMMANDS ---
        # Apply the direction flips (linear and angular)
        linear = -self.linear_x * 60
        angular = self.angular_z * 250
        
        # Calculate raw speeds
        left_raw = int(linear + angular)
        right_raw = int(linear - angular)
        
        # APPLY LEFT TRIM SUBSIDY
        # This gives the left motor a head start to correct for its weakness
        left_val = int(left_raw * self.LEFT_TRIM)
        right_val = right_raw

        command = f"!G 1 {-right_val}\r!G 2 {left_val}\r"
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

        d_left = left_ticks - self.last_left_ticks
        d_right = right_ticks - self.last_right_ticks
        
        dist_left = (d_left / self.TICKS_PER_REV) * (2 * math.pi * self.WHEEL_RADIUS)
        dist_right = (d_right / self.TICKS_PER_REV) * (2 * math.pi * self.WHEEL_RADIUS)
        
        d_center = (dist_left + dist_right) / 2.0
        d_th = (dist_right - dist_left) / self.WHEEL_BASE

        # Velocity Calculation
        vx = d_center / dt
        vth = d_th / dt

        self.x += d_center * math.cos(self.th)
        self.y += d_center * math.sin(self.th)
        self.th += d_th

        # Prepare Odometry Message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint" 
        
        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.th / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.th / 2.0)

        # Twist
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vth

        # Trust Factor (Covariance)
        odom.pose.covariance = [0.001] * 36
        odom.twist.covariance = [0.001] * 36
        
        self.odom_pub.publish(odom)

        self.last_left_ticks = left_ticks
        self.last_right_ticks = right_ticks
        self.last_time = current_time

    def destroy_node(self):
        # Stop motors on shutdown
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