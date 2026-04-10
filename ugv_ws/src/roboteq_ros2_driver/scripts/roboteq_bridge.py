#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros
import serial
import math
import time

class RoboteqBridge(Node):
    def __init__(self):
        super().__init__('roboteq_bridge')
        
        # --- ROVER CONSTANTS (Fill these in!) ---
        self.WHEEL_RADIUS = 0.05       # Meters (e.g., 0.05 for 10cm wheel)
        self.WHEEL_BASE = 0.3          # Meters (Distance between left/right wheels)
        self.TICKS_PER_REV = 2000      # Encoder pulses per 1 full wheel turn
        # ----------------------------------------

        # 1. Serial Setup
        self.ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=0.01)
        self.get_logger().info('Roboteq Bridge: Connected & Requesting Encoders')
        self.ser.write(b"!C 1 0\r!C 2 0\r")
        time.sleep(0.1) 
        
        self.get_logger().info('Roboteq Bridge: Connected & Resetting Encoders')

        # 2. ROS 2 Pubs/Subs
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.velocity_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 3. State Variables
        self.linear_x = 0.0
        self.angular_z = 0.0
        
        # Odometry state
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
        """Main loop: Reads Encoders, Calculates Odom, and Sends Motor Commands"""
        
        # --- PART A: READ ENCODERS (?C) ---
        self.ser.reset_input_buffer()
        
        # 2. Command the Roboteq to send Encoders
        self.ser.write(b"?C\r")
        
        # 3. Give the Roboteq a tiny moment to process (Pi 5 is very fast!)
        time.sleep(0.01)

        # 4. Read everything currently in the serial buffer
        if self.ser.in_waiting > 0:
            raw_data = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
            lines = raw_data.split('\r')
            
            for line in lines:
                clean_line = line.strip()
                # Use 'in' instead of 'startswith' to be extra safe
                if "C=" in clean_line:
                    try:
                        # Extract numbers from C=123:456
                        content = clean_line.split('=')[1]
                        parts = content.split(':')
                        
                        right_ticks = -int(parts[0])
                        left_ticks = int(parts[1])
                        
                        # This triggers the publisher!
                        self.calculate_odometry(left_ticks, right_ticks)
                        # Log it once so we know it worked
                       # self.get_logger().info(f"Odom Active: L{left_ticks} R{right_ticks}")
                        break 
                    except (IndexError, ValueError):
                        continue
        # --- PART B: SEND MOTOR COMMANDS (!G) ---
        linear = self.linear_x * 60
        angular = self.angular_z * 275
        
        left_val = int(linear + angular)
        right_val = int(linear - angular)

        # Mirror Logic
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
        
        # Calculate change in ticks
        d_left = left_ticks - self.last_left_ticks
        d_right = right_ticks - self.last_right_ticks
        
        # Convert ticks to meters
        dist_left = (d_left / self.TICKS_PER_REV) * (2 * math.pi * self.WHEEL_RADIUS)
        dist_right = (d_right / self.TICKS_PER_REV) * (2 * math.pi * self.WHEEL_RADIUS)
        
        # Distance center and rotation
        d_center = (dist_left + dist_right) / 2.0
        d_th = (dist_right - dist_left) / self.WHEEL_BASE

        # Update pose
        self.x += d_center * math.cos(self.th)
        self.y += d_center * math.sin(self.th)
        self.th += d_th

        # Prepare Odometry Message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        # Set Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        # Convert Theta to Quaternion
        odom.pose.pose.orientation.z = math.sin(self.th / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.th / 2.0)
        
        self.odom_pub.publish(odom)

        #Prepare Transform Message
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # Set translation from your calculated x, y
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        #Set rotation using the same math as odom orientation
        t.transform.rotation.z = math.sin(self.th / 2.0)
        t.transform.rotation.w = math.cos(self.th / 2.0)

        #Broadcast
        self.tf_broadcaster.sendTransform(t)


        # Save for next loop
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

