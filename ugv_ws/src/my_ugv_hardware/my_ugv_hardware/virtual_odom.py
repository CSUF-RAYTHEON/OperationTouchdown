#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
import math

class VirtualOdom(Node):
    def __init__(self):
        super().__init__('virtual_odom')
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Robot State
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.vx = 0.0
        self.vth = 0.0
        
        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.update_position)

    def cmd_vel_callback(self, msg):
        self.vx = msg.linear.x
        self.vth = msg.angular.z

    def update_position(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        # Calculate displacement
        self.x += self.vx * math.cos(self.th) * dt
        self.y += self.vx * math.sin(self.th) * dt
        self.th += self.vth * dt

        # Broadcast the Transform (This moves the box in Foxglove!)
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = math.sin(self.th / 2.0)
        t.transform.rotation.w = math.cos(self.th / 2.0)
        
        self.tf_broadcaster.sendTransform(t)
        self.last_time = current_time

def main():
    rclpy.init()
    node = VirtualOdom()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

