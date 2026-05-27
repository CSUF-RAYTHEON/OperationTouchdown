#!/usr/bin/env python3
"""Pass Nav2 velocity commands to the motor bridge (optional linear.x sign fix)."""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelNavRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_nav_relay')
        self.declare_parameter('invert_linear_x', False)
        self.declare_parameter('input_topic', 'cmd_vel_nav_out')
        self.declare_parameter('output_topic', 'cmd_vel')
        self._invert = self.get_parameter('invert_linear_x').value
        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value

        self.pub = self.create_publisher(Twist, out_topic, 10)
        self.sub = self.create_subscription(Twist, in_topic, self._cb, 10)
        self.get_logger().info(
            f'Nav cmd relay: {in_topic} -> {out_topic} '
            f'(invert_linear_x={self._invert})'
        )

    def _cb(self, msg: Twist):
        out = Twist()
        out.linear.x = -msg.linear.x if self._invert else msg.linear.x
        out.linear.y = msg.linear.y
        out.linear.z = msg.linear.z
        out.angular.x = msg.angular.x
        out.angular.y = msg.angular.y
        out.angular.z = msg.angular.z
        self.pub.publish(out)


def main():
    rclpy.init()
    node = CmdVelNavRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
