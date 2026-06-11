#!/usr/bin/env python3
"""
Xbox controller speed adjustment and emergency stop node.

Button mapping (Xbox on Linux /dev/input/js0):
  Button 0 = A  — increase speed
  Button 1 = B  — decrease speed
  Button 5 = RB — hold to drive (teleop enable)
  Button 10 = R3 (right stick click) — toggle EMERGENCY STOP

Pipeline:
  teleop  -> /cmd_vel_teleop
  mission/kill -> /cmd_vel
  Nav2    -> /cmd_vel_nav
  This node merges them (teleop wins when RB held) and publishes /cmd_vel_motor
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


class JoySpeedEstop(Node):
    SPEED_STEPS = [
        (0.012, 0.55),
        (0.015, 0.60),
        (0.025, 0.65),
        (0.030, 0.70),
        (0.040, 0.75),
        (0.060, 0.85),
        (0.080, 0.95),
        (0.100, 1.05),
        (0.120, 1.15),
        (0.150, 1.20),
    ]
    DEFAULT_STEP = 0

    def __init__(self):
        super().__init__("joy_speed_estop")
        self.speed_idx = self.DEFAULT_STEP
        self.estopped = False
        self.hw_killed = False
        self._rb_held = False
        self._prev_buttons = []
        self._teleop_cmd = Twist()
        self._mission_cmd = Twist()
        self._nav_cmd = Twist()

        self.sub_joy = self.create_subscription(Joy, "/joy", self.joy_cb, 10)
        self.sub_teleop = self.create_subscription(Twist, "/cmd_vel_teleop", self.teleop_cb, 10)
        self.sub_mission = self.create_subscription(Twist, "/cmd_vel", self.mission_cb, 10)
        self.sub_nav = self.create_subscription(Twist, "/cmd_vel_nav", self.nav_cb, 10)
        self.sub_estop = self.create_subscription(Bool, "/e_stop", self.estop_hw_cb, 10)
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel_motor", 10)

        self.merge_timer = self.create_timer(0.1, self.merge_timer_cb)
        self.estop_timer = self.create_timer(0.1, self.estop_timer_cb)

        max_lin, max_ang = self.SPEED_STEPS[self.speed_idx]
        self.get_logger().info(
            "JoySpeedEstop ready — hold RB to drive, A/B=speed, R3=e-stop"
        )
        self.get_logger().info(
            f"Speed step {self.speed_idx}: max_linear={max_lin} m/s, max_angular={max_ang} rad/s"
        )

    def joy_cb(self, msg: Joy):
        buttons = list(msg.buttons)
        prev = self._prev_buttons if self._prev_buttons else buttons
        self._rb_held = len(buttons) > 5 and buttons[5] == 1

        def rising(idx):
            return (
                len(buttons) > idx
                and buttons[idx] == 1
                and (len(prev) <= idx or prev[idx] == 0)
            )

        if rising(0):
            self.speed_idx = min(self.speed_idx + 1, len(self.SPEED_STEPS) - 1)
            max_lin, max_ang = self.SPEED_STEPS[self.speed_idx]
            self.get_logger().info(
                f"Speed step {self.speed_idx}: max_linear={max_lin} m/s, max_angular={max_ang} rad/s"
            )

        if rising(1):
            self.speed_idx = max(self.speed_idx - 1, 0)
            max_lin, max_ang = self.SPEED_STEPS[self.speed_idx]
            self.get_logger().info(
                f"Speed step {self.speed_idx}: max_linear={max_lin} m/s, max_angular={max_ang} rad/s"
            )

        if rising(10):
            self.estopped = not self.estopped
            if self.estopped:
                self.get_logger().warn(
                    "*** EMERGENCY STOP ACTIVE — press R3 to resume ***"
                )
            else:
                self.get_logger().info("E-stop cleared — robot may move again")

        self._prev_buttons = buttons

    def estop_hw_cb(self, msg: Bool):
        self.hw_killed = msg.data

    def teleop_cb(self, msg: Twist):
        self._teleop_cmd = msg

    def mission_cb(self, msg: Twist):
        self._mission_cmd = msg

    def nav_cb(self, msg: Twist):
        self._nav_cmd = msg

    def _clamp(self, msg: Twist) -> Twist:
        max_lin, max_ang = self.SPEED_STEPS[self.speed_idx]
        out = Twist()
        out.linear.x = max(-max_lin, min(max_lin, msg.linear.x))
        out.angular.z = max(-max_ang, min(max_ang, msg.angular.z))
        return out

    def _is_nonzero(self, msg: Twist) -> bool:
        return abs(msg.linear.x) > 1e-4 or abs(msg.angular.z) > 1e-4

    def merge_timer_cb(self):
        if self.estopped or self.hw_killed:
            return

        if self._rb_held:
            self.pub_cmd.publish(self._clamp(self._teleop_cmd))
        elif self._is_nonzero(self._mission_cmd):
            self.pub_cmd.publish(self._clamp(self._mission_cmd))
        else:
            self.pub_cmd.publish(self._clamp(self._nav_cmd))

    def estop_timer_cb(self):
        if self.estopped or self.hw_killed:
            self.pub_cmd.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = JoySpeedEstop()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
