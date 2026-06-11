#!/usr/bin/env python3
"""
Xbox controller speed adjustment and emergency stop node.

Button mapping (Xbox on Linux /dev/input/js0):
  Button 0 = A  — increase speed
  Button 1 = B  — decrease speed
  Button 10 = R3 (right stick click) — toggle EMERGENCY STOP

Speed steps (max_linear_m_s, max_angular_rad_s):
  0: (0.012, 0.55)  minimum
  1: (0.015, 0.60)
  2: (0.025, 0.65)  low-speed test point
  3: (0.030, 0.70)
  4: (0.040, 0.75)
  5: (0.060, 0.85)
  6: (0.080, 0.95)
  7: (0.100, 1.05)
  8: (0.120, 1.15)
  9: (0.150, 1.20)  max

Pipeline:
  teleop + Nav2 both publish to /cmd_vel
  This node subscribes /cmd_vel, clamps to speed step limits, republishes to /cmd_vel_motor
  roboteq_bridge subscribes /cmd_vel_motor (remapped in launch file)

E-STOP (R3 pressed):
  - Blocks /cmd_vel forwarding
  - Publishes zero velocity to /cmd_vel_motor at 10 Hz
  - Press R3 again to clear
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class JoySpeedEstop(Node):
    # Each entry is (max_linear_m_s, max_angular_rad_s)
    SPEED_STEPS = [
        (0.012, 0.55),  # step 0 — increased angular to reliably break static friction
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
    DEFAULT_STEP = 0  # start at minimum

    def __init__(self):
        super().__init__("joy_speed_estop")
        self.speed_idx = self.DEFAULT_STEP
        self.estopped = False
        self._prev_buttons = []

        self.sub_joy = self.create_subscription(Joy, "/joy", self.joy_cb, 10)
        self.sub_cmd = self.create_subscription(Twist, "/cmd_vel", self.cmd_cb, 10)
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel_motor", 10)

        # Publish zero velocity at 10 Hz when e-stopped
        self.estop_timer = self.create_timer(0.1, self.estop_timer_cb)

        max_lin, max_ang = self.SPEED_STEPS[self.speed_idx]
        self.get_logger().info(
            "JoySpeedEstop ready — A=speed up, B=speed down, R3=e-stop toggle"
        )
        self.get_logger().info(
            f"Speed step {self.speed_idx}: max_linear={max_lin} m/s, max_angular={max_ang} rad/s"
        )

    def joy_cb(self, msg: Joy):
        buttons = list(msg.buttons)
        prev = self._prev_buttons if self._prev_buttons else buttons

        def rising(idx):
            return (
                len(buttons) > idx
                and buttons[idx] == 1
                and (len(prev) <= idx or prev[idx] == 0)
            )

        # A (0) — increase speed
        if rising(0):
            self.speed_idx = min(self.speed_idx + 1, len(self.SPEED_STEPS) - 1)
            max_lin, max_ang = self.SPEED_STEPS[self.speed_idx]
            self.get_logger().info(
                f"Speed step {self.speed_idx}: max_linear={max_lin} m/s, max_angular={max_ang} rad/s"
            )

        # B (1) — decrease speed
        if rising(1):
            self.speed_idx = max(self.speed_idx - 1, 0)
            max_lin, max_ang = self.SPEED_STEPS[self.speed_idx]
            self.get_logger().info(
                f"Speed step {self.speed_idx}: max_linear={max_lin} m/s, max_angular={max_ang} rad/s"
            )

        # R3 (10) — toggle e-stop
        if rising(10):
            self.estopped = not self.estopped
            if self.estopped:
                self.get_logger().warn(
                    "*** EMERGENCY STOP ACTIVE — press R3 to resume ***"
                )
            else:
                self.get_logger().info("E-stop cleared — robot may move again")

        self._prev_buttons = buttons

    def cmd_cb(self, msg: Twist):
        if self.estopped:
            return
        max_lin, max_ang = self.SPEED_STEPS[self.speed_idx]
        out = Twist()
        out.linear.x = max(-max_lin, min(max_lin, msg.linear.x))
        out.angular.z = max(-max_ang, min(max_ang, msg.angular.z))
        self.pub_cmd.publish(out)

    def estop_timer_cb(self):
        if self.estopped:
            self.pub_cmd.publish(Twist())  # zero velocity


def main(args=None):
    rclpy.init(args=args)
    node = JoySpeedEstop()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
