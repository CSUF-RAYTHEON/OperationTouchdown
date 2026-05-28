#!/usr/bin/env python3
"""Kill switch monitor node.

Monitors a hardware GPIO kill switch (via RPi.GPIO) or a software
/kill_switch_hw Bool topic and immediately stops all motion by
publishing to /e_stop and /cmd_vel.

Competition requirement: motion must halt within 3 seconds of activation.
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except (ImportError, RuntimeError):
    GPIO_AVAILABLE = False


class KillSwitch(Node):
    """Hardware and software kill switch with <3 second halt guarantee."""

    def __init__(self):
        super().__init__('kill_switch')

        self.declare_parameter('gpio_pin',        17)      # BCM pin number
        self.declare_parameter('use_gpio',        True)    # False = software-only
        self.declare_parameter('active_low',      True)    # True = pulled high, switch grounds
        self.declare_parameter('poll_rate_hz',    20.0)    # GPIO poll frequency

        self._gpio_pin   = self.get_parameter('gpio_pin').value
        self._use_gpio   = self.get_parameter('use_gpio').value
        self._active_low = self.get_parameter('active_low').value
        self._poll_rate  = self.get_parameter('poll_rate_hz').value

        self._killed = False

        # Publishers
        self._estop_pub  = self.create_publisher(Bool,  '/e_stop',  1)
        self._cmd_pub    = self.create_publisher(Twist, '/cmd_vel', 1)

        # Software kill switch subscriber
        self._hw_sub = self.create_subscription(
            Bool, '/kill_switch_hw', self._hw_cb, 1
        )

        # GPIO setup
        if self._use_gpio and GPIO_AVAILABLE:
            try:
                GPIO.setmode(GPIO.BCM)
                pull = GPIO.PUD_UP if self._active_low else GPIO.PUD_DOWN
                GPIO.setup(self._gpio_pin, GPIO.IN, pull_up_down=pull)
                GPIO.add_event_detect(
                    self._gpio_pin,
                    GPIO.BOTH,
                    callback=self._gpio_cb,
                    bouncetime=50
                )
                self.get_logger().info(
                    f'Kill switch GPIO {self._gpio_pin} configured (active_low={self._active_low})'
                )
            except Exception as e:
                self.get_logger().error(f'GPIO setup failed: {e} — falling back to poll')
                self._use_gpio = False
        elif self._use_gpio and not GPIO_AVAILABLE:
            self.get_logger().warn('RPi.GPIO not available — using software kill switch only')

        # Polling timer (backup / software mode)
        period = 1.0 / self._poll_rate
        self._poll_timer = self.create_timer(period, self._poll)

        # Periodic zero-velocity publisher to ensure halt is maintained
        self._halt_timer = self.create_timer(0.1, self._maintain_halt)

        self.get_logger().info('KillSwitch node ready')

    # ──────────────────────────────────────────────────────────────────────────

    def _gpio_cb(self, channel: int):
        """GPIO interrupt callback (runs in RPi.GPIO thread)."""
        try:
            state = GPIO.input(channel)
            activated = (state == GPIO.LOW) if self._active_low else (state == GPIO.HIGH)
            if activated and not self._killed:
                self._activate()
            elif not activated and self._killed:
                self._deactivate()
        except Exception as e:
            self.get_logger().error(f'GPIO callback error: {e}')

    def _hw_cb(self, msg: Bool):
        """Software /kill_switch_hw topic callback."""
        if msg.data and not self._killed:
            self._activate()
        elif not msg.data and self._killed:
            self._deactivate()

    def _poll(self):
        """Polling fallback for GPIO (when interrupt unavailable)."""
        if not self._use_gpio or not GPIO_AVAILABLE:
            return
        try:
            state     = GPIO.input(self._gpio_pin)
            activated = (state == GPIO.LOW) if self._active_low else (state == GPIO.HIGH)
            if activated and not self._killed:
                self._activate()
            elif not activated and self._killed:
                self._deactivate()
        except Exception:
            pass

    def _maintain_halt(self):
        """Continuously publish zero velocity while killed to guarantee <3s halt."""
        if self._killed:
            self._publish_zero()

    # ──────────────────────────────────────────────────────────────────────────

    def _activate(self):
        self._killed = True
        self.get_logger().fatal('KILL SWITCH ACTIVATED — halting all motion NOW')

        # Publish e_stop True
        estop = Bool()
        estop.data = True
        self._estop_pub.publish(estop)

        # Immediately zero velocity (belt-and-suspenders)
        self._publish_zero()

    def _deactivate(self):
        self._killed = False
        self.get_logger().warn('Kill switch released — resuming normal operation')
        estop = Bool()
        estop.data = False
        self._estop_pub.publish(estop)

    def _publish_zero(self):
        self._cmd_pub.publish(Twist())

    def destroy_node(self):
        if GPIO_AVAILABLE:
            try:
                GPIO.cleanup()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KillSwitch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
