#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RoboteqDriver(Node):
    def __init__(self):
        super().__init__('motor_driver')
        
        # 1. Create a Subscriber for /cmd_vel
        # This listens for Twist messages (from keyboard or Foxglove)
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.velocity_callback,
            10) # Queue size
        
        self.get_logger().info('Roboteq Bridge Node has started and is listening for /cmd_vel...')

    def velocity_callback(self, msg):
        """
        This function runs every time a new movement command is received.
        """
        # Linear velocity (forward/backward) in m/s
        linear_x = msg.linear.x
        # Angular velocity (turning) in rad/s
        angular_z = msg.angular.z

        # --- KINEMATICS LOGIC ---
        # For a differential drive rover:
        # Left wheel = Linear - Angular
        # Right wheel = Linear + Angular
        left_motor_speed = linear_x - angular_z
        right_motor_speed = linear_x + angular_z

        # 2. LOG THE OUTPUT
        # For now, we print to console. 
        # Next step: Send these to the serial port for the Roboteq!
        self.get_logger().info(
            f'INPUT: Linear={linear_x:.2f} Ang={angular_z:.2f} | '
            f'OUTPUT: Left={left_motor_speed:.2f} Right={right_motor_speed:.2f}'
        )

        # TODO: self.serial_port.write(f"!G 1 {left_motor_speed}\r")
        # TODO: self.serial_port.write(f"!G 2 {right_motor_speed}\r")

def main(args=None):
    rclpy.init(args=args)
    node = RoboteqDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

