#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import serial

class LoraReceiver(Node):
    def __init__(self):
        super().__init__('lora_receiver')
        
        # SB Components Dongle usually shows up as /dev/ttyUSB0 or USB1
        # The default baud rate for E22/E220 series is 9600
        try:
            # TIP: If it fails, check if your dongle is actually on /dev/ttyUSB0
            self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
            self.get_logger().info('LoRa Dongle Connected on /dev/ttyUSB0')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to LoRa Dongle: {e}')
            self.ser = None

        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.timer = self.create_timer(0.1, self.read_lora)

    def read_lora(self):
        if self.ser and self.ser.in_waiting > 0:
            try:
                # Decoding with 'ignore' prevents the node from crashing 
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                
                if line:
                    self.get_logger().info(f'Received: {line}')
                    # Here you can add logic to parse "GOTO:x:y" into PoseStamped
            except Exception as e:
                self.get_logger().error(f'Error reading serial: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = LoraReceiver()
    try:
        # This is what keeps the script from exiting immediately!
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        if node.ser:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

# THIS IS THE IGNITION SWITCH
if __name__ == '__main__':
    main()


