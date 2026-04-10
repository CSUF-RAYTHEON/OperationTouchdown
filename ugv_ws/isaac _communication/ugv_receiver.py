import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import serial
import threading

class UgvReceiver(Node):
        def __init__(self):
            super().__init__('ugv_receiver')
            # This publisher tells Nav2 where the UAV is on the map
            self.goal_pub = self.create_publisher(PoseStamped, '/uav_position', 10)

            try:
                # Ensure baud matches the UAV (115200 for 5mph responsiveness)
                self.ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
                self.get_logger().info("LoRa Receiver Active. Tracking UAV...")
            except Exception as e:
                self.get_logger().error(f"Serial Error: {e}")

            threading.Thread(target=self.listen_loop, daemon=True).start()

        def listen_loop(self):
            while rclpy.ok():
                if self.ser.in_waiting > 0:
                    try:
                        line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                        if line.startswith("<POS") and line.endswith(">"):
                            parts = line.strip("<>").split(",")
                            # parts[1]=X, parts[2]=Y(Alt), parts[3]=Z(Forward)
                            self.process_uav_data(float(parts[1]), float(parts[2]), float(parts[3]))
                    except:
                        continue

        def process_uav_data(self, x, y, z):
            msg = PoseStamped()
            msg.header.frame_id = "map" 
            msg.header.stamp = self.get_clock().now().to_msg()

   
            msg.pose.position.x = x
            msg.pose.position.y = z # Z in VO is 'Forward/Depth' on the ground map
            msg.pose.position.z = y # Y in VO is 'Height'

            self.goal_pub.publish(msg)
            self.get_logger().info(f"UAV spotted at: X={x:.2f}, Z={z:.2f}, ALT={y:.2f}")

def main():
        rclpy.init()
        rclpy.spin(UgvReceiver())
        rclpy.shutdown()

if __name__ == "__main__":
        main()
