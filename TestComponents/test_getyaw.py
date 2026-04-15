# This code tests getting the yaw from the drone by reading the attitude mavlink message sent by the pixhawk.
from pymavlink import mavutil
import time

def get_yaw(master):
    print("Waiting for heartbeat from Pixhawk...")
    master.wait_heartbeat()
    print("Heartbeat Received\n")

    print("Listening for ATTITUDE messages...")
    
    msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=3)
    if msg:
        print(f"Returning ATTITUDE message with yaw: {msg.yaw}")
        return float(msg.yaw)
    else:
        print("Failed to receive ATTITUDE message returning 9999")
        return float(9999)


if __name__ == "__main__":
    serial_port = '/dev/serial0'
    baudrate =  57600
    source_system = 1
    source_component = 191

    print("\nConnecting to Pixhawk & Waiting for Heartbeat")
    master = mavutil.mavlink_connection(serial_port, baud=baudrate, source_system=source_system, source_component=source_component)
    master.target_system = 1 # Send messages to system 1(drone/vehicle #1)
    master.target_component = 1 # Send messages to flight controller "autopilot"
    master.wait_heartbeat()
    print("Heartbeat Received & Connection Established")
    print(f"Source System: {master.source_system}, Source Component: {master.source_component}, Target System: {master.target_system}, Target Component: {master.target_component}, Connection Type: {serial_port}, Baudrate: {baudrate}")

    try:
        print(f"Current Yaw: {get_yaw(master)}")
        time.sleep(1)
    except KeyboardInterrupt:
        print("Stopped getting yaw")


