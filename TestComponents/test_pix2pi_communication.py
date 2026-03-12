# This code tests the communications between the Pixhawk and the Raspberry Pi.
# The Pi sends a command to receive a heartbeat of the Pixhawk, if it gets to
# print that it received the heartbeat then there is succesfull communication
# between the Pi and Pixhawk. It also prints some basic info about the Pixhawk. 

from pymavlink import mavutil

def heartbeat():
    print("Waiting for heartbeat from Pixhawk...")
    master.wait_heartbeat()
    print("Heartbeat Received\n")

    print(f"Heartbeat from system (system {master.target_system} component {master.target_component})")
    print(f"Using MAVLink 2.0: {master.mavlink20()} \n\n\n")

if __name__ == "__main__":
    serial_port = '/dev/serial0'
    baudrate =  57600
    source_system = 1
    source_component = 191

    print("\nConnecting to Pixhawk...")
    master = mavutil.mavlink_connection(serial_port, baud=baudrate, source_system=source_system, source_component=source_component)
    heartbeat()