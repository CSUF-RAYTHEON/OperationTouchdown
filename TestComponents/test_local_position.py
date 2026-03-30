# This code tests getting the local position and storing those values to the variables in the global variable files,
# we calculate our custom local positioning and store the value in a global variable. This file specifically 
# compartmentalizes the steps that generally only need to be done once.
from pymavlink import mavutil
import global_variables
import time

def test_request_local_position_messages(master):
    print(f"Entered test_request_local_position_messages() for Target System: {master.target_system} & Target Component: {master.target_component}")

    # 1. We do this because by default pixhawk does not send certain messages, the LOCAL_POSITION_NED message being one of them.
    #    So we send a command telling the pixhawk to send a message at a certain interval, in this case we tell it to send the 
    #    LOCAL_POSITION_NED message at a rate of 1HZ (1e7 microseconds / 10). 
    master.mav.command_long_send(master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
    1e7 / 10, 0, 0, 0, 0, 0)

def test_set_origin_local_position(master):
    print(f"Entered test_set_origin_local_position() for Target System: {master.target_system} & Target Component: {master.target_component}")

    # 1. Here we read the LOCAL_POSITION_NED messages, this message contains the local position of the drone in the NED (North, East, Down) coordinate frame. 
    #    The origin (0,0,0) is initilized at different times, so for consistency purposes we want to read the LOCAL_POSITION_NED messages to confirm where the 
    #    drone thinks it is relative to the origin it set. This is important for future tests that rely on the drone moving to specific coordinates, so we can 
    #    ensure that the drone's local position is accurate and consistent with our expectations. We also print out the values to the terminal for logging purposes.
    #    Lastly, we calculate the current position using our custom coordinates and then we store the values in the global variables we imported from the 
    #    global variable file so we have access to them whenever. 
    print("Reading LOCAL_POSITION_NED messages & Setting Origins...")
    start_time = time.time()
    while time.time() - start_time < 5: # Read LOCAL_POSITION_NED messages for 5 seconds
        local_position_ned_msg = master.recv_match(type=['LOCAL_POSITION_NED'], blocking=True, timeout=2) # Receive a LOCAL_POSITION_NED message and block up to 2 seconds
        if local_position_ned_msg is not None:
            global_variables.origin_drone_x = local_position_ned_msg.x
            global_variables.origin_drone_y = local_position_ned_msg.y
            global_variables.origin_drone_z = local_position_ned_msg.z
            global_variables.current_position_drone_x = local_position_ned_msg.x
            global_variables.current_position_drone_y = local_position_ned_msg.y
            global_variables.current_position_drone_z = local_position_ned_msg.z
            global_variables.current_position_custom_x = local_position_ned_msg.x - global_variables.origin_drone_x
            global_variables.current_position_custom_y = local_position_ned_msg.y - global_variables.origin_drone_y
            global_variables.current_position_custom_z = local_position_ned_msg.z - global_variables.origin_drone_z
            print(f"Received LOCAL_POSITION_NED message with x: {local_position_ned_msg.x}, y: {local_position_ned_msg.y}, z: {local_position_ned_msg.z}")
            break
    else:
        print("Timed out waiting for LOCAL_POSITION_NED messages")
    
    print(f"First drone origin values: x: {global_variables.origin_drone_x}, y: {global_variables.origin_drone_y}, z: {global_variables.origin_drone_z}")
    print(f"First drone current position values: x: {global_variables.current_position_drone_x}, y: {global_variables.current_position_drone_y}, z: {global_variables.current_position_drone_z}")
    print(f"First custom current position values: x: {global_variables.current_position_custom_x}, y: {global_variables.current_position_custom_y}, z: {global_variables.current_position_custom_z}")


def test_update_local_position(master):
    # 1. Read LOCAL_POSITION_NED message and update current position values.
    
    print(f"Entered test_update_local_position() for Target System: {master.target_system} & Target Component: {master.target_component}")

    local_position_ned_msg = master.recv_match(type=['LOCAL_POSITION_NED'], blocking=True, timeout=2) # Receive a LOCAL_POSITION_NED message and block up to 2 seconds
    if local_position_ned_msg is not None:
        global_variables.current_position_drone_x = local_position_ned_msg.x
        global_variables.current_position_drone_y = local_position_ned_msg.y
        global_variables.current_position_drone_z = local_position_ned_msg.z
        global_variables.current_position_custom_x = local_position_ned_msg.x - global_variables.origin_drone_x
        global_variables.current_position_custom_y = local_position_ned_msg.y - global_variables.origin_drone_y
        global_variables.current_position_custom_z = local_position_ned_msg.z - global_variables.origin_drone_z
        print(f"Updated current position drone values with x: {local_position_ned_msg.x}, y: {local_position_ned_msg.y}, z: {local_position_ned_msg.z}")
        print(f"Updated current position custom values with x: {global_variables.current_position_custom_x}, y: {global_variables.current_position_custom_y}, z: {global_variables.current_position_custom_z}")
    else:
        print("Timed out waiting for LOCAL_POSITION_NED messages")

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

    test_request_local_position_messages(master)
    test_set_origin_local_position(master)
    test_update_local_position(master)