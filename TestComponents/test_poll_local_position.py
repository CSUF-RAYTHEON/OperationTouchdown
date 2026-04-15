# This code tests getting the local position and storing those values to the variables in the global variable files,
# we calculate our custom local positioning and store the value in a global variable. We constatly poll and print the 
# local position values until we stop the script, this is to ensure that we are able to continuously read the local position
# and see that the values change as we move the drone around.
from pymavlink import mavutil
import global_variables
import time

def poll_local_position(master):
    print(f"Entered test_poll_local_position() for Target System: {master.target_system} & Target Component: {master.target_component}")

    # 1. We do this because by default pixhawk does not send certain messages, the LOCAL_POSITION_NED message being one of them.
    #    So we send a command telling the pixhawk to send a message at a certain interval, in this case we tell it to send the 
    #    LOCAL_POSITION_NED message at a rate of 1HZ (1e7 microseconds / 10). 
    master.mav.command_long_send(master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
    1e7 / 10, 0, 0, 0, 0, 0)
    
    # 2. We initialize the variables to 9999.0, this is because we want to have a default value in case we fail to read the LOCAL_POSITION_NED messages, 
    #    so that we can still print out the values to the terminal for logging purposes. This is important because if we fail to read the LOCAL_POSITION_NED messages
    #    then we want to know that we failed and what the values are, rather than just having an error or no output at all. So this is a way to ensure that we have 
    #    some output even if we fail to read the messages. We set it to 9999.0 because that is a value that is unlikely to be the position of the drone,
    #    so it serves as a clear indicator that we failed to read the messages and/or it did not update from the default value. We don't set the origin_custom values
    #    because they will always be 0.0 on startup, so wherever we begin our script that position will be the custom origin, the custom current position values use 
    #    the custom origin as its reference point.
    global_variables.origin_drone_x = 9999.0
    global_variables.origin_drone_y = 9999.0
    global_variables.origin_drone_z = 9999.0
    global_variables.origin_custom_x = 0.0
    global_variables.origin_custom_y = 0.0
    global_variables.origin_custom_z = 0.0
    global_variables.current_position_drone_x = 9999.0
    global_variables.current_position_drone_y = 9999.0
    global_variables.current_position_drone_z = 9999.0  
    global_variables.current_position_custom_x = 9999.0
    global_variables.current_position_custom_y = 9999.0
    global_variables.current_position_custom_z = 9999.0

    # 3. It can reject/delete the newest messeges that the Pixhawk sends to the Pi, this is because
    #    the message buffer overflows. We solve the overflow issue by using the recv_match() helper function from
    #    the pymavlink/mavutil library to read any incoming messages and clear the buffer.
    #.   IMPORTANT: SIMPLY DOING TIME.SLEEP() WILL NOT WORK AS WE NEED TO ALSO READ/CLEAR THE BUFFER
    print("Waiting 3 seconds to read/clear message queue...")
    start_time = time.time()
    while time.time() - start_time < 3:
        master.recv_match(blocking=False) # Grab any waiting message and immediately discard it
        time.sleep(0.1)
    master.wait_heartbeat()
    print("Done waiting\n")

    # 4. Here we read the LOCAL_POSITION_NED messages, this message contains the local position of the drone in the NED (North, East, Down) coordinate frame. 
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

    # 5. We continuously read the LOCAL_POSITION_NED messages and update the current position values and print them for logging
    try:
        while True: # Continuously read LOCAL_POSITION_NED messages and update current position values
            local_position_ned_msg = master.recv_match(type=['LOCAL_POSITION_NED'], blocking=True, timeout=2) # Receive a LOCAL_POSITION_NED message and block up to 2 seconds
            if local_position_ned_msg is not None:
                global_variables.current_position_drone_x = local_position_ned_msg.x
                global_variables.current_position_drone_y = local_position_ned_msg.y
                global_variables.current_position_drone_z = local_position_ned_msg.z
                global_variables.current_position_custom_x = local_position_ned_msg.x - global_variables.origin_drone_x
                global_variables.current_position_custom_y = local_position_ned_msg.y - global_variables.origin_drone_y
                global_variables.current_position_custom_z = local_position_ned_msg.z - global_variables.origin_drone_z

                print(f"Drone current position values: x: {global_variables.current_position_drone_x}, y: {global_variables.current_position_drone_y}, z: {global_variables.current_position_drone_z}")
                print(f"Custom current position values: x: {global_variables.current_position_custom_x}, y: {global_variables.current_position_custom_y}, z: {global_variables.current_position_custom_z}")
            else:
                print("Timed out waiting for LOCAL_POSITION_NED messages")
    except KeyboardInterrupt:
        print("Stopped polling local position")

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

    poll_local_position(master)