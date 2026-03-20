from pymavlink import mavutil
import time

x_local_ned = 0.0
y_local_ned = 0.0
z_local_ned = 0.0

def test_origin(master):
    print(f"Entered test_origin() for Target System: {master.target_system} & Target Component: {master.target_component}")

    # 1. It will reject a switch/delete newest messeges that the Pixhawk sends to the Pi, this is because
    # the message buffer overflows. We solve the overflow issue by using the recv_match() helper function from
    # the pymavlink/mavutil library to read any incoming messages and clear the buffer.
    # IMPORTANT: SIMPLY DOING TIME.SLEEP() WILL NOT WORK AS WE NEED TO ALSO READ/CLEAR THE BUFFER
    print("Waiting 3 seconds to read/clear message queue...")
    start_time = time.time()
    while time.time() - start_time < 3:
        master.recv_match(blocking=False) # Grab any waiting message and immediately discard it
        time.sleep(0.1)
    master.wait_heartbeat()
    print("Done waiting\n")

    #2. Here we read the LOCAL_POSITION_NED messages, this message contains the local position of the drone in the NED (North, East, Down) coordinate frame. 
    #   The origin (0,0,0) is initilized at different times, so for consistency purposes we want to read the LOCAL_POSITION_NED messages to confirm where the 
    #   drone thinks the origin is. This is important for future tests that rely on the drone moving to specific coordinates, so we can ensure that the drone's 
    #   local position is accurate and consistent with our expectations. We also print out the values to the terminal for logging purposes.
    print("Reading LOCAL_POSITION_NED messages...")
    start_time = time.time()
    while time.time() - start_time < 5: # Read LOCAL_POSITION_NED messages for 5 seconds
        local_position_ned_msg = master.recv_match(type=['LOCAL_POSITION_NED'], blocking=True, timeout=2) # Receive a LOCAL_POSITION_NED message and block up to 2 seconds
        if local_position_ned_msg is not None:
            x_local_ned = local_position_ned_msg.x
            y_local_ned = local_position_ned_msg.y
            z_local_ned = local_position_ned_msg.z
            print(f"Received LOCAL_POSITION_NED message with x: {x_local_ned}, y: {y_local_ned}, z: {z_local_ned}")
            break
    else:
        print("Timed out waiting for LOCAL_POSITION_NED messages")
    
    print(f"Final LOCAL_POSITION_NED values: x: {x_local_ned}, y: {y_local_ned}, z: {z_local_ned}\n")



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

    test_origin(master)