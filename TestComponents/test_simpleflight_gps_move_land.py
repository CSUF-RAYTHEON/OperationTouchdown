from test_change_flight_mode import change_flight_mode
from test_arm_drone import arm_drone
from test_origin import test_origin
from pymavlink import mavutil
import time


def test_simpleflight_gps_move_land(master):
    print(f"Entered test_simpleflight_gps_move_land() for Target System: {master.target_system} & Target Component: {master.target_component}")

    # 1. It will reject a switch/delete newest messeges that the Pixhawk sends to the Pi, this is because
    #    the message buffer overflows. We solve the overflow issue by using the recv_match() helper function from
    #    the pymavlink/mavutil library to read any incoming messages and clear the buffer.
    #    IMPORTANT: SIMPLY DOING TIME.SLEEP() WILL NOT WORK AS WE NEED TO ALSO READ/CLEAR THE BUFFER
    print("Waiting 3 seconds to read/clear message queue...")
    start_time = time.time()
    while time.time() - start_time < 3:
        master.recv_match(blocking=False) # Grab any waiting message and immediately discard it
        time.sleep(0.1)
    master.wait_heartbeat()
    print("Done waiting\n")
    
    # 2. Takeoff using mavlink message SET_POSITION_TARGET_LOCAL_NED
    print("Taking off to 10 meters altitude AGV using SET_POSITION_TARGET_LOCAL_NED message...")
    master.mav.set_position_target_local_ned_send(0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, 
        ( mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE),
        0, 0, -10,
        0, 0, 0,
        0, 0, 0,
        0, 0)






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

    change_flight_mode(master, "GUIDED")
    arm_drone(master)
    test_origin(master)
    test_simpleflight_gps_move_land(master)
