from pymavlink import mavutil
from test_change_flight_mode import change_flight_mode
import time

def arm_drone(master):
    print(f"Entered arm_drone() & Setting Arming Parameters for Target System: {master.target_system} & Target Component: {master.target_component}")
    params = {"ARMING_REQUIRE": 1, "ARMING_CHECK": 1, "ARMING_ACCTHRESH": 0.255, "ARMING_MAGTHRESH": 50, "ARMING_NEED_LOC": 0}
    for name, value in params.items():
        try:
            master.mav.param_set_send(master.target_system, master.target_component, name.encode(), float(value), mavutil.mavlink.MAV_PARAM_TYPE_INT32)
            print(f"Set Parameter ({name}) = {value}")
            msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
            print(f"MESSAGE: {msg.get_type()}")
            print(f"DATA: {msg.to_dict()}\n")
            if not msg:
                continue

        except Exception as e:
            print(f"Failed to set {name}: {e}", end=" ")
    print("\nParameters set. You may need to reboot FCU for sensors to reinit.")

    print("Arming Drone Motors")
    master.arducopter_arm()
    print("Reading message buffer to catch up to arming change...")
    start_time = time.time()
    while time.time() - start_time < 3: # Continously read messages for up to 3 seconds
        msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=1) # Recieve a message and block up to 1 second
        if msg and master.flightmode == flight_mode:
            print(f"SUCCESS! Pymavlink caught up and sees: {master.flightmode} flight mode & Base Mode(MAV_MODE_FLAGS): {master.base_mode}\n")
            break
    else:
        # If the loop finishes the 3 seconds without breaking, it timed out
        print(f"FAILURE! Timed out waiting for {flight_mode}. Current mode seen: {master.flightmode} & Base Mode(MAV_MODE_FLAGS): {master.base_mode}\n")


if __name__ == "__main__":
    serial_port = '/dev/serial0'
    baudrate =  57600
    source_system = 1
    source_component = 191

    print("\nConnecting to Pixhawk & Waiting for Heartbeat...")
    master = mavutil.mavlink_connection(serial_port, baud=baudrate, source_system=source_system, source_component=source_component)
    master.wait_heartbeat()
    print(f"Heartbeat Received, Source System: {source_system}, Source Component: {source_component}, Connection Type: {serial_port}, Baudrate: {baudrate}")

    change_flight_mode(master, "GUIDED")
    arm_drone(master)