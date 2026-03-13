from pymavlink import mavutil
from test_change_flight_mode import change_flight_mode
import time

def arm_drone(master):
    # 1. Setting some arming parameters for the drone, these parameters dicate under what conditions the drone will arm.
    #    These are not all of the parameters however, so if for some reason other parameters are changed then it could fail
    #    to arm. The ARMING_REQUIRE is a parameter for planes, not for drones, this distinction is important as changing
    #    parameters for a plane can result in the drone not arming. So we set ARMING_REQUIRE = 1 which is its default value.
    #    This is to ensure it is always 1 whenever we arm to prevent being unable to arm. We also print out the values it
    #    becomes and the associated parameter. Both are printed to the termal for logging purposes.  
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

    # 2. Here we arm the drone using  the built-in helper function from pymavlink/mavutil library
    print("Arming Drone Motors")
    master.arducopter_arm()

    # 3. Flush the buffer until we catch the correct command acknowledement(cmd ack) by pulling the next cmd ack from the queue continously.
    #    This is for logging purposes to confirm that the arming command was sent and accepted by the flight controller. It also serves to 
    #    clear the buffer of any old messages until we catch the correct one that shows the drone is armed.
    print("Reading message buffer to catch up to arming change...")
    start_time = time.time()
    while time.time() - start_time < 3: # Continously read command acknowledgements for 3 seconds
        command_ack_msg = master.recv_match(type=['COMMAND_ACK'], blocking=True, timeout=2) # Receive a command acknowledgement message and block up to 2 seconds
        if command_ack_msg is not None:
            if command_ack_msg.command == 400 and command_ack_msg.result == 0:
                print(f"Command Acknowledgment received for MAV_CMD_COMPONENT_ARM_DISARM(CMD #400) with result MAV_RESULT_ACCEPTED(0)")
                print(f"{command_ack_msg.get_type()}: {command_ack_msg.to_dict()}\n")
                break
    else:
        if command_ack_msg is not None:
            print(f"Command Acknowledgment received but timed out with wrong command or result: CMD #{command_ack_msg.command} & CMD Result #{command_ack_msg.result}")
            print(f"{command_ack_msg.get_type()}: {command_ack_msg.to_dict()}")
        else:
            print("Timed out waiting for command acknowledgement message")


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