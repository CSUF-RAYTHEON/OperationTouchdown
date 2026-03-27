# This code tests the arming of the drone. It first sets some arming parameters to ensure that the drone can arm, then it sends the arming command, 
# and finally it checks if the drone is armed. It also prints out various messages to the terminal for logging purposes. It also goes through the
# process of clearing the messae buffer to ensure that we get the correct command acknowledement and heartbeat messages that confirm the drone is armed. 
# This is important because if the buffer is not cleared then we can get old messages that do not reflect the current state of the drone, which can 
# lead to confusion and incorrect assumptions about whether the drone is armed or not. So we actively read and clear the buffer until we catch the correct
# messages that confirm the drone is armed. So it also shows how to read command acknowledement messages and heartbeat messages.
from pymavlink import mavutil
from test_change_flight_mode import change_flight_mode
import time

def arm_drone(master):
    print(f"Entered arm_drone() & Setting Arming Parameters for Target System: {master.target_system} & Target Component: {master.target_component}")

    # 1. Setting some arming parameters for the drone, these parameters dicate under what conditions the drone will arm.
    #    These are not all of the parameters however, so if for some reason other parameters are changed then it could fail
    #    to arm. The ARMING_REQUIRE is a parameter for planes, not for drones, this distinction is important as changing
    #    parameters for a plane can result in the drone not arming. So we set ARMING_REQUIRE = 1 which is its default value.
    #    This is to ensure it is always 1 whenever we arm to prevent being unable to arm. We also print out the values it
    #    becomes and the associated parameter. Both are printed to the termal for logging purposes.  
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
    master.wait_heartbeat()
    print("\nParameters set. You may need to reboot FCU for sensors to reinit.")

    # 2. Here we arm the drone using the built-in helper function from pymavlink/mavutil library
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

    # 4. Here we check if the drone is armed using the built-in helper function from pymavlink/mavutil library to ensure that the drone is armed. This is because sometimes 
    #    the drone can fail to arm due to various reasons such as bad parameters, bad GPS lock, or bad sensor readings. So this is a backup to ensure that the drone 
    #    is armed and ready to fly. The previous step is more for logging purposes to confirm that the arming command was sent and accepted by the flight controller, 
    #    but this step is to ensure that the drone is actually armed. And we also print out the result to the terminal for logging purposes.
    print("Checking if drone armed...")
    start_time = time.time()
    while time.time() - start_time < 3: # Wait for up too 3 seconds for confirmation that the motors armed
        master.motors_armed_wait()
        if master.motors_armed():
            print("Drone is armed and ready to fly\n")
            break
        else:
            print("Escaped motors_armed_wait() but drone did not arm")
    else:
        if master.motors_armed():
            print("Timed out waiting for confirmation that drone is armed. But motors show they are armed.\n")
        else:
            print("Motors failed to arm and timed out waiting for confirmation that drone is armed.\n")


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