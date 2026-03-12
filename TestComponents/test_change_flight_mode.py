# This code tests and shows how to switch flight modes using the Pi and confirm it switched modes.
# It also goes into some detail about the message queue/buffer, and how it must be kept from overflowing
# to prevent any unwanted errors/behavior. And the necessary steps for startup.
from pymavlink import mavutil
import time

def change_flight_mode(master, flight_mode):
    print("Waiting for heartbeat from Pixhawk...")
    master.wait_heartbeat()
    print("Heartbeat Received")
    print(f"Entered change_flight_mode() for Target System: {master.target_system} & Target Component: {master.target_component}")

    # 1. ArduPilot will actively reject a flight mode switch if its (EKF) hasn't secured a solid GPS lock 
    # and stabilized its sensors. So this loop waits for 5 seconds to allow for this to happen.
    # It will also reject a switch/delete newest messeges that the Pixhawk sends to the Pi, this is because
    # the message buffer overflows. We solve the overflow issue by using the recv_match() helper function from
    # the pymavlink/mavutil library to read any incoming messages and clear the buffer.
    # IMPORTANT: SIMPLY DOING TIME.SLEEP() WILL NOT WORK AS WE NEED TO ALSO READ/CLEAR THE BUFFER
    print("Waiting 3 seconds for sensors, GPS, and reading message queue...")
    start_time = time.time()
    while time.time() - start_time < 3:
        master.recv_match(blocking=False) # Grab any waiting message and immediately discard it
        time.sleep(0.1) # A tiny 0.1s sleep keep CPU from maxing out at 100%
    print("Done waiting!\n")

    # 2. Switch flight mode using the built-in helper function from pymavlink/mavutil library
    print(f"Switching to {flight_mode} flight mode...")
    master.set_mode(flight_mode)

    # 3. Flush the buffer until we catch the correct command acknowledement(cmd ack) by pulling the next cmd ack from the queue. 
    print("Reading message buffer to catch command acknowledgment...")
    start_time = time.time()
    while time.time() - start_time < 3: # Continously read command acknowledgements for 3 seconds
        command_ack_msg = master.recv_match(type=['COMMAND_ACK'], blocking=True, timeout=2) # Receive a command acknowledgement message and block up to 2 seconds
        if command_ack_msg is not None:
            if command_ack_msg.command == 176 and command_ack_msg.result == 0:
                print(f"Command Acknowledgment received for MAV_CMD_DO_SET_MODE(CMD #176) with result MAV_RESULT_ACCEPTED(0)")
                print(f"{command_ack_msg.get_type()}: {command_ack_msg.to_dict()}\n")
                break
    else:
        if command_ack_msg is not None:
            print(f"Command Acknowledgment received but timed out with wrong command or result: CMD #{command_ack_msg.command} & CMD Result #{command_ack_msg.result}")
            print(f"{command_ack_msg.get_type()}: {command_ack_msg.to_dict()}")
        else:
            print("Timed out waiting for command acknowledgement message")

    # 4. Flush the buffer until we catch the updated heartbeat by pulling the next heartbeat from the queue. Pymavlink caches the flight mode based
    # on the LAST heartbeat it read. If we print the mode immediately, it will falsely print the wrong mode. To solve this we must actively pull 
    # new heartbeats from the queue until we catch up to the message that proves the flight controller actually did switch modes.
    print("Reading message buffer to get latest heartbeat...")
    start_time = time.time()
    while time.time() - start_time < 3: # Continously read heartbeats for 3 seconds
        heartbeat_msg = master.recv_match(type=['HEARTBEAT'], blocking=True, timeout=2) # Recieve a heartbeat message and block up to 2 second
        if heartbeat_msg and master.flightmode == flight_mode:
            print(f"SUCCESS! Pymavlink caught up and sees: {master.flightmode} flight mode & Base Mode(MAV_MODE_FLAGS): {master.base_mode}\n")
            break
    else:
        print(f"FAILURE! Timed out waiting for {flight_mode}. Current mode seen: {master.flightmode} & Base Mode(MAV_MODE_FLAGS): {master.base_mode}\n")

if __name__ == "__main__":
    serial_port = '/dev/serial0'
    baudrate =  57600
    source_system = 1
    source_component = 191

    print("\nConnecting to Pixhawk...")
    master = mavutil.mavlink_connection(serial_port, baud=baudrate, source_system=source_system, source_component=source_component)
    master.target_system = 1 # Send messages to system 1(drone/vehicle #1)
    master.target_component = 1 # Send messages to flight controller "autopilot"

    change_flight_mode(master, "GUIDED")