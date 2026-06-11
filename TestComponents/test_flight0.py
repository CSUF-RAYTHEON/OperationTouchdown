from pymavlink import mavutil
from test_change_flight_mode import change_flight_mode
from test_arm_drone import arm_drone
import test_move
import test_local_position

if __name__ == "__main__":
    serial_port = '/dev/ttyS3'
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
    test_local_position.request_local_position_messages(master)
    test_local_position.set_origin_local_position(master)
    test_local_position.update_local_position(master)
    test_move.takeoff(master, 3)
    test_move.move(master, 3, 3, -3)
    test_move.land_current_position(master)