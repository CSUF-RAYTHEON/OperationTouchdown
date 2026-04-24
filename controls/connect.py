# This code establishes a connection to the pixhawk and returns a mavlink master object, depedning on which function is used it uses either a UART or USB connection.
# A port may only establish a connection with a single process at a time. 
from pymavlink import mavutil

def connect_UART0():
    serial_port = '/dev/serial0'
    baudrate =  57600
    source_system = 1
    source_component = 191

    print("\nConnecting to Pixhawk via UART0 & waiting for heartbeat")
    master = mavutil.mavlink_connection(serial_port, baud=baudrate, source_system=source_system, source_component=source_component)
    master.target_system = 1 # Send messages to system 1(drone/vehicle #1)
    master.target_component = 1 # Send messages to flight controller "autopilot"
    master.wait_heartbeat()
    print("Heartbeat received & connection established for UART0")
    print(f"Source System: {master.source_system}, Source Component: {master.source_component}, Target System: {master.target_system}, Target Component: {master.target_component}, Connection Type: {serial_port}, Baudrate: {baudrate}")
    
    return master

def connect_UART2():
    serial_port = '/dev/ttyAMA2'
    baudrate =  57600
    source_system = 1
    source_component = 191

    print("\nConnecting to Pixhawk via UART2 & waiting for heartbeat")
    master = mavutil.mavlink_connection(serial_port, baud=baudrate, source_system=source_system, source_component=source_component)
    master.target_system = 1 # Send messages to system 1(drone/vehicle #1)
    master.target_component = 1 # Send messages to flight controller "autopilot"
    master.wait_heartbeat()
    print("Heartbeat received & connection established for UART2")
    print(f"Source System: {master.source_system}, Source Component: {master.source_component}, Target System: {master.target_system}, Target Component: {master.target_component}, Connection Type: {serial_port}, Baudrate: {baudrate}")
    
    return master

if __name__ == "__main__":
    master1 = connect_UART0()
    master2 = connect_UART2()

    master1.close()
    master2.close()