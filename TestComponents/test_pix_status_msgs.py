# This code tests the receiving of status/vehicle messages from the Pixhawk, essentially testing
# that the Pi is indeed receiving the status messages from the Pixhawk. 
from pymavlink import mavutil
import time

def pixhawk_status_messages():
    print("Waiting for heartbeat from Pixhawk...")
    master.wait_heartbeat()
    print("Heartbeat Received\n")

    print("Listening for STATUSTEXT (pre-arm) messages. Press Ctrl-C to stop.")
    
    try:
        while True:
            msg = master.recv_match(type='STATUSTEXT', blocking=True, timeout=5)
            if msg:
                try:
                    print("STATUSTEXT:", msg.text)
                except Exception:
                    print("STATUSTEXT (raw):", msg)
            else:
                # occasionally poll sys_status to show some telemetry
                s = master.recv_match(type='SYS_STATUS', blocking=False)
                if s:
                    print("SYS_STATUS: battery:", getattr(s, 'battery_remaining', 'N/A'), "%")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n\nStopped")


if __name__ == "__main__":
    serial_port = '/dev/serial0'
    baudrate =  57600
    source_system = 1
    source_component = 191

    print("\nConnecting to Pixhawk...")
    master = mavutil.mavlink_connection(serial_port, baud=baudrate, source_system=source_system, source_component=source_component)
    pixhawk_status_messages()