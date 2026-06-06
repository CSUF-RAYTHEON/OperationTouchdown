#!/usr/bin/env python3
"""Automated 5-second drive test over LoRa."""

import serial
import time

# Update this to match the port where your sending LoRa module is plugged in.
# For Mac it might be '/dev/cu.usbserial-140'
# For Raspberry Pi it might be '/dev/ttyUSB0' or '/dev/serial0'
PORT = '/dev/ttyUSB0'
BAUD = 9600

def run_auto_test():
    try:
        print(f"Connecting to LoRa on {PORT} at {BAUD} baud...")
        ser = serial.Serial(PORT, BAUD, timeout=1)
        time.sleep(1)  # Give the serial port a second to initialize

        # 1. Send STRAIGHT
        print("Sending command: STRAIGHT")
        # Ensure we add the newline (\n) so the UGV bridge knows the message is complete
        ser.write(b'BACKWARDS\n')

        # 2. Wait exactly 12 seconds
        print("Driving forward... waiting 90 seconds.")
        for i in range(5, 0, -1):
            print(f"{i}...")
            time.sleep(1)

        # 3. Send STOP
        print("Sending command: STOP")
        ser.write(b'STOP\n')

        # Give the hardware a fraction of a second to finish the RF transmission
        time.sleep(0.5)
        print("Test complete. UGV should be stopped.")

    except Exception as e:
        print(f"Hardware Error: {e}")
        print("Check if the port is correct and the LoRa module is plugged in.")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial port closed.")

if __name__ == '__main__':
    run_auto_test()