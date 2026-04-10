import serial
import time

# Configure the serial port (Check if your device is /dev/ttyUSB0 or /dev/ttyACM0)
ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)

def send_command(cmd):
    ser.write(f"{cmd}\r".encode())
    return ser.readline().decode().strip()

try:
    print("--- Starting Motor Test ---")
    
    # 1. Spin Motor 1 (Right) at 200/1000 power
    send_command("!G 1 200")
    # 2. Spin Motor 2 (Left) at 200/1000 power
    send_command("!G 2 200")
    
    print("Motors running... reading encoder data:")
    
    # 3. Read encoders 10 times during the spin
    for _ in range(10):
        m1_enc = send_command("?C 1") # Query Encoder 1
        m2_enc = send_command("?C 2") # Query Encoder 2
        print(f"Encoders -> Right: {m1_enc} | Left: {m2_enc}")
        time.sleep(0.2)

finally:
    # 4. ALWAYS stop the motors
    send_command("!G 1 0")
    send_command("!G 2 0")
    print("--- Test Complete: Motors Stopped ---")
    ser.close()

