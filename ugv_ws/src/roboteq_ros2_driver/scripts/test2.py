import serial
import time

# Connection to Max3232 Level Shifter on Pi 5
ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=0.01)

def send_movement(linear, angular):
    """
    linear: -1000 to 1000 (Speed)
    angular: -1000 to 1000 (Steering: Negative=Left, Positive=Right)
    """
    # 1. Calculate Differential Drive speeds
    left_base = linear + angular
    right_base = linear - angular

    # 2. Apply your Mirror Logic
    # Based on your tests: Right (Ch 1) Forward is Negative
    # Left (Ch 2) Forward is Positive
    right_cmd = -right_base
    left_cmd = left_base

    # 3. Safety Clamp (Don't exceed 1000 or -1000)
    right_cmd = max(min(right_cmd, 1000), -1000)
    left_cmd = max(min(left_cmd, 1000), -1000)

    # 4. Write to Roboteq
    ser.write(f"!G 1 {int(right_cmd)}\r!G 2 {int(left_cmd)}\r".encode())

def run_stage(name, linear, angular, duration):
    print(f"--- STAGE: {name} ---")
    send_movement(linear, angular)
    
    start_time = time.time()
    while time.time() - start_time < duration:
        ser.write(b"?C 1\r?C 2\r")
        response = ser.read_all().decode().strip()
        if response:
            print(f"Encoders: {response.replace('\r', ' ')}")
        time.sleep(0.1)

try:
    # Test 1: Straight Forward
    run_stage("Forward", 120, 0, 2)

    # Test 2: Wide Right Arc (Right wheel slows down, Left stays fast)
    run_stage("Right Arc", 250, 45, 2)

    # Test 3: Sharp Left Arc (Left wheel slows down, Right stays fast)
    run_stage("Left Arc", 200, -45, 2)

finally:
    send_movement(0, 0)
    ser.close()
    print("--- Test Complete: Safety Stop Engaged ---")
