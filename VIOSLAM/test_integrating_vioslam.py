import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import multiprocessing as mp
import numpy as np
from multiprocessing import shared_memory
from broadcaster import camera_broadcaster
from calculatevioslam_updateposition1 import calculatevioslam_updateposition
from TestComponents import test_getyaw
from controls import move
from TestComponents import test_use_externalnav
from TestComponents import test_change_flight_mode
from pymavlink import mavutil

if __name__ == "__main__":
    # 1. Set the multiprocessing start method to 'spawn' this is so that when a new process is started it
    #    doesn't inherit the memory of the parent process, and instead creates its own memory space and also
    #    its own python interpreter. Effectively isolating the processes from each other except for the shared memory.
    mp.set_start_method('spawn', force=True)
    
    W, H = 640, 400
    RGB_BYTES = W * H * 3
    GRAY_BYTES = W * H
    DEPTH_BYTES = W * H * 2  # 16-bit depth uses 2 bytes per pixel
    CALIB_BYTES = 3 * 3 * 8 # 3x3 matrix of float64 (8 bytes each)
    YAW_BYTES = 8 # float64 for yaw value
    COORD_BYTES = 8 # float64 for coordinate value

    # 2. Create the shared memory variables for RGB, gray, depth frames, and camera calibration matrix
    print("Broadcaster tester allocating shared memory...")
    shm_rgb = shared_memory.SharedMemory(create=True, size=RGB_BYTES, name="oak_rgb")
    shm_gray = shared_memory.SharedMemory(create=True, size=GRAY_BYTES, name="oak_gray")
    shm_depth = shared_memory.SharedMemory(create=True, size=DEPTH_BYTES, name="oak_depth")
    shm_calib = shared_memory.SharedMemory(create=True, size=CALIB_BYTES, name="oak_calib")
    shm_yaw = shared_memory.SharedMemory(create=True, size=YAW_BYTES, name="pixhawk_yaw")
    shm_x_coord = shared_memory.SharedMemory(create=True, size=COORD_BYTES, name="pixhawk_x_coord")
    shm_y_coord = shared_memory.SharedMemory(create=True, size=COORD_BYTES, name="pixhawk_y_coord")
    shm_z_coord = shared_memory.SharedMemory(create=True, size=COORD_BYTES, name="pixhawk_z_coord")

    shared_yaw = np.ndarray((1,), dtype=np.float64, buffer=shm_yaw.buf)
    shared_yaw[0] = 999.0  # VIO script looks for this to know it hasn't updated yet

    # 2. Create the mutex lock for the camera frame variables, camera calibration variable, and uart tx port variable
    camera_frame_mutex = mp.Lock()
    camera_calibration_mutex = mp.Lock()
    uart_tx_mutex = mp.Lock()
    yaw_mutex = mp.Lock()
    position_mutex = mp.Lock()

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

    initial_yaw_rad = test_getyaw.get_yaw(master)
    with yaw_mutex:
        shared_yaw[0] = initial_yaw_rad
        print(f"Initial yaw set in shared memory: {shared_yaw[0]} radians")

    # 3. Define the independent processes
    broadcaster_process = mp.Process(target=camera_broadcaster, args=(camera_frame_mutex, camera_calibration_mutex))
    calculatevioslam_updateposition_process = mp.Process(target=calculatevioslam_updateposition, args=(camera_frame_mutex, uart_tx_mutex, camera_calibration_mutex, yaw_mutex, position_mutex))
    
    with uart_tx_mutex:
        test_use_externalnav.use_externalnav(master)
        test_change_flight_mode.change_flight_mode(master)
        master.close()

    try:
        # 4. Start the processes
        broadcaster_process.start()
        calculatevioslam_updateposition_process.start()

        move.takeoff(master, 3, uart_tx_mutex)
        move.move(master, 3, 3, -3, uart_tx_mutex, position_mutex)
        move.land_current_position(master, uart_tx_mutex)
        # 5. Wait until the viewer window is closed
        calculatevioslam_updateposition_process.join()
        
        # 6. kill the broadcaster
        broadcaster_process.terminate()
        broadcaster_process.join()
    except KeyboardInterrupt:
        print("\nBroadcaster tester caught keyboard interrupt. Shutting down...")
    finally:
        # 7. Cleanup all three blocks
        print("Broadcaster tester cleaning up shared memory...")
        shm_rgb.close()
        shm_rgb.unlink()
        shm_gray.close()
        shm_gray.unlink()
        shm_depth.close()
        shm_depth.unlink()
        shm_calib.close()
        shm_calib.unlink()
        shm_yaw.close()
        shm_yaw.unlink()
        shm_x_coord.close()
        shm_x_coord.unlink()
        shm_y_coord.close()
        shm_y_coord.unlink()
        shm_z_coord.close()
        shm_z_coord.unlink()
        print("Integrating vioslam tester processes terminated safely.")