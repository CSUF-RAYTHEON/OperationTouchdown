import time
import numpy as np
import multiprocessing as mp
from multiprocessing import shared_memory
from pymavlink import mavutil

def takeoff(master, height):
    print(f"Entered takeoff() for Target System: {master.target_system} & Target Component: {master.target_component}")
    # 1. Takeoff to specified height (positive value for takeoff command) 
    master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, height)

    time.sleep(2) # Wait for 2 seconds to allow the drone to takeoff and stabilize at the new height

def land_current_position(master):
    print(f"Entered land_current_position() for Target System: {master.target_system} & Target Component: {master.target_component}")
    # 1. Land by using land command at current position. 
    master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)
    
    time.sleep(2) # Wait for 2 seconds to allow the drone to land and stabilize

def goto(master, x, y, z):
    # 1. Send SET_POSITION_TARGET_LOCAL_NED message using the active MAVLink connection
    master.mav.set_position_target_local_ned_send(0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, 
        ( mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE),
        x, y, -z, # X, Y, Z (NED) -> (NEU)
        0, 0, 0, # VX, VY, VZ (not used)
        0, 0, 0, # AX, AY, AZ (not used)
        0, 0)    # YAW, YAW_RATE (not used)

def hold_position(master, x, y, z, duration):
    # 1. Continuously stream the target position using the active connection
    start_time = time.time()
    while time.time() - start_time < duration:
        goto(master, x, y, z)
        time.sleep(0.05)

def move(master, x, y, z, shared_local_position_ned, local_position_ned_mutex):
    # Safely read the initial Pixhawk fused coordinates
    with local_position_ned_mutex:
        local_x = shared_local_position_ned[0]
        local_y = shared_local_position_ned[1]
        local_z = -shared_local_position_ned[2]
        
    # 1. Tell drone to move to specified position until within 0.8 meters
    while(abs(local_x - x) > 0.5 or abs(local_y - y) > 0.5 or abs(local_z - z) > 0.5):
        
        # Send the continuous stream of target coordinates
        hold_position(master, x, y, z, 1.0)
        
        # Safely re-check the Pixhawk's fused coordinates after holding position
        with local_position_ned_mutex:
            local_x = shared_local_position_ned[0]
            local_y = shared_local_position_ned[1]
            local_z = -shared_local_position_ned[2]

def set_speed(master, speed_m_s):
    print(f"Setting speed limit to {speed_m_s} m/s")
    master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0,
        1,          # Param 1: Speed type (1 = Ground Speed)
        speed_m_s,  # Param 2: Speed in m/s (e.g., 1.0)
        -1,         # Param 3: Throttle (-1 to ignore)
        0, 0, 0, 0  # Params 4-7: Not used
    )

def main(shared_local_position_ned, local_position_ned_mutex):
    from controls.connect import connect_UART0
    from controls.externalnav import externalnav
    from flightmode import change_flight_mode
    from controls.affinitypriority import set_core_and_priority 

    set_core_and_priority(3, None) # Core 4, Normal Priority
    shared_local_position_ned = np.ndarray((3,), dtype=np.float64, buffer=shm_local_position_ned.buf)

    master = connect_UART0()
    change_flight_mode(master, "GUIDED")
    set_speed(master, 0.5) # Set speed limit to 0.5 m/s
    externalnav(master) # Set External Navigation parameters to use VIO SLAM as main navigation source

    takeoff(master, 4.0) # Takeoff to 4 meter altitude
    move(master, 2.0, 0.0, 3.0, shared_local_position_ned, local_position_ned_mutex)
    land_current_position(master) # Land at current position
    time.sleep(3) # Wait for 3 seconds to allow the drone to land and stabilize before ending the program

if __name__ == "__main__":
    mp.set_start_method('spawn', force=True)
    
    W, H = 640, 400
    RGB_BYTES = W * H * 3
    GRAY_BYTES = W * H
    DEPTH_BYTES = W * H * 2 
    CALIB_BYTES = 3 * 3 * 8 
    ATTITUDE_BYTES = 3 * 8 
    POSITION_BYTES = 3 * 8 
    LOCAL_POSITION_NED_BYTES = 3 * 8
    BOOL_BYTES = 2
    TARGET_BYTES = 4 * 8

    print("Movement tester allocating shared memory...")
    shm_rgb = shared_memory.SharedMemory(create=True, size=RGB_BYTES, name="oak_rgb")
    shm_gray = shared_memory.SharedMemory(create=True, size=GRAY_BYTES, name="oak_gray")
    shm_depth = shared_memory.SharedMemory(create=True, size=DEPTH_BYTES, name="oak_depth")
    shm_calib = shared_memory.SharedMemory(create=True, size=CALIB_BYTES, name="oak_calib")
    shm_attitude = shared_memory.SharedMemory(create=True, size=ATTITUDE_BYTES, name="attitude")
    shm_position = shared_memory.SharedMemory(create=True, size=POSITION_BYTES, name="position")
    shm_local_position_ned = shared_memory.SharedMemory(create=True, size=LOCAL_POSITION_NED_BYTES, name="local_position_ned")
    shm_slam_enabled = shared_memory.SharedMemory(create=True, size=BOOL_BYTES, name="slam_enabled")
    shm_slam_target = shared_memory.SharedMemory(create=True, size=TARGET_BYTES, name="slam_target")
    shm_slam_trigger = shared_memory.SharedMemory(create=True, size=BOOL_BYTES, name="slam_trigger")
    print("Movement tester finished allocating shared memory...")

    rgb_frame_mutex = mp.Lock()
    gray_frame_mutex = mp.Lock()
    depth_frame_mutex = mp.Lock()
    attitude_mutex = mp.Lock()
    position_mutex = mp.Lock()
    local_position_ned_mutex = mp.Lock()
    slam_trigger_mutex = mp.Lock()
    slam_enabled_mutex = mp.Lock()

    from vioslam.slam import slam
    from vioslam.broadcaster import broadcaster
    from vioslam.vio import vio
    broadcaster_process = mp.Process(target=broadcaster, args=(rgb_frame_mutex, gray_frame_mutex, depth_frame_mutex, attitude_mutex, local_position_ned_mutex,))
    vio_process = mp.Process(target=vio, args=(gray_frame_mutex, depth_frame_mutex, attitude_mutex, position_mutex, slam_trigger_mutex,))
    slam_process = mp.Process(target=slam, args=(rgb_frame_mutex, depth_frame_mutex, attitude_mutex, position_mutex, slam_enabled_mutex, slam_trigger_mutex,))
    main_process = mp.Process(target=main, args=(shm_local_position_ned, local_position_ned_mutex))

    try:
        broadcaster_process.start()
        time.sleep(3)
        vio_process.start()
        time.sleep(3)
        slam_process.start()
        time.sleep(5)
        main_process.start()
        time.sleep(1)
        main_process.join()
        
    except KeyboardInterrupt:
        print("Movement tester caught keyboard interrupt. Shutting down...")
    finally:
        broadcaster_process.terminate()
        vio_process.terminate()
        slam_process.terminate()
        main_process.terminate()

        broadcaster_process.join()
        vio_process.join()
        slam_process.join()
        main_process.join()

        print("Movement tester cleaning up shared memory...")
        shm_rgb.close()
        shm_rgb.unlink()
        shm_gray.close()
        shm_gray.unlink()
        shm_depth.close()
        shm_depth.unlink()
        shm_calib.close()
        shm_calib.unlink()
        shm_attitude.close()
        shm_attitude.unlink()
        shm_position.close()
        shm_position.unlink()
        shm_local_position_ned.close()
        shm_local_position_ned.unlink()
        shm_slam_enabled.close()
        shm_slam_enabled.unlink()
        shm_slam_target.close()
        shm_slam_target.unlink()
        shm_slam_trigger.close()
        shm_slam_trigger.unlink()
        print("Movement tester processes terminated safely.")