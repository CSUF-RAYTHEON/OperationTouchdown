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