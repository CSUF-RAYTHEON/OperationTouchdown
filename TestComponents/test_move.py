# This code tests the movement of the drone using custom local positioning, we use the local position messages to calculate our own custom local position 
# and then we use those values to tell the drone to move to specific coordinates relative to its current position. 
import global_variables
from test_local_position import test_update_local_position
import time

def test_takeoff(master, height):
    print(f"Entered test_takeoff() for Target System: {master.target_system} & Target Component: {master.target_component}")

    # 1. Takeoff to specified height (negative value for NED frame)
    test_move(master, 0, 0, -height)

def test_land_current_position(master):
    print(f"Entered test_land_current_position() for Target System: {master.target_system} & Target Component: {master.target_component}")

    # 1. Land by using land command, this command using latitude and longitude however if set to 0 for both then it will ignore them
    #    and instead land at current position. This allows the code to work for gps and non-gps implementations. 
    master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_NAV_LAND,
    0, 0, 0, 0, 0, 0, 0, 0)

def test_move(master, x, y, z):
    print(f"Entered test_move() for Target System: {master.target_system} & Target Component: {master.target_component}")

    # 1. Tell drone to move to specified position relative to current position using custom origin coordinates reference frame, until within 0.8 meters of target position
    while(abs(global_variables.current_position_custom_x - x) > 0.8 and abs(global_variables.current_position_custom_y - y) > 0.8 and abs(global_variables.current_position_custom_z - z) > 0.8):
        test_hold_position(master, global_variables.origin_drone_x + x, global_variables.origin_drone_y + y, global_variables.origin_drone_z + z, 2)
        test_update_local_position(master)



def test_goto(master, x, y, z):
    # 1. Send SET_POSITION_TARGET_LOCAL_NED message to move the drone to the specified position
    master.mav.set_position_target_local_ned_send(0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, 
        ( mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE),
        x, y, z, # X, Y, Z (NED)
        0, 0, 0,
        0, 0, 0,
        0, 0)


def test_hold_position(master, x, y, z, duration):
    # 1. Continously tell drone to go to specified position for specified duration
    start_time = time.time()
    while time.time() - start_time < duration:
        test_goto(master, x, y, z)
        time.sleep(0.5)