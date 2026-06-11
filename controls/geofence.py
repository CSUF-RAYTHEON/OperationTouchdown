# This code includes a utility function that changes parameters such that the drone uses a geofence
from pymavlink import mavutil

def geofence(master):
    print(f"Entered geofence() & Setting Geofence Parameters for Target System: {master.target_system} & Target Component: {master.target_component}")

    # 1. Setting Geofence parameters for the drone, these parameters dicate what the drone will use for Geofence and positioning.
    #    These are not all of the parameters however, so if for some reason other parameters are changed then it could fail
    #    to arm or work properly. We also print out the values it becomes and the associated parameter. Both are printed to the terminal for logging purposes.  
    params = {"FENCE_ENABLE": 1, "FENCE_TYPE": 2, "FENCE_ACTION": 2, "FENCE_ALT_MAX": 20, "FENCE_ALT_MAX_TP": 2}
    for name, value in params.items():
        try:
            master.mav.param_set_send(master.target_system, master.target_component, name.encode(), float(value), mavutil.mavlink.MAV_PARAM_TYPE_INT32)
            print(f"Set Parameter ({name}) = {value}")
            msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
            print(f"MESSAGE: {msg.get_type()}")
            print(f"DATA: {msg.to_dict()}\n")
            if not msg:
                continue

        except Exception as e:
            print(f"Failed to set {name}: {e}", end=" ")
    master.wait_heartbeat()
    print("\nParameters set. You may need to reboot FCU for sensors to reinit.")

def disable_geofence(master):
    print(f"Entered disable_geofence() & Setting Geofence Parameters for Target System: {master.target_system} & Target Component: {master.target_component}")

    # 1. Setting Geofence parameters for the drone, these parameters dicate what the drone will use for Geofence and positioning.
    #    These are not all of the parameters however, so if for some reason other parameters are changed then it could fail
    #    to arm or work properly. We also print out the values it becomes and the associated parameter. Both are printed to the terminal for logging purposes.  
    params = {"FENCE_ENABLE": 0}
    for name, value in params.items():
        try:
            master.mav.param_set_send(master.target_system, master.target_component, name.encode(), float(value), mavutil.mavlink.MAV_PARAM_TYPE_INT32)
            print(f"Set Parameter ({name}) = {value}")
            msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
            print(f"MESSAGE: {msg.get_type()}")
            print(f"DATA: {msg.to_dict()}\n")
            if not msg:
                continue

        except Exception as e:
            print(f"Failed to set {name}: {e}", end=" ")
    master.wait_heartbeat()
    print("\nParameters set. You may need to reboot FCU for sensors to reinit.")

if __name__ == "__main__":
    from controls.connect import connect_UART0
    master = connect_UART0()
    geofence(master)