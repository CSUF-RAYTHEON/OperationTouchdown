from pymavlink import mavutil
#from test_disable_safety2 import set_param
import serial
import time


master = mavutil.mavlink_connection('/dev/serial0', baud=57600)

def heartbeat():
    print("Waiting for heartbeat from Pixhawk...")
    master.wait_heartbeat()
    print(f"Heartbeat from system (system {master.target_system} component {master.target_component})")
    print(f"Using MAVLink 2.0: {master.mavlink20()} \n\n\n")

def arm_motors():
    print("Arming Drone Component(Motors)...\n")
    master.arducopter_arm()
    # master.mav.command_long_send(
    #     master.target_system,
    #     master.target_component,
    #     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    #     0,
    #     1,
    #     0, 0, 0, 0, 0, 0
    # )
    heartbeat()
    time.sleep(2)
    print(master.motors_armed())

    # 2. Switch to GUIDED mode
    # We fetch the specific integer ID for 'GUIDED' mode from ArduPilot's mapping
    mode_id = master.mode_mapping()['GUIDED']
    print("Switching to GUIDED mode...")
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )

    heartbeat()
    master.recv_match(type='HEARTBEAT', blocking=True)
    time.sleep(2)
    print(master.flightmode)
    master.motors_armed_wait()
    print("Motors Armed!\n")

    print(f"base {master.base_mode}")

def disarm_motors():
    print("Disarming Drone Component(Motors)...\n")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, # 1 = ARM, 0 = DISARM
        0, 0, 0, 0, 0, 0
    )
    master.motors_disarmed_wait()
    print("Motors Disarmed!\n")

def set_nogps_mode():
    master.set_mode_apm("STABILIZE")
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        b"GPS_TYPE",
        float(0),
        mavutil.mavlink.MAV_PARAM_TYPE_INT32
    )

def disable_safety_checks(master):
    """Disable safety and arming checks for bench tests."""
    print("Disabling safety switch and arming checks...")
    params = {"ARMING_REQUIRE": 1, "ARMING_CHECK": 1, "ARMING_ACCTHRESH": 0.255, "ARMING_MAGTHRESH": 50, "ARMING_NEED_LOC": 0}
    for name, value in params.items():
        try:
            master.mav.param_set_send(master.target_system, master.target_component,
                                      name.encode(), float(value),
                                      mavutil.mavlink.MAV_PARAM_TYPE_INT32)
            print(f"Set Parameter ({name}) = {value}")
            msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
            print(f"MESSAGE: {msg.get_type()}")
            print(f"DATA: {msg.to_dict()}")

            time.sleep(0.2)
        except Exception as e:
            print(f"Failed to set {name}: {e}")

    print("\nParameters sent. You may need to reboot FCU for sensors to reinit.")


master = mavutil.mavlink_connection('/dev/serial0', baud=57600)
print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Heartbeat OK listening for STATUSTEXT (pre-arm) messages. Press Ctrl-C to stop.")

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
    print("Stopped")



heartbeat()
#set_nogps_mode()
disable_safety_checks(master)
#master.set_mode_manual()
#master.param_set_send("COM_ARM_WO_GPS",1)
#master.param_set_send("EKF3_REQ_GPS",0)
arm_motors()
disarm_motors()
while True:
    msg = master.recv_match(blocking=True)
    if not msg:
        continue
    print(msg)