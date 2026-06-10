#!/usr/bin/env python3
"""
UGV Boot Launcher — reads Xbox controller without ROS.
Runs as a systemd service at boot.

Controls (D-pad, hold 2s):
  D-pad UP    → ros2 launch master_nav2_akida.launch.py (Nav2 + Akida)
  D-pad DOWN  → ros2 launch master_akida.launch.py     (controller + Akida)
  D-pad LEFT or RIGHT → kill running launch
"""
import time
import subprocess
import os
import signal
import sys
from pathlib import Path

# Try evdev first, fall back to raw js0 read
try:
    import evdev
    USE_EVDEV = True
except ImportError:
    USE_EVDEV = False

WORKSPACE = "/home/ugv/Desktop/OperationTouchdown/ugv_ws"
LAUNCH_CMD_NAV2 = [
    "bash", "-c",
    f"source /opt/ros/jazzy/setup.bash && source {WORKSPACE}/install/setup.bash && "
    f"ros2 launch my_ugv_bringup master_nav2_akida.launch.py"
]
LAUNCH_CMD_BASE = [
    "bash", "-c",
    f"source /opt/ros/jazzy/setup.bash && source {WORKSPACE}/install/setup.bash && "
    f"ros2 launch my_ugv_bringup master_akida.launch.py"
]

HOLD_DURATION = 2.0  # seconds to hold before triggering

launch_proc = None

def kill_launch():
    global launch_proc
    # Always kill any ros2 launch processes system-wide, regardless of how
    # they were started (SSH, launcher, etc.)
    print("[launcher] Stopping all ROS2 processes...")
    subprocess.run(["pkill", "-SIGTERM", "-f", "ros2 launch"], check=False)
    time.sleep(3)
    # Force-kill anything still alive
    subprocess.run(["pkill", "-SIGKILL", "-f", "ros2 launch"], check=False)
    subprocess.run(["pkill", "-SIGKILL", "-f", "ros2_launch"], check=False)
    if launch_proc:
        try:
            os.killpg(os.getpgid(launch_proc.pid), signal.SIGKILL)
        except Exception:
            pass
        launch_proc = None
    print("[launcher] ROS2 stopped.")

def start_launch(with_nav2: bool):
    global launch_proc
    kill_launch()
    cmd = LAUNCH_CMD_NAV2 if with_nav2 else LAUNCH_CMD_BASE
    print(f"[launcher] Starting ROS launch (nav2={with_nav2})...")
    launch_proc = subprocess.Popen(cmd, preexec_fn=os.setsid)

def run_with_evdev():
    import evdev
    from evdev import ecodes

    # Wait for joystick to appear
    while not Path("/dev/input/js0").exists():
        print("[launcher] Waiting for /dev/input/js0...")
        time.sleep(1)

    # Find the Xbox controller event device
    devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
    joy = None
    for d in devices:
        if "Xbox" in d.name or "Microsoft" in d.name or "js0" in d.path:
            joy = d
            break
    if not joy:
        joy = evdev.InputDevice("/dev/input/js0")

    print(f"[launcher] Monitoring: {joy.name} at {joy.path}")
    print("[launcher] Hold D-pad UP   (2s) = launch Nav2 + Akida")
    print("[launcher] Hold D-pad DOWN (2s) = launch controller + Akida")
    print("[launcher] Hold D-pad LEFT/RIGHT (2s) = stop launch")

    dpad_down_time = {}

    for event in joy.read_loop():
        if event.type == ecodes.EV_ABS:
            if event.code == ecodes.ABS_HAT0Y:  # up/down axis
                if event.value == -1:  # D-pad UP pressed
                    dpad_down_time['up'] = time.monotonic()
                elif event.value == 1:  # D-pad DOWN pressed
                    dpad_down_time['down'] = time.monotonic()
                elif event.value == 0:  # released
                    for direction in ['up', 'down']:
                        if direction in dpad_down_time:
                            held = time.monotonic() - dpad_down_time.pop(direction)
                            if held >= HOLD_DURATION:
                                if direction == 'up':
                                    start_launch(with_nav2=True)
                                elif direction == 'down':
                                    start_launch(with_nav2=False)

            elif event.code == ecodes.ABS_HAT0X:  # left/right axis
                if event.value != 0:  # D-pad LEFT or RIGHT pressed
                    dpad_down_time['lr'] = time.monotonic()
                else:  # released
                    if 'lr' in dpad_down_time:
                        held = time.monotonic() - dpad_down_time.pop('lr')
                        if held >= HOLD_DURATION:
                            kill_launch()

def run_raw_js():
    """Fallback: read raw /dev/input/js0 binary events."""
    import struct
    JS_EVENT_AXIS   = 0x02
    # axis 7 = ABS_HAT0Y: value < 0 = up, value > 0 = down, 0 = center
    # axis 6 = ABS_HAT0X: value != 0 = left/right, 0 = center
    AXIS_HAT0X = 6
    AXIS_HAT0Y = 7

    while not Path("/dev/input/js0").exists():
        print("[launcher] Waiting for /dev/input/js0...")
        time.sleep(1)

    print("[launcher] Using raw js0 reader (evdev not available)")
    dpad_down_time = {}

    with open("/dev/input/js0", "rb") as js:
        while True:
            data = js.read(8)
            if len(data) < 8:
                continue
            time_ms, value, type_, number = struct.unpack("IhBB", data)
            if type_ & JS_EVENT_AXIS:
                if number == AXIS_HAT0Y:
                    if value < 0:  # D-pad UP
                        dpad_down_time['up'] = time.monotonic()
                    elif value > 0:  # D-pad DOWN
                        dpad_down_time['down'] = time.monotonic()
                    else:  # released
                        for direction in ['up', 'down']:
                            if direction in dpad_down_time:
                                held = time.monotonic() - dpad_down_time.pop(direction)
                                if held >= HOLD_DURATION:
                                    if direction == 'up':
                                        start_launch(with_nav2=True)
                                    elif direction == 'down':
                                        start_launch(with_nav2=False)

                elif number == AXIS_HAT0X:
                    if value != 0:  # D-pad LEFT or RIGHT
                        dpad_down_time['lr'] = time.monotonic()
                    else:  # released
                        if 'lr' in dpad_down_time:
                            held = time.monotonic() - dpad_down_time.pop('lr')
                            if held >= HOLD_DURATION:
                                kill_launch()

if __name__ == "__main__":
    print("[launcher] UGV Boot Launcher started")
    if USE_EVDEV:
        run_with_evdev()
    else:
        run_raw_js()
