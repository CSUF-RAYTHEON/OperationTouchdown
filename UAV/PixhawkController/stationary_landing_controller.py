# Stationary Landing Controller
#
# Uses MAVLink to command Pixhawk velocity
# Converts camera frame to body frame
# Implements velocity control
# Lands drone and cuts motors
# Has a takeoff function for testing 
#
# Assumptions:
# Downward facing camera
# Camera centered on drone
# Pixhawk stabilizes drone

from pymavlink import mavutil
import time

# PD control gains.  Kp drives the drone toward the tag; Kd opposes the rate
# of change of the position error so the drone "brakes" as it approaches the
# target instead of coasting through on inertia.  Pure P-control caused the
# drone to overshoot the tag and lose it from the camera FOV.
Kp_xy = 0.35
Kd_xy = 0.25
Kp_z  = 0.3

# Safety cap on velocity commands (m/s) at full altitude.  This is scaled
# down linearly as the drone descends so low-altitude micro-adjustments
# can't lunge past the tag.
MAX_VELOCITY = 0.3

# Below this altitude (m) lateral gain and max velocity are scaled down
# linearly with altitude.  At touchdown height the gain is reduced to
# MIN_GAIN_SCALE of its cruise value, giving small, deliberate corrections.
GAIN_SCALE_ALTITUDE = 1.5
MIN_GAIN_SCALE = 0.3

# Maximum lateral error allowed before descent is throttled, expressed as a
# fraction of the current altitude.  Roughly: the tag must stay within a
# ~20° cone below the drone or descent slows to 20% of its commanded rate.
# Without this the drone descends while drifting, the camera FOV shrinks,
# and the tag falls out of frame.
DESCENT_XY_RATIO = 0.35

# If no measurement has been seen for this many seconds we discard the
# filter/derivative history and re-seed from the next measurement.  This
# prevents a giant D-term spike when the tag reappears after a brief loss.
STALE_STATE_TIMEOUT = 0.5


class StationaryLandingController:

    def __init__(self, connection_string, baudrate):

        print("[INFO] Connecting to Pixhawk...")

        self.master = mavutil.mavlink_connection(connection_string, baud=baudrate)
        self.master.target_system = 1 # Send messages to system 1(drone/vehicle #1)
        self.master.target_component = 1 # Send messages to flight controller "autopilot"
        print("Waiting for heartbeat...")
        self.master.wait_heartbeat()
        print("Heartbeat Received & Connection Established")
        print(f"Source System: {self.master.source_system}, Source Component: {self.master.source_component}, Target System: {self.master.target_system}, Target Component: {self.master.target_component}, Connection Type: {connection_string}, Baudrate: {baudrate}")
        print("[INFO] Pixhawk Connected")

        # None signals "not yet initialised"; the first detection seeds the
        # filter directly so the initial velocity command is always correct.
        # Starting at 0 caused the drone to compute error_z = 0 - 0.3 = -0.3
        # on the very first frame, sending an ASCEND command instead of DESCEND.
        self.prev_x = None
        self.prev_y = None
        self.prev_z = None
        self.prev_t = None
    
    def heartbeat(self):
        print("Waiting for heartbeat from Pixhawk...")
        self.master.wait_heartbeat()
        print(f"Heartbeat from system (system {self.master.target_system} component {self.master.target_component})")
        print(f"Using MAVLink 2.0: {self.master.mavlink20()} \n\n\n")

    def arm_motors(self):
        print(f"Entered arm_drone() & Setting Arming Parameters for Target System: {self.master.target_system} & Target Component: {self.master.target_component}")

        # 1. Setting some arming parameters for the drone, these parameters dicate under what conditions the drone will arm.
        #    These are not all of the parameters however, so if for some reason other parameters are changed then it could fail
        #    to arm. The ARMING_REQUIRE is a parameter for planes, not for drones, this distinction is important as changing
        #    parameters for a plane can result in the drone not arming. So we set ARMING_REQUIRE = 1 which is its default value.
        #    This is to ensure it is always 1 whenever we arm to prevent being unable to arm. We also print out the values it
        #    becomes and the associated parameter. Both are printed to the termal for logging purposes.  
        params = {"ARMING_REQUIRE": 1, "ARMING_CHECK": 1, "ARMING_ACCTHRESH": 0.3, "ARMING_MAGTHRESH": 75, "ARMING_NEED_LOC": 0}
        for name, value in params.items():
            try:
                self.master.mav.param_set_send(self.master.target_system, self.master.target_component, name.encode(), float(value), mavutil.mavlink.MAV_PARAM_TYPE_INT32)
                print(f"Set Parameter ({name}) = {value}")
                msg = self.master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
                print(f"MESSAGE: {msg.get_type()}")
                print(f"DATA: {msg.to_dict()}\n")
                if not msg:
                    continue

            except Exception as e:
                print(f"Failed to set {name}: {e}", end=" ")
        self.master.wait_heartbeat()
        print("\nParameters set. You may need to reboot FCU for sensors to reinit.")

        # 2. Here we arm the drone using the built-in helper function from pymavlink/mavutil library
        print("Arming Drone Motors")
        self.master.arducopter_arm()

        # 3. Flush the buffer until we catch the correct command acknowledement(cmd ack) by pulling the next cmd ack from the queue continously.
        #    This is for logging purposes to confirm that the arming command was sent and accepted by the flight controller. It also serves to 
        #    clear the buffer of any old messages until we catch the correct one that shows the drone is armed.
        print("Reading message buffer to catch up to arming change...")
        start_time = time.time()
        while time.time() - start_time < 3: # Continously read command acknowledgements for 3 seconds
            command_ack_msg = self.master.recv_match(type=['COMMAND_ACK'], blocking=True, timeout=2) # Receive a command acknowledgement message and block up to 2 seconds
            if command_ack_msg is not None:
                if command_ack_msg.command == 400 and command_ack_msg.result == 0:
                    print(f"Command Acknowledgment received for MAV_CMD_COMPONENT_ARM_DISARM(CMD #400) with result MAV_RESULT_ACCEPTED(0)")
                    print(f"{command_ack_msg.get_type()}: {command_ack_msg.to_dict()}\n")
                    break
        else:
            if command_ack_msg is not None:
                print(f"Command Acknowledgment received but timed out with wrong command or result: CMD #{command_ack_msg.command} & CMD Result #{command_ack_msg.result}")
                print(f"{command_ack_msg.get_type()}: {command_ack_msg.to_dict()}")
            else:
                print("Timed out waiting for command acknowledgement message")

        # 4. Here we check if the drone is armed using the built-in helper function from pymavlink/mavutil library to ensure that the drone is armed. This is because sometimes 
        #    the drone can fail to arm due to various reasons such as bad parameters, bad GPS lock, or bad sensor readings. So this is a backup to ensure that the drone 
        #    is armed and ready to fly. The previous step is more for logging purposes to confirm that the arming command was sent and accepted by the flight controller, 
        #    but this step is to ensure that the drone is actually armed. And we also print out the result to the terminal for logging purposes.
        print("Checking if drone armed...")
        start_time = time.time()
        while time.time() - start_time < 3: # Wait for up too 3 seconds for confirmation that the motors armed
            self.master.motors_armed_wait()
            if self.master.motors_armed():
                print("Drone is armed and ready to fly\n")
                break
            else:
                print("Escaped motors_armed_wait() but drone did not arm")
        else:
            if self.master.motors_armed():
                print("Timed out waiting for confirmation that drone is armed. But motors show they are armed.\n")
            else:
                print("Motors failed to arm and timed out waiting for confirmation that drone is armed.\n")

    def disarm_motors(self):
        """
        Disarm the drone
        """
        print("Disarming Drone Component(Motors)...\n")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0, # 1 = ARM, 0 = DISARM
            0, 0, 0, 0, 0, 0
        )
        self.master.motors_disarmed_wait()
        print("Motors Disarmed!\n")

    def change_flight_mode(self, flight_mode):
        """
        Change the flight mode of the drone and confirm the change by reading messages from the buffer.
        """
        print(f"Entered change_flight_mode() for Target System: {self.master.target_system} & Target Component: {self.master.target_component}")

        # 1. ArduPilot will actively reject a flight mode switch if its (EKF) hasn't secured a solid GPS lock 
        #    and stabilized its sensors. So this loop waits for 3 seconds to allow for this to happen.
        #    It will also reject a switch/delete newest messeges that the Pixhawk sends to the Pi, this is because
        #    the message buffer overflows. We solve the overflow issue by using the recv_match() helper function from
        #    the pymavlink/mavutil library to read any incoming messages and clear the buffer.
        #    IMPORTANT: SIMPLY DOING TIME.SLEEP() WILL NOT WORK AS WE NEED TO ALSO READ/CLEAR THE BUFFER
        print("Waiting 3 seconds for sensors, GPS, and reading message queue...")
        start_time = time.time()
        while time.time() - start_time < 3:
            self.master.recv_match(blocking=False) # Grab any waiting message and immediately discard it
            time.sleep(0.1)
        self.master.wait_heartbeat()
        print("Done waiting\n")

        # 2. Switch flight mode using the built-in helper function from pymavlink/mavutil library
        print(f"Switching to {flight_mode} flight mode...")
        self.master.set_mode(flight_mode)

        # 3. Flush the buffer until we catch the correct command acknowledement(cmd ack) by pulling the next cmd ack from the queue. 
        #    This is for logging purposes to confirm that the mode change command was sent and accepted by the flight controller. It also 
        #    serves to clear the buffer of any old messages until we catch the correct one that shows the drone is in the specified mode.
        print("Reading message buffer to catch command acknowledgment...")
        start_time = time.time()
        while time.time() - start_time < 3: # Continously read command acknowledgements for 3 seconds
            command_ack_msg = self.master.recv_match(type=['COMMAND_ACK'], blocking=True, timeout=2) # Receive a command acknowledgement message and block up to 2 seconds
            if command_ack_msg is not None:
                if command_ack_msg.command == 176 and command_ack_msg.result == 0:
                    print(f"Command Acknowledgment received for MAV_CMD_DO_SET_MODE(CMD #176) with result MAV_RESULT_ACCEPTED(0)")
                    print(f"{command_ack_msg.get_type()}: {command_ack_msg.to_dict()}\n")
                    break
        else:
            if command_ack_msg is not None:
                print(f"Command Acknowledgment received but timed out with wrong command or result: CMD #{command_ack_msg.command} & CMD Result #{command_ack_msg.result}")
                print(f"{command_ack_msg.get_type()}: {command_ack_msg.to_dict()}")
            else:
                print("Timed out waiting for command acknowledgement message")

        # 4. Flush the buffer until we catch the updated heartbeat by pulling the next heartbeat from the queue. Pymavlink caches the flight mode based
        #    on the LAST heartbeat it read. If we print the mode immediately, it will falsely print the wrong mode. To solve this we must actively pull 
        #    new heartbeats from the queue until we catch up to the message that proves the flight controller actually did switch modes.
        print("Reading message buffer to get latest heartbeat...")
        start_time = time.time()
        while time.time() - start_time < 3: # Continously read heartbeats for 3 seconds
            heartbeat_msg = self.master.recv_match(type=['HEARTBEAT'], blocking=True, timeout=2) # Recieve a heartbeat message and block up to 2 second
            if (heartbeat_msg is not None) and (self.master.flightmode == flight_mode):
                print(f"Succesfully switched to: {self.master.flightmode} flight mode & Base Mode(MAV_MODE_FLAGS): {self.master.base_mode}\n")
                break
        else:
            print(f"Timed out waiting for {flight_mode}. Current mode seen: {self.master.flightmode} & Base Mode(MAV_MODE_FLAGS): {self.master.base_mode}\n")

    def stationary_landing(self):
        """
        Switch to LAND mode and wait for the drone to touch down.

        MAV_CMD_NAV_LAND is a navigation waypoint command and is not reliably
        honoured by ArduCopter while in GUIDED mode.  Switching to LAND mode
        directly is the correct approach — ArduCopter will descend, touch down,
        and auto-disarm when it detects zero throttle at ground level.
        """
        print("[INFO] Switching to LAND mode...")
        self.change_flight_mode("LAND")

        # Wait for motors to auto-disarm, which confirms touchdown.
        # LAND mode descends at ~0.5–1 m/s; 20 s is a safe upper bound
        # from the ~0.3 m trigger altitude (expected ~1–2 s), but guards
        # against higher-altitude triggering edge cases.
        print("[INFO] Waiting for touchdown and auto-disarm...")
        start = time.time()
        while time.time() - start < 20:
            if not self.master.motors_armed():
                print("[INFO] Motors disarmed — touchdown confirmed.")
                return
            time.sleep(0.5)

        # Motors are still armed after 20 s; force-disarm as a safety fallback.
        print("[WARN] Touchdown not confirmed after 20 s. Force-disarming motors.")
        self.disarm_motors()

    def disable_safety_checks(self):
        """
        Disable safety and arming checks for bench tests.
        """
        print("Disabling safety switch and arming checks...")
        params = {"ARMING_REQUIRE": 1, "ARMING_CHECK": 1, "ARMING_ACCTHRESH": 0.255, "ARMING_MAGTHRESH": 50, "ARMING_NEED_LOC": 0}
        for name, value in params.items():
            try:
                self.master.mav.param_set_send(self.master.target_system, self.master.target_component,
                                        name.encode(), float(value),
                                        mavutil.mavlink.MAV_PARAM_TYPE_INT32)
                print(f"Set Parameter ({name}) = {value}")
                msg = self.master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
                print(f"MESSAGE: {msg.get_type()}")
                print(f"DATA: {msg.to_dict()}")

                time.sleep(0.2)
            except Exception as e:
                print(f"Failed to set {name}: {e}")

        print("\nParameters sent. You may need to reboot FCU for sensors to reinit.")

    def takeoff_to_altitude(self, meters):
        """
        Take off to the specified altitude and block until the drone
        has actually reached within 0.3 m of the target.

        A fixed time.sleep(5) was too short for a 3 m climb in many
        conditions.  Once the main loop starts sending velocity commands
        they override the climb, so the drone must fully reach altitude
        BEFORE the control loop begins.
        """
        print(f"[INFO] Taking off to {meters} meters...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0,
            meters
        )

        print(f"[INFO] Waiting for altitude {meters} m ...")
        timeout = 30  # seconds — generous upper bound
        start = time.time()
        while time.time() - start < timeout:
            msg = self.master.recv_match(
                type="LOCAL_POSITION_NED", blocking=True, timeout=1
            )
            if msg is not None:
                # In NED frame z is negative when above home, so altitude = -z
                altitude = -msg.z
                print(f"[INFO] Altitude: {altitude:.2f} m / {meters} m target")
                if altitude >= meters - 0.3:
                    print(f"[INFO] Target altitude reached ({altitude:.2f} m).")
                    return
            time.sleep(0.1)

        print(f"[WARN] Altitude timeout — proceeding anyway.")

    def send_velocity(self, vx, vy, vz):
        """
        Send velocity command to Pixhawk
        """
        # Send the velocity command to the Pixhawk using MAVLink
        self.master.mav.set_position_target_local_ned_send(
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111,
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            0, 0
        )

    def convert_camera_to_body_frame(self, cam_x, cam_y, cam_z):
        """
        Convert camera frame to the body frame
        """
        # Camera frame to body frame conversion
        #
        # Downward camera: (Got this from chat, we need to check this with actual testing))
        # body_x = -cam_y
        # body_y =  cam_x
        # body_z =  cam_z

        body_x = -cam_y
        body_y = cam_x
        body_z = cam_z

        return body_x, body_y, body_z
    
    def adjust_velocity_and_send(self, body_x, body_y, body_z):
        """
        PD velocity control with altitude-aware gains for precise landing.

        Four mechanisms work together to prevent the overshoot/tag-loss
        behaviour seen with pure P-control:

          1. EMA on the measurement rejects single-frame pose noise.
             alpha=0.5 (was 0.7) — still smooths but with half the lag.
          2. Derivative term opposes rapid changes in position error so the
             drone brakes as it nears the target instead of coasting through
             on inertia.
          3. Lateral gain and MAX_VELOCITY scale linearly with altitude
             below GAIN_SCALE_ALTITUDE, so corrections shrink as the camera
             FOV shrinks.
          4. Vertical descent is throttled whenever the XY error is large
             relative to the current altitude (DESCENT_XY_RATIO).  This is
             the single most important change: without it the drone keeps
             diving while drifting, the FOV collapses, and the tag falls
             out of frame.
        """
        now = time.time()

        # (Re-)seed the filter from the first valid measurement, OR after
        # a long gap (tag was lost) — comparing the current frame against
        # a stale prev_* would produce a huge spurious D-term spike.
        if self.prev_x is None or (now - self.prev_t) > STALE_STATE_TIMEOUT:
            self.prev_x = body_x
            self.prev_y = body_y
            self.prev_z = body_z
            self.prev_t = now

        # Lighter EMA than before (was 0.7).  Heavy filtering created lag,
        # and lag + P-control = overshoot.
        alpha = 0.5
        filt_x = alpha * self.prev_x + (1 - alpha) * body_x
        filt_y = alpha * self.prev_y + (1 - alpha) * body_y
        filt_z = alpha * self.prev_z + (1 - alpha) * body_z

        # Clamp dt to avoid div-by-zero on the seeded frame and to suppress
        # D-term spikes after a frame skip.
        dt = max(min(now - self.prev_t, 0.2), 0.01)

        # Derivative of position == derivative of error (target is 0).
        dx = (filt_x - self.prev_x) / dt
        dy = (filt_y - self.prev_y) / dt

        # Adaptive deadband — tighter near the ground where every cm matters,
        # looser at altitude where small offsets aren't worth twitching for.
        deadband = 0.03 if filt_z < 1.0 else 0.05
        err_x = 0.0 if abs(filt_x) < deadband else filt_x
        err_y = 0.0 if abs(filt_y) < deadband else filt_y

        # Smooth altitude taper on lateral authority.  At cruise altitude
        # gain_scale == 1.0; at touchdown it falls to MIN_GAIN_SCALE.
        if filt_z < GAIN_SCALE_ALTITUDE:
            gain_scale = max(MIN_GAIN_SCALE, filt_z / GAIN_SCALE_ALTITUDE)
        else:
            gain_scale = 1.0

        vx = gain_scale * (Kp_xy * err_x + Kd_xy * dx)
        vy = gain_scale * (Kp_xy * err_y + Kd_xy * dy)

        # --- Vertical control ---
        TARGET_Z = 0.3
        error_z = filt_z - TARGET_Z
        vz = 0.0 if abs(error_z) < 0.05 else Kp_z * error_z

        # Couple descent to XY alignment.  When off-center, slow descent to
        # 20% of commanded rate (but don't stop entirely — we still want to
        # make progress while the XY loop catches up).  Only throttles
        # downward velocity; ascent for "tag too close" is left untouched.
        xy_err = max(abs(filt_x), abs(filt_y))
        if vz > 0 and xy_err > DESCENT_XY_RATIO * max(filt_z, 0.3):
            vz *= 0.2

        # Per-axis cap, scaled with altitude so the drone can't lunge near
        # the ground.  Vertical cap stays at full MAX_VELOCITY.
        max_v_xy = MAX_VELOCITY * gain_scale
        vx = max(min(vx, max_v_xy), -max_v_xy)
        vy = max(min(vy, max_v_xy), -max_v_xy)
        vz = max(min(vz, MAX_VELOCITY), -MAX_VELOCITY)

        # Persist filtered state for the next call's derivative + EMA.
        self.prev_x = filt_x
        self.prev_y = filt_y
        self.prev_z = filt_z
        self.prev_t = now

        self.send_velocity(vx, vy, vz)

        # Return the commanded velocity so the caller can show it on the OSD
        # for live tuning.  Existing callers that ignore the return value
        # continue to work unchanged.
        return vx, vy, vz
