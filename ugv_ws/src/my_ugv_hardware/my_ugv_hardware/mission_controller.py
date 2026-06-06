#!/usr/bin/env python3
"""Mission state machine for Raytheon Autonomous Vehicle Competition.

Handles all three challenge modes and logs required events per
competition rules Appendix B.
"""

import math
import os
import time
from datetime import datetime
from enum import Enum, auto
from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String


class MissionState(Enum):
    IDLE        = auto()
    CHALLENGE_1 = auto()
    CHALLENGE_2 = auto()
    CHALLENGE_3 = auto()
    NAVIGATING  = auto()
    AT_GOAL     = auto()
    COMPLETE    = auto()


class MissionController(Node):
    """State machine that drives the UGV through competition challenges."""

    def __init__(self):
        super().__init__('mission_controller')

        # ── parameters ──────────────────────────────────────────────────────
        self.declare_parameter('challenge_mode',   2)
        self.declare_parameter('straight_speed',   0.1)   # m/s
        self.declare_parameter('goal_tolerance',   1.52)  # m (5 feet)
        self.declare_parameter('mission_log_path', '/home/ugv/mission_log.txt')
        self.declare_parameter('c1_travel_time',   30.0)  # seconds

        self._challenge_mode  = self.get_parameter('challenge_mode').value
        self._straight_speed  = self.get_parameter('straight_speed').value
        self._goal_tolerance  = self.get_parameter('goal_tolerance').value
        self._log_path        = self.get_parameter('mission_log_path').value
        self._c1_travel_time  = self.get_parameter('c1_travel_time').value

        # ── state ────────────────────────────────────────────────────────────
        self._state           = MissionState.IDLE
        self._goal_pose: Optional[PoseStamped] = None
        self._nav_start_time: Optional[float]  = None
        self._c1_start_time:  Optional[float]  = None
        self._ugv_start_time: Optional[float]  = None
        self._mission_start_time: Optional[float] = None

        self._current_x  = 0.0
        self._current_y  = 0.0
        self._current_vx = 0.0

        # ── ROS interfaces ───────────────────────────────────────────────────
        self._cmd_pub   = self.create_publisher(Twist,  '/cmd_vel',  10)
        self._state_pub = self.create_publisher(String, '/mission/state', 10)

        self._goal_sub   = self.create_subscription(
            PoseStamped, '/lora/goal_pose',   self._goal_cb,    10)
        self._cmd_sub    = self.create_subscription(
            String,      '/lora/mission_cmd', self._mission_cmd_cb, 10)
        self._odom_sub   = self.create_subscription(
            __import__('nav_msgs.msg', fromlist=['Odometry']).Odometry,
            '/odom', self._odom_cb, 10)

        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 10 Hz state-machine tick
        self._timer = self.create_timer(0.1, self._tick)

        # Open log file
        os.makedirs(os.path.dirname(self._log_path), exist_ok=True)
        self._log(f'=== Mission Controller started — challenge_mode={self._challenge_mode} ===')
        self.get_logger().info(
            f'MissionController ready — challenge_mode={self._challenge_mode}'
        )

    # ──────────────────────────────────────────────────────────────────────────
    # Logging
    # ──────────────────────────────────────────────────────────────────────────

    def _log(self, msg: str):
        """Log to ROS logger AND mission log file."""
        ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        line = f'[{ts}] {msg}'
        self.get_logger().info(msg)
        try:
            with open(self._log_path, 'a') as f:
                f.write(line + '\n')
        except Exception as e:
            self.get_logger().warn(f'Log write failed: {e}')

    # ──────────────────────────────────────────────────────────────────────────
    # Callbacks
    # ──────────────────────────────────────────────────────────────────────────

    def _odom_cb(self, msg):
        """Track current pose and speed from odometry."""
        self._current_x  = msg.pose.pose.position.x
        self._current_y  = msg.pose.pose.position.y
        self._current_vx = msg.twist.twist.linear.x

    def _goal_cb(self, msg: PoseStamped):
        """Handle GOAL pose received from LoRa."""
        gx = msg.pose.position.x
        gy = msg.pose.position.y
        self._log(f'DESTINATION RECEIVED from LoRa — x={gx:.3f} y={gy:.3f}')
        self._log(f'LOCATION OF DESTINATION: ({gx:.3f}, {gy:.3f}) in map frame')
        self._log(f'UGV RECEIPT OF DESTINATION LOCATION at {datetime.now().isoformat()}')

        self._goal_pose = msg

        if self._state in (MissionState.CHALLENGE_2,
                           MissionState.CHALLENGE_3,
                           MissionState.IDLE,
                           MissionState.AT_GOAL):
            self._start_navigation(msg)

    def _mission_cmd_cb(self, msg: String):
        """Handle mission commands from LoRa (STRAIGHT, STOP, etc.)."""
        cmd = msg.data.strip().upper()
        self._log(f'LORA MISSION COMMAND RECEIVED: {cmd}')

        if cmd == 'STRAIGHT':
            self._log('UAV START TIME: Challenge 1 initiated by UAV STRAIGHT command')
            self._transition(MissionState.CHALLENGE_1)

        elif cmd == 'STOP':
            self._log('STOP command received — halting UGV')
            self._stop_motors()
            self._transition(MissionState.COMPLETE)

        elif cmd == 'START':
            mode = self._challenge_mode
            if mode == 1:
                self._transition(MissionState.CHALLENGE_1)
            elif mode == 2:
                self._transition(MissionState.CHALLENGE_2)
            elif mode == 3:
                self._transition(MissionState.CHALLENGE_3)

    # ──────────────────────────────────────────────────────────────────────────
    # State transitions
    # ──────────────────────────────────────────────────────────────────────────

    def _transition(self, new_state: MissionState):
        self._log(f'STATE: {self._state.name} → {new_state.name}')
        self._state = new_state
        state_msg = String()
        state_msg.data = new_state.name
        self._state_pub.publish(state_msg)

        if new_state == MissionState.CHALLENGE_1:
            self._start_challenge_1()
        elif new_state in (MissionState.CHALLENGE_2, MissionState.CHALLENGE_3):
            self._log(f'Waiting for goal coordinates via LoRa (Challenge {new_state.name[-1]})')

    def _start_challenge_1(self):
        """Drive straight at constant speed for C1."""
        self._c1_start_time  = time.monotonic()
        self._mission_start_time = time.monotonic()
        self._ugv_start_time = time.monotonic()
        self._log(f'UGV START TIME: {datetime.now().isoformat()}')
        self._log(f'CHALLENGE 1: driving straight at {self._straight_speed} m/s')

    def _start_navigation(self, goal: PoseStamped):
        """Send a Nav2 NavigateToPose goal."""
        gx = goal.pose.position.x
        gy = goal.pose.position.y
        self._ugv_start_time = time.monotonic()
        self._nav_start_time = time.monotonic()
        self._log(f'UGV START TIME: {datetime.now().isoformat()}')
        self._log(f'Sending Nav2 goal: ({gx:.3f}, {gy:.3f})')
        self._transition(MissionState.NAVIGATING)

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('Nav2 action server not available — will retry')
            return

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal

        self._log(f'UGV GENERATED PATH: Nav2 computing path to ({gx:.3f}, {gy:.3f})')
        future = self._nav_client.send_goal_async(
            nav_goal,
            feedback_callback=self._nav_feedback_cb
        )
        future.add_done_callback(self._nav_goal_response_cb)

    def _nav_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 rejected goal')
            return
        self._log('Nav2 accepted goal — navigating')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_cb)

    def _nav_feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        dist = fb.distance_remaining
        self._log(f'UGV SPEED: {self._current_vx:.3f} m/s | distance_remaining={dist:.2f} m')

    def _nav_result_cb(self, future):
        result = future.result().result
        self._log(f'Nav2 navigation result code: {result}')
        self._on_arrived()

    # ──────────────────────────────────────────────────────────────────────────
    # State machine tick
    # ──────────────────────────────────────────────────────────────────────────

    def _tick(self):
        if self._state == MissionState.CHALLENGE_1:
            self._tick_c1()
        elif self._state == MissionState.NAVIGATING:
            self._tick_navigating()

    def _tick_c1(self):
        """Publish forward cmd_vel and check 30s timeout."""
        elapsed = time.monotonic() - self._c1_start_time
        if elapsed >= self._c1_travel_time:
            self._log(f'CHALLENGE 1 COMPLETE: 30s elapsed — stopping')
            self._log(f'UGV END TIME: {datetime.now().isoformat()}')
            self._stop_motors()
            self._transition(MissionState.COMPLETE)
            return

        cmd = Twist()
        cmd.linear.x = self._straight_speed
        self._cmd_pub.publish(cmd)

    def _tick_navigating(self):
        """Monitor distance to goal; stop when within tolerance."""
        if self._goal_pose is None:
            return

        gx = self._goal_pose.pose.position.x
        gy = self._goal_pose.pose.position.y
        dist = math.hypot(gx - self._current_x, gy - self._current_y)

        if dist <= self._goal_tolerance:
            self._log(f'UGV within {dist:.2f}m of goal (tolerance={self._goal_tolerance}m)')
            self._on_arrived()

    def _on_arrived(self):
        """Called when UGV reaches goal."""
        if self._state in (MissionState.AT_GOAL, MissionState.COMPLETE):
            return

        self._stop_motors()

        # Cancel Nav2 if still running
        if self._nav_client.server_is_ready():
            pass  # Nav2 result_cb already fired or tolerance check beat it

        end_time = datetime.now().isoformat()
        self._log(f'UGV END TIME: {end_time}')
        self._log(f'UAV END TIME (estimated): {end_time} (UAV should land now)')
        self._log('AT GOAL — waiting for UAV to land')
        self._transition(MissionState.AT_GOAL)

    def _stop_motors(self):
        try:
            cmd = Twist()
            self._cmd_pub.publish(cmd)
        except Exception:
            pass
        self.get_logger().info('Motors stopped')

    def destroy_node(self):
        self._stop_motors()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
