# Copyright 2026
# Licensed under the Apache License, Version 2.0
"""ROS 2 driver node that wraps xgolib.py for the XGO-Lite2 quadruped.

Design highlights
-----------------
* Command interfaces
    - /cmd_vel          (geometry_msgs/Twist)              -> move_x / move_y / turn
    - /body_posture     (xgo_lite2_msgs/BodyPosture)       -> translation / attitude
    - /joint_cmds       (sensor_msgs/JointState, rad)      -> motor()
    - /leg_cmd          (xgo_lite2_msgs/LegPosition, mm)   -> leg()
    - /arm_cmd          (geometry_msgs/Point, mm; y unused)-> arm()
    - /claw_cmd         (std_msgs/UInt8, 0=open..255=closed)-> claw()
    - /mark_time_height (std_msgs/UInt8, mm)               -> mark_time()
    - /motor_speed      (std_msgs/UInt8, 1..255)           -> motor_speed()
* Services
    - /set_imu_mode         (std_srvs/SetBool)             -> imu()
    - /set_perform_mode     (std_srvs/SetBool)             -> perform()
    - /set_arm_mode         (std_srvs/SetBool)             -> arm_mode()
    - /set_calibration      (std_srvs/SetBool)             -> calibration()
    - /set_gait_type        (xgo_lite2_msgs/SetGaitType)   -> gait_type()
    - /set_pace             (xgo_lite2_msgs/SetPace)       -> pace()
    - /set_motor_load       (xgo_lite2_msgs/SetMotorLoad)  -> (un)load_motor()
    - /reset                (std_srvs/Trigger)             -> reset()
* Actions
    - /play_action          (xgo_lite2_msgs/PlayPreset)    -> action(id)
* Observations
    - /joint_states     (sensor_msgs/JointState, rad)      <- read_motor()
    - /imu/data         (sensor_msgs/Imu)                  <- read_roll/pitch/yaw()
    - /battery_state    (sensor_msgs/BatteryState)         <- read_battery()
    - /odom             (nav_msgs/Odometry, open-loop)     <- integrated /cmd_vel

At startup the node calls ``read_firmware()`` and, when ``version=='auto'``,
selects the xgolib parameter table for XGO-Mini (firmware string starting
with ``M``) or XGO-Lite otherwise.

Because the underlying serial protocol is single-threaded and its reads block
for up to ~1 s, all API access is serialised behind a single ``threading.Lock``
and the node runs on a ``MultiThreadedExecutor`` so a slow read cannot stall
command subscribers.
"""

from __future__ import annotations

import math
import threading
import time
from typing import List, Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Point, Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Imu, JointState
from std_msgs.msg import UInt8
from std_srvs.srv import SetBool, Trigger

from xgo_lite2_msgs.action import PlayPreset
from xgo_lite2_msgs.msg import BodyPosture, LegPosition
from xgo_lite2_msgs.srv import SetGaitType, SetMotorLoad, SetPace

from .xgolib import XGO


DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi


# Default joint name <-> motor id mapping.
# Per the Elecfreaks official documentation, ``read_motor()`` returns 12 leg
# servo angles corresponding to motor ids
# [11,12,13, 21,22,23, 31,32,33, 41,42,43] where the first digit encodes the
# leg (1=LF, 2=RF, 3=RR, 4=LR) and the second digit encodes the joint
# (1=lower/knee, 2=middle/thigh, 3=upper/hip).
# The claw (id 51) and arm servos (52, 53) are handled through dedicated
# topics (``claw_cmd`` / ``arm_cmd``), not through ``joint_states``.
DEFAULT_MOTOR_IDS: List[int] = [
    11, 12, 13,
    21, 22, 23,
    31, 32, 33,
    41, 42, 43,
]
DEFAULT_JOINT_NAMES: List[str] = [
    'lf_lower', 'lf_middle', 'lf_upper',
    'rf_lower', 'rf_middle', 'rf_upper',
    'rr_lower', 'rr_middle', 'rr_upper',
    'lr_lower', 'lr_middle', 'lr_upper',
]


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class XgoDriverNode(Node):
    """ROS 2 wrapper around :class:`xgolib.XGO`."""

    def __init__(self) -> None:
        super().__init__('xgo_lite2_driver')

        # ---------------- parameters ----------------
        self.declare_parameter('serial_port', '/dev/ttyAMA0')
        self.declare_parameter('baudrate', 115200)
        # 'auto' -> query firmware and pick xgolite / xgomini automatically.
        self.declare_parameter('version', 'auto')

        # cmd_vel scaling. Input is assumed [m/s] / [rad/s], output is xgolib step
        # units. xgolib turn() step is in deg/s (limit 100), so gain 57.3 maps
        # 1 rad/s -> 57.3 deg/s. linear_gain_* map 1 m/s -> full-scale step.
        self.declare_parameter('linear_gain_x', 25.0)
        self.declare_parameter('linear_gain_y', 18.0)
        self.declare_parameter('angular_gain', 57.3)
        self.declare_parameter('cmd_vel_timeout', 0.5)

        # Joint mapping (must have same length).
        self.declare_parameter('joint_names', DEFAULT_JOINT_NAMES)
        self.declare_parameter('motor_ids', DEFAULT_MOTOR_IDS)

        # Publish rates [Hz]; set to 0 to disable a stream.
        self.declare_parameter('joint_state_rate_hz', 20.0)
        self.declare_parameter('imu_rate_hz', 20.0)
        self.declare_parameter('battery_rate_hz', 1.0)
        self.declare_parameter('odom_rate_hz', 20.0)

        # Frames.
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('imu_frame_id', 'imu_link')
        self.declare_parameter('publish_odom_tf', False)

        # PlayPreset action defaults.
        self.declare_parameter('default_preset_duration', 3.0)
        self.declare_parameter('preset_feedback_rate_hz', 5.0)

        self._load_parameters()

        # ---------------- hardware ----------------
        self._hw_lock = threading.Lock()
        try:
            # Open with a safe default first; version may be re-applied below
            # after firmware auto-detection.
            init_version = ('xgolite' if self._version == 'auto'
                            else self._version)
            self.dog = XGO(self._serial_port, baud=self._baudrate,
                           version=init_version)
        except Exception as exc:  # pragma: no cover - depends on hardware
            self.get_logger().error(
                f'Failed to open XGO serial port {self._serial_port}: {exc}')
            raise

        # Detect firmware / model per official doc guidance.
        self._detect_and_apply_version()

        # ---------------- executor / cbgs ----------------
        # Reentrant so the executor can service reads and writes concurrently;
        # thread-safety is enforced by ``self._hw_lock``.
        self._cb_cmd = ReentrantCallbackGroup()
        self._cb_read = ReentrantCallbackGroup()

        # ---------------- QoS ----------------
        cmd_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST, depth=10)
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=10)

        # ---------------- publishers ----------------
        self.pub_joint = self.create_publisher(
            JointState, 'joint_states', sensor_qos)
        self.pub_imu = self.create_publisher(Imu, 'imu/data', sensor_qos)
        self.pub_batt = self.create_publisher(
            BatteryState, 'battery_state', sensor_qos)
        self.pub_odom = self.create_publisher(Odometry, 'odom', sensor_qos)

        self._tf_bc = None
        if self._publish_odom_tf:
            from tf2_ros import TransformBroadcaster  # lazy import
            self._tf_bc = TransformBroadcaster(self)

        # ---------------- subscribers ----------------
        self.create_subscription(
            Twist, 'cmd_vel', self._on_cmd_vel, cmd_qos,
            callback_group=self._cb_cmd)
        self.create_subscription(
            BodyPosture, 'body_posture', self._on_body_posture, cmd_qos,
            callback_group=self._cb_cmd)
        self.create_subscription(
            JointState, 'joint_cmds', self._on_joint_cmds, cmd_qos,
            callback_group=self._cb_cmd)
        self.create_subscription(
            LegPosition, 'leg_cmd', self._on_leg_cmd, cmd_qos,
            callback_group=self._cb_cmd)
        # Extra hardware primitives exposed as topics (fire-and-forget).
        self.create_subscription(
            Point, 'arm_cmd', self._on_arm_cmd, cmd_qos,
            callback_group=self._cb_cmd)
        self.create_subscription(
            UInt8, 'claw_cmd', self._on_claw_cmd, cmd_qos,
            callback_group=self._cb_cmd)
        self.create_subscription(
            UInt8, 'mark_time_height', self._on_mark_time, cmd_qos,
            callback_group=self._cb_cmd)
        self.create_subscription(
            UInt8, 'motor_speed', self._on_motor_speed, cmd_qos,
            callback_group=self._cb_cmd)

        # ---------------- services ----------------
        self.create_service(
            SetBool, 'set_imu_mode', self._srv_set_imu,
            callback_group=self._cb_cmd)
        self.create_service(
            SetBool, 'set_perform_mode', self._srv_set_perform,
            callback_group=self._cb_cmd)
        self.create_service(
            SetBool, 'set_arm_mode', self._srv_set_arm_mode,
            callback_group=self._cb_cmd)
        self.create_service(
            SetBool, 'set_calibration', self._srv_set_calibration,
            callback_group=self._cb_cmd)
        self.create_service(
            SetGaitType, 'set_gait_type', self._srv_set_gait,
            callback_group=self._cb_cmd)
        self.create_service(
            SetPace, 'set_pace', self._srv_set_pace,
            callback_group=self._cb_cmd)
        self.create_service(
            SetMotorLoad, 'set_motor_load', self._srv_set_motor_load,
            callback_group=self._cb_cmd)
        self.create_service(
            Trigger, 'reset', self._srv_reset,
            callback_group=self._cb_cmd)

        # ---------------- action server ----------------
        self._action_srv = ActionServer(
            self,
            PlayPreset,
            'play_action',
            execute_callback=self._exec_play_action,
            goal_callback=lambda _goal: GoalResponse.ACCEPT,
            cancel_callback=lambda _gh: CancelResponse.ACCEPT,
            callback_group=self._cb_cmd,
        )

        # ---------------- state ----------------
        self._cmd_vel_lock = threading.Lock()
        self._last_cmd_vel = (0.0, 0.0, 0.0)  # vx [m/s], vy [m/s], wz [rad/s]
        self._last_cmd_vel_time: Optional[rclpy.time.Time] = None
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0
        self._last_odom_time: Optional[rclpy.time.Time] = None

        # ---------------- timers ----------------
        self._maybe_create_timer(
            self._joint_state_rate_hz, self._pub_joint_states)
        self._maybe_create_timer(self._imu_rate_hz, self._pub_imu)
        self._maybe_create_timer(self._battery_rate_hz, self._pub_battery)
        self._maybe_create_timer(self._odom_rate_hz, self._pub_odom)
        self._maybe_create_timer(10.0, self._watchdog_cmd_vel)

        self.get_logger().info(
            f'XGO driver ready on {self._serial_port} '
            f'(version={self._version}).')

    # ---------------------------------------------------------------- helpers
    def _load_parameters(self) -> None:
        gp = self.get_parameter
        self._serial_port = gp('serial_port').value
        self._baudrate = int(gp('baudrate').value)
        self._version = gp('version').value

        self._linear_gain_x = float(gp('linear_gain_x').value)
        self._linear_gain_y = float(gp('linear_gain_y').value)
        self._angular_gain = float(gp('angular_gain').value)
        self._cmd_vel_timeout = float(gp('cmd_vel_timeout').value)

        self._joint_names = list(gp('joint_names').value)
        self._motor_ids = [int(i) for i in gp('motor_ids').value]
        if len(self._joint_names) != len(self._motor_ids):
            raise RuntimeError(
                'joint_names and motor_ids parameters must have identical '
                f'length ({len(self._joint_names)} vs {len(self._motor_ids)}).')
        self._joint_index = {n: i for i, n in enumerate(self._joint_names)}

        self._joint_state_rate_hz = float(gp('joint_state_rate_hz').value)
        self._imu_rate_hz = float(gp('imu_rate_hz').value)
        self._battery_rate_hz = float(gp('battery_rate_hz').value)
        self._odom_rate_hz = float(gp('odom_rate_hz').value)

        self._base_frame_id = gp('base_frame_id').value
        self._odom_frame_id = gp('odom_frame_id').value
        self._imu_frame_id = gp('imu_frame_id').value
        self._publish_odom_tf = bool(gp('publish_odom_tf').value)

        self._default_preset_duration = float(
            gp('default_preset_duration').value)
        self._preset_feedback_rate_hz = float(
            gp('preset_feedback_rate_hz').value)

    def _maybe_create_timer(self, hz: float, cb) -> None:
        if hz <= 0.0:
            return
        self.create_timer(1.0 / hz, cb, callback_group=self._cb_read)

    def _detect_and_apply_version(self) -> None:
        """Query firmware and re-apply the correct xgolib parameter table."""
        fw = ''
        try:
            with self._hw_lock:
                fw = self.dog.read_firmware() or ''
        except Exception as exc:  # pragma: no cover - hardware
            self.get_logger().warn(f'read_firmware failed: {exc}')
            return
        self.get_logger().info(f'XGO firmware string: "{fw}"')
        if self._version != 'auto':
            return
        # Per official doc: firmware starting with 'M' -> XGO-MINI, else LITE.
        detected = 'xgomini' if fw.startswith('M') else 'xgolite'
        self.get_logger().info(f'Auto-detected model: {detected}')
        try:
            # Re-init to apply detected parameter table without reopening serial.
            from .xgolib import changePara
            changePara(detected)
            self._version = detected
        except Exception as exc:  # pragma: no cover - hardware
            self.get_logger().warn(f'Failed to apply detected version: {exc}')

    # ---------------------------------------------------------- cmd handlers
    def _on_cmd_vel(self, msg: Twist) -> None:
        vx = clamp(msg.linear.x, -1e3, 1e3)
        vy = clamp(msg.linear.y, -1e3, 1e3)
        wz = clamp(msg.angular.z, -1e3, 1e3)

        step_x = vx * self._linear_gain_x
        step_y = vy * self._linear_gain_y
        step_w = wz * self._angular_gain

        with self._cmd_vel_lock:
            self._last_cmd_vel = (vx, vy, wz)
            self._last_cmd_vel_time = self.get_clock().now()

        with self._hw_lock:
            self.dog.move_x(step_x)
            self.dog.move_y(step_y)
            self.dog.turn(step_w)

    def _on_body_posture(self, msg: BodyPosture) -> None:
        with self._hw_lock:
            self.dog.translation(
                ['x', 'y', 'z'],
                [msg.translation.x, msg.translation.y, msg.translation.z])
            self.dog.attitude(
                ['r', 'p', 'y'],
                [msg.attitude.x, msg.attitude.y, msg.attitude.z])

    def _on_joint_cmds(self, msg: JointState) -> None:
        if not msg.name or not msg.position:
            return
        ids: List[int] = []
        vals: List[float] = []
        for name, pos in zip(msg.name, msg.position):
            idx = self._joint_index.get(name)
            if idx is None:
                self.get_logger().warn(
                    f'joint_cmds: unknown joint "{name}", ignored.')
                continue
            ids.append(self._motor_ids[idx])
            vals.append(float(pos) * RAD2DEG)
        if not ids:
            return
        with self._hw_lock:
            self.dog.motor(ids, vals)

    def _on_leg_cmd(self, msg: LegPosition) -> None:
        if msg.leg_id < 1 or msg.leg_id > 4:
            self.get_logger().warn(
                f'leg_cmd: leg_id must be 1..4, got {msg.leg_id}.')
            return
        with self._hw_lock:
            self.dog.leg(
                int(msg.leg_id),
                [msg.position.x, msg.position.y, msg.position.z])

    def _on_arm_cmd(self, msg: Point) -> None:
        # Only x and z are used by xgolib.arm(); y is ignored (planar arm).
        with self._hw_lock:
            self.dog.arm(float(msg.x), float(msg.z))

    def _on_claw_cmd(self, msg: UInt8) -> None:
        # 0 = fully open, 255 = fully closed (per official doc).
        with self._hw_lock:
            self.dog.claw(int(msg.data))

    def _on_mark_time(self, msg: UInt8) -> None:
        # 0 = stop, 10..35 mm = leg-raise height for marching in place.
        with self._hw_lock:
            self.dog.mark_time(int(msg.data))

    def _on_motor_speed(self, msg: UInt8) -> None:
        # 1..255 servo rotation speed (only meaningful when driving motors directly).
        with self._hw_lock:
            self.dog.motor_speed(max(1, int(msg.data)))

    def _watchdog_cmd_vel(self) -> None:
        """Stop the base if no cmd_vel has arrived recently."""
        if self._cmd_vel_timeout <= 0.0:
            return
        with self._cmd_vel_lock:
            last_time = self._last_cmd_vel_time
            last_cmd = self._last_cmd_vel
        if last_time is None:
            return
        dt = (self.get_clock().now() - last_time).nanoseconds * 1e-9
        if dt > self._cmd_vel_timeout and last_cmd != (0.0, 0.0, 0.0):
            with self._cmd_vel_lock:
                self._last_cmd_vel = (0.0, 0.0, 0.0)
            with self._hw_lock:
                self.dog.stop()

    # ---------------------------------------------------------------- srvs
    def _srv_set_imu(self, req: SetBool.Request,
                     resp: SetBool.Response) -> SetBool.Response:
        with self._hw_lock:
            self.dog.imu(1 if req.data else 0)
        resp.success = True
        resp.message = f'IMU mode {"on" if req.data else "off"}'
        return resp

    def _srv_set_perform(self, req: SetBool.Request,
                         resp: SetBool.Response) -> SetBool.Response:
        with self._hw_lock:
            self.dog.perform(1 if req.data else 0)
        resp.success = True
        resp.message = f'Perform mode {"on" if req.data else "off"}'
        return resp

    def _srv_set_arm_mode(self, req: SetBool.Request,
                          resp: SetBool.Response) -> SetBool.Response:
        with self._hw_lock:
            self.dog.arm_mode(0x01 if req.data else 0x00)
        resp.success = True
        resp.message = f'Arm stable mode {"on" if req.data else "off"}'
        return resp

    def _srv_set_calibration(self, req: SetBool.Request,
                             resp: SetBool.Response) -> SetBool.Response:
        # WARNING: places the robot in an unloaded calibration pose. See doc.
        state = 'start' if req.data else 'end'
        with self._hw_lock:
            self.dog.calibration(state)
        resp.success = True
        resp.message = f'calibration {state}'
        return resp

    def _srv_set_motor_load(self, req: SetMotorLoad.Request,
                            resp: SetMotorLoad.Response) -> SetMotorLoad.Response:
        leg_id = int(req.leg_id)
        if leg_id < 0 or leg_id > 5:
            resp.success = False
            resp.message = 'leg_id must be 0 (all) or 1..5'
            return resp
        with self._hw_lock:
            if req.load:
                if leg_id == 0:
                    self.dog.load_allmotor()
                else:
                    self.dog.load_motor(leg_id)
            else:
                if leg_id == 0:
                    self.dog.unload_allmotor()
                else:
                    self.dog.unload_motor(leg_id)
        resp.success = True
        resp.message = (f'{"load" if req.load else "unload"} '
                        f'{"all" if leg_id == 0 else f"leg {leg_id}"}')
        return resp

    def _srv_set_gait(self, req: SetGaitType.Request,
                      resp: SetGaitType.Response) -> SetGaitType.Response:
        if req.gait_type not in ('trot', 'walk', 'high_walk'):
            resp.success = False
            resp.message = f'Unknown gait: {req.gait_type}'
            return resp
        with self._hw_lock:
            self.dog.gait_type(req.gait_type)
        resp.success = True
        resp.message = f'gait set to {req.gait_type}'
        return resp

    def _srv_set_pace(self, req: SetPace.Request,
                      resp: SetPace.Response) -> SetPace.Response:
        if req.pace not in ('normal', 'slow', 'high'):
            resp.success = False
            resp.message = f'Unknown pace: {req.pace}'
            return resp
        with self._hw_lock:
            self.dog.pace(req.pace)
        resp.success = True
        resp.message = f'pace set to {req.pace}'
        return resp

    def _srv_reset(self, _req: Trigger.Request,
                   resp: Trigger.Response) -> Trigger.Response:
        with self._hw_lock:
            self.dog.reset()
        resp.success = True
        resp.message = 'reset issued'
        return resp

    # ---------------------------------------------------------------- action
    def _exec_play_action(self, goal_handle):
        req: PlayPreset.Goal = goal_handle.request
        result = PlayPreset.Result()
        action_id = int(req.action_id)
        # Official doc: valid ids are [1, 255]; ids 1..20 are the documented
        # preset behaviours, higher values are firmware-reserved (255 = reset).
        if action_id < 1 or action_id > 255:
            result.success = False
            result.message = 'action_id must be in [1, 255]'
            goal_handle.abort()
            return result
        if action_id > 20:
            self.get_logger().warn(
                f'action_id {action_id} is outside documented range [1,20].')

        duration = float(req.duration) if req.duration > 0.0 \
            else self._default_preset_duration

        with self._hw_lock:
            self.dog.action(action_id)

        feedback = PlayPreset.Feedback()
        rate = max(1.0, self._preset_feedback_rate_hz)
        period = 1.0 / rate
        start = time.monotonic()
        while True:
            elapsed = time.monotonic() - start
            if goal_handle.is_cancel_requested:
                with self._hw_lock:
                    self.dog.reset()
                goal_handle.canceled()
                result.success = False
                result.message = 'canceled by client'
                return result
            if elapsed >= duration:
                break
            feedback.status = 'playing'
            feedback.progress = float(min(1.0, elapsed / duration))
            goal_handle.publish_feedback(feedback)
            time.sleep(period)

        goal_handle.succeed()
        result.success = True
        result.message = 'action completed'
        return result

    # ---------------------------------------------------------- observations
    def _pub_joint_states(self) -> None:
        try:
            with self._hw_lock:
                angles_deg = self.dog.read_motor()
        except Exception as exc:  # pragma: no cover - hardware
            self.get_logger().warn(f'read_motor failed: {exc}')
            return
        if not angles_deg:
            return
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        n = min(len(angles_deg), len(self._joint_names))
        js.name = self._joint_names[:n]
        js.position = [float(a) * DEG2RAD for a in angles_deg[:n]]
        self.pub_joint.publish(js)

    def _pub_imu(self) -> None:
        try:
            with self._hw_lock:
                roll = self.dog.read_roll()
                pitch = self.dog.read_pitch()
                yaw = self.dog.read_yaw()
        except Exception as exc:  # pragma: no cover - hardware
            self.get_logger().warn(f'read imu failed: {exc}')
            return
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._imu_frame_id
        msg.orientation = euler_to_quaternion(
            float(roll) * DEG2RAD,
            float(pitch) * DEG2RAD,
            float(yaw) * DEG2RAD,
        )
        # Coarse orientation covariance; angular vel & linear accel unavailable
        # from xgolib, mark them invalid via -1 on the first diagonal entry.
        msg.orientation_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01,
        ]
        msg.angular_velocity_covariance = [-1.0] + [0.0] * 8
        msg.linear_acceleration_covariance = [-1.0] + [0.0] * 8
        self.pub_imu.publish(msg)

    def _pub_battery(self) -> None:
        try:
            with self._hw_lock:
                level = self.dog.read_battery()
        except Exception as exc:  # pragma: no cover - hardware
            self.get_logger().warn(f'read_battery failed: {exc}')
            return
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.percentage = float(level) / 100.0
        msg.present = True
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        msg.power_supply_technology = \
            BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        self.pub_batt.publish(msg)

    def _pub_odom(self) -> None:
        now = self.get_clock().now()
        if self._last_odom_time is None:
            self._last_odom_time = now
            return
        dt = (now - self._last_odom_time).nanoseconds * 1e-9
        self._last_odom_time = now
        if dt <= 0.0:
            return

        with self._cmd_vel_lock:
            vx, vy, wz = self._last_cmd_vel

        # Simple 2D integration using the last commanded velocity.
        c = math.cos(self._odom_yaw)
        s = math.sin(self._odom_yaw)
        self._odom_x += (vx * c - vy * s) * dt
        self._odom_y += (vx * s + vy * c) * dt
        self._odom_yaw += wz * dt

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self._odom_frame_id
        odom.child_frame_id = self._base_frame_id
        odom.pose.pose.position.x = self._odom_x
        odom.pose.pose.position.y = self._odom_y
        odom.pose.pose.orientation = euler_to_quaternion(
            0.0, 0.0, self._odom_yaw)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        self.pub_odom.publish(odom)

        if self._tf_bc is not None:
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = self._odom_frame_id
            t.child_frame_id = self._base_frame_id
            t.transform.translation.x = self._odom_x
            t.transform.translation.y = self._odom_y
            t.transform.translation.z = 0.0
            t.transform.rotation = odom.pose.pose.orientation
            self._tf_bc.sendTransform(t)

    # ---------------------------------------------------------- lifecycle
    def destroy_node(self) -> bool:
        try:
            with self._hw_lock:
                self.dog.stop()
        except Exception:  # pragma: no cover - hardware
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = XgoDriverNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
