#!/usr/bin/env python3
"""
PX4 Offboard Control Node using MAVSDK-Python and ROS 2.

This node provides ROS 2 interfaces for controlling a PX4 drone via MAVSDK:
- Subscribes to /cmd_vel for velocity commands
- Provides services for arm/disarm, offboard start/stop, and landing
- Publishes telemetry data (position, velocity, state)

Features:
- Robust connection handling (works regardless of startup order)
- Automatic reconnection if connection is lost
- Safe timeout behavior (zero velocity on command timeout)

Coordinate Frame Mapping (ROS -> NED):
- /cmd_vel linear.x  -> North velocity (m/s)
- /cmd_vel linear.y  -> East velocity (m/s)
- /cmd_vel linear.z  -> Up velocity (m/s, converted to -Down internally)
- /cmd_vel angular.z -> Yaw rate (deg/s)

Author: Generated for PX4 + Isaac Sim + Pegasus integration
"""

import asyncio
import math
import threading
import time
from enum import Enum
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist, PoseStamped, TwistStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String, Float32
from std_srvs.srv import SetBool, Trigger

from mavsdk import System
from mavsdk.offboard import (
    OffboardError,
    VelocityNedYaw,
    VelocityBodyYawspeed,
    AccelerationNed,
    AttitudeRate,
    Attitude,
)


class ControlMode(Enum):
    """Control mode for the offboard controller."""
    VELOCITY = "velocity"
    ATTITUDE_RATE = "attitude_rate"
    ATTITUDE = "attitude"


class DroneState(Enum):
    """Enum representing the drone's current state."""
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    ARMED = "armed"
    OFFBOARD = "offboard"
    LANDING = "landing"


class OffboardControlNode(Node):
    """
    ROS 2 Node for PX4 offboard control using MAVSDK.

    This node runs an asyncio event loop in a background thread to handle
    MAVSDK's async API while maintaining ROS 2 compatibility.
    """

    def __init__(self):
        super().__init__('px4_offboard_control')

        # Declare parameters
        self.declare_parameter('drone_id', 0)
        self.declare_parameter('connection_url', 'udpout://127.0.0.1:14280')
        self.declare_parameter('setpoint_rate_hz', 20.0)
        self.declare_parameter('cmd_timeout_sec', 0.5)
        self.declare_parameter('takeoff_altitude_m', 1.5)
        self.declare_parameter('yaw_mode', 'rate')  # 'rate' or 'angle'
        self.declare_parameter('control_mode', 'velocity')  # 'velocity' or 'acceleration'
        self.declare_parameter('reconnect_interval_sec', 3.0)
        self.declare_parameter('hover_thrust', 0.5)  # Normalized thrust for hover (0-1)
        self.declare_parameter('imu_rate_hz', 50.0)  # IMU telemetry stream rate from PX4
        self.declare_parameter('odom_rate_hz', 50.0)  # Odom/telemetry publish rate

        # Get parameters
        self.drone_id = self.get_parameter('drone_id').get_parameter_value().integer_value
        self.connection_url = self.get_parameter('connection_url').get_parameter_value().string_value
        self.setpoint_rate_hz = self.get_parameter('setpoint_rate_hz').get_parameter_value().double_value
        self.cmd_timeout_sec = self.get_parameter('cmd_timeout_sec').get_parameter_value().double_value
        self.takeoff_altitude_m = self.get_parameter('takeoff_altitude_m').get_parameter_value().double_value
        self.yaw_mode = self.get_parameter('yaw_mode').get_parameter_value().string_value
        self.control_mode = self.get_parameter('control_mode').get_parameter_value().string_value
        self.reconnect_interval_sec = self.get_parameter('reconnect_interval_sec').get_parameter_value().double_value
        self.hover_thrust = self.get_parameter('hover_thrust').get_parameter_value().double_value
        self.imu_rate_hz = self.get_parameter('imu_rate_hz').get_parameter_value().double_value
        self.odom_rate_hz = self.get_parameter('odom_rate_hz').get_parameter_value().double_value

        self.get_logger().info(f"Drone ID: {self.drone_id}")
        self.get_logger().info(f"Connection URL: {self.connection_url}")
        self.get_logger().info(f"Setpoint rate: {self.setpoint_rate_hz} Hz")
        self.get_logger().info(f"Command timeout: {self.cmd_timeout_sec} s")
        self.get_logger().info(f"Yaw mode: {self.yaw_mode}")
        self.get_logger().info(f"Control mode: {self.control_mode}")
        self.get_logger().info(f"Hover thrust: {self.hover_thrust} (fallback/initial value)")
        self.get_logger().info(f"Mode switching: Using PX4 thrust setpoint for smooth transitions")

        # State variables (thread-safe access via locks)
        self._lock = threading.Lock()
        self._drone_state = DroneState.DISCONNECTED
        self._is_connected = False
        self._is_ready = False
        self._is_armed = False
        self._is_offboard = False
        self._is_healthy = False
        self._connection_lost = False

        # Commanded velocity (NED frame, yaw rate in deg/s)
        self._cmd_north_m_s = 0.0
        self._cmd_east_m_s = 0.0
        self._cmd_down_m_s = 0.0
        self._cmd_yaw_deg_s = 0.0
        self._cmd_yaw_deg = 0.0  # For angle mode
        self._last_cmd_time = 0.0

        # Commanded acceleration (NED frame)
        self._cmd_accel_north = 0.0
        self._cmd_accel_east = 0.0
        self._cmd_accel_down = 0.0
        self._use_accel_mode = False
        self._last_accel_cmd_time = 0.0

        # Commanded attitude rate (body frame, deg/s) + thrust
        self._cmd_roll_rate_deg_s = 0.0
        self._cmd_pitch_rate_deg_s = 0.0
        self._cmd_yaw_rate_deg_s = 0.0
        self._cmd_thrust = self.hover_thrust  # Normalized 0-1, configured for hover
        self._last_attitude_rate_cmd_time = 0.0

        # Commanded attitude (body frame, degrees) + thrust
        self._cmd_roll_deg = 0.0
        self._cmd_pitch_deg = 0.0
        self._cmd_yaw_deg_attitude = 0.0
        self._cmd_attitude_thrust = self.hover_thrust
        self._last_attitude_cmd_time = 0.0

        # Control mode (velocity, attitude_rate, or attitude)
        self._control_mode = ControlMode.VELOCITY

        # Telemetry data
        self._position_ned = (0.0, 0.0, 0.0)  # North, East, Down
        self._velocity_ned = (0.0, 0.0, 0.0)
        self._attitude_euler = (0.0, 0.0, 0.0)  # Roll, Pitch, Yaw (degrees)
        self._position_global = (0.0, 0.0, 0.0)  # Lat, Lon, Alt
        self._px4_thrust_setpoint = self.hover_thrust  # Current thrust setpoint from PX4
        self._imu_accel_flu = [0.0, 0.0, 9.81]  # Cached linear acceleration (FLU, m/s^2)
        self._angular_velocity_frd = (0.0, 0.0, 0.0)  # Cached angular velocity (roll, pitch, yaw) rad/s

        # MAVSDK system
        self._drone: Optional[System] = None
        self._asyncio_loop: Optional[asyncio.AbstractEventLoop] = None
        self._asyncio_thread: Optional[threading.Thread] = None
        self._running = True
        self._telemetry_tasks = []

        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers (relative names so they respect namespace)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self._cmd_vel_callback,
            10
        )

        # Optional acceleration command subscriber
        self.cmd_accel_sub = self.create_subscription(
            Vector3Stamped,
            'cmd_accel',
            self._cmd_accel_callback,
            10
        )

        # Attitude rate command subscriber (for manipulation mode)
        # Twist mapping: angular.x=roll_rate, angular.y=pitch_rate, angular.z=yaw_rate (deg/s)
        #                linear.z=thrust (normalized 0-1)
        self.cmd_attitude_rate_sub = self.create_subscription(
            Twist,
            'cmd_attitude_rate',
            self._cmd_attitude_rate_callback,
            10
        )

        # Attitude command subscriber (for position controller output)
        # Twist mapping: angular.x=roll_deg, angular.y=pitch_deg, angular.z=yaw_deg
        #                linear.z=thrust (normalized 0-1)
        self.cmd_attitude_sub = self.create_subscription(
            Twist,
            'cmd_attitude',
            self._cmd_attitude_callback,
            10
        )

        # Publishers (relative names so they respect namespace)
        self.position_pub = self.create_publisher(PoseStamped, 'telemetry/position', 10)
        self.velocity_pub = self.create_publisher(TwistStamped, 'telemetry/velocity', 10)
        self.odom_pub = self.create_publisher(Odometry, 'telemetry/odom', 10)
        self.state_pub = self.create_publisher(String, 'telemetry/state', 10)
        self.control_mode_pub = self.create_publisher(String, 'control_mode', 10)
        self.thrust_setpoint_pub = self.create_publisher(Float32, 'telemetry/thrust_setpoint', 10)
        self.imu_pub = self.create_publisher(Imu, 'telemetry/imu', 10)

        # Services (relative names so they respect namespace)
        self.arm_srv = self.create_service(SetBool, 'arm', self._arm_callback)
        self.offboard_start_srv = self.create_service(Trigger, 'offboard_start', self._offboard_start_callback)
        self.offboard_stop_srv = self.create_service(Trigger, 'offboard_stop', self._offboard_stop_callback)
        self.land_srv = self.create_service(Trigger, 'land', self._land_callback)
        # Mode switch service: true=attitude_rate mode, false=velocity mode
        self.set_control_mode_srv = self.create_service(
            SetBool, 'set_control_mode', self._set_control_mode_callback
        )
        # Attitude mode switch service
        self.set_attitude_mode_srv = self.create_service(
            Trigger, 'set_attitude_mode', self._set_attitude_mode_callback
        )

        # Timer for publishing telemetry
        odom_period = 1.0 / self.odom_rate_hz
        self.telemetry_timer = self.create_timer(odom_period, self._publish_telemetry)

        # High-rate timer for IMU publishing (decoupled from MAVSDK stream rate)
        imu_period = 1.0 / self.imu_rate_hz
        self.imu_timer = self.create_timer(imu_period, self._publish_imu)

        # Start asyncio event loop in background thread
        self._start_asyncio_loop()

        self.get_logger().info("PX4 Offboard Control Node initialized")
        self.get_logger().info("Control modes: VELOCITY (default), ATTITUDE_RATE, ATTITUDE")
        self.get_logger().info("Velocity mode: cmd_vel.linear.x->North, .y->East, .z->Up; angular.z->yaw rate")
        self.get_logger().info("Attitude rate mode: cmd_attitude_rate.angular.x/y/z->roll/pitch/yaw rates; linear.z->thrust")
        self.get_logger().info("Attitude mode: cmd_attitude.angular.x/y/z->roll/pitch/yaw deg; linear.z->thrust")
        self.get_logger().info("Use /set_control_mode (true=attitude_rate, false=velocity) or /set_attitude_mode")

    def _start_asyncio_loop(self):
        """Start the asyncio event loop in a background thread."""
        self._asyncio_loop = asyncio.new_event_loop()
        self._asyncio_thread = threading.Thread(target=self._run_asyncio_loop, daemon=True)
        self._asyncio_thread.start()

    def _run_asyncio_loop(self):
        """Run the asyncio event loop."""
        asyncio.set_event_loop(self._asyncio_loop)
        self._asyncio_loop.run_until_complete(self._main_async())

    def _reset_state(self):
        """Reset all state variables on disconnection."""
        with self._lock:
            self._drone_state = DroneState.DISCONNECTED
            self._is_connected = False
            self._is_ready = False
            self._is_armed = False
            self._is_offboard = False
            self._is_healthy = False

        # Cleanup the old drone object to release resources
        if self._drone is not None:
            try:
                # Force garbage collection of the System object
                del self._drone
            except Exception:
                pass
            self._drone = None

        # Force garbage collection to clean up mavsdk_server
        import gc
        gc.collect()

    async def _main_async(self):
        """Main async function with connection management and auto-reconnect."""
        # Create System object with unique gRPC port based on drone_id
        # This prevents multiple mavsdk_server instances from conflicting
        grpc_port = 50051 + self.drone_id
        self.get_logger().info(f"Using MAVSDK gRPC port: {grpc_port}")
        self._drone = System(port=grpc_port)

        while self._running:
            try:
                await self._connect_and_run()
            except Exception as e:
                error_str = str(e)
                error_type = type(e).__name__
                if "grpc" in error_str.lower() or "socket" in error_str.lower() or "rpc" in error_type.lower():
                    self.get_logger().warn(f"Connection issue (will retry): {error_type}: {e}")
                else:
                    self.get_logger().error(f"Connection error ({error_type}): {e}")

            if self._running:
                self._reset_state()
                self.get_logger().info(f"Reconnecting in {self.reconnect_interval_sec} seconds...")
                await asyncio.sleep(self.reconnect_interval_sec)

                # Create fresh System object for reconnect (with same unique gRPC port)
                grpc_port = 50051 + self.drone_id
                self._drone = System(port=grpc_port)

    async def _connect_and_run(self):
        """Connect to the drone and run telemetry tasks."""
        with self._lock:
            self._drone_state = DroneState.CONNECTING

        self.get_logger().info(f"Connecting to drone at {self.connection_url}...")

        # Connect with timeout to prevent indefinite blocking
        try:
            await asyncio.wait_for(
                self._drone.connect(system_address=self.connection_url),
                timeout=10.0
            )
        except asyncio.TimeoutError:
            self.get_logger().warn("Initial connect() timed out, continuing anyway...")

        # Wait for connection by polling for telemetry data
        # This is more reliable than connection_state() which may not emit if already connected
        self.get_logger().info("Waiting for drone connection (PX4 can be started before or after this node)...")

        connected = False
        max_attempts = 60  # 60 seconds total
        attempt = 0

        while not connected and attempt < max_attempts and self._running:
            attempt += 1

            # Try multiple methods to establish connection
            try:
                # Method 1: Try to get telemetry (this also triggers MAVSDK to send heartbeats)
                async def get_first_telemetry():
                    async for _ in self._drone.telemetry.attitude_euler():
                        return True
                    return False

                connected = await asyncio.wait_for(get_first_telemetry(), timeout=1.0)

            except asyncio.TimeoutError:
                # Method 2: Try connection_state() as backup
                try:
                    async def check_connection_state():
                        async for state in self._drone.core.connection_state():
                            return state.is_connected
                        return False

                    is_conn = await asyncio.wait_for(check_connection_state(), timeout=0.5)
                    if is_conn:
                        connected = True
                except asyncio.TimeoutError:
                    pass

                if not connected and attempt % 3 == 0:  # Log every 3 seconds
                    self.get_logger().info(f"Waiting for drone... ({attempt}s)")

            except Exception as e:
                # During initial connection, telemetry plugin may not be ready yet
                # This is expected - just log at debug level and retry
                error_str = str(e)
                if "not been initialized" in error_str or "Did you run" in error_str:
                    # Expected during connection setup - only log at debug level
                    if attempt == 1 or attempt % 10 == 0:  # Log occasionally
                        self.get_logger().debug(f"Waiting for MAVSDK telemetry to initialize... ({attempt}s)")
                else:
                    # Unexpected error - log as warning
                    self.get_logger().warn(f"Connection probe error: {type(e).__name__}: {e}")

                import traceback
                self.get_logger().debug(traceback.format_exc())
                await asyncio.sleep(0.5)

        if connected:
            self.get_logger().info("Drone connected!")
            with self._lock:
                self._drone_state = DroneState.CONNECTED
                self._is_connected = True
            # Allow connection to stabilize
            await asyncio.sleep(0.5)
        else:
            raise Exception("Connection timeout - no response from drone")

        # Start monitoring tasks one at a time with small delays to avoid overwhelming MAVSDK
        self._telemetry_tasks = []

        # Health monitor first (needed for ready check)
        self._telemetry_tasks.append(asyncio.create_task(self._monitor_health()))
        await asyncio.sleep(0.1)

        # Position and attitude
        self._telemetry_tasks.append(asyncio.create_task(self._monitor_position()))
        self._telemetry_tasks.append(asyncio.create_task(self._monitor_attitude()))
        await asyncio.sleep(0.1)

        # Actuator control target (for reading PX4's thrust setpoint)
        self._telemetry_tasks.append(asyncio.create_task(self._monitor_actuator_control()))
        await asyncio.sleep(0.1)

        # IMU: high-rate angular velocity + cached linear acceleration
        self._telemetry_tasks.append(asyncio.create_task(self._monitor_imu()))
        self._telemetry_tasks.append(asyncio.create_task(self._monitor_imu_accel()))
        await asyncio.sleep(0.1)

        # Flight mode and armed state
        self._telemetry_tasks.append(asyncio.create_task(self._monitor_armed_state()))
        self._telemetry_tasks.append(asyncio.create_task(self._monitor_flight_mode()))
        await asyncio.sleep(0.1)

        # Setpoint loop
        self._telemetry_tasks.append(asyncio.create_task(self._setpoint_loop()))

        # Connection monitor (starts after delay internally)
        self._telemetry_tasks.append(asyncio.create_task(self._monitor_connection()))

        # Wait for drone to be ready (with tolerance for brief connection hiccups)
        self.get_logger().info("Waiting for drone to be ready...")
        ready_timeout = 30.0
        start_time = time.time()
        disconnect_grace_count = 0
        max_disconnect_grace = 10  # Allow up to 1 second of disconnect states

        while self._running and (time.time() - start_time) < ready_timeout:
            with self._lock:
                is_healthy = self._is_healthy
                is_connected = self._is_connected

            if not is_connected:
                disconnect_grace_count += 1
                if disconnect_grace_count > max_disconnect_grace:
                    raise Exception("Connection lost while waiting for ready state")
            else:
                disconnect_grace_count = 0

            if is_healthy and is_connected:
                break

            await asyncio.sleep(0.1)

        with self._lock:
            if self._is_healthy:
                self._is_ready = True
                self.get_logger().info("Drone is ready! Use /arm and /offboard_start services.")
            else:
                self.get_logger().warn("Drone health check timeout - proceeding anyway (position may not be valid)")
                self._is_ready = True

        # Wait for tasks to complete (they run until connection lost or shutdown)
        try:
            # Use wait with FIRST_EXCEPTION to catch any task failure
            done, pending = await asyncio.wait(
                self._telemetry_tasks,
                return_when=asyncio.FIRST_EXCEPTION
            )

            # Check for exceptions
            for task in done:
                exc = task.exception()
                if exc:
                    # Log the specific task that failed
                    self.get_logger().debug(f"Task failed: {exc}")
                    raise exc

        except asyncio.CancelledError:
            self.get_logger().info("Tasks cancelled")
        except Exception as e:
            # Check if it's a gRPC error (common with MAVSDK)
            error_str = str(e)
            if "grpc" in error_str.lower() or "socket" in error_str.lower():
                self.get_logger().warn(f"gRPC/Socket error detected, will reconnect: {type(e).__name__}")
            else:
                self.get_logger().error(f"Task error: {e}")
            raise
        finally:
            # Cancel remaining tasks
            for task in self._telemetry_tasks:
                if not task.done():
                    task.cancel()
                    try:
                        await task
                    except asyncio.CancelledError:
                        pass
            # Small delay to allow cleanup
            await asyncio.sleep(0.5)

    async def _monitor_connection(self):
        """Monitor connection state and trigger reconnect on loss."""
        # Wait a bit for connection to stabilize before monitoring
        await asyncio.sleep(2.0)

        # Track consecutive disconnected states to avoid false positives
        disconnect_count = 0
        disconnect_threshold = 3  # Require 3 consecutive disconnects

        async for state in self._drone.core.connection_state():
            if not state.is_connected:
                disconnect_count += 1
                if disconnect_count >= disconnect_threshold:
                    self.get_logger().warn("Connection to drone lost!")
                    with self._lock:
                        self._is_connected = False
                        self._is_ready = False
                        self._drone_state = DroneState.DISCONNECTED
                    raise Exception("Connection lost")
            else:
                disconnect_count = 0  # Reset on successful connection state

    async def _monitor_armed_state(self):
        """Monitor armed state from telemetry."""
        try:
            async for is_armed in self._drone.telemetry.armed():
                with self._lock:
                    self._is_armed = is_armed
                    if is_armed:
                        if self._drone_state in [DroneState.CONNECTED, DroneState.CONNECTING]:
                            self._drone_state = DroneState.ARMED
                    else:
                        if self._drone_state == DroneState.ARMED:
                            self._drone_state = DroneState.CONNECTED
        except Exception as e:
            self.get_logger().debug(f"Armed state monitor ended: {e}")
            raise

    async def _monitor_flight_mode(self):
        """Monitor flight mode from telemetry."""
        try:
            async for flight_mode in self._drone.telemetry.flight_mode():
                mode_str = str(flight_mode)
                with self._lock:
                    if "OFFBOARD" in mode_str.upper():
                        self._is_offboard = True
                        if self._is_armed:
                            self._drone_state = DroneState.OFFBOARD
                    else:
                        self._is_offboard = False
                        if self._is_armed and self._drone_state == DroneState.OFFBOARD:
                            self._drone_state = DroneState.ARMED
        except Exception as e:
            self.get_logger().debug(f"Flight mode monitor ended: {e}")
            raise

    async def _monitor_position(self):
        """Monitor position and velocity from telemetry."""
        try:
            async for position in self._drone.telemetry.position_velocity_ned():
                with self._lock:
                    self._position_ned = (
                        position.position.north_m,
                        position.position.east_m,
                        position.position.down_m
                    )
                    self._velocity_ned = (
                        position.velocity.north_m_s,
                        position.velocity.east_m_s,
                        position.velocity.down_m_s
                    )
        except Exception as e:
            self.get_logger().debug(f"Position monitor ended: {e}")
            raise

    async def _monitor_attitude(self):
        """Monitor attitude from telemetry."""
        try:
            async for attitude in self._drone.telemetry.attitude_euler():
                with self._lock:
                    self._attitude_euler = (
                        attitude.roll_deg,
                        attitude.pitch_deg,
                        attitude.yaw_deg
                    )
        except Exception as e:
            self.get_logger().debug(f"Attitude monitor ended: {e}")
            raise

    async def _monitor_health(self):
        """Monitor health status."""
        try:
            async for health in self._drone.telemetry.health():
                with self._lock:
                    self._is_healthy = (
                        health.is_local_position_ok or
                        (health.is_global_position_ok and health.is_home_position_ok)
                    )
        except Exception as e:
            self.get_logger().debug(f"Health monitor ended: {e}")
            raise

    async def _monitor_actuator_control(self):
        """
        Monitor actuator control target to read PX4's current thrust setpoint.

        This gives us the actual thrust being commanded by PX4's internal controllers,
        which is more reliable than estimation for mode switching.
        """
        try:
            async for control in self._drone.telemetry.actuator_control_target():
                # control.group[0].controls[3] is the thrust control (index 3 = throttle/thrust)
                # It's normalized to [-1, 1] range, we need [0, 1]
                if len(control.group) > 0 and len(control.group[0].controls) > 3:
                    raw_thrust = control.group[0].controls[3]
                    # Convert from [-1, 1] to [0, 1] range
                    # PX4 uses -1 to 1, where 0 is mid-throttle
                    normalized_thrust = (raw_thrust + 1.0) / 2.0

                    with self._lock:
                        # Only update if we're in velocity mode (in attitude/attitude_rate mode, we control thrust directly)
                        if self._control_mode not in (ControlMode.ATTITUDE_RATE, ControlMode.ATTITUDE) and self._is_offboard:
                            self._px4_thrust_setpoint = max(0.0, min(1.0, normalized_thrust))
        except Exception as e:
            self.get_logger().debug(f"Actuator control monitor ended: {e}")
            raise

    async def _monitor_imu(self):
        """
        Cache angular velocity from the ATTITUDE_QUATERNION MAVLink stream.

        The actual IMU publishing is done by the _publish_imu timer at
        imu_rate_hz, decoupled from MAVSDK's stream rate.
        """
        try:
            try:
                await self._drone.telemetry.set_rate_attitude_quaternion(
                    self.imu_rate_hz)
            except Exception:
                pass  # best effort

            async for ang_vel in (
                    self._drone.telemetry.attitude_angular_velocity_body()):
                with self._lock:
                    self._angular_velocity_frd = (
                        ang_vel.roll_rad_s,
                        ang_vel.pitch_rad_s,
                        ang_vel.yaw_rad_s,
                    )
        except Exception as e:
            self.get_logger().debug(f"Angular velocity monitor ended: {e}")
            raise

    async def _monitor_imu_accel(self):
        """
        Cache linear acceleration from the HIGHRES_IMU stream (~3-4 Hz).

        This runs as a background task; the main _monitor_imu reads
        the cached values at the higher attitude stream rate.
        """
        try:
            try:
                await self._drone.telemetry.set_rate_imu(self.imu_rate_hz)
            except Exception:
                pass  # best effort — PX4 SITL may not honor this

            async for imu_data in self._drone.telemetry.imu():
                with self._lock:
                    self._imu_accel_flu = [
                        imu_data.acceleration_frd.forward_m_s2,
                        -imu_data.acceleration_frd.right_m_s2,
                        -imu_data.acceleration_frd.down_m_s2,
                    ]
        except Exception as e:
            self.get_logger().debug(f"IMU accel monitor ended: {e}")
            raise

    async def _setpoint_loop(self):
        """
        Continuously send setpoints at the configured rate.
        This is required to maintain offboard mode.

        Supports two control modes:
        - VELOCITY: Standard velocity control (for flying)
        - ATTITUDE_RATE: Direct attitude rate + thrust control (for manipulation)
        """
        period = 1.0 / self.setpoint_rate_hz

        while self._running:
            try:
                with self._lock:
                    is_offboard = self._is_offboard
                    is_connected = self._is_connected
                    current_time = time.time()
                    control_mode = self._control_mode

                    # Check for command timeouts
                    vel_timeout = (current_time - self._last_cmd_time) > self.cmd_timeout_sec
                    accel_timeout = (current_time - self._last_accel_cmd_time) > self.cmd_timeout_sec
                    attitude_rate_timeout = (current_time - self._last_attitude_rate_cmd_time) > self.cmd_timeout_sec

                    if control_mode == ControlMode.ATTITUDE:
                        # Attitude angle control mode
                        attitude_timeout = (current_time - self._last_attitude_cmd_time) > self.cmd_timeout_sec
                        if not attitude_timeout:
                            att_roll_deg = self._cmd_roll_deg
                            att_pitch_deg = self._cmd_pitch_deg
                            att_yaw_deg = self._cmd_yaw_deg_attitude
                            att_thrust = self._cmd_attitude_thrust
                        else:
                            # Timeout - command level attitude and maintain hover thrust
                            att_roll_deg, att_pitch_deg = 0.0, 0.0
                            att_yaw_deg = self._attitude_euler[2]  # Hold current yaw
                            att_thrust = self.hover_thrust
                    elif control_mode == ControlMode.ATTITUDE_RATE:
                        # Attitude rate control mode
                        if not attitude_rate_timeout:
                            roll_rate = self._cmd_roll_rate_deg_s
                            pitch_rate = self._cmd_pitch_rate_deg_s
                            yaw_rate = self._cmd_yaw_rate_deg_s
                            thrust = self._cmd_thrust
                        else:
                            # Timeout - command zero rates and maintain hover thrust
                            roll_rate, pitch_rate, yaw_rate = 0.0, 0.0, 0.0
                            thrust = self.hover_thrust
                    else:
                        # Velocity control mode
                        if self._use_accel_mode and not accel_timeout:
                            # Use acceleration mode
                            accel_north = self._cmd_accel_north
                            accel_east = self._cmd_accel_east
                            accel_down = self._cmd_accel_down
                            use_accel = True
                        elif not vel_timeout:
                            # Use velocity mode
                            north = self._cmd_north_m_s
                            east = self._cmd_east_m_s
                            down = self._cmd_down_m_s
                            yaw_rate = self._cmd_yaw_deg_s
                            yaw_deg = self._cmd_yaw_deg
                            use_accel = False
                        else:
                            # Timeout - command zero velocity for safety
                            north, east, down = 0.0, 0.0, 0.0
                            yaw_rate = 0.0
                            yaw_deg = self._attitude_euler[2]  # Hold current yaw
                            use_accel = False

                    yaw_mode = self.yaw_mode

                if not is_connected:
                    await asyncio.sleep(period)
                    continue

                if is_offboard:
                    if control_mode == ControlMode.ATTITUDE:
                        # Send attitude angle setpoint (for position controller)
                        await self._drone.offboard.set_attitude(
                            Attitude(att_roll_deg, att_pitch_deg, att_yaw_deg, att_thrust)
                        )
                    elif control_mode == ControlMode.ATTITUDE_RATE:
                        # Send attitude rate setpoint (for manipulation mode)
                        await self._drone.offboard.set_attitude_rate(
                            AttitudeRate(roll_rate, pitch_rate, yaw_rate, thrust)
                        )
                    elif use_accel:
                        # Send acceleration setpoint
                        try:
                            await self._drone.offboard.set_acceleration_ned(
                                AccelerationNed(accel_north, accel_east, accel_down)
                            )
                        except Exception as e:
                            self.get_logger().warn(f"Acceleration setpoint failed: {e}")
                            # Fall back to velocity mode
                            await self._drone.offboard.set_velocity_ned(
                                VelocityNedYaw(0.0, 0.0, 0.0, yaw_deg)
                            )
                    elif yaw_mode == 'rate':
                        # Use yaw rate mode (body frame velocity)
                        await self._drone.offboard.set_velocity_body(
                            VelocityBodyYawspeed(north, east, down, yaw_rate)
                        )
                    else:
                        # Use yaw angle mode (NED frame velocity)
                        await self._drone.offboard.set_velocity_ned(
                            VelocityNedYaw(north, east, down, yaw_deg)
                        )

                await asyncio.sleep(period)

            except asyncio.CancelledError:
                raise
            except Exception as e:
                self.get_logger().error(f"Setpoint loop error: {e}")
                await asyncio.sleep(period)

    def _cmd_vel_callback(self, msg: Twist):
        """
        Handle incoming velocity commands from /cmd_vel.

        Mapping (ROS convention to NED):
        - linear.x  -> North velocity (m/s)
        - linear.y  -> East velocity (m/s)
        - linear.z  -> Up velocity, negated to Down (m/s)
        - angular.z -> Yaw rate (deg/s)
        """
        with self._lock:
            # Auto-switch to VELOCITY mode when receiving velocity commands
            if self._control_mode != ControlMode.VELOCITY:
                self._control_mode = ControlMode.VELOCITY
                self.get_logger().info(
                    "Auto-switched to VELOCITY mode (received cmd_vel)")

            self._cmd_north_m_s = msg.linear.x
            self._cmd_east_m_s = msg.linear.y
            self._cmd_down_m_s = -msg.linear.z  # Up is positive in ROS, Down is positive in NED
            self._cmd_yaw_deg_s = msg.angular.z  # Yaw rate in deg/s

            # For angle mode, integrate yaw rate (simplified)
            self._cmd_yaw_deg += msg.angular.z * (1.0 / self.setpoint_rate_hz)
            self._cmd_yaw_deg = self._cmd_yaw_deg % 360.0

            self._last_cmd_time = time.time()
            self._use_accel_mode = False

    def _cmd_accel_callback(self, msg: Vector3Stamped):
        """
        Handle incoming acceleration commands from /cmd_accel.

        Mapping (ROS convention to NED):
        - vector.x -> North acceleration (m/s^2)
        - vector.y -> East acceleration (m/s^2)
        - vector.z -> Up acceleration, negated to Down (m/s^2)
        """
        with self._lock:
            self._cmd_accel_north = msg.vector.x
            self._cmd_accel_east = msg.vector.y
            self._cmd_accel_down = -msg.vector.z  # Up is positive in ROS, Down is positive in NED
            self._last_accel_cmd_time = time.time()
            self._use_accel_mode = True

    def _cmd_attitude_rate_callback(self, msg: Twist):
        """
        Handle incoming attitude rate commands from /cmd_attitude_rate.

        Used for manipulation mode where direct attitude rate control is needed.

        Mapping:
        - angular.x -> Roll rate (deg/s, positive = right wing down)
        - angular.y -> Pitch rate (deg/s, positive = nose up)
        - angular.z -> Yaw rate (deg/s, positive = nose right)
        - linear.z  -> Thrust (normalized 0-1, ~0.5 for hover)

        Note: angular rates follow body frame convention.
        """
        with self._lock:
            # Auto-switch to ATTITUDE_RATE mode when receiving rate commands
            if self._control_mode != ControlMode.ATTITUDE_RATE:
                self._control_mode = ControlMode.ATTITUDE_RATE
                self.get_logger().info(
                    "Auto-switched to ATTITUDE_RATE mode (received cmd_attitude_rate)")

            self._cmd_roll_rate_deg_s = msg.angular.x
            self._cmd_pitch_rate_deg_s = msg.angular.y
            self._cmd_yaw_rate_deg_s = msg.angular.z
            # Thrust: clamp to 0-1 range
            self._cmd_thrust = max(0.0, min(1.0, msg.linear.z))
            self._last_attitude_rate_cmd_time = time.time()

    def _cmd_attitude_callback(self, msg: Twist):
        """
        Handle incoming attitude angle commands from /cmd_attitude.

        Used for position controller output where attitude angles are computed
        externally and PX4's inner attitude controller handles rate control.

        Mapping:
        - angular.x -> Roll (deg, positive = right wing down)
        - angular.y -> Pitch (deg, positive = nose up)
        - angular.z -> Yaw (deg, positive = nose right)
        - linear.z  -> Thrust (normalized 0-1, ~0.5 for hover)
        """
        with self._lock:
            # Auto-switch to ATTITUDE mode when receiving attitude commands
            if self._control_mode != ControlMode.ATTITUDE:
                self._control_mode = ControlMode.ATTITUDE
                self.get_logger().info(
                    "Auto-switched to ATTITUDE mode (received cmd_attitude)")

            self._cmd_roll_deg = msg.angular.x
            self._cmd_pitch_deg = msg.angular.y
            self._cmd_yaw_deg_attitude = msg.angular.z
            self._cmd_attitude_thrust = max(0.0, min(1.0, msg.linear.z))
            self._last_attitude_cmd_time = time.time()

    def _publish_telemetry(self):
        """Publish telemetry data to ROS 2 topics."""
        with self._lock:
            pos = self._position_ned
            vel = self._velocity_ned
            att = self._attitude_euler
            state = self._drone_state
            is_armed = self._is_armed
            is_offboard = self._is_offboard
            is_connected = self._is_connected
            is_ready = self._is_ready
            control_mode = self._control_mode

        now = self.get_clock().now().to_msg()

        # Publish control mode
        mode_msg = String()
        mode_msg.data = control_mode.value
        self.control_mode_pub.publish(mode_msg)

        # Publish PX4 thrust setpoint (only when connected)
        if is_connected:
            with self._lock:
                px4_thrust = self._px4_thrust_setpoint
            thrust_msg = Float32()
            thrust_msg.data = px4_thrust
            self.thrust_setpoint_pub.publish(thrust_msg)

        # Publish state (always, even when disconnected)
        state_msg = String()
        state_str = state.value
        if is_connected:
            if is_armed:
                state_str += "/armed"
            else:
                state_str += "/disarmed"
            if is_offboard:
                state_str += "/offboard"
            if is_ready:
                state_str += "/ready"
        state_msg.data = state_str
        self.state_pub.publish(state_msg)

        # Skip other telemetry if not connected
        if not is_connected:
            return

        # Publish position as PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = "map"
        # PX4 NED: pos[0]=North, pos[1]=East, pos[2]=Down
        # Isaac Sim world frame: X=East, Y=North, Z=Up
        pose_msg.pose.position.x = pos[1]   # East  -> sim X
        pose_msg.pose.position.y = pos[0]   # North -> sim Y
        pose_msg.pose.position.z = -pos[2]  # Down  -> Up (sim Z)

        # Full quaternion from Euler angles (roll, pitch, yaw) - ZYX convention
        roll_rad = math.radians(att[0])
        pitch_rad = math.radians(att[1])
        # Convert PX4 yaw (clockwise from North, NED) to ENU/ROS yaw (CCW from East)
        yaw_rad = math.radians(90.0 - att[2])

        cr = math.cos(roll_rad / 2)
        sr = math.sin(roll_rad / 2)
        cp = math.cos(pitch_rad / 2)
        sp = math.sin(pitch_rad / 2)
        cy = math.cos(yaw_rad / 2)
        sy = math.sin(yaw_rad / 2)

        pose_msg.pose.orientation.w = cr * cp * cy + sr * sp * sy
        pose_msg.pose.orientation.x = sr * cp * cy - cr * sp * sy
        pose_msg.pose.orientation.y = cr * sp * cy + sr * cp * sy
        pose_msg.pose.orientation.z = cr * cp * sy - sr * sp * cy
        self.position_pub.publish(pose_msg)

        # Publish velocity as TwistStamped
        twist_msg = TwistStamped()
        twist_msg.header.stamp = now
        twist_msg.header.frame_id = "base_link"
        # PX4 NED: vel[0]=North, vel[1]=East, vel[2]=Down
        # Isaac Sim world frame: X=East, Y=North, Z=Up
        twist_msg.twist.linear.x = vel[1]   # East  -> sim X
        twist_msg.twist.linear.y = vel[0]   # North -> sim Y
        twist_msg.twist.linear.z = -vel[2]  # Down  -> Up (sim Z)
        self.velocity_pub.publish(twist_msg)

        # Publish combined odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = "map"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose = pose_msg.pose
        odom_msg.twist.twist = twist_msg.twist
        self.odom_pub.publish(odom_msg)

    def _publish_imu(self):
        """Publish IMU data at imu_rate_hz from cached telemetry."""
        with self._lock:
            if not self._is_connected:
                return
            att = self._attitude_euler
            ang_vel = self._angular_velocity_frd
            accel = list(self._imu_accel_flu)

        imu_msg = Imu()
        # Use ROS clock (will use /clock topic if use_sim_time=True)
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "base_link"

        # Orientation: NED euler -> ENU quaternion
        roll_rad = math.radians(att[0])
        pitch_rad = math.radians(att[1])
        yaw_rad = math.radians(90.0 - att[2])

        cr = math.cos(roll_rad / 2)
        sr = math.sin(roll_rad / 2)
        cp = math.cos(pitch_rad / 2)
        sp = math.sin(pitch_rad / 2)
        cy = math.cos(yaw_rad / 2)
        sy = math.sin(yaw_rad / 2)

        imu_msg.orientation.w = cr * cp * cy + sr * sp * sy
        imu_msg.orientation.x = sr * cp * cy - cr * sp * sy
        imu_msg.orientation.y = cr * sp * cy + sr * cp * sy
        imu_msg.orientation.z = cr * cp * sy - sr * sp * cy

        # Angular velocity: FRD -> FLU
        imu_msg.angular_velocity.x = ang_vel[0]
        imu_msg.angular_velocity.y = -ang_vel[1]
        imu_msg.angular_velocity.z = -ang_vel[2]

        # Linear acceleration (already FLU from _monitor_imu_accel)
        imu_msg.linear_acceleration.x = accel[0]
        imu_msg.linear_acceleration.y = accel[1]
        imu_msg.linear_acceleration.z = accel[2]

        self.imu_pub.publish(imu_msg)

    def _check_ready(self) -> tuple:
        """Check if drone is connected and ready. Returns (is_ready, error_message)."""
        with self._lock:
            is_connected = self._is_connected
            is_ready = self._is_ready

        if not is_connected:
            return False, "Drone not connected. Wait for 'Drone connected!' message."
        if not is_ready:
            return False, "Drone not ready. Wait for 'Drone is ready!' message."
        return True, ""

    def _arm_callback(self, request: SetBool.Request, response: SetBool.Response):
        """Handle arm/disarm service request."""
        ready, error = self._check_ready()
        if not ready:
            response.success = False
            response.message = f"Cannot {'arm' if request.data else 'disarm'}: {error}"
            self.get_logger().error(response.message)
            return response

        if request.data:
            future = asyncio.run_coroutine_threadsafe(
                self._arm_async(),
                self._asyncio_loop
            )
        else:
            future = asyncio.run_coroutine_threadsafe(
                self._disarm_async(),
                self._asyncio_loop
            )

        try:
            success, message = future.result(timeout=10.0)
            response.success = success
            response.message = message
        except Exception as e:
            response.success = False
            response.message = f"{'Arm' if request.data else 'Disarm'} failed: {e}"
            self.get_logger().error(response.message)

        return response

    async def _arm_async(self):
        """Async arm operation."""
        try:
            self.get_logger().info("Arming...")
            await self._drone.action.arm()
            self.get_logger().info("Armed successfully")
            return True, "Armed successfully"
        except Exception as e:
            self.get_logger().error(f"Arm failed: {e}")
            return False, str(e)

    async def _disarm_async(self):
        """Async disarm operation."""
        try:
            self.get_logger().info("Disarming...")
            await self._drone.action.disarm()
            self.get_logger().info("Disarmed successfully")
            return True, "Disarmed successfully"
        except Exception as e:
            self.get_logger().error(f"Disarm failed: {e}")
            return False, str(e)

    def _offboard_start_callback(self, request: Trigger.Request, response: Trigger.Response):
        """Handle offboard start service request."""
        ready, error = self._check_ready()
        if not ready:
            response.success = False
            response.message = f"Cannot start offboard: {error}"
            self.get_logger().error(response.message)
            return response

        future = asyncio.run_coroutine_threadsafe(
            self._offboard_start_async(),
            self._asyncio_loop
        )
        try:
            success, message = future.result(timeout=10.0)
            response.success = success
            response.message = message
        except Exception as e:
            response.success = False
            response.message = f"Offboard start failed: {e}"
            self.get_logger().error(response.message)

        return response

    async def _offboard_start_async(self):
        """
        Async offboard start operation.

        IMPORTANT: Must send initial setpoint before starting offboard mode.
        """
        try:
            self.get_logger().info("Starting offboard mode...")

            # First, send initial setpoint (required by PX4)
            self.get_logger().info("Sending initial setpoints...")

            with self._lock:
                yaw_deg = self._attitude_euler[2]
                control_mode = self._control_mode

            # Send setpoints for ~0.5 seconds to ensure PX4 receives them
            for _ in range(10):
                if control_mode == ControlMode.ATTITUDE:
                    await self._drone.offboard.set_attitude(
                        Attitude(0.0, 0.0, yaw_deg, self.hover_thrust)
                    )
                elif control_mode == ControlMode.ATTITUDE_RATE:
                    await self._drone.offboard.set_attitude_rate(
                        AttitudeRate(0.0, 0.0, 0.0, self.hover_thrust)
                    )
                else:
                    await self._drone.offboard.set_velocity_ned(
                        VelocityNedYaw(0.0, 0.0, 0.0, yaw_deg)
                    )
                await asyncio.sleep(0.05)

            # Now start offboard mode
            await self._drone.offboard.start()

            self.get_logger().info("Offboard mode started successfully")
            return True, "Offboard started"

        except OffboardError as e:
            error_msg = f"Offboard start failed: {e}"
            self.get_logger().error(error_msg)
            self.get_logger().error("Hint: Make sure the drone is armed first")
            return False, error_msg
        except Exception as e:
            error_msg = f"Offboard start failed: {e}"
            self.get_logger().error(error_msg)
            return False, error_msg

    def _offboard_stop_callback(self, request: Trigger.Request, response: Trigger.Response):
        """Handle offboard stop service request."""
        ready, error = self._check_ready()
        if not ready:
            response.success = False
            response.message = f"Cannot stop offboard: {error}"
            self.get_logger().error(response.message)
            return response

        future = asyncio.run_coroutine_threadsafe(
            self._offboard_stop_async(),
            self._asyncio_loop
        )
        try:
            success, message = future.result(timeout=10.0)
            response.success = success
            response.message = message
        except Exception as e:
            response.success = False
            response.message = f"Offboard stop failed: {e}"
            self.get_logger().error(response.message)

        return response

    async def _offboard_stop_async(self):
        """Async offboard stop operation."""
        try:
            self.get_logger().info("Stopping offboard mode...")
            await self._drone.offboard.stop()
            self.get_logger().info("Offboard mode stopped")
            return True, "Offboard stopped"
        except Exception as e:
            error_msg = f"Offboard stop failed: {e}"
            self.get_logger().error(error_msg)
            return False, error_msg

    def _land_callback(self, request: Trigger.Request, response: Trigger.Response):
        """Handle land service request."""
        ready, error = self._check_ready()
        if not ready:
            response.success = False
            response.message = f"Cannot land: {error}"
            self.get_logger().error(response.message)
            return response

        future = asyncio.run_coroutine_threadsafe(
            self._land_async(),
            self._asyncio_loop
        )
        try:
            success, message = future.result(timeout=30.0)
            response.success = success
            response.message = message
        except Exception as e:
            response.success = False
            response.message = f"Land failed: {e}"
            self.get_logger().error(response.message)

        return response

    async def _land_async(self):
        """Async land operation."""
        try:
            self.get_logger().info("Landing...")

            with self._lock:
                self._drone_state = DroneState.LANDING

            # Stop offboard first if active
            try:
                await self._drone.offboard.stop()
            except Exception:
                pass  # May not be in offboard mode

            # Command landing
            await self._drone.action.land()

            self.get_logger().info("Landing command sent")
            return True, "Landing initiated"

        except Exception as e:
            error_msg = f"Land failed: {e}"
            self.get_logger().error(error_msg)
            return False, error_msg

    def _set_control_mode_callback(self, request: SetBool.Request, response: SetBool.Response):
        """
        Handle control mode switch service request.

        data=True:  Switch to ATTITUDE_RATE mode (for manipulation)
        data=False: Switch to VELOCITY mode (for flying)
        """
        if request.data:
            mode_name = "ATTITUDE_RATE"
        else:
            mode_name = "VELOCITY"

        # Send initial setpoints immediately via async call
        future = asyncio.run_coroutine_threadsafe(
            self._switch_control_mode_async(request.data),
            self._asyncio_loop
        )

        try:
            success, message = future.result(timeout=5.0)
            response.success = success
            response.message = message
        except Exception as e:
            response.success = False
            response.message = f"Mode switch failed: {e}"
            self.get_logger().error(response.message)

        return response

    def _set_attitude_mode_callback(self, request: Trigger.Request, response: Trigger.Response):
        """Handle switching to ATTITUDE mode (angle-based attitude control)."""
        future = asyncio.run_coroutine_threadsafe(
            self._switch_to_attitude_mode_async(),
            self._asyncio_loop
        )

        try:
            success, message = future.result(timeout=5.0)
            response.success = success
            response.message = message
        except Exception as e:
            response.success = False
            response.message = f"Mode switch to ATTITUDE failed: {e}"
            self.get_logger().error(response.message)

        return response

    async def _switch_to_attitude_mode_async(self):
        """Switch to ATTITUDE mode with smooth transition."""
        try:
            # Read current state
            with self._lock:
                thrust = self._px4_thrust_setpoint
                yaw_deg = self._attitude_euler[2]

            # Fallback to estimated thrust if PX4 setpoint seems wrong
            if thrust < 0.2 or thrust > 0.9:
                thrust = self._estimate_current_thrust()

            self.get_logger().info(
                f"Switching to ATTITUDE mode: yaw={yaw_deg:.1f} deg, "
                f"thrust={thrust:.3f}"
            )

            # Pre-send attitude setpoints for smooth transition
            for _ in range(10):
                await self._drone.offboard.set_attitude(
                    Attitude(0.0, 0.0, yaw_deg, thrust)
                )
                await asyncio.sleep(0.05)

            # Update state variables
            with self._lock:
                self._control_mode = ControlMode.ATTITUDE
                self._cmd_roll_deg = 0.0
                self._cmd_pitch_deg = 0.0
                self._cmd_yaw_deg_attitude = yaw_deg
                self._cmd_attitude_thrust = thrust
                self._last_attitude_cmd_time = time.time()

            self.get_logger().info("Control mode switched to: ATTITUDE")
            return True, "Control mode set to ATTITUDE"

        except Exception as e:
            error_msg = f"Mode switch to ATTITUDE failed: {e}"
            self.get_logger().error(error_msg)
            return False, error_msg

    def _estimate_current_thrust(self):
        """
        Estimate the current thrust being used based on attitude and velocity.

        This estimates the normalized thrust (0-1) needed to maintain current flight state.
        Uses physics-based calculation from current roll/pitch angles and vertical motion.

        Returns:
            float: Estimated normalized thrust (0-1)
        """
        with self._lock:
            roll_deg = self._attitude_euler[0]
            pitch_deg = self._attitude_euler[1]
            vel_down = self._velocity_ned[2]  # Positive is downward
            last_cmd_up = -self._cmd_down_m_s  # Last commanded upward velocity

        # Convert angles to radians
        roll_rad = math.radians(roll_deg)
        pitch_rad = math.radians(pitch_deg)

        # Calculate tilt compensation factor
        # When tilted, more thrust is needed to maintain altitude
        # Total_thrust = hover_thrust / (cos(roll) * cos(pitch))
        cos_roll = math.cos(roll_rad)
        cos_pitch = math.cos(pitch_rad)
        tilt_factor = 1.0 / (cos_roll * cos_pitch) if (cos_roll > 0.1 and cos_pitch > 0.1) else 1.0

        # Base thrust from tilt
        thrust = self.hover_thrust * tilt_factor

        # Add dynamic component based on vertical velocity
        # If ascending (vel_down < 0), we need more thrust
        # If descending (vel_down > 0), we need less thrust
        k_vel_dynamics = 0.08  # Thrust adjustment per m/s of velocity
        thrust -= vel_down * k_vel_dynamics

        # Add commanded velocity component
        # This captures what the velocity controller was trying to achieve
        k_cmd_vel = 0.12  # Thrust adjustment per m/s of commanded velocity
        thrust += last_cmd_up * k_cmd_vel

        # Clamp to safe range
        thrust = max(0.15, min(0.85, thrust))

        return thrust

    async def _switch_control_mode_async(self, to_attitude_rate: bool):
        """
        Async mode switch with immediate setpoint sending.
        This ensures smooth transition between control modes.

        When switching to attitude rate mode, we use PX4's current thrust setpoint
        instead of estimation for smooth, stable transitions.
        """
        try:
            if to_attitude_rate:
                mode_name = "ATTITUDE_RATE"

                # Use PX4's current thrust setpoint (more reliable than estimation)
                with self._lock:
                    thrust = self._px4_thrust_setpoint
                    roll_deg = self._attitude_euler[0]
                    pitch_deg = self._attitude_euler[1]
                    vel_down = self._velocity_ned[2]
                    last_cmd_up = -self._cmd_down_m_s

                # Fallback to estimated thrust if PX4 setpoint is not available or seems wrong
                if thrust < 0.2 or thrust > 0.9:
                    thrust_estimated = self._estimate_current_thrust()
                    self.get_logger().warn(
                        f"PX4 thrust setpoint ({thrust:.3f}) out of range, "
                        f"using estimation ({thrust_estimated:.3f})"
                    )
                    thrust = thrust_estimated

                self.get_logger().info(
                    f"Switching to ATTITUDE_RATE mode: "
                    f"roll={roll_deg:.1f}°, pitch={pitch_deg:.1f}°, "
                    f"vel_down={vel_down:.2f} m/s, last_cmd_up={last_cmd_up:.2f} m/s, "
                    f"PX4 thrust setpoint={thrust:.3f}"
                )

                # Send attitude rate setpoints immediately (multiple times for reliability)
                for _ in range(10):  # Send for ~0.5 seconds
                    await self._drone.offboard.set_attitude_rate(
                        AttitudeRate(0.0, 0.0, 0.0, thrust)
                    )
                    await asyncio.sleep(0.05)

                # Now update the state variables
                with self._lock:
                    self._control_mode = ControlMode.ATTITUDE_RATE
                    self._cmd_roll_rate_deg_s = 0.0
                    self._cmd_pitch_rate_deg_s = 0.0
                    self._cmd_yaw_rate_deg_s = 0.0
                    self._cmd_thrust = thrust
                    self._last_attitude_rate_cmd_time = time.time()

            else:
                mode_name = "VELOCITY"

                # When switching back to velocity mode, command zero velocity (hover)
                with self._lock:
                    yaw_deg = self._attitude_euler[2]
                    last_thrust = self._cmd_thrust

                # Estimate initial vertical velocity based on thrust difference from hover
                # If thrust was above hover, we might have been ascending
                # Start with zero velocity and let PX4 velocity controller take over
                self.get_logger().info(
                    f"Switching to VELOCITY mode: last_thrust={last_thrust:.3f}"
                )

                # Send velocity setpoints immediately
                for _ in range(10):  # Send for ~0.5 seconds
                    await self._drone.offboard.set_velocity_ned(
                        VelocityNedYaw(0.0, 0.0, 0.0, yaw_deg)
                    )
                    await asyncio.sleep(0.05)

                # Now update the state variables
                with self._lock:
                    self._control_mode = ControlMode.VELOCITY
                    self._cmd_north_m_s = 0.0
                    self._cmd_east_m_s = 0.0
                    self._cmd_down_m_s = 0.0
                    self._cmd_yaw_deg_s = 0.0
                    self._last_cmd_time = time.time()

            self.get_logger().info(f"Control mode switched to: {mode_name}")
            return True, f"Control mode set to {mode_name}"

        except Exception as e:
            error_msg = f"Mode switch failed: {e}"
            self.get_logger().error(error_msg)
            return False, error_msg

    def destroy_node(self):
        """Clean up on node destruction."""
        self.get_logger().info("Shutting down...")
        self._running = False

        # Cancel asyncio tasks
        if self._asyncio_loop is not None:
            # Cancel all telemetry tasks
            for task in self._telemetry_tasks:
                if not task.done():
                    self._asyncio_loop.call_soon_threadsafe(task.cancel)

            self._asyncio_loop.call_soon_threadsafe(self._asyncio_loop.stop)

        if self._asyncio_thread is not None:
            self._asyncio_thread.join(timeout=2.0)

        super().destroy_node()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    node = OffboardControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
