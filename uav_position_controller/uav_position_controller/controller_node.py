#!/usr/bin/env python3
"""
UAV Position Controller Node.

Cascaded PID controller: position -> velocity -> attitude + thrust.
Outputs attitude angle setpoints to offboard_node which relays them
to PX4 via MAVSDK set_attitude(). PX4's inner attitude controller
handles rate control and motor mixing.

Designed for eventual UAV-manipulator integration where attitude-level
control authority is required.
"""

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import SetBool


class PIDController:
    """PID controller with anti-windup (back-calculation)."""

    def __init__(self, kp, ki, kd,
                 output_min=-float('inf'), output_max=float('inf')):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral = 0.0
        self.prev_error = 0.0
        self.first_run = True

    def compute(self, error, dt):
        """Compute PID output."""
        if dt <= 0:
            return 0.0

        p_term = self.kp * error

        self.integral += error * dt
        i_term = self.ki * self.integral

        if self.first_run:
            d_term = 0.0
            self.first_run = False
        else:
            d_term = self.kd * (error - self.prev_error) / dt

        self.prev_error = error

        output = p_term + i_term + d_term
        output_sat = max(self.output_min, min(self.output_max, output))

        # Anti-windup: back-calculate integral
        if self.ki != 0 and output != output_sat:
            self.integral = (output_sat - p_term - d_term) / self.ki

        return output_sat

    def reset(self):
        """Reset controller state."""
        self.integral = 0.0
        self.prev_error = 0.0
        self.first_run = True


def quaternion_to_euler(x, y, z, w):
    """Convert quaternion to euler angles (roll, pitch, yaw) in radians."""
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class PositionControllerNode(Node):
    """Cascaded PID position controller for PX4 UAV."""

    def __init__(self):
        super().__init__('uav_position_controller')

        # Declare parameters
        self._declare_parameters()
        self._load_parameters()

        # Initialize PID controllers
        self._init_controllers()

        # Current state (from odometry)
        self.current_position = [0.0, 0.0, 0.0]
        self.current_velocity = [0.0, 0.0, 0.0]
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_yaw = 0.0

        # Target
        self.target_position = [0.0, 0.0, 1.5]
        self.target_yaw = 0.0

        # Controller state
        self.controller_enabled = False
        self.odom_received = False
        self.last_control_time = None
        self.drone_is_offboard = False

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, 'telemetry/odom', self._odom_callback, 10)
        self.target_sub = self.create_subscription(
            PoseStamped, 'target_pose', self._target_callback, 10)
        self.state_sub = self.create_subscription(
            String, 'telemetry/state', self._state_callback, 10)

        # Publishers
        self.cmd_attitude_pub = self.create_publisher(
            Twist, 'cmd_attitude', 10)
        self.status_pub = self.create_publisher(
            String, 'controller/status', 10)

        # Services
        self.enable_srv = self.create_service(
            SetBool, 'controller/enable', self._enable_callback)

        # Control loop timer
        self.control_timer = self.create_timer(
            1.0 / self.control_rate, self._control_loop)

        # Status publishing timer
        self.status_timer = self.create_timer(0.1, self._publish_status)

        self.get_logger().info('UAV Position Controller initialized')
        self.get_logger().info(
            f'Control rate: {self.control_rate} Hz, '
            f'max tilt: {self.max_tilt_deg} deg, '
            f'hover thrust: {self.hover_thrust}')

    def _declare_parameters(self):
        """Declare all ROS 2 parameters."""
        # Position PID gains
        self.declare_parameter('pos_kp_xy', 1.2)
        self.declare_parameter('pos_ki_xy', 0.0)
        self.declare_parameter('pos_kd_xy', 0.0)
        self.declare_parameter('pos_kp_z', 1.5)
        self.declare_parameter('pos_ki_z', 0.05)
        self.declare_parameter('pos_kd_z', 0.0)

        # Velocity PID gains
        self.declare_parameter('vel_kp_xy', 1.5)
        self.declare_parameter('vel_ki_xy', 0.2)
        self.declare_parameter('vel_kd_xy', 0.05)
        self.declare_parameter('vel_kp_z', 3.0)
        self.declare_parameter('vel_ki_z', 0.5)
        self.declare_parameter('vel_kd_z', 0.1)

        # Physical parameters
        self.declare_parameter('gravity', 9.81)
        self.declare_parameter('hover_thrust', 0.5)
        self.declare_parameter('max_tilt_deg', 25.0)
        self.declare_parameter('max_velocity_xy', 2.0)
        self.declare_parameter('max_velocity_z', 1.5)
        self.declare_parameter('min_thrust', 0.1)
        self.declare_parameter('max_thrust', 0.9)
        self.declare_parameter('control_rate', 50.0)

    def _load_parameters(self):
        """Load parameters from ROS 2 parameter server."""
        self.pos_kp_xy = self.get_parameter(
            'pos_kp_xy').get_parameter_value().double_value
        self.pos_ki_xy = self.get_parameter(
            'pos_ki_xy').get_parameter_value().double_value
        self.pos_kd_xy = self.get_parameter(
            'pos_kd_xy').get_parameter_value().double_value
        self.pos_kp_z = self.get_parameter(
            'pos_kp_z').get_parameter_value().double_value
        self.pos_ki_z = self.get_parameter(
            'pos_ki_z').get_parameter_value().double_value
        self.pos_kd_z = self.get_parameter(
            'pos_kd_z').get_parameter_value().double_value

        self.vel_kp_xy = self.get_parameter(
            'vel_kp_xy').get_parameter_value().double_value
        self.vel_ki_xy = self.get_parameter(
            'vel_ki_xy').get_parameter_value().double_value
        self.vel_kd_xy = self.get_parameter(
            'vel_kd_xy').get_parameter_value().double_value
        self.vel_kp_z = self.get_parameter(
            'vel_kp_z').get_parameter_value().double_value
        self.vel_ki_z = self.get_parameter(
            'vel_ki_z').get_parameter_value().double_value
        self.vel_kd_z = self.get_parameter(
            'vel_kd_z').get_parameter_value().double_value

        self.gravity = self.get_parameter(
            'gravity').get_parameter_value().double_value
        self.hover_thrust = self.get_parameter(
            'hover_thrust').get_parameter_value().double_value
        self.max_tilt_deg = self.get_parameter(
            'max_tilt_deg').get_parameter_value().double_value
        self.max_velocity_xy = self.get_parameter(
            'max_velocity_xy').get_parameter_value().double_value
        self.max_velocity_z = self.get_parameter(
            'max_velocity_z').get_parameter_value().double_value
        self.min_thrust = self.get_parameter(
            'min_thrust').get_parameter_value().double_value
        self.max_thrust = self.get_parameter(
            'max_thrust').get_parameter_value().double_value
        self.control_rate = self.get_parameter(
            'control_rate').get_parameter_value().double_value

    def _init_controllers(self):
        """Initialize PID controllers."""
        # Position PIDs (output: desired velocity m/s)
        self.pos_pid_x = PIDController(
            self.pos_kp_xy, self.pos_ki_xy, self.pos_kd_xy,
            -self.max_velocity_xy, self.max_velocity_xy)
        self.pos_pid_y = PIDController(
            self.pos_kp_xy, self.pos_ki_xy, self.pos_kd_xy,
            -self.max_velocity_xy, self.max_velocity_xy)
        self.pos_pid_z = PIDController(
            self.pos_kp_z, self.pos_ki_z, self.pos_kd_z,
            -self.max_velocity_z, self.max_velocity_z)

        # Velocity PIDs (output: desired acceleration m/s^2)
        max_accel_xy = self.gravity * math.tan(
            math.radians(self.max_tilt_deg))
        self.vel_pid_x = PIDController(
            self.vel_kp_xy, self.vel_ki_xy, self.vel_kd_xy,
            -max_accel_xy, max_accel_xy)
        self.vel_pid_y = PIDController(
            self.vel_kp_xy, self.vel_ki_xy, self.vel_kd_xy,
            -max_accel_xy, max_accel_xy)
        self.vel_pid_z = PIDController(
            self.vel_kp_z, self.vel_ki_z, self.vel_kd_z,
            -self.gravity * 0.5, self.gravity * 0.8)

    def _reset_all_pids(self):
        """Reset all PID controllers."""
        self.pos_pid_x.reset()
        self.pos_pid_y.reset()
        self.pos_pid_z.reset()
        self.vel_pid_x.reset()
        self.vel_pid_y.reset()
        self.vel_pid_z.reset()

    def _odom_callback(self, msg):
        """Handle odometry from offboard_node."""
        self.current_position[0] = msg.pose.pose.position.x
        self.current_position[1] = msg.pose.pose.position.y
        self.current_position[2] = msg.pose.pose.position.z

        self.current_velocity[0] = msg.twist.twist.linear.x
        self.current_velocity[1] = msg.twist.twist.linear.y
        self.current_velocity[2] = msg.twist.twist.linear.z

        q = msg.pose.pose.orientation
        self.current_roll, self.current_pitch, self.current_yaw = (
            quaternion_to_euler(q.x, q.y, q.z, q.w))

        self.odom_received = True

    def _target_callback(self, msg):
        """Handle target pose."""
        self.target_position[0] = msg.pose.position.x
        self.target_position[1] = msg.pose.position.y
        self.target_position[2] = msg.pose.position.z

        q = msg.pose.orientation
        _, _, yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)
        self.target_yaw = yaw

        self.get_logger().info(
            f'New target: [{msg.pose.position.x:.2f}, '
            f'{msg.pose.position.y:.2f}, {msg.pose.position.z:.2f}], '
            f'yaw={math.degrees(yaw):.1f} deg')

    def _state_callback(self, msg):
        """Detect offboard+armed state from offboard_node."""
        state = msg.data.lower()
        was_offboard = self.drone_is_offboard
        self.drone_is_offboard = (
            'offboard' in state and 'armed' in state)

        if self.drone_is_offboard and not was_offboard:
            self._enable_controller()
        elif not self.drone_is_offboard and was_offboard:
            self._disable_controller()

    def _enable_callback(self, request, response):
        """Handle enable/disable service."""
        if request.data:
            self._enable_controller()
            response.message = 'Controller enabled'
        else:
            self._disable_controller()
            response.message = 'Controller disabled'
        response.success = True
        return response

    def _enable_controller(self):
        """Enable controller and reset PIDs."""
        self._reset_all_pids()
        self.controller_enabled = True
        self.last_control_time = None
        # Hold current position
        self.target_position = list(self.current_position)
        self.target_yaw = self.current_yaw
        self.get_logger().info(
            f'Controller ENABLED - holding position '
            f'[{self.current_position[0]:.2f}, '
            f'{self.current_position[1]:.2f}, '
            f'{self.current_position[2]:.2f}]')

    def _disable_controller(self):
        """Disable the controller."""
        self.controller_enabled = False
        self.get_logger().info('Controller DISABLED')

    def _control_loop(self):
        """Main control loop at control_rate Hz."""
        if not self.controller_enabled or not self.odom_received:
            return

        # Compute dt
        now = self.get_clock().now()
        if self.last_control_time is None:
            self.last_control_time = now
            return
        dt = (now - self.last_control_time).nanoseconds * 1e-9
        self.last_control_time = now
        if dt <= 0 or dt > 0.1:
            dt = 1.0 / self.control_rate

        # === Position PID (world frame) ===
        pos_err_x = self.target_position[0] - self.current_position[0]
        pos_err_y = self.target_position[1] - self.current_position[1]
        pos_err_z = self.target_position[2] - self.current_position[2]

        des_vel_x = self.pos_pid_x.compute(pos_err_x, dt)
        des_vel_y = self.pos_pid_y.compute(pos_err_y, dt)
        des_vel_z = self.pos_pid_z.compute(pos_err_z, dt)

        # === Velocity PID (world frame) ===
        vel_err_x = des_vel_x - self.current_velocity[0]
        vel_err_y = des_vel_y - self.current_velocity[1]
        vel_err_z = des_vel_z - self.current_velocity[2]

        acc_x = self.vel_pid_x.compute(vel_err_x, dt)
        acc_y = self.vel_pid_y.compute(vel_err_y, dt)
        acc_z = self.vel_pid_z.compute(vel_err_z, dt) + self.gravity

        # === Acceleration to Attitude Conversion ===
        # Transform world-frame acceleration (ENU, yaw CCW from East) to body frame.
        # ENU axes: X=East, Y=North, Z=Up. Body x = forward.
        # Rotation ENU->body about Z (using R(-ψ) to body FLU, then flip Y to FRD):
        # FLU: [acc_flu_x] = [ cosψ  sinψ] [acc_x]
        #      [acc_flu_y]   [-sinψ  cosψ] [acc_y]
        # Convert FLU to FRD: acc_body_y = -acc_flu_y (right positive)
        cos_yaw = math.cos(self.current_yaw)
        sin_yaw = math.sin(self.current_yaw)

        acc_body_x = acc_x * cos_yaw + acc_y * sin_yaw
        acc_body_y = acc_x * sin_yaw - acc_y * cos_yaw

        # Compute desired roll and pitch
        desired_pitch = math.atan2(-acc_body_x, self.gravity)
        desired_roll = math.atan2(acc_body_y, self.gravity)

        # Clip to max tilt
        max_tilt_rad = math.radians(self.max_tilt_deg)
        desired_roll = max(-max_tilt_rad, min(max_tilt_rad, desired_roll))
        desired_pitch = max(
            -max_tilt_rad, min(max_tilt_rad, desired_pitch))

        # Compute thrust with tilt compensation
        cos_r = math.cos(desired_roll)
        cos_p = math.cos(desired_pitch)
        if cos_r > 0.1 and cos_p > 0.1:
            tilt_compensation = 1.0 / (cos_r * cos_p)
        else:
            tilt_compensation = 1.0
        thrust_cmd = (
            (acc_z / self.gravity) * self.hover_thrust * tilt_compensation)
        thrust_cmd = max(self.min_thrust, min(self.max_thrust, thrust_cmd))

        # Yaw: convert internal ENU yaw (rad, CCW from East) to PX4 NED yaw (deg, CW from North)
        desired_yaw_deg_enu = math.degrees(self.target_yaw)
        desired_yaw = 90.0 - desired_yaw_deg_enu

        # === Publish attitude command ===
        cmd = Twist()
        cmd.angular.x = math.degrees(desired_roll)
        cmd.angular.y = math.degrees(desired_pitch)
        cmd.angular.z = desired_yaw
        cmd.linear.z = thrust_cmd
        self.cmd_attitude_pub.publish(cmd)

    def _publish_status(self):
        """Publish controller status."""
        msg = String()
        if self.controller_enabled:
            msg.data = 'enabled'
        else:
            msg.data = 'disabled'
        self.status_pub.publish(msg)


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = PositionControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
