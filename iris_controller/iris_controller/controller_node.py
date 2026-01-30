#!/usr/bin/env python3
"""
Iris Quadrotor Controller Node
Cascaded PID Controller for position, velocity, attitude, and rate control
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from sensor_msgs.msg import Imu
import numpy as np
from dataclasses import dataclass
import math


@dataclass
class PIDGains:
    """PID gain parameters"""
    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0


class PIDController:
    """Simple PID controller with anti-windup"""

    def __init__(self, kp: float, ki: float, kd: float,
                 output_min: float = -float('inf'),
                 output_max: float = float('inf')):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max

        self.integral = 0.0
        self.prev_error = 0.0
        self.first_run = True

    def compute(self, error: float, dt: float) -> float:
        if dt <= 0:
            return 0.0

        # Proportional term
        p_term = self.kp * error

        # Integral term with anti-windup
        self.integral += error * dt
        i_term = self.ki * self.integral

        # Derivative term
        if self.first_run:
            d_term = 0.0
            self.first_run = False
        else:
            d_term = self.kd * (error - self.prev_error) / dt

        self.prev_error = error

        # Compute output with saturation
        output = p_term + i_term + d_term
        output_saturated = np.clip(output, self.output_min, self.output_max)

        # Anti-windup: back-calculate integral
        if self.ki != 0 and output != output_saturated:
            self.integral = (output_saturated - p_term - d_term) / self.ki

        return output_saturated

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.first_run = True


def quaternion_to_euler(q):
    """Convert quaternion to euler angles (roll, pitch, yaw)"""
    x, y, z, w = q

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


class IrisController(Node):
    """Main controller node for Iris quadrotor"""

    def __init__(self):
        super().__init__('iris_controller')

        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                # Position controller gains
                ('pos_kp_xy', 1.0),
                ('pos_ki_xy', 0.0),
                ('pos_kd_xy', 0.5),
                ('pos_kp_z', 2.0),
                ('pos_ki_z', 0.1),
                ('pos_kd_z', 0.5),
                # Velocity controller gains
                ('vel_kp_xy', 2.0),
                ('vel_ki_xy', 0.1),
                ('vel_kd_xy', 0.1),
                ('vel_kp_z', 5.0),
                ('vel_ki_z', 0.5),
                ('vel_kd_z', 0.2),
                # Attitude controller gains
                ('att_kp_rp', 6.0),
                ('att_ki_rp', 0.0),
                ('att_kd_rp', 0.5),
                ('att_kp_yaw', 3.0),
                ('att_ki_yaw', 0.0),
                ('att_kd_yaw', 0.3),
                # Rate controller gains
                ('rate_kp_rp', 0.15),
                ('rate_ki_rp', 0.0),
                ('rate_kd_rp', 0.01),
                ('rate_kp_yaw', 0.2),
                ('rate_ki_yaw', 0.0),
                ('rate_kd_yaw', 0.0),
                # Physical parameters
                ('mass', 1.5),  # kg
                ('gravity', 9.81),
                ('max_tilt_angle', 0.5),  # rad (~28 deg)
                ('hover_rotor_speed', 600.0),  # Rotor angular velocity for hover (rad/s)
                ('max_rotor_speed', 1000.0),   # Maximum rotor angular velocity (rad/s)
                ('min_rotor_speed', 0.0),      # Minimum rotor angular velocity (rad/s)
                # Control rate
                ('control_rate', 100.0),  # Hz
            ]
        )

        # Get parameters
        self.mass = self.get_parameter('mass').value
        self.gravity = self.get_parameter('gravity').value
        self.max_tilt = self.get_parameter('max_tilt_angle').value
        self.hover_rotor_speed = self.get_parameter('hover_rotor_speed').value
        self.max_rotor_speed = self.get_parameter('max_rotor_speed').value
        self.min_rotor_speed = self.get_parameter('min_rotor_speed').value

        # Initialize PID controllers
        self._init_controllers()

        # State variables
        self.current_pose = None
        self.current_twist = None
        self.current_imu = None

        # Setpoints
        self.target_position = np.array([0.0, 0.0, 1.0])  # Default hover at 1m
        self.target_yaw = 0.0
        self.armed = False

        # Timing
        self.last_control_time = self.get_clock().now()

        # QoS profile
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped, '/drone0/state/pose', self.pose_callback, qos)
        self.twist_sub = self.create_subscription(
            TwistStamped, '/drone0/state/twist', self.twist_callback, qos)
        self.imu_sub = self.create_subscription(
            Imu, '/drone0/sensors/imu', self.imu_callback, qos)

        # Target setpoint subscriber
        self.target_sub = self.create_subscription(
            PoseStamped, '/drone0/target_pose', self.target_callback, 10)

        # Arm/disarm subscriber
        self.arm_sub = self.create_subscription(
            Bool, '/drone0/arm', self.arm_callback, 10)

        # Publishers for rotor commands
        self.rotor_pubs = []
        for i in range(4):
            pub = self.create_publisher(Float64, f'/drone0/control/rotor{i}/ref', 10)
            self.rotor_pubs.append(pub)

        # Control timer
        control_rate = self.get_parameter('control_rate').value
        self.control_timer = self.create_timer(1.0 / control_rate, self.control_loop)

        self.get_logger().info('Iris Controller initialized')
        self.get_logger().info(f'Hover rotor speed: {self.hover_rotor_speed} rad/s')
        self.get_logger().info('Publish to /drone0/target_pose to set target position')
        self.get_logger().info('Publish to /drone0/arm (Bool) to arm/disarm')

    def _init_controllers(self):
        """Initialize all PID controllers"""
        # Position controllers (output: desired velocity)
        self.pos_pid_x = PIDController(
            self.get_parameter('pos_kp_xy').value,
            self.get_parameter('pos_ki_xy').value,
            self.get_parameter('pos_kd_xy').value,
            -2.0, 2.0  # Max velocity m/s
        )
        self.pos_pid_y = PIDController(
            self.get_parameter('pos_kp_xy').value,
            self.get_parameter('pos_ki_xy').value,
            self.get_parameter('pos_kd_xy').value,
            -2.0, 2.0
        )
        self.pos_pid_z = PIDController(
            self.get_parameter('pos_kp_z').value,
            self.get_parameter('pos_ki_z').value,
            self.get_parameter('pos_kd_z').value,
            -2.0, 2.0
        )

        # Velocity controllers (output: desired acceleration/attitude)
        self.vel_pid_x = PIDController(
            self.get_parameter('vel_kp_xy').value,
            self.get_parameter('vel_ki_xy').value,
            self.get_parameter('vel_kd_xy').value,
            -5.0, 5.0  # Max acceleration m/s^2
        )
        self.vel_pid_y = PIDController(
            self.get_parameter('vel_kp_xy').value,
            self.get_parameter('vel_ki_xy').value,
            self.get_parameter('vel_kd_xy').value,
            -5.0, 5.0
        )
        self.vel_pid_z = PIDController(
            self.get_parameter('vel_kp_z').value,
            self.get_parameter('vel_ki_z').value,
            self.get_parameter('vel_kd_z').value,
            -10.0, 20.0  # More thrust for climbing
        )

        # Attitude controllers (output: desired angular rate)
        self.att_pid_roll = PIDController(
            self.get_parameter('att_kp_rp').value,
            self.get_parameter('att_ki_rp').value,
            self.get_parameter('att_kd_rp').value,
            -3.0, 3.0  # Max angular rate rad/s
        )
        self.att_pid_pitch = PIDController(
            self.get_parameter('att_kp_rp').value,
            self.get_parameter('att_ki_rp').value,
            self.get_parameter('att_kd_rp').value,
            -3.0, 3.0
        )
        self.att_pid_yaw = PIDController(
            self.get_parameter('att_kp_yaw').value,
            self.get_parameter('att_ki_yaw').value,
            self.get_parameter('att_kd_yaw').value,
            -2.0, 2.0
        )

        # Rate controllers (output: motor torque commands)
        self.rate_pid_roll = PIDController(
            self.get_parameter('rate_kp_rp').value,
            self.get_parameter('rate_ki_rp').value,
            self.get_parameter('rate_kd_rp').value,
            -100.0, 100.0
        )
        self.rate_pid_pitch = PIDController(
            self.get_parameter('rate_kp_rp').value,
            self.get_parameter('rate_ki_rp').value,
            self.get_parameter('rate_kd_rp').value,
            -100.0, 100.0
        )
        self.rate_pid_yaw = PIDController(
            self.get_parameter('rate_kp_yaw').value,
            self.get_parameter('rate_ki_yaw').value,
            self.get_parameter('rate_kd_yaw').value,
            -100.0, 100.0
        )

    def pose_callback(self, msg: PoseStamped):
        self.current_pose = msg

    def twist_callback(self, msg: TwistStamped):
        self.current_twist = msg

    def imu_callback(self, msg: Imu):
        self.current_imu = msg

    def target_callback(self, msg: PoseStamped):
        self.target_position = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])
        # Extract yaw from quaternion
        q = msg.pose.orientation
        _, _, self.target_yaw = quaternion_to_euler([q.x, q.y, q.z, q.w])
        self.get_logger().info(f'New target: pos={self.target_position}, yaw={self.target_yaw:.2f}')

    def arm_callback(self, msg: Bool):
        """Handle arm/disarm commands"""
        if msg.data:
            self.arm()
        else:
            self.disarm()

    def arm(self):
        """Arm the controller"""
        self.armed = True
        self._reset_all_controllers()
        self.get_logger().info('Controller ARMED')

    def disarm(self):
        """Disarm the controller"""
        self.armed = False
        self._stop_motors()
        self.get_logger().info('Controller DISARMED')

    def _reset_all_controllers(self):
        """Reset all PID controllers"""
        self.pos_pid_x.reset()
        self.pos_pid_y.reset()
        self.pos_pid_z.reset()
        self.vel_pid_x.reset()
        self.vel_pid_y.reset()
        self.vel_pid_z.reset()
        self.att_pid_roll.reset()
        self.att_pid_pitch.reset()
        self.att_pid_yaw.reset()
        self.rate_pid_roll.reset()
        self.rate_pid_pitch.reset()
        self.rate_pid_yaw.reset()

    def _stop_motors(self):
        """Stop all motors"""
        for pub in self.rotor_pubs:
            msg = Float64()
            msg.data = 0.0
            pub.publish(msg)

    def control_loop(self):
        """Main control loop"""
        if not self.armed:
            return

        if self.current_pose is None or self.current_twist is None:
            return

        # Calculate dt
        current_time = self.get_clock().now()
        dt = (current_time - self.last_control_time).nanoseconds * 1e-9
        self.last_control_time = current_time

        if dt <= 0 or dt > 0.1:  # Sanity check
            dt = 0.01

        # Get current state
        pos = np.array([
            self.current_pose.pose.position.x,
            self.current_pose.pose.position.y,
            self.current_pose.pose.position.z
        ])

        q = self.current_pose.pose.orientation
        roll, pitch, yaw = quaternion_to_euler([q.x, q.y, q.z, q.w])

        vel = np.array([
            self.current_twist.twist.linear.x,
            self.current_twist.twist.linear.y,
            self.current_twist.twist.linear.z
        ])

        ang_vel = np.array([
            self.current_twist.twist.angular.x,
            self.current_twist.twist.angular.y,
            self.current_twist.twist.angular.z
        ])

        # ============ Position Control ============
        pos_error = self.target_position - pos

        des_vel_x = self.pos_pid_x.compute(pos_error[0], dt)
        des_vel_y = self.pos_pid_y.compute(pos_error[1], dt)
        des_vel_z = self.pos_pid_z.compute(pos_error[2], dt)

        # ============ Velocity Control ============
        vel_error_x = des_vel_x - vel[0]
        vel_error_y = des_vel_y - vel[1]
        vel_error_z = des_vel_z - vel[2]

        # Desired accelerations in world frame
        acc_x = self.vel_pid_x.compute(vel_error_x, dt)
        acc_y = self.vel_pid_y.compute(vel_error_y, dt)
        acc_z = self.vel_pid_z.compute(vel_error_z, dt) + self.gravity  # Feedforward gravity

        # Convert desired acceleration to thrust command
        # Normalized thrust: 1.0 = hover thrust
        thrust_normalized = acc_z / self.gravity
        thrust_normalized = np.clip(thrust_normalized, 0, 2.0)  # Max 2x hover

        # Desired roll and pitch from horizontal accelerations
        # Rotated by current yaw
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)

        acc_x_body = acc_x * cos_yaw + acc_y * sin_yaw
        acc_y_body = -acc_x * sin_yaw + acc_y * cos_yaw

        des_pitch = np.clip(np.arctan2(acc_x_body, self.gravity + acc_z), -self.max_tilt, self.max_tilt)
        des_roll = np.clip(np.arctan2(-acc_y_body, self.gravity + acc_z), -self.max_tilt, self.max_tilt)
        des_yaw = self.target_yaw

        # ============ Attitude Control ============
        roll_error = des_roll - roll
        pitch_error = des_pitch - pitch
        yaw_error = des_yaw - yaw

        # Wrap yaw error to [-pi, pi]
        while yaw_error > np.pi:
            yaw_error -= 2 * np.pi
        while yaw_error < -np.pi:
            yaw_error += 2 * np.pi

        des_roll_rate = self.att_pid_roll.compute(roll_error, dt)
        des_pitch_rate = self.att_pid_pitch.compute(pitch_error, dt)
        des_yaw_rate = self.att_pid_yaw.compute(yaw_error, dt)

        # ============ Rate Control ============
        roll_rate_error = des_roll_rate - ang_vel[0]
        pitch_rate_error = des_pitch_rate - ang_vel[1]
        yaw_rate_error = des_yaw_rate - ang_vel[2]

        roll_torque = self.rate_pid_roll.compute(roll_rate_error, dt)
        pitch_torque = self.rate_pid_pitch.compute(pitch_rate_error, dt)
        yaw_torque = self.rate_pid_yaw.compute(yaw_rate_error, dt)

        # ============ Motor Mixing ============
        # Iris quadrotor motor configuration (X configuration):
        # Motor 0: Front-Right (CW)  - +roll, -pitch, -yaw
        # Motor 1: Rear-Left (CW)    - -roll, +pitch, -yaw
        # Motor 2: Front-Left (CCW)  - -roll, -pitch, +yaw
        # Motor 3: Rear-Right (CCW)  - +roll, +pitch, +yaw

        # Base rotor speed for current thrust level
        base_speed = self.hover_rotor_speed * np.sqrt(thrust_normalized)

        # Scale torques to rotor speed deltas
        torque_scale = 50.0  # Tuning parameter for torque to rotor speed
        roll_delta = roll_torque * torque_scale
        pitch_delta = pitch_torque * torque_scale
        yaw_delta = yaw_torque * torque_scale

        motor_0 = base_speed + roll_delta - pitch_delta - yaw_delta
        motor_1 = base_speed - roll_delta + pitch_delta - yaw_delta
        motor_2 = base_speed - roll_delta - pitch_delta + yaw_delta
        motor_3 = base_speed + roll_delta + pitch_delta + yaw_delta

        # Clamp motor values
        motors = np.array([motor_0, motor_1, motor_2, motor_3])
        motors = np.clip(motors, self.min_rotor_speed, self.max_rotor_speed)

        # Publish motor commands
        for i, pub in enumerate(self.rotor_pubs):
            msg = Float64()
            msg.data = float(motors[i])
            pub.publish(msg)

    def set_target(self, x: float, y: float, z: float, yaw: float = 0.0):
        """Set target position and yaw"""
        self.target_position = np.array([x, y, z])
        self.target_yaw = yaw
        self.get_logger().info(f'Target set to: [{x:.2f}, {y:.2f}, {z:.2f}], yaw={yaw:.2f}')


def main(args=None):
    rclpy.init(args=args)
    controller = IrisController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.disarm()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
