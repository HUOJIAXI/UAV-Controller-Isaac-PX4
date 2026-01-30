#!/usr/bin/env python3
"""
Launch file for Iris Quadrotor Controller
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    hover_rotor_speed_arg = DeclareLaunchArgument(
        'hover_rotor_speed',
        default_value='600.0',
        description='Rotor angular velocity for hover (rad/s)'
    )

    # Controller node
    controller_node = Node(
        package='iris_controller',
        executable='controller_node',
        name='iris_controller',
        output='screen',
        parameters=[{
            # Position controller gains
            'pos_kp_xy': 1.0,
            'pos_ki_xy': 0.0,
            'pos_kd_xy': 0.5,
            'pos_kp_z': 2.0,
            'pos_ki_z': 0.1,
            'pos_kd_z': 0.5,
            # Velocity controller gains
            'vel_kp_xy': 2.0,
            'vel_ki_xy': 0.1,
            'vel_kd_xy': 0.1,
            'vel_kp_z': 5.0,
            'vel_ki_z': 0.5,
            'vel_kd_z': 0.2,
            # Attitude controller gains
            'att_kp_rp': 6.0,
            'att_ki_rp': 0.0,
            'att_kd_rp': 0.5,
            'att_kp_yaw': 3.0,
            'att_ki_yaw': 0.0,
            'att_kd_yaw': 0.3,
            # Rate controller gains
            'rate_kp_rp': 0.15,
            'rate_ki_rp': 0.0,
            'rate_kd_rp': 0.01,
            'rate_kp_yaw': 0.2,
            'rate_ki_yaw': 0.0,
            'rate_kd_yaw': 0.0,
            # Physical parameters
            'mass': 1.5,
            'gravity': 9.81,
            'max_tilt_angle': 0.5,
            'hover_rotor_speed': LaunchConfiguration('hover_rotor_speed'),
            'max_rotor_speed': 1000.0,
            'min_rotor_speed': 0.0,
            'control_rate': 100.0,
        }]
    )

    return LaunchDescription([
        hover_rotor_speed_arg,
        controller_node,
    ])
