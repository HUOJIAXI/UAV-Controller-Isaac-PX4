"""
Launch file for UAV Position Controller.

Starts offboard_node (PX4 interface) + position controller + optional GUI.

Examples:
    # Default (single drone with GUI)
    ros2 launch uav_position_controller position_control.launch.py

    # Without GUI
    ros2 launch uav_position_controller position_control.launch.py gui:=false

    # Multi-drone
    ros2 launch uav_position_controller position_control.launch.py drone_id:=1 port:=14281
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    """Generate the launch description."""
    # Add pyenv site-packages to PYTHONPATH for mavsdk
    pyenv_site_packages = os.path.expanduser(
        '~/.pyenv/versions/3.12.12/lib/python3.12/site-packages'
    )
    existing_pythonpath = os.environ.get('PYTHONPATH', '')
    new_pythonpath = (
        f'{pyenv_site_packages}:{existing_pythonpath}'
        if existing_pythonpath else pyenv_site_packages)

    set_pythonpath = SetEnvironmentVariable('PYTHONPATH', new_pythonpath)

    # Launch arguments
    drone_id_arg = DeclareLaunchArgument(
        'drone_id', default_value='0',
        description='Drone ID for namespacing (0 = no namespace)')
    port_arg = DeclareLaunchArgument(
        'port', default_value='14280',
        description='MAVLink UDP port')
    host_arg = DeclareLaunchArgument(
        'host', default_value='127.0.0.1',
        description='MAVLink host address')
    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value='',
        description='Custom namespace (overrides drone_id)')
    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Launch control panel GUI')
    hover_thrust_arg = DeclareLaunchArgument(
        'hover_thrust', default_value='0.6',
        description='Normalized hover thrust (0-1)')
    imu_rate_hz_arg = DeclareLaunchArgument(
        'imu_rate_hz', default_value='50.0',
        description='IMU telemetry publish rate (Hz)')
    odom_rate_hz_arg = DeclareLaunchArgument(
        'odom_rate_hz', default_value='50.0',
        description='Odom/telemetry publish rate (Hz)')

    # Rate control parameters
    use_rate_control_arg = DeclareLaunchArgument(
        'use_rate_control', default_value='true',
        description='Use rate control (true) or attitude control (false)')
    att_kp_roll_arg = DeclareLaunchArgument(
        'att_kp_roll', default_value='2.0',
        description='Attitude rate PID proportional gain for roll')
    att_ki_roll_arg = DeclareLaunchArgument(
        'att_ki_roll', default_value='0.0',
        description='Attitude rate PID integral gain for roll')
    att_kd_roll_arg = DeclareLaunchArgument(
        'att_kd_roll', default_value='0.0',
        description='Attitude rate PID derivative gain for roll')
    att_kp_pitch_arg = DeclareLaunchArgument(
        'att_kp_pitch', default_value='2.0',
        description='Attitude rate PID proportional gain for pitch')
    att_ki_pitch_arg = DeclareLaunchArgument(
        'att_ki_pitch', default_value='0.0',
        description='Attitude rate PID integral gain for pitch')
    att_kd_pitch_arg = DeclareLaunchArgument(
        'att_kd_pitch', default_value='0.0',
        description='Attitude rate PID derivative gain for pitch')
    att_kp_yaw_arg = DeclareLaunchArgument(
        'att_kp_yaw', default_value='1.0',
        description='Attitude rate PID proportional gain for yaw')
    att_ki_yaw_arg = DeclareLaunchArgument(
        'att_ki_yaw', default_value='0.0',
        description='Attitude rate PID integral gain for yaw')
    att_kd_yaw_arg = DeclareLaunchArgument(
        'att_kd_yaw', default_value='0.0',
        description='Attitude rate PID derivative gain for yaw')
    max_roll_rate_arg = DeclareLaunchArgument(
        'max_roll_rate', default_value='100.0',
        description='Maximum roll rate (deg/s)')
    max_pitch_rate_arg = DeclareLaunchArgument(
        'max_pitch_rate', default_value='100.0',
        description='Maximum pitch rate (deg/s)')
    max_yaw_rate_arg = DeclareLaunchArgument(
        'max_yaw_rate', default_value='30.0',
        description='Maximum yaw rate (deg/s)')

    # Compute namespace
    node_namespace = PythonExpression([
        "'", LaunchConfiguration('namespace'),
        "' if '", LaunchConfiguration('namespace'),
        "' else ('drone' + '", LaunchConfiguration('drone_id'),
        "' if '", LaunchConfiguration('drone_id'),
        "' != '0' else '')"])

    # Connection URL
    connection_url = PythonExpression([
        "'udpout://' + '", LaunchConfiguration('host'),
        "' + ':' + '", LaunchConfiguration('port'), "'"])

    # 1. Offboard node (PX4 interface)
    offboard_node = Node(
        package='px4_offboard_control',
        executable='offboard_node',
        name='px4_offboard_control',
        namespace=node_namespace,
        output='screen',
        emulate_tty=True,
        parameters=[{
            'connection_url': connection_url,
            'drone_id': PythonExpression([
                "int('", LaunchConfiguration('drone_id'), "')"]),
            'setpoint_rate_hz': 20.0,
            'cmd_timeout_sec': 0.5,
            'yaw_mode': 'angle',
            'hover_thrust': LaunchConfiguration('hover_thrust'),
            'imu_rate_hz': LaunchConfiguration('imu_rate_hz'),
            'odom_rate_hz': LaunchConfiguration('odom_rate_hz'),
            'use_sim_time': True,
        }])

    # 2. Position controller
    controller_node = Node(
        package='uav_position_controller',
        executable='controller_node',
        name='uav_position_controller',
        namespace=node_namespace,
        output='screen',
        emulate_tty=True,
        parameters=[{
            # Position PID gains
            'pos_kp_xy': 1.2,
            'pos_ki_xy': 0.0,
            'pos_kd_xy': 0.0,
            'pos_kp_z': 1.2,
            'pos_ki_z': 0.0,
            'pos_kd_z': 0.0,
            # Velocity PID gains
            'vel_kp_xy': 1.5,
            'vel_ki_xy': 0.2,
            'vel_kd_xy': 0.05,
            'vel_kp_z': 2.0,
            'vel_ki_z': 0.0,
            'vel_kd_z': 0.1,
            # Physical parameters
            'gravity': 9.81,
            'hover_thrust': LaunchConfiguration('hover_thrust'),
            'max_tilt_deg': 25.0,
            'max_velocity_xy': 2.0,
            'max_velocity_z': 1.5,
            'min_thrust': 0.1,
            'max_thrust': 0.9,
            'control_rate': 50.0,
            # Rate control parameters
            'use_rate_control': LaunchConfiguration('use_rate_control'),
            'att_kp_roll': LaunchConfiguration('att_kp_roll'),
            'att_ki_roll': LaunchConfiguration('att_ki_roll'),
            'att_kd_roll': LaunchConfiguration('att_kd_roll'),
            'att_kp_pitch': LaunchConfiguration('att_kp_pitch'),
            'att_ki_pitch': LaunchConfiguration('att_ki_pitch'),
            'att_kd_pitch': LaunchConfiguration('att_kd_pitch'),
            'att_kp_yaw': LaunchConfiguration('att_kp_yaw'),
            'att_ki_yaw': LaunchConfiguration('att_ki_yaw'),
            'att_kd_yaw': LaunchConfiguration('att_kd_yaw'),
            'max_roll_rate': LaunchConfiguration('max_roll_rate'),
            'max_pitch_rate': LaunchConfiguration('max_pitch_rate'),
            'max_yaw_rate': LaunchConfiguration('max_yaw_rate'),
            'use_sim_time': True,
        }])

    # 3. Control panel (optional)
    control_panel_node = Node(
        package='uav_position_controller',
        executable='control_panel',
        name='uav_control_panel',
        namespace=node_namespace,
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('gui')))

    return LaunchDescription([
        set_pythonpath,
        drone_id_arg,
        port_arg,
        host_arg,
        namespace_arg,
        gui_arg,
        hover_thrust_arg,
        imu_rate_hz_arg,
        odom_rate_hz_arg,
        use_rate_control_arg,
        att_kp_roll_arg,
        att_ki_roll_arg,
        att_kd_roll_arg,
        att_kp_pitch_arg,
        att_ki_pitch_arg,
        att_kd_pitch_arg,
        att_kp_yaw_arg,
        att_ki_yaw_arg,
        att_kd_yaw_arg,
        max_roll_rate_arg,
        max_pitch_rate_arg,
        max_yaw_rate_arg,
        offboard_node,
        controller_node,
        control_panel_node,
    ])
