"""
Launch file for PX4 Offboard Control Node.

This launch file starts the offboard control node with configurable parameters.
Supports multiple UAVs via drone_id and port parameters.

Examples:
    # Single drone (default)
    ros2 launch px4_offboard_control offboard.launch.py

    # Multiple drones with different ports
    ros2 launch px4_offboard_control offboard.launch.py drone_id:=0 port:=14280
    ros2 launch px4_offboard_control offboard.launch.py drone_id:=1 port:=14281
    ros2 launch px4_offboard_control offboard.launch.py drone_id:=2 port:=14282

    # Custom namespace
    ros2 launch px4_offboard_control offboard.launch.py namespace:=uav1 port:=14280
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    """Generate the launch description."""

    # Add pyenv site-packages to PYTHONPATH for mavsdk
    pyenv_site_packages = os.path.expanduser(
        '~/.pyenv/versions/3.12.12/lib/python3.12/site-packages'
    )
    existing_pythonpath = os.environ.get('PYTHONPATH', '')
    new_pythonpath = f"{pyenv_site_packages}:{existing_pythonpath}" if existing_pythonpath else pyenv_site_packages

    set_pythonpath = SetEnvironmentVariable('PYTHONPATH', new_pythonpath)

    # Declare launch arguments
    drone_id_arg = DeclareLaunchArgument(
        'drone_id',
        default_value='0',
        description='Drone ID for namespacing (0, 1, 2, ...). drone_id=0 uses no namespace.'
    )

    port_arg = DeclareLaunchArgument(
        'port',
        default_value='14280',
        description='MAVLink UDP port for this drone (e.g., 14280, 14281, 14282)'
    )

    host_arg = DeclareLaunchArgument(
        'host',
        default_value='127.0.0.1',
        description='MAVLink host address'
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Custom namespace (overrides drone_id-based namespace if set)'
    )

    setpoint_rate_arg = DeclareLaunchArgument(
        'setpoint_rate_hz',
        default_value='20.0',
        description='Setpoint publishing rate in Hz (minimum 10 Hz recommended)'
    )

    cmd_timeout_arg = DeclareLaunchArgument(
        'cmd_timeout_sec',
        default_value='0.5',
        description='Command timeout in seconds (zero velocity sent if no command received)'
    )

    takeoff_altitude_arg = DeclareLaunchArgument(
        'takeoff_altitude_m',
        default_value='1.5',
        description='Default takeoff altitude in meters'
    )

    yaw_mode_arg = DeclareLaunchArgument(
        'yaw_mode',
        default_value='rate',
        description='Yaw control mode: "rate" for yaw rate control, "angle" for yaw angle control'
    )

    control_mode_arg = DeclareLaunchArgument(
        'control_mode',
        default_value='velocity',
        description='Control mode: "velocity" or "acceleration"'
    )

    # Build connection URL from host and port
    connection_url = PythonExpression([
        "'udpout://' + '",
        LaunchConfiguration('host'),
        "' + ':' + '",
        LaunchConfiguration('port'),
        "'"
    ])

    # Build namespace: use custom namespace if provided, otherwise use drone_id
    # For drone_id=0 with no custom namespace, use empty namespace (backward compatible)
    node_namespace = PythonExpression([
        "'",
        LaunchConfiguration('namespace'),
        "' if '",
        LaunchConfiguration('namespace'),
        "' else ('drone' + '",
        LaunchConfiguration('drone_id'),
        "' if '",
        LaunchConfiguration('drone_id'),
        "' != '0' else '')"
    ])

    # Create the node
    offboard_node = Node(
        package='px4_offboard_control',
        executable='offboard_node',
        name='px4_offboard_control',
        namespace=node_namespace,
        output='screen',
        emulate_tty=True,
        parameters=[{
            'connection_url': connection_url,
            'drone_id': PythonExpression(["int('", LaunchConfiguration('drone_id'), "')"]),
            'setpoint_rate_hz': LaunchConfiguration('setpoint_rate_hz'),
            'cmd_timeout_sec': LaunchConfiguration('cmd_timeout_sec'),
            'takeoff_altitude_m': LaunchConfiguration('takeoff_altitude_m'),
            'yaw_mode': LaunchConfiguration('yaw_mode'),
            'control_mode': LaunchConfiguration('control_mode'),
        }]
    )

    return LaunchDescription([
        set_pythonpath,
        drone_id_arg,
        port_arg,
        host_arg,
        namespace_arg,
        setpoint_rate_arg,
        cmd_timeout_arg,
        takeoff_altitude_arg,
        yaw_mode_arg,
        control_mode_arg,
        offboard_node,
    ])
