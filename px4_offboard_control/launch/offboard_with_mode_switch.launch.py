"""
Launch file for PX4 Offboard Control with Mode Switch Node.

This launch file starts both the offboard control node and the mode switch node,
enabling seamless switching between velocity mode (flying) and attitude rate mode
(manipulation).

Examples:
    # Single drone with mode switch
    ros2 launch px4_offboard_control offboard_with_mode_switch.launch.py

    # With custom parameters
    ros2 launch px4_offboard_control offboard_with_mode_switch.launch.py drone_id:=0 port:=14280

    # Disable keyboard control (for programmatic control only)
    ros2 launch px4_offboard_control offboard_with_mode_switch.launch.py enable_keyboard:=false
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

    enable_keyboard_arg = DeclareLaunchArgument(
        'enable_keyboard',
        default_value='true',
        description='Enable keyboard control in mode switch node'
    )

    default_thrust_arg = DeclareLaunchArgument(
        'default_thrust',
        default_value='0.5',
        description='Default thrust value for attitude rate mode (0-1, ~0.5 for hover)'
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

    # Offboard control node
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
            'yaw_mode': 'rate',
            'hover_thrust': LaunchConfiguration('default_thrust'),
        }]
    )

    # Mode switch node
    mode_switch_node = Node(
        package='px4_offboard_control',
        executable='mode_switch',
        name='mode_switch',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'namespace': node_namespace,
            'enable_keyboard': PythonExpression([
                "'", LaunchConfiguration('enable_keyboard'), "'.lower() == 'true'"
            ]),
            'default_thrust': LaunchConfiguration('default_thrust'),
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
        enable_keyboard_arg,
        default_thrust_arg,
        offboard_node,
        mode_switch_node,
    ])
