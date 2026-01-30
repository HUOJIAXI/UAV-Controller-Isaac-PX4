"""
Launch file for multi-drone UAV Position Controller.

Launches N drones, each with its own offboard_node + position controller
+ optional control panel. Ports and namespaces are auto-allocated.

Port allocation:
    drone_id=0 -> MAVLink port 14280, gRPC port 50051, namespace: /
    drone_id=1 -> MAVLink port 14281, gRPC port 50052, namespace: /drone1
    drone_id=2 -> MAVLink port 14282, gRPC port 50053, namespace: /drone2

Examples:
    # 2 drones with GUI for each
    ros2 launch uav_position_controller multi_drone.launch.py num_drones:=2

    # 3 drones, no GUI
    ros2 launch uav_position_controller multi_drone.launch.py num_drones:=3 gui:=false

    # 2 drones starting from a different base port
    ros2 launch uav_position_controller multi_drone.launch.py num_drones:=2 base_port:=14540
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
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
    num_drones_arg = DeclareLaunchArgument(
        'num_drones', default_value='2',
        description='Number of drones to launch')
    base_port_arg = DeclareLaunchArgument(
        'base_port', default_value='14280',
        description='Base MAVLink UDP port (incremented per drone)')
    host_arg = DeclareLaunchArgument(
        'host', default_value='127.0.0.1',
        description='MAVLink host address')
    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Launch control panel GUI for each drone')
    hover_thrust_arg = DeclareLaunchArgument(
        'hover_thrust', default_value='0.5',
        description='Normalized hover thrust (0-1)')

    # We need to resolve num_drones and base_port at generation time
    # since we can't dynamically create nodes with LaunchConfiguration.
    # Use OpaqueFunction for this.
    from launch.actions import OpaqueFunction

    def launch_drones(context):
        num_drones = int(
            LaunchConfiguration('num_drones').perform(context))
        base_port = int(
            LaunchConfiguration('base_port').perform(context))
        host = LaunchConfiguration('host').perform(context)
        gui = LaunchConfiguration('gui').perform(context)
        hover_thrust = LaunchConfiguration(
            'hover_thrust').perform(context)

        nodes = []

        for i in range(num_drones):
            drone_id = i
            port = base_port + i
            namespace = f'drone{drone_id}' if drone_id > 0 else ''
            connection_url = f'udpout://{host}:{port}'

            # Offboard node
            nodes.append(Node(
                package='px4_offboard_control',
                executable='offboard_node',
                name='px4_offboard_control',
                namespace=namespace,
                output='screen',
                emulate_tty=True,
                parameters=[{
                    'connection_url': connection_url,
                    'drone_id': drone_id,
                    'setpoint_rate_hz': 20.0,
                    'cmd_timeout_sec': 0.5,
                    'yaw_mode': 'angle',
                    'hover_thrust': float(hover_thrust),
                }]))

            # Position controller
            nodes.append(Node(
                package='uav_position_controller',
                executable='controller_node',
                name='uav_position_controller',
                namespace=namespace,
                output='screen',
                emulate_tty=True,
                parameters=[{
                    'pos_kp_xy': 1.2,
                    'pos_ki_xy': 0.0,
                    'pos_kd_xy': 0.0,
                    'pos_kp_z': 1.5,
                    'pos_ki_z': 0.05,
                    'pos_kd_z': 0.0,
                    'vel_kp_xy': 1.5,
                    'vel_ki_xy': 0.2,
                    'vel_kd_xy': 0.05,
                    'vel_kp_z': 3.0,
                    'vel_ki_z': 0.5,
                    'vel_kd_z': 0.1,
                    'gravity': 9.81,
                    'hover_thrust': float(hover_thrust),
                    'max_tilt_deg': 25.0,
                    'max_velocity_xy': 2.0,
                    'max_velocity_z': 1.5,
                    'min_thrust': 0.1,
                    'max_thrust': 0.9,
                    'control_rate': 50.0,
                }]))

            # Control panel (optional)
            if gui.lower() == 'true':
                nodes.append(Node(
                    package='uav_position_controller',
                    executable='control_panel',
                    name='uav_control_panel',
                    namespace=namespace,
                    output='screen',
                    emulate_tty=True))

        return nodes

    return LaunchDescription([
        set_pythonpath,
        num_drones_arg,
        base_port_arg,
        host_arg,
        gui_arg,
        hover_thrust_arg,
        OpaqueFunction(function=launch_drones),
    ])
