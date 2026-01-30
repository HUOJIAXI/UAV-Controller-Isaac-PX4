#
TODO: the switch between 2 modes is still unstable. The two modes all work, but when velocity changes to attitute, the uav will fall down.

# PX4 Offboard Control (ROS 2 + MAVSDK)

A ROS 2 package for controlling a PX4 drone via MAVSDK-Python. Designed for use with PX4 SITL running in Isaac Sim with Pegasus.

## Features

- **Velocity Control**: Subscribe to `/cmd_vel` for velocity commands
- **Acceleration Control**: Optional `/cmd_accel` topic for acceleration setpoints
- **ROS 2 Services**: Arm/disarm, offboard start/stop, and landing
- **Telemetry Publishing**: Position, velocity, and state information
- **Safety**: Automatic zero-velocity command on timeout
- **Configurable**: ROS 2 parameters for connection, rates, and control modes

## Prerequisites

- ROS 2 Humble or Iron
- Python 3.8+
- MAVSDK-Python (`pip install mavsdk`)
- PX4 SITL running (e.g., via Isaac Sim + Pegasus)

## Build Instructions

```bash
# Navigate to workspace
cd ~/Desktop/quadrotor_ws

# Build the package
colcon build --packages-select px4_offboard_control

# Source the workspace
source install/setup.bash
```

## Running the Node

### Using Launch File (Recommended)

```bash
# Default configuration (connects to udpout://127.0.0.1:14280)
ros2 launch px4_offboard_control offboard.launch.py

# With custom connection URL
ros2 launch px4_offboard_control offboard.launch.py connection_url:=udpout://127.0.0.1:14280

# With custom parameters
ros2 launch px4_offboard_control offboard.launch.py \
    connection_url:=udpout://127.0.0.1:14280 \
    setpoint_rate_hz:=20.0 \
    cmd_timeout_sec:=0.5 \
    yaw_mode:=rate
```

### Direct Node Execution

```bash
ros2 run px4_offboard_control offboard_node --ros-args \
    -p connection_url:=udpout://127.0.0.1:14280 \
    -p setpoint_rate_hz:=20.0
```

## ROS 2 Interfaces

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands (primary control) |
| `/cmd_accel` | `geometry_msgs/Vector3Stamped` | Acceleration commands (optional) |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/telemetry/position` | `geometry_msgs/PoseStamped` | Current position (NED->ROS converted) |
| `/telemetry/velocity` | `geometry_msgs/TwistStamped` | Current velocity (NED->ROS converted) |
| `/telemetry/odom` | `nav_msgs/Odometry` | Combined odometry |
| `/telemetry/state` | `std_msgs/String` | Drone state (connected/armed/offboard) |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/arm` | `std_srvs/SetBool` | Arm (true) or disarm (false) |
| `/offboard_start` | `std_srvs/Trigger` | Start offboard mode |
| `/offboard_stop` | `std_srvs/Trigger` | Stop offboard mode |
| `/land` | `std_srvs/Trigger` | Land the drone |

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `drone_id` | `0` | Drone ID for namespacing (0 = no namespace) |
| `port` | `14280` | MAVLink UDP port |
| `host` | `127.0.0.1` | MAVLink host address |
| `namespace` | `` | Custom namespace (overrides drone_id) |
| `setpoint_rate_hz` | `20.0` | Setpoint publishing rate (Hz) |
| `cmd_timeout_sec` | `0.5` | Command timeout (seconds) |
| `takeoff_altitude_m` | `1.5` | Default takeoff altitude (meters) |
| `yaw_mode` | `rate` | Yaw control: `rate` or `angle` |
| `control_mode` | `velocity` | Control mode: `velocity` or `acceleration` |
| `reconnect_interval_sec` | `2.0` | Auto-reconnect interval (seconds) |

## Coordinate Frame Mapping

### /cmd_vel to NED Velocity

| cmd_vel Field | NED Mapping | Description |
|---------------|-------------|-------------|
| `linear.x` | North (m/s) | Forward velocity |
| `linear.y` | East (m/s) | Right velocity |
| `linear.z` | -Down (m/s) | Up velocity (negated internally) |
| `angular.z` | Yaw rate (deg/s) | Rotation rate |

### /cmd_accel to NED Acceleration

| cmd_accel Field | NED Mapping | Description |
|-----------------|-------------|-------------|
| `vector.x` | North (m/s²) | Forward acceleration |
| `vector.y` | East (m/s²) | Right acceleration |
| `vector.z` | -Down (m/s²) | Up acceleration (negated internally) |

## Example Usage

### 1. Start the Node

```bash
# Terminal 1: Start the offboard control node
ros2 launch px4_offboard_control offboard.launch.py
```

### 2. Arm the Drone

```bash
# Terminal 2: Arm the drone
ros2 service call /arm std_srvs/srv/SetBool "{data: true}"
```

### 3. Start Offboard Mode

```bash
# Start offboard mode (drone will hover)
ros2 service call /offboard_start std_srvs/srv/Trigger
```

### 4. Send Velocity Commands

```bash
# Move forward at 1 m/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}" -r 10

# Move up at 0.5 m/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.5}, angular: {z: 0.0}}" -r 10

# Rotate (yaw) at 15 deg/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {z: 15.0}}" -r 10

# Move forward-right while ascending
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.5, z: 0.3}, angular: {z: 0.0}}" -r 10
```

### 5. Stop and Land

```bash
# Stop offboard mode
ros2 service call /offboard_stop std_srvs/srv/Trigger

# Land
ros2 service call /land std_srvs/srv/Trigger

# Disarm (after landing)
ros2 service call /arm std_srvs/srv/SetBool "{data: false}"
```

### 6. Monitor Telemetry

```bash
# View position
ros2 topic echo /telemetry/position

# View velocity
ros2 topic echo /telemetry/velocity

# View state
ros2 topic echo /telemetry/state

# View combined odometry
ros2 topic echo /telemetry/odom
```

## Multi-Drone Setup

This package supports controlling multiple UAVs simultaneously. Each drone needs:
1. A unique MAVLink port (configured in Pegasus/Isaac Sim)
2. A unique drone_id (for automatic namespace) or custom namespace
3. Each controller uses a unique MAVSDK gRPC port (50051 + drone_id) internally

### Launch Multiple Drones

```bash
# Terminal 1: Drone 0 (default, no namespace)
ros2 launch px4_offboard_control offboard.launch.py drone_id:=0 port:=14280

# Terminal 2: Drone 1 (namespace: /drone1)
ros2 launch px4_offboard_control offboard.launch.py drone_id:=1 port:=14281

# Terminal 3: Drone 2 (namespace: /drone2)
ros2 launch px4_offboard_control offboard.launch.py drone_id:=2 port:=14282
```

### Multi-UAV Control - Complete Workflow

#### Step 1: Launch Controllers (separate terminals)

```bash
# Terminal 1 - UAV0 Controller
ros2 launch px4_offboard_control offboard.launch.py drone_id:=0 port:=14280

# Terminal 2 - UAV1 Controller
ros2 launch px4_offboard_control offboard.launch.py drone_id:=1 port:=14281
```

#### Step 2: Arm Both Drones

```bash
# Arm UAV0
ros2 service call /arm std_srvs/srv/SetBool "{data: true}"

# Arm UAV1
ros2 service call /drone1/arm std_srvs/srv/SetBool "{data: true}"
```

#### Step 3: Start Offboard Mode

```bash
# Start offboard on UAV0
ros2 service call /offboard_start std_srvs/srv/Trigger

# Start offboard on UAV1
ros2 service call /drone1/offboard_start std_srvs/srv/Trigger
```

#### Step 4: Send Velocity Commands

The arm and offboard mode should start one by one (finish the arm and offboard process in two terminals, and then can control the two drones in one terminal), when arm, the rotor starts to rotate, you should offboard start quickly and give the ascend command before the rotors stop.

```bash
# UAV0: Ascend at 0.5 m/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {z: 0.5}}" -r 10

# UAV1: Ascend at 0.5 m/s (separate terminal)
ros2 topic pub /drone1/cmd_vel geometry_msgs/msg/Twist "{linear: {z: 0.5}}" -r 10

# UAV0: Move forward at 1 m/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}}" -r 10

# UAV1: Move forward at 1 m/s
ros2 topic pub /drone1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}}" -r 10

# UAV0: Move right at 0.5 m/s while ascending
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {y: 0.5, z: 0.3}}" -r 10

# UAV1: Move left at 0.5 m/s while ascending
ros2 topic pub /drone1/cmd_vel geometry_msgs/msg/Twist "{linear: {y: -0.5, z: 0.3}}" -r 10

# UAV0: Rotate (yaw) at 15 deg/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 15.0}}" -r 10

# UAV1: Rotate (yaw) at -15 deg/s (opposite direction)
ros2 topic pub /drone1/cmd_vel geometry_msgs/msg/Twist "{angular: {z: -15.0}}" -r 10
```

#### Step 5: Land Both Drones

```bash
# Land UAV0
ros2 service call /land std_srvs/srv/Trigger

# Land UAV1
ros2 service call /drone1/land std_srvs/srv/Trigger
```

#### Step 6: Disarm (after landing)

```bash
# Disarm UAV0
ros2 service call /arm std_srvs/srv/SetBool "{data: false}"

# Disarm UAV1
ros2 service call /drone1/arm std_srvs/srv/SetBool "{data: false}"
```

### Multi-UAV Quick Reference

| Action | UAV0 Command | UAV1 Command |
|--------|--------------|--------------|
| Arm | `ros2 service call /arm std_srvs/srv/SetBool "{data: true}"` | `ros2 service call /drone1/arm std_srvs/srv/SetBool "{data: true}"` |
| Disarm | `ros2 service call /arm std_srvs/srv/SetBool "{data: false}"` | `ros2 service call /drone1/arm std_srvs/srv/SetBool "{data: false}"` |
| Offboard Start | `ros2 service call /offboard_start std_srvs/srv/Trigger` | `ros2 service call /drone1/offboard_start std_srvs/srv/Trigger` |
| Offboard Stop | `ros2 service call /offboard_stop std_srvs/srv/Trigger` | `ros2 service call /drone1/offboard_stop std_srvs/srv/Trigger` |
| Land | `ros2 service call /land std_srvs/srv/Trigger` | `ros2 service call /drone1/land std_srvs/srv/Trigger` |
| Ascend | `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {z: 0.5}}" -r 10` | `ros2 topic pub /drone1/cmd_vel geometry_msgs/msg/Twist "{linear: {z: 0.5}}" -r 10` |
| Forward | `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}}" -r 10` | `ros2 topic pub /drone1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}}" -r 10` |
| Hover | `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{}" -r 10` | `ros2 topic pub /drone1/cmd_vel geometry_msgs/msg/Twist "{}" -r 10` |

### Monitor Multiple Drones

```bash
# UAV0 telemetry
ros2 topic echo /telemetry/position
ros2 topic echo /telemetry/velocity
ros2 topic echo /telemetry/state

# UAV1 telemetry
ros2 topic echo /drone1/telemetry/position
ros2 topic echo /drone1/telemetry/velocity
ros2 topic echo /drone1/telemetry/state

# List all topics (see both drones)
ros2 topic list | grep -E "(cmd_vel|telemetry)"

# List all services (see both drones)
ros2 service list | grep -E "(arm|offboard|land)"
```

### Custom Namespace

```bash
# Use custom namespace instead of drone_id
ros2 launch px4_offboard_control offboard.launch.py namespace:=leader port:=14280
ros2 launch px4_offboard_control offboard.launch.py namespace:=follower1 port:=14281

# Control with custom namespace
ros2 service call /leader/arm std_srvs/srv/SetBool "{data: true}"
ros2 service call /follower1/arm std_srvs/srv/SetBool "{data: true}"
ros2 service call /leader/offboard_start std_srvs/srv/Trigger
ros2 service call /follower1/offboard_start std_srvs/srv/Trigger
```

### Topic/Service Naming

| drone_id | namespace | Topics/Services |
|----------|-----------|-----------------|
| 0 | (none) | `/cmd_vel`, `/arm`, `/telemetry/...` |
| 1 | drone1 | `/drone1/cmd_vel`, `/drone1/arm`, ... |
| 2 | drone2 | `/drone2/cmd_vel`, `/drone2/arm`, ... |
| - | custom | `/custom/cmd_vel`, `/custom/arm`, ... |

### Internal Port Allocation

Each drone controller uses unique internal ports to avoid conflicts:

| drone_id | MAVLink Port | MAVSDK gRPC Port |
|----------|--------------|------------------|
| 0 | 14280 | 50051 |
| 1 | 14281 | 50052 |
| 2 | 14282 | 50053 |

## Acceleration Mode (Optional)

To use acceleration control instead of velocity:

```bash
# Send acceleration command (m/s²)
ros2 topic pub /cmd_accel geometry_msgs/msg/Vector3Stamped \
    "{header: {frame_id: 'base_link'}, vector: {x: 0.5, y: 0.0, z: 0.0}}" -r 10
```

Note: Acceleration mode requires PX4 support and may not work in all configurations. The node will fall back to velocity mode if acceleration setpoints fail.

## Safety Features

1. **Command Timeout**: If no velocity command is received for `cmd_timeout_sec`, the node automatically sends zero velocity to prevent runaway behavior.

2. **Initial Setpoint**: The node sends initial setpoints before starting offboard mode, as required by PX4.

3. **Continuous Setpoints**: Setpoints are published at `setpoint_rate_hz` (default 20 Hz) to maintain offboard mode.

4. **Connection Monitoring**: The node waits for drone connection and health status before enabling control.

5. **Auto-Reconnect**: If connection is lost, the node automatically attempts to reconnect every `reconnect_interval_sec` seconds.

6. **Startup Order Independent**: Works regardless of whether you start the controller or PX4 first.

## Troubleshooting

### "Offboard start failed"

- Ensure the drone is armed before starting offboard mode
- Check that PX4 has valid position estimate (GPS or local position)
- Verify the connection URL is correct

### "Connection timeout"

- Verify PX4 SITL is running
- Check the MAVLink port (default 14280 for companion computer)
- Ensure no other application is using the port

### No telemetry data

- Wait for drone connection (check logs)
- Verify PX4 is publishing telemetry data
- Check QoS settings if using different middleware

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    ROS 2 Node (rclpy)                           │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │  Main Thread (ROS 2 Spin)                                │   │
│  │  - Service callbacks                                     │   │
│  │  - Telemetry publisher timer                             │   │
│  │  - cmd_vel subscriber callback                           │   │
│  └─────────────────────────────────────────────────────────┘   │
│                              │                                   │
│                              │ Thread-safe queue/lock            │
│                              ▼                                   │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │  Background Thread (asyncio event loop)                  │   │
│  │  - MAVSDK connection                                     │   │
│  │  - Telemetry monitoring tasks                            │   │
│  │  - Setpoint publishing loop (20 Hz)                      │   │
│  │  - Async service handlers                                │   │
│  └─────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
                              │
                              │ UDP MAVLink
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                    PX4 SITL (Isaac Sim + Pegasus)               │
│                    Port: 14280 (Companion Computer)             │
└─────────────────────────────────────────────────────────────────┘
```

## License

MIT License
