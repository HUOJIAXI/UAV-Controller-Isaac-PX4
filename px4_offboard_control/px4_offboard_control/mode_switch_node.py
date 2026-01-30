#!/usr/bin/env python3
"""
Mode Switch Node for PX4 Offboard Control.

This node provides a higher-level interface for switching between control modes:
- VELOCITY mode: For standard flying operations
- ATTITUDE_RATE mode: For manipulation tasks requiring direct attitude control

Features:
- Unified command interface that routes to appropriate topic based on mode
- Keyboard control for testing
- Automatic mode switching based on external triggers
- Status monitoring and display

Author: Generated for aerial manipulation integration
"""

import sys
import threading
import termios
import tty
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger


class ControlMode(Enum):
    """Control mode enum matching offboard_node."""
    VELOCITY = "velocity"
    ATTITUDE_RATE = "attitude_rate"


class ModeSwitchNode(Node):
    """
    Mode switch node for managing control modes and providing unified command interface.

    This node acts as a bridge between high-level control and the offboard node,
    allowing seamless switching between flying (velocity) and manipulation (attitude rate) modes.
    """

    def __init__(self):
        super().__init__('mode_switch')

        # Parameters
        self.declare_parameter('namespace', '')
        self.declare_parameter('enable_keyboard', True)
        self.declare_parameter('default_thrust', 0.5)  # Hover thrust for attitude mode

        self.namespace = self.get_parameter('namespace').get_parameter_value().string_value
        self.enable_keyboard = self.get_parameter('enable_keyboard').get_parameter_value().bool_value
        self.default_thrust = self.get_parameter('default_thrust').get_parameter_value().double_value

        # Build topic/service prefix
        if self.namespace:
            self.prefix = f'/{self.namespace}'
        else:
            self.prefix = ''

        # Current state
        self._current_mode = ControlMode.VELOCITY
        self._drone_state = "unknown"
        self._is_armed = False
        self._is_offboard = False

        # Subscribers for state monitoring
        self.control_mode_sub = self.create_subscription(
            String,
            f'{self.prefix}/control_mode',
            self._control_mode_callback,
            10
        )
        self.state_sub = self.create_subscription(
            String,
            f'{self.prefix}/telemetry/state',
            self._state_callback,
            10
        )

        # Publishers for commands
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            f'{self.prefix}/cmd_vel',
            10
        )
        self.cmd_attitude_rate_pub = self.create_publisher(
            Twist,
            f'{self.prefix}/cmd_attitude_rate',
            10
        )

        # Unified command subscriber (routes to appropriate topic based on mode)
        self.unified_cmd_sub = self.create_subscription(
            Twist,
            'unified_cmd',
            self._unified_cmd_callback,
            10
        )

        # Service clients
        self.set_mode_client = self.create_client(
            SetBool,
            f'{self.prefix}/set_control_mode'
        )
        self.arm_client = self.create_client(
            SetBool,
            f'{self.prefix}/arm'
        )
        self.offboard_start_client = self.create_client(
            Trigger,
            f'{self.prefix}/offboard_start'
        )
        self.offboard_stop_client = self.create_client(
            Trigger,
            f'{self.prefix}/offboard_stop'
        )
        self.land_client = self.create_client(
            Trigger,
            f'{self.prefix}/land'
        )

        # Services for external mode switching
        self.switch_to_velocity_srv = self.create_service(
            Trigger,
            'switch_to_velocity',
            self._switch_to_velocity_callback
        )
        self.switch_to_attitude_rate_srv = self.create_service(
            Trigger,
            'switch_to_attitude_rate',
            self._switch_to_attitude_rate_callback
        )

        # Status timer
        self.status_timer = self.create_timer(2.0, self._print_status)

        # Keyboard control thread
        self._running = True
        self._keyboard_thread = None
        if self.enable_keyboard:
            self._keyboard_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
            self._keyboard_thread.start()

        self.get_logger().info("Mode Switch Node initialized")
        self.get_logger().info(f"Namespace: '{self.namespace}' (prefix: '{self.prefix}')")
        self._print_help()

    def _control_mode_callback(self, msg: String):
        """Update current control mode from offboard node."""
        try:
            self._current_mode = ControlMode(msg.data)
        except ValueError:
            self.get_logger().warn(f"Unknown control mode: {msg.data}")

    def _state_callback(self, msg: String):
        """Update drone state from offboard node."""
        self._drone_state = msg.data
        self._is_armed = "armed" in msg.data.lower()
        self._is_offboard = "offboard" in msg.data.lower()

    def _unified_cmd_callback(self, msg: Twist):
        """
        Handle unified command and route to appropriate topic.

        In VELOCITY mode: routes to /cmd_vel
        In ATTITUDE_RATE mode: routes to /cmd_attitude_rate
        """
        if self._current_mode == ControlMode.VELOCITY:
            self.cmd_vel_pub.publish(msg)
        else:
            # For attitude rate mode, use linear.z as thrust if not specified
            if msg.linear.z == 0.0:
                msg.linear.z = self.default_thrust
            self.cmd_attitude_rate_pub.publish(msg)

    def _switch_to_velocity_callback(self, request, response):
        """Service callback to switch to velocity mode."""
        success = self._set_control_mode(ControlMode.VELOCITY)
        response.success = success
        response.message = "Switched to VELOCITY mode" if success else "Failed to switch mode"
        return response

    def _switch_to_attitude_rate_callback(self, request, response):
        """Service callback to switch to attitude rate mode."""
        success = self._set_control_mode(ControlMode.ATTITUDE_RATE)
        response.success = success
        response.message = "Switched to ATTITUDE_RATE mode" if success else "Failed to switch mode"
        return response

    def _set_control_mode(self, mode: ControlMode) -> bool:
        """Set control mode via service call to offboard node."""
        if not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("set_control_mode service not available")
            return False

        request = SetBool.Request()
        request.data = (mode == ControlMode.ATTITUDE_RATE)

        future = self.set_mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            result = future.result()
            if result.success:
                self.get_logger().info(f"Control mode set to: {mode.value}")
                return True
            else:
                self.get_logger().error(f"Failed to set mode: {result.message}")
                return False
        else:
            self.get_logger().error("Service call failed")
            return False

    def _call_trigger_service(self, client, service_name: str) -> bool:
        """Helper to call a Trigger service."""
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(f"{service_name} service not available")
            return False

        request = Trigger.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            result = future.result()
            self.get_logger().info(f"{service_name}: {result.message}")
            return result.success
        else:
            self.get_logger().error(f"{service_name} service call failed")
            return False

    def _call_arm_service(self, arm: bool) -> bool:
        """Helper to call arm/disarm service."""
        if not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("arm service not available")
            return False

        request = SetBool.Request()
        request.data = arm
        future = self.arm_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            result = future.result()
            action = "Arm" if arm else "Disarm"
            self.get_logger().info(f"{action}: {result.message}")
            return result.success
        else:
            self.get_logger().error("Arm service call failed")
            return False

    def _print_status(self):
        """Print current status periodically."""
        mode_str = self._current_mode.value.upper()
        self.get_logger().info(
            f"[Status] Mode: {mode_str} | State: {self._drone_state}"
        )

    def _print_help(self):
        """Print keyboard control help."""
        help_text = """
╔══════════════════════════════════════════════════════════════╗
║                   Mode Switch Keyboard Control                ║
╠══════════════════════════════════════════════════════════════╣
║  Mode Switching:                                              ║
║    v - Switch to VELOCITY mode (flying)                       ║
║    m - Switch to ATTITUDE_RATE mode (manipulation)            ║
║                                                               ║
║  Drone Control:                                               ║
║    a - Arm                                                    ║
║    d - Disarm                                                 ║
║    o - Start offboard mode                                    ║
║    p - Stop offboard mode                                     ║
║    l - Land                                                   ║
║                                                               ║
║  Velocity Mode Commands (WASD + QE + RF):                     ║
║    w/s - Forward/Backward        q/e - Yaw left/right         ║
║    a/d - Left/Right              r/f - Up/Down                ║
║                                                               ║
║  Attitude Rate Mode Commands (IJKL + UO + TG):                ║
║    i/k - Pitch forward/back      u/o - Yaw left/right         ║
║    j/l - Roll left/right         t/g - Thrust up/down         ║
║                                                               ║
║    SPACE - Stop all motion                                    ║
║    h - Show this help                                         ║
║    ESC/Ctrl+C - Quit                                          ║
╚══════════════════════════════════════════════════════════════╝
"""
        print(help_text)

    def _keyboard_loop(self):
        """Handle keyboard input in a separate thread."""
        # Velocity command increments
        vel_linear = 0.5  # m/s
        vel_angular = 15.0  # deg/s

        # Attitude rate command increments
        rate_rp = 5.0  # deg/s for roll/pitch rate
        rate_yaw = 10.0  # deg/s for yaw rate
        thrust_inc = 0.05  # thrust increment

        current_thrust = self.default_thrust

        try:
            # Set terminal to raw mode for single character input
            old_settings = termios.tcgetattr(sys.stdin)
            tty.setraw(sys.stdin.fileno())

            while self._running:
                try:
                    ch = sys.stdin.read(1)
                    if not ch:
                        continue

                    # Handle escape and Ctrl+C
                    if ch == '\x1b' or ch == '\x03':  # ESC or Ctrl+C
                        self._running = False
                        break

                    ch = ch.lower()

                    # Mode switching
                    if ch == 'v':
                        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                        self._set_control_mode(ControlMode.VELOCITY)
                        tty.setraw(sys.stdin.fileno())
                    elif ch == 'm':
                        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                        self._set_control_mode(ControlMode.ATTITUDE_RATE)
                        tty.setraw(sys.stdin.fileno())

                    # Drone control
                    elif ch == 'a':
                        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                        self._call_arm_service(True)
                        tty.setraw(sys.stdin.fileno())
                    elif ch == 'd':
                        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                        self._call_arm_service(False)
                        tty.setraw(sys.stdin.fileno())
                    elif ch == 'o':
                        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                        self._call_trigger_service(self.offboard_start_client, "offboard_start")
                        tty.setraw(sys.stdin.fileno())
                    elif ch == 'p':
                        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                        self._call_trigger_service(self.offboard_stop_client, "offboard_stop")
                        tty.setraw(sys.stdin.fileno())
                    elif ch == 'l':
                        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                        self._call_trigger_service(self.land_client, "land")
                        tty.setraw(sys.stdin.fileno())

                    # Help
                    elif ch == 'h':
                        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                        self._print_help()
                        tty.setraw(sys.stdin.fileno())

                    # Stop all motion
                    elif ch == ' ':
                        msg = Twist()
                        if self._current_mode == ControlMode.ATTITUDE_RATE:
                            msg.linear.z = current_thrust
                        self._unified_cmd_callback(msg)

                    # Velocity mode commands (WASD + QE + RF)
                    elif self._current_mode == ControlMode.VELOCITY:
                        msg = Twist()
                        if ch == 'w':
                            msg.linear.x = vel_linear
                        elif ch == 's':
                            msg.linear.x = -vel_linear
                        elif ch == 'a':
                            msg.linear.y = -vel_linear
                        elif ch == 'd':
                            msg.linear.y = vel_linear
                        elif ch == 'r':
                            msg.linear.z = vel_linear
                        elif ch == 'f':
                            msg.linear.z = -vel_linear
                        elif ch == 'q':
                            msg.angular.z = vel_angular
                        elif ch == 'e':
                            msg.angular.z = -vel_angular

                        if any([msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z]):
                            self.cmd_vel_pub.publish(msg)

                    # Attitude rate mode commands (IJKL + UO + TG)
                    elif self._current_mode == ControlMode.ATTITUDE_RATE:
                        msg = Twist()
                        msg.linear.z = current_thrust  # Always include thrust

                        if ch == 'i':
                            msg.angular.y = rate_rp  # Pitch forward (nose down)
                        elif ch == 'k':
                            msg.angular.y = -rate_rp  # Pitch back (nose up)
                        elif ch == 'j':
                            msg.angular.x = -rate_rp  # Roll left
                        elif ch == 'l':
                            msg.angular.x = rate_rp  # Roll right
                        elif ch == 'u':
                            msg.angular.z = rate_yaw  # Yaw left
                        elif ch == 'o':
                            msg.angular.z = -rate_yaw  # Yaw right
                        elif ch == 't':
                            current_thrust = min(1.0, current_thrust + thrust_inc)
                            msg.linear.z = current_thrust
                        elif ch == 'g':
                            current_thrust = max(0.0, current_thrust - thrust_inc)
                            msg.linear.z = current_thrust

                        self.cmd_attitude_rate_pub.publish(msg)

                except Exception as e:
                    pass  # Ignore read errors

        except Exception as e:
            self.get_logger().error(f"Keyboard control error: {e}")
        finally:
            # Restore terminal settings
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            except Exception:
                pass

    def destroy_node(self):
        """Clean up on node destruction."""
        self._running = False
        if self._keyboard_thread is not None:
            self._keyboard_thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    node = ModeSwitchNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
