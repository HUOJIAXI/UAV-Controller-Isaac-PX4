#!/usr/bin/env python3
"""
UAV Position Controller - Control Panel.

Tkinter GUI for controlling the UAV position controller via offboard_node.
Uses ROS 2 services for arm/offboard/land and publishes target pose to
the position controller.
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

import tkinter as tk
from tkinter import ttk


def quaternion_to_euler(x, y, z, w):
    """Convert quaternion to euler angles (roll, pitch, yaw) in radians."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class ControlPanelNode(Node):
    """ROS 2 node for the control panel."""

    def __init__(self):
        super().__init__('uav_control_panel')

        # State subscribers
        self.odom_sub = self.create_subscription(
            Odometry, 'telemetry/odom', self._odom_callback, 10)
        self.state_sub = self.create_subscription(
            String, 'telemetry/state', self._state_callback, 10)
        self.mode_sub = self.create_subscription(
            String, 'control_mode', self._mode_callback, 10)
        self.ctrl_status_sub = self.create_subscription(
            String, 'controller/status', self._ctrl_status_callback, 10)

        # Target publisher
        self.target_pub = self.create_publisher(
            PoseStamped, 'target_pose', 10)

        # Service clients
        self.arm_client = self.create_client(SetBool, 'arm')
        self.offboard_start_client = self.create_client(
            Trigger, 'offboard_start')
        self.offboard_stop_client = self.create_client(
            Trigger, 'offboard_stop')
        self.land_client = self.create_client(Trigger, 'land')
        self.set_attitude_mode_client = self.create_client(
            Trigger, 'set_attitude_mode')
        self.controller_enable_client = self.create_client(
            SetBool, 'controller/enable')

        # State data
        self.current_position = [0.0, 0.0, 0.0]
        self.current_velocity = [0.0, 0.0, 0.0]
        self.current_attitude = [0.0, 0.0, 0.0]  # roll, pitch, yaw (rad)
        self.drone_state = 'disconnected'
        self.control_mode = 'velocity'
        self.controller_status = 'disabled'
        self.last_service_msg = ''

    def _odom_callback(self, msg):
        """Handle odometry."""
        self.current_position = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z]
        self.current_velocity = [
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z]
        q = msg.pose.pose.orientation
        self.current_attitude = list(
            quaternion_to_euler(q.x, q.y, q.z, q.w))

    def _state_callback(self, msg):
        """Handle drone state."""
        self.drone_state = msg.data

    def _mode_callback(self, msg):
        """Handle control mode."""
        self.control_mode = msg.data

    def _ctrl_status_callback(self, msg):
        """Handle controller status."""
        self.controller_status = msg.data

    def send_target(self, right_body, fwd_body, dz, yaw=0.0):
        """
        Publish target pose using body-frame offsets (user: forward/right).

        Internally use aircraft FLU axes (x=fwd, y=left). User "Right" is -body Y.
        Vertical input dz is relative to current altitude (body Up).
        """
        cur_x, cur_y, cur_z = self.current_position
        yaw_curr = self.current_attitude[2]  # ENU yaw, CCW about +Z

        # Map user offsets to FLU: x=fwd, y=left
        x_fwd = fwd_body
        y_left = -right_body

        cos_y = math.cos(yaw_curr)
        sin_y = math.sin(yaw_curr)
        # body->world: world = Rz(yaw) * body
        dx = x_fwd * cos_y - y_left * sin_y
        dy = x_fwd * sin_y + y_left * cos_y

        target_x = cur_x + dx
        target_y = cur_y + dy
        target_z = cur_z + dz

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(target_x)
        msg.pose.position.y = float(target_y)
        msg.pose.position.z = float(target_z)
        msg.pose.orientation.z = math.sin(yaw / 2)
        msg.pose.orientation.w = math.cos(yaw / 2)
        self.target_pub.publish(msg)
        self.get_logger().info(
            f'Target body offsets: fwd {fwd_body:.2f} m, right {right_body:.2f} m, up {dz:.2f} m, '
            f'world: [{target_x:.2f}, {target_y:.2f}, {target_z:.2f}], '
            f'yaw_cmd={math.degrees(yaw):.1f} deg')

    def call_trigger_service(self, client, name):
        """Call a Trigger service (non-blocking from GUI thread)."""
        if not client.wait_for_service(timeout_sec=1.0):
            self.last_service_msg = f'{name}: service not available'
            self.get_logger().warn(self.last_service_msg)
            return False

        request = Trigger.Request()
        future = client.call_async(request)
        # Wait for result in this (background) thread
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is not None:
            result = future.result()
            self.last_service_msg = f'{name}: {result.message}'
            self.get_logger().info(self.last_service_msg)
            return result.success
        else:
            self.last_service_msg = f'{name}: timeout'
            self.get_logger().warn(self.last_service_msg)
            return False

    def call_setbool_service(self, client, value, name):
        """Call a SetBool service (non-blocking from GUI thread)."""
        if not client.wait_for_service(timeout_sec=1.0):
            self.last_service_msg = f'{name}: service not available'
            self.get_logger().warn(self.last_service_msg)
            return False

        request = SetBool.Request()
        request.data = value
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is not None:
            result = future.result()
            self.last_service_msg = f'{name}: {result.message}'
            self.get_logger().info(self.last_service_msg)
            return result.success
        else:
            self.last_service_msg = f'{name}: timeout'
            self.get_logger().warn(self.last_service_msg)
            return False


class ControlPanelGUI:
    """Tkinter GUI for UAV position controller."""

    def __init__(self, node):
        self.node = node
        self.startup_running = False

        # Create main window
        self.root = tk.Tk()
        self.root.title('UAV Position Controller Panel')
        self.root.geometry('720x720')
        self.root.configure(bg='#2b2b2b')

        # Style
        self.style = ttk.Style()
        self.style.theme_use('clam')
        self.style.configure(
            'TFrame', background='#2b2b2b')
        self.style.configure(
            'TLabel', background='#2b2b2b', foreground='white',
            font=('Helvetica', 10))
        self.style.configure(
            'TLabelframe', background='#2b2b2b', foreground='white')
        self.style.configure(
            'TLabelframe.Label', background='#2b2b2b',
            foreground='#00ff00', font=('Helvetica', 11, 'bold'))

        self._create_widgets()
        self._start_update_loop()

    def _create_widgets(self):
        """Create all GUI widgets."""
        main_frame = ttk.Frame(self.root, padding='10')
        main_frame.pack(fill=tk.BOTH, expand=True)

        # === Status Display ===
        status_frame = ttk.LabelFrame(
            main_frame, text='Status', padding='10')
        status_frame.pack(fill=tk.X, pady=5)

        # Drone state
        state_frame = ttk.Frame(status_frame)
        state_frame.pack(fill=tk.X, pady=2)
        ttk.Label(state_frame, text='Drone:', width=12).pack(side=tk.LEFT)
        self.state_label = ttk.Label(
            state_frame, text='disconnected',
            foreground='red', font=('Courier', 10))
        self.state_label.pack(side=tk.LEFT, padx=10)

        # Control mode + controller status
        mode_frame = ttk.Frame(status_frame)
        mode_frame.pack(fill=tk.X, pady=2)
        ttk.Label(mode_frame, text='Mode:', width=12).pack(side=tk.LEFT)
        self.mode_label = ttk.Label(
            mode_frame, text='velocity | controller: disabled',
            font=('Courier', 10))
        self.mode_label.pack(side=tk.LEFT, padx=10)

        # Position
        pos_frame = ttk.Frame(status_frame)
        pos_frame.pack(fill=tk.X, pady=2)
        ttk.Label(pos_frame, text='Position:', width=12).pack(side=tk.LEFT)
        self.pos_label = ttk.Label(
            pos_frame,
            text='East:   0.000  North:   0.000  Up:   0.000',
            font=('Courier', 10))
        self.pos_label.pack(side=tk.LEFT, padx=10)

        # Velocity
        vel_frame = ttk.Frame(status_frame)
        vel_frame.pack(fill=tk.X, pady=2)
        ttk.Label(vel_frame, text='Velocity:', width=12).pack(side=tk.LEFT)
        self.vel_label = ttk.Label(
            vel_frame,
            text='Vfwd(body): 0.000  Vright(body): 0.000  Vup: 0.000',
            font=('Courier', 10))
        self.vel_label.pack(side=tk.LEFT, padx=10)

        # Attitude
        att_frame = ttk.Frame(status_frame)
        att_frame.pack(fill=tk.X, pady=2)
        ttk.Label(att_frame, text='Attitude:', width=12).pack(side=tk.LEFT)
        self.att_label = ttk.Label(
            att_frame, text='R:  0.00   P:  0.00   Y:  0.00  deg',
            font=('Courier', 10))
        self.att_label.pack(side=tk.LEFT, padx=10)

        # === Drone Control ===
        ctrl_frame = ttk.LabelFrame(
            main_frame, text='Drone Control', padding='10')
        ctrl_frame.pack(fill=tk.X, pady=5)

        btn_frame = ttk.Frame(ctrl_frame)
        btn_frame.pack(fill=tk.X)

        self.start_btn = tk.Button(
            btn_frame, text='ARM & START',
            command=self._on_start,
            bg='#006400', fg='white',
            font=('Helvetica', 11, 'bold'), width=14, height=2)
        self.start_btn.pack(side=tk.LEFT, padx=3)

        tk.Button(
            btn_frame, text='OFFBOARD STOP',
            command=self._on_offboard_stop,
            bg='#b8860b', fg='white',
            font=('Helvetica', 10), width=14, height=2
        ).pack(side=tk.LEFT, padx=3)

        tk.Button(
            btn_frame, text='LAND',
            command=self._on_land,
            bg='#ff8c00', fg='white',
            font=('Helvetica', 10), width=14, height=2
        ).pack(side=tk.LEFT, padx=3)

        tk.Button(
            btn_frame, text='DISARM',
            command=self._on_disarm,
            bg='#8b0000', fg='white',
            font=('Helvetica', 10), width=14, height=2
        ).pack(side=tk.LEFT, padx=3)

        # Service feedback
        self.svc_label = ttk.Label(
            ctrl_frame, text='', foreground='#aaaaaa',
            font=('Helvetica', 9))
        self.svc_label.pack(fill=tk.X, pady=3)

        # === Target Position ===
        target_frame = ttk.LabelFrame(
            main_frame, text='Target Position', padding='10')
        target_frame.pack(fill=tk.X, pady=5)

        # Slider rows (body-frame offsets: fwd/right relative to current yaw; Up is delta Z)
        for label, var_name, lo, hi, default, unit in [
            ('Fwd (body):', 'y_var', -5.0, 5.0, 0.0, 'm'),
            ('Right (body):', 'x_var', -5.0, 5.0, 0.0, 'm'),
            ('Up ΔZ (body):', 'z_var', -3.0, 5.0, 0.0, 'm'),
            ('Yaw:', 'yaw_var', -180.0, 180.0, 0.0, 'deg'),
        ]:
            row = ttk.Frame(target_frame)
            row.pack(fill=tk.X, pady=2)
            ttk.Label(row, text=label, width=5).pack(side=tk.LEFT)
            var = tk.DoubleVar(value=default)
            setattr(self, var_name, var)
            ttk.Scale(
                row, from_=lo, to=hi, variable=var,
                orient=tk.HORIZONTAL, length=300
            ).pack(side=tk.LEFT, padx=5)
            ttk.Entry(row, textvariable=var, width=8).pack(
                side=tk.LEFT, padx=5)
            ttk.Label(row, text=unit).pack(side=tk.LEFT)

        # Send / Hold buttons
        tgt_btn_frame = ttk.Frame(target_frame)
        tgt_btn_frame.pack(fill=tk.X, pady=5)

        tk.Button(
            tgt_btn_frame, text='Send Target',
            command=self._on_send_target,
            bg='#1e90ff', fg='white',
            font=('Helvetica', 11, 'bold'), width=14
        ).pack(side=tk.LEFT, padx=5)

        tk.Button(
            tgt_btn_frame, text='Hold Position',
            command=self._on_hold_position,
            bg='#4682b4', fg='white',
            font=('Helvetica', 10), width=14
        ).pack(side=tk.LEFT, padx=5)

        # === Quick Actions ===
        quick_frame = ttk.LabelFrame(
            main_frame, text='Quick Actions', padding='10')
        quick_frame.pack(fill=tk.X, pady=5)

        row1 = ttk.Frame(quick_frame)
        row1.pack(fill=tk.X, pady=3)

        for text, args in [
            ('Takeoff 1m', (0, 0, 1)),
            ('Takeoff 2m', (0, 0, 2)),
            ('Takeoff 3m', (0, 0, 3)),
        ]:
            tk.Button(
                row1, text=text,
                command=lambda a=args: self._quick_target(*a),
                bg='#228b22', fg='white',
                font=('Helvetica', 10), width=11
            ).pack(side=tk.LEFT, padx=3)

        row2 = ttk.Frame(quick_frame)
        row2.pack(fill=tk.X, pady=3)

        for text, d in [
            # Body-frame deltas: forward/right are relative to current yaw
            ('Fwd 0.5', (0.0, 0.5, 0)),
            ('Back 0.5', (0.0, -0.5, 0)),
            ('Left 0.5', (-0.5, 0.0, 0)),   # left = -right_body
            ('Right 0.5', (0.5, 0.0, 0)),   # right = +right_body
            ('Up 0.5', (0, 0, 0.5)),
            ('Down 0.5', (0, 0, -0.5)),
        ]:
            tk.Button(
                row2, text=text,
                command=lambda delta=d: self._move_relative(*delta),
                bg='#696969', fg='white', width=9
            ).pack(side=tk.LEFT, padx=2)

        row3 = ttk.Frame(quick_frame)
        row3.pack(fill=tk.X, pady=3)

        tk.Button(
            row3, text='Yaw L 15',
            command=lambda: self._yaw_relative(15),   # CCW (left) = +yaw
            bg='#696969', fg='white', width=9
        ).pack(side=tk.LEFT, padx=2)

        tk.Button(
            row3, text='Yaw R 15',
            command=lambda: self._yaw_relative(-15),  # CW (right) = -yaw
            bg='#696969', fg='white', width=9
        ).pack(side=tk.LEFT, padx=2)

        # === Emergency Stop ===
        em_frame = ttk.Frame(main_frame)
        em_frame.pack(fill=tk.X, pady=10)

        tk.Button(
            em_frame, text='EMERGENCY STOP',
            command=self._on_emergency_stop,
            bg='#dc143c', fg='white',
            font=('Helvetica', 14, 'bold'),
            width=30, height=2
        ).pack()

    def _start_update_loop(self):
        """Start periodic display updates."""
        self._update_display()

    def _update_display(self):
        """Update telemetry display."""
        pos = self.node.current_position
        vel = self.node.current_velocity
        att = [math.degrees(a) for a in self.node.current_attitude]

        self.state_label.config(text=self.node.drone_state)
        state_lower = self.node.drone_state.lower()
        if 'offboard' in state_lower:
            self.state_label.config(foreground='#00ff00')
        elif 'armed' in state_lower:
            self.state_label.config(foreground='#ffff00')
        elif 'connected' in state_lower:
            self.state_label.config(foreground='#ffa500')
        else:
            self.state_label.config(foreground='red')

        self.mode_label.config(
            text=f'{self.node.control_mode} | '
                 f'controller: {self.node.controller_status}')

        # Position shown in world ENU to avoid ambiguity
        self.pos_label.config(
            text=f'East: {pos[0]:7.3f}  North: {pos[1]:7.3f}  Up: {pos[2]:7.3f}')

        # Velocities shown in body frame (FLU) for intuitive forward/right readout
        yaw = att[2]  # rad, ENU
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        v_fwd = vel[0] * cos_y + vel[1] * sin_y
        v_right = -vel[0] * sin_y + vel[1] * cos_y
        self.vel_label.config(
            text=f'Vfwd(body): {v_fwd:6.3f}  Vright(body): {v_right:6.3f}  '
                 f'Vup: {vel[2]:6.3f}')
        self.att_label.config(
            text=f'R: {att[0]:6.2f}   P: {att[1]:6.2f}   '
                 f'Y: {att[2]:6.2f}  deg')

        # Update service feedback
        if self.node.last_service_msg:
            self.svc_label.config(text=self.node.last_service_msg)

        self.root.after(50, self._update_display)

    def _on_start(self):
        """ARM & START: arm, set attitude mode, offboard start, enable."""
        if self.startup_running:
            return
        self.startup_running = True
        self.start_btn.config(text='STARTING...', bg='#555555')
        thread = threading.Thread(
            target=self._startup_sequence, daemon=True)
        thread.start()

    def _startup_sequence(self):
        """Run startup sequence in background thread."""
        try:
            # 1. Arm
            ok = self.node.call_setbool_service(
                self.node.arm_client, True, 'Arm')
            if not ok:
                return
            time.sleep(0.5)

            # 2. Switch to ATTITUDE mode
            ok = self.node.call_trigger_service(
                self.node.set_attitude_mode_client, 'Set attitude mode')
            if not ok:
                return
            time.sleep(0.2)

            # 3. Start offboard
            ok = self.node.call_trigger_service(
                self.node.offboard_start_client, 'Offboard start')
            if not ok:
                return
            time.sleep(0.5)

            # 4. Enable controller
            self.node.call_setbool_service(
                self.node.controller_enable_client, True,
                'Enable controller')
        finally:
            self.root.after(0, self._startup_done)

    def _startup_done(self):
        """Reset button after startup sequence."""
        self.startup_running = False
        self.start_btn.config(text='ARM & START', bg='#006400')

    def _on_offboard_stop(self):
        """Stop offboard mode."""
        thread = threading.Thread(target=lambda: (
            self.node.call_setbool_service(
                self.node.controller_enable_client, False,
                'Disable controller'),
            self.node.call_trigger_service(
                self.node.offboard_stop_client, 'Offboard stop'),
        ), daemon=True)
        thread.start()

    def _on_land(self):
        """Land the drone."""
        thread = threading.Thread(target=lambda: (
            self.node.call_setbool_service(
                self.node.controller_enable_client, False,
                'Disable controller'),
            self.node.call_trigger_service(
                self.node.land_client, 'Land'),
        ), daemon=True)
        thread.start()

    def _on_disarm(self):
        """Disarm the drone."""
        thread = threading.Thread(target=lambda: (
            self.node.call_setbool_service(
                self.node.controller_enable_client, False,
                'Disable controller'),
            self.node.call_setbool_service(
                self.node.arm_client, False, 'Disarm'),
        ), daemon=True)
        thread.start()

    def _on_emergency_stop(self):
        """Emergency stop: land + disable + stop offboard."""
        thread = threading.Thread(target=lambda: (
            self.node.call_setbool_service(
                self.node.controller_enable_client, False,
                'Disable controller'),
            self.node.call_trigger_service(
                self.node.land_client, 'Emergency land'),
        ), daemon=True)
        thread.start()

    def _on_send_target(self):
        """Send target from sliders (body-frame offsets, delta Z)."""
        x_body = self.x_var.get()
        y_body = self.y_var.get()
        dz = self.z_var.get()
        yaw = math.radians(self.yaw_var.get())
        self.node.send_target(x_body, y_body, dz, yaw)

    def _on_hold_position(self):
        """Hold current position: zero offsets, keep current yaw, ΔZ=0."""
        att = self.node.current_attitude
        self.x_var.set(0.0)
        self.y_var.set(0.0)
        self.z_var.set(0.0)
        self.yaw_var.set(round(math.degrees(att[2]), 1))
        self.node.send_target(0.0, 0.0, 0.0, att[2])

    def _quick_target(self, x_body, y_body, dz):
        """Send a quick target (body offsets, delta Z)."""
        yaw = math.radians(self.yaw_var.get())
        self._reset_offsets()
        self.x_var.set(x_body)
        self.y_var.set(y_body)
        self.z_var.set(dz)
        self.node.send_target(x_body, y_body, dz, yaw)

    def _move_relative(self, dx_body, dy_body, dz):
        """Quick action: reset offsets then apply body-frame delta (non-accumulating)."""
        self._reset_offsets()
        x = dx_body
        y = dy_body
        z = dz
        self.x_var.set(round(x, 2))
        self.y_var.set(round(y, 2))
        self.z_var.set(round(z, 2))
        yaw = math.radians(self.yaw_var.get())
        self.node.send_target(x, y, z, yaw)

    def _reset_offsets(self):
        """Zero body-frame offset sliders (keep yaw)."""
        self.x_var.set(0.0)
        self.y_var.set(0.0)
        self.z_var.set(0.0)

    def _yaw_relative(self, dyaw_deg):
        """Rotate yaw relative to current target."""
        yaw = self.yaw_var.get() + dyaw_deg
        # Wrap to [-180, 180]
        while yaw > 180:
            yaw -= 360
        while yaw < -180:
            yaw += 360
        self.yaw_var.set(round(yaw, 1))
        x = self.x_var.get()
        y = self.y_var.get()
        z = self.z_var.get()
        self.node.send_target(x, y, z, math.radians(yaw))

    def run(self):
        """Run the GUI main loop."""
        self.root.mainloop()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = ControlPanelNode()

    # Spin ROS in a separate thread
    spin_thread = threading.Thread(
        target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()

    gui = ControlPanelGUI(node)

    try:
        gui.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
