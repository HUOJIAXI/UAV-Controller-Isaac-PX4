#!/usr/bin/env python3
"""
Iris Quadrotor Control Panel
GUI interface for controlling the Iris quadrotor in Isaac Sim
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Imu
import tkinter as tk
from tkinter import ttk
import threading
import numpy as np
import math


def quaternion_to_euler(q):
    """Convert quaternion to euler angles (roll, pitch, yaw)"""
    x, y, z, w = q

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
    """ROS2 node for control panel"""

    def __init__(self):
        super().__init__('control_panel')

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # State subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped, '/drone0/state/pose', self.pose_callback, qos)
        self.twist_sub = self.create_subscription(
            TwistStamped, '/drone0/state/twist', self.twist_callback, qos)

        # Target position publisher
        self.target_pub = self.create_publisher(PoseStamped, '/drone0/target_pose', 10)

        # Arm/disarm publisher
        self.arm_pub = self.create_publisher(Bool, '/drone0/arm', 10)

        # Direct rotor publishers for emergency stop
        self.rotor_pubs = []
        for i in range(4):
            pub = self.create_publisher(Float64, f'/drone0/control/rotor{i}/ref', 10)
            self.rotor_pubs.append(pub)

        # State data
        self.current_position = [0.0, 0.0, 0.0]
        self.current_velocity = [0.0, 0.0, 0.0]
        self.current_attitude = [0.0, 0.0, 0.0]  # roll, pitch, yaw
        self.current_ang_vel = [0.0, 0.0, 0.0]

    def pose_callback(self, msg: PoseStamped):
        self.current_position = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ]
        q = msg.pose.orientation
        self.current_attitude = list(quaternion_to_euler([q.x, q.y, q.z, q.w]))

    def twist_callback(self, msg: TwistStamped):
        self.current_velocity = [
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z
        ]
        self.current_ang_vel = [
            msg.twist.angular.x,
            msg.twist.angular.y,
            msg.twist.angular.z
        ]

    def send_target(self, x, y, z, yaw=0.0):
        """Send target position"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        # Convert yaw to quaternion
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(yaw / 2)
        msg.pose.orientation.w = math.cos(yaw / 2)
        self.target_pub.publish(msg)
        self.get_logger().info(f'Target sent: [{x:.2f}, {y:.2f}, {z:.2f}], yaw={yaw:.2f}')

    def send_arm(self, arm: bool):
        """Send arm/disarm command"""
        msg = Bool()
        msg.data = arm
        self.arm_pub.publish(msg)
        self.get_logger().info(f'Arm command sent: {arm}')

    def emergency_stop(self):
        """Stop all motors immediately"""
        self.send_arm(False)
        for pub in self.rotor_pubs:
            msg = Float64()
            msg.data = 0.0
            pub.publish(msg)
        self.get_logger().warn('EMERGENCY STOP ACTIVATED')


class ControlPanelGUI:
    """Tkinter GUI for control panel"""

    def __init__(self, node: ControlPanelNode):
        self.node = node
        self.armed = False
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 1.0
        self.target_yaw = 0.0

        # Create main window
        self.root = tk.Tk()
        self.root.title("Iris Quadrotor Control Panel")
        self.root.geometry("700x650")
        self.root.configure(bg='#2b2b2b')

        # Style configuration
        self.style = ttk.Style()
        self.style.theme_use('clam')
        self.style.configure('TFrame', background='#2b2b2b')
        self.style.configure('TLabel', background='#2b2b2b', foreground='white', font=('Helvetica', 10))
        self.style.configure('TLabelframe', background='#2b2b2b', foreground='white')
        self.style.configure('TLabelframe.Label', background='#2b2b2b', foreground='#00ff00', font=('Helvetica', 11, 'bold'))
        self.style.configure('TButton', font=('Helvetica', 10))
        self.style.configure('Emergency.TButton', font=('Helvetica', 12, 'bold'))
        self.style.configure('Arm.TButton', font=('Helvetica', 11, 'bold'))

        self._create_widgets()
        self._start_update_loop()

    def _create_widgets(self):
        """Create all GUI widgets"""
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)

        # === Status Display ===
        status_frame = ttk.LabelFrame(main_frame, text="Status", padding="10")
        status_frame.pack(fill=tk.X, pady=5)

        # Position
        pos_frame = ttk.Frame(status_frame)
        pos_frame.pack(fill=tk.X, pady=2)
        ttk.Label(pos_frame, text="Position:", width=12).pack(side=tk.LEFT)
        self.pos_label = ttk.Label(pos_frame, text="X: 0.00  Y: 0.00  Z: 0.00", font=('Courier', 10))
        self.pos_label.pack(side=tk.LEFT, padx=10)

        # Velocity
        vel_frame = ttk.Frame(status_frame)
        vel_frame.pack(fill=tk.X, pady=2)
        ttk.Label(vel_frame, text="Velocity:", width=12).pack(side=tk.LEFT)
        self.vel_label = ttk.Label(vel_frame, text="Vx: 0.00  Vy: 0.00  Vz: 0.00", font=('Courier', 10))
        self.vel_label.pack(side=tk.LEFT, padx=10)

        # Attitude
        att_frame = ttk.Frame(status_frame)
        att_frame.pack(fill=tk.X, pady=2)
        ttk.Label(att_frame, text="Attitude:", width=12).pack(side=tk.LEFT)
        self.att_label = ttk.Label(att_frame, text="R: 0.00  P: 0.00  Y: 0.00", font=('Courier', 10))
        self.att_label.pack(side=tk.LEFT, padx=10)

        # === Arm/Disarm Section ===
        arm_frame = ttk.LabelFrame(main_frame, text="Control", padding="10")
        arm_frame.pack(fill=tk.X, pady=5)

        arm_btn_frame = ttk.Frame(arm_frame)
        arm_btn_frame.pack(fill=tk.X)

        self.arm_btn = tk.Button(
            arm_btn_frame, text="ARM", command=self.toggle_arm,
            bg='#006400', fg='white', font=('Helvetica', 12, 'bold'),
            width=15, height=2
        )
        self.arm_btn.pack(side=tk.LEFT, padx=5)

        self.arm_status_label = ttk.Label(arm_btn_frame, text="DISARMED", foreground='red', font=('Helvetica', 14, 'bold'))
        self.arm_status_label.pack(side=tk.LEFT, padx=20)

        # === Target Position ===
        target_frame = ttk.LabelFrame(main_frame, text="Target Position", padding="10")
        target_frame.pack(fill=tk.X, pady=5)

        # X position
        x_frame = ttk.Frame(target_frame)
        x_frame.pack(fill=tk.X, pady=3)
        ttk.Label(x_frame, text="X:", width=5).pack(side=tk.LEFT)
        self.x_var = tk.DoubleVar(value=0.0)
        self.x_scale = ttk.Scale(x_frame, from_=-5.0, to=5.0, variable=self.x_var, orient=tk.HORIZONTAL, length=300)
        self.x_scale.pack(side=tk.LEFT, padx=5)
        self.x_entry = ttk.Entry(x_frame, textvariable=self.x_var, width=8)
        self.x_entry.pack(side=tk.LEFT, padx=5)
        ttk.Label(x_frame, text="m").pack(side=tk.LEFT)

        # Y position
        y_frame = ttk.Frame(target_frame)
        y_frame.pack(fill=tk.X, pady=3)
        ttk.Label(y_frame, text="Y:", width=5).pack(side=tk.LEFT)
        self.y_var = tk.DoubleVar(value=0.0)
        self.y_scale = ttk.Scale(y_frame, from_=-5.0, to=5.0, variable=self.y_var, orient=tk.HORIZONTAL, length=300)
        self.y_scale.pack(side=tk.LEFT, padx=5)
        self.y_entry = ttk.Entry(y_frame, textvariable=self.y_var, width=8)
        self.y_entry.pack(side=tk.LEFT, padx=5)
        ttk.Label(y_frame, text="m").pack(side=tk.LEFT)

        # Z position (altitude)
        z_frame = ttk.Frame(target_frame)
        z_frame.pack(fill=tk.X, pady=3)
        ttk.Label(z_frame, text="Z:", width=5).pack(side=tk.LEFT)
        self.z_var = tk.DoubleVar(value=1.0)
        self.z_scale = ttk.Scale(z_frame, from_=0.0, to=5.0, variable=self.z_var, orient=tk.HORIZONTAL, length=300)
        self.z_scale.pack(side=tk.LEFT, padx=5)
        self.z_entry = ttk.Entry(z_frame, textvariable=self.z_var, width=8)
        self.z_entry.pack(side=tk.LEFT, padx=5)
        ttk.Label(z_frame, text="m").pack(side=tk.LEFT)

        # Yaw
        yaw_frame = ttk.Frame(target_frame)
        yaw_frame.pack(fill=tk.X, pady=3)
        ttk.Label(yaw_frame, text="Yaw:", width=5).pack(side=tk.LEFT)
        self.yaw_var = tk.DoubleVar(value=0.0)
        self.yaw_scale = ttk.Scale(yaw_frame, from_=-180.0, to=180.0, variable=self.yaw_var, orient=tk.HORIZONTAL, length=300)
        self.yaw_scale.pack(side=tk.LEFT, padx=5)
        self.yaw_entry = ttk.Entry(yaw_frame, textvariable=self.yaw_var, width=8)
        self.yaw_entry.pack(side=tk.LEFT, padx=5)
        ttk.Label(yaw_frame, text="deg").pack(side=tk.LEFT)

        # Send target button
        send_btn = tk.Button(
            target_frame, text="Send Target", command=self.send_target,
            bg='#1e90ff', fg='white', font=('Helvetica', 11, 'bold'),
            width=15, height=1
        )
        send_btn.pack(pady=10)

        # === Quick Actions ===
        quick_frame = ttk.LabelFrame(main_frame, text="Quick Actions", padding="10")
        quick_frame.pack(fill=tk.X, pady=5)

        quick_btn_frame = ttk.Frame(quick_frame)
        quick_btn_frame.pack(fill=tk.X)

        tk.Button(
            quick_btn_frame, text="Takeoff (1m)", command=lambda: self.quick_command(0, 0, 1),
            bg='#228b22', fg='white', font=('Helvetica', 10), width=12
        ).pack(side=tk.LEFT, padx=3)

        tk.Button(
            quick_btn_frame, text="Takeoff (2m)", command=lambda: self.quick_command(0, 0, 2),
            bg='#228b22', fg='white', font=('Helvetica', 10), width=12
        ).pack(side=tk.LEFT, padx=3)

        tk.Button(
            quick_btn_frame, text="Land", command=lambda: self.quick_command(
                self.node.current_position[0],
                self.node.current_position[1], 0.1),
            bg='#ff8c00', fg='white', font=('Helvetica', 10), width=12
        ).pack(side=tk.LEFT, padx=3)

        tk.Button(
            quick_btn_frame, text="Hold Position", command=self.hold_position,
            bg='#4682b4', fg='white', font=('Helvetica', 10), width=12
        ).pack(side=tk.LEFT, padx=3)

        # Direction buttons
        dir_frame = ttk.Frame(quick_frame)
        dir_frame.pack(fill=tk.X, pady=10)

        ttk.Label(dir_frame, text="Move (0.5m step):").pack(side=tk.LEFT, padx=5)

        tk.Button(
            dir_frame, text="Forward", command=lambda: self.move_relative(0.5, 0, 0),
            bg='#696969', fg='white', width=8
        ).pack(side=tk.LEFT, padx=2)

        tk.Button(
            dir_frame, text="Back", command=lambda: self.move_relative(-0.5, 0, 0),
            bg='#696969', fg='white', width=8
        ).pack(side=tk.LEFT, padx=2)

        tk.Button(
            dir_frame, text="Left", command=lambda: self.move_relative(0, 0.5, 0),
            bg='#696969', fg='white', width=8
        ).pack(side=tk.LEFT, padx=2)

        tk.Button(
            dir_frame, text="Right", command=lambda: self.move_relative(0, -0.5, 0),
            bg='#696969', fg='white', width=8
        ).pack(side=tk.LEFT, padx=2)

        tk.Button(
            dir_frame, text="Up", command=lambda: self.move_relative(0, 0, 0.5),
            bg='#696969', fg='white', width=8
        ).pack(side=tk.LEFT, padx=2)

        tk.Button(
            dir_frame, text="Down", command=lambda: self.move_relative(0, 0, -0.5),
            bg='#696969', fg='white', width=8
        ).pack(side=tk.LEFT, padx=2)

        # === Emergency Stop ===
        emergency_frame = ttk.Frame(main_frame)
        emergency_frame.pack(fill=tk.X, pady=10)

        self.emergency_btn = tk.Button(
            emergency_frame, text="EMERGENCY STOP", command=self.emergency_stop,
            bg='#dc143c', fg='white', font=('Helvetica', 14, 'bold'),
            width=30, height=2
        )
        self.emergency_btn.pack()

        # === Info ===
        info_frame = ttk.Frame(main_frame)
        info_frame.pack(fill=tk.X, pady=5)
        ttk.Label(
            info_frame,
            text="Use the controller node: ros2 run iris_controller controller_node",
            foreground='#888888'
        ).pack()

    def _start_update_loop(self):
        """Start the GUI update loop"""
        self._update_display()

    def _update_display(self):
        """Update display with current state"""
        pos = self.node.current_position
        vel = self.node.current_velocity
        att = [math.degrees(a) for a in self.node.current_attitude]

        self.pos_label.config(text=f"X: {pos[0]:7.3f}  Y: {pos[1]:7.3f}  Z: {pos[2]:7.3f}")
        self.vel_label.config(text=f"Vx: {vel[0]:6.3f}  Vy: {vel[1]:6.3f}  Vz: {vel[2]:6.3f}")
        self.att_label.config(text=f"R: {att[0]:6.2f}  P: {att[1]:6.2f}  Y: {att[2]:6.2f}")

        # Schedule next update
        self.root.after(50, self._update_display)

    def toggle_arm(self):
        """Toggle arm/disarm state"""
        self.armed = not self.armed
        if self.armed:
            self.arm_btn.config(text="DISARM", bg='#8b0000')
            self.arm_status_label.config(text="ARMED", foreground='#00ff00')
            # Arm the controller
            self.node.send_arm(True)
            # Send initial hover target
            self.node.send_target(0.0, 0.0, 1.0, 0.0)
        else:
            self.arm_btn.config(text="ARM", bg='#006400')
            self.arm_status_label.config(text="DISARMED", foreground='red')
            self.node.send_arm(False)
            self.node.emergency_stop()

    def send_target(self):
        """Send target position from sliders"""
        if not self.armed:
            return
        x = self.x_var.get()
        y = self.y_var.get()
        z = self.z_var.get()
        yaw = math.radians(self.yaw_var.get())
        self.node.send_target(x, y, z, yaw)

    def quick_command(self, x, y, z, yaw=0.0):
        """Send quick command"""
        if not self.armed:
            return
        self.x_var.set(x)
        self.y_var.set(y)
        self.z_var.set(z)
        self.yaw_var.set(math.degrees(yaw))
        self.node.send_target(x, y, z, yaw)

    def hold_position(self):
        """Hold current position"""
        if not self.armed:
            return
        pos = self.node.current_position
        self.x_var.set(pos[0])
        self.y_var.set(pos[1])
        self.z_var.set(max(pos[2], 0.5))  # At least 0.5m altitude
        self.node.send_target(pos[0], pos[1], max(pos[2], 0.5), 0.0)

    def move_relative(self, dx, dy, dz):
        """Move relative to current target"""
        if not self.armed:
            return
        x = self.x_var.get() + dx
        y = self.y_var.get() + dy
        z = max(self.z_var.get() + dz, 0.1)  # Minimum altitude
        self.x_var.set(x)
        self.y_var.set(y)
        self.z_var.set(z)
        self.node.send_target(x, y, z, math.radians(self.yaw_var.get()))

    def emergency_stop(self):
        """Emergency stop"""
        self.armed = False
        self.arm_btn.config(text="ARM", bg='#006400')
        self.arm_status_label.config(text="DISARMED", foreground='red')
        self.node.emergency_stop()

    def run(self):
        """Run the GUI main loop"""
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = ControlPanelNode()

    # Spin ROS in a separate thread
    spin_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()

    # Create and run GUI
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
