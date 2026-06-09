import os
import sys
import time
import math as m
import traceback
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped, Quaternion, Vector3, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu
from transforms3d.euler import quat2euler
from transforms3d.quaternions import mat2quat
from transforms3d.euler import euler2mat

from mavros_msgs.msg import State, AttitudeTarget, RCIn
from mavros_msgs.srv import CommandBool, SetMode

from pyJoules.handler.csv_handler import CSVHandler
from pyJoules.device.rapl_device import RaplPackageDomain, RaplCoreDomain
from pyJoules.energy_meter import EnergyContext

from point_control_px4.script.quad_model_ENU import Quadrotor
from point_control_px4.script.generate_nmpc_mavros import QuadrotorMPC2
from point_control_px4.script.Logger_pqr import Logger


class OffboardControl(Node):
    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land_mavros')

        self.mocap_k = -1
        self.full_rotations = 0
        self.made_it = 0

        self.sim = bool(int(input("Select which thrust model to be used. Write 1 for threotical and 0 for fitting: ")))
        print(f"{'threotical' if self.sim else 'fitting'}")
        self.double_speed = bool(int(input("Double Speed Trajectories? Press 1 for Yes and 0 for No: ")))

        self.ctrl_loop_time_log = []
        self.x_log, self.y_log, self.z_log, self.yaw_log = [], [], [], []
        self.throttle_log, self.roll_log, self.pitch_log = [], [], []
        self.p_u_log, self.q_u_log, self.r_u_log = [], [], []
        self.x_ref_log, self.y_ref_log, self.z_ref_log, self.yaw_ref_log = [], [], [], []
        self.vx_log, self.vy_log, self.vz_log, self.p_log, self.q_log, self.r_log = [], [], [], [], [], []
        self.mpc_timel_array = []
        self.ctrl_callback_timel_log = []

        self.mode_channel = 5
        self.pyjoules_on = False
        if self.pyjoules_on:
            self.csv_handler = CSVHandler('mpc_cpu_energy.csv')

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ---------------------------
        # MAVROS publishers
        # ---------------------------
        self.attitude_target_pub = self.create_publisher(
            AttitudeTarget,
            '/mavros/setpoint_raw/attitude',
            pub_qos
        )

        self.position_setpoint_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            pub_qos
        )
        self.vision_pose_pub = self.create_publisher(PoseStamped, "/mavros/vision_pose/pose", pub_qos)
        self.vision_twist_pub = self.create_publisher(
            TwistStamped, "/mavros/vision_speed/speed_twist", pub_qos
        )

        # self.state_input_ref_log_publisher_ = self.create_publisher(
        #     Float64MultiArray, '/state_input_ref_log', pub_qos
        # )
        # self.state_input_ref_log_msg = Float64MultiArray()

        # ---------------------------
        # MAVROS subscribers
        # ---------------------------
        self.vins_odom_sub = self.create_subscription(Odometry, '/odometry', self.vins_odom_cb, sensor_qos)
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            sensor_qos
        )

        self.mavros_pos_sub = self.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self.mavros_pos, sensor_qos
        )
        self.mavros_vel_sub = self.create_subscription(
            TwistStamped, "/mavros/local_position/velocity_local", self.mavros_vel, sensor_qos
        )

        self.rc_sub = self.create_subscription(
            RCIn,
            '/mavros/rc/in',
            self.rc_channel_callback,
            sensor_qos
        )

        # ---------------------------
        # MAVROS services
        # ---------------------------
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /mavros/cmd/arming ...')

        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /mavros/set_mode ...')

        # ---------------------------
        # init vars
        # ---------------------------
        self.offboard_mode_rc_switch_on = False 
        self.current_state = State()

        self.cushion_time = 10.0
        self.flight_time = 30.0
        self.time_before_land = self.flight_time + 2 * self.cushion_time
        print(f"time_before_land: {self.time_before_land}")

        self.offboard_setpoint_counter = 0

        self.T0 = time.time()
        self.time_from_start = time.time() - self.T0
        self.first_iteration = True
        self.mode_save = ""

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.p = 0.0
        self.q = 0.0
        self.r = 0.0
        self.state_vector = np.zeros((9, 1))
        self.nr_state = np.zeros((4, 1))

        if self.sim:
            print("Using simulator throttle from force conversion function")
            self.MASS = 1.287
            # self.MOTOR_CONSTANT = 0.000001699265381
            self.MOTOR_CONSTANT = 0.000002699265381
            self.MOTOR_VELOCITY_ARMED = 100.0
            self.MOTOR_INPUT_SCALING = 2870.159048
        else:
            print("Using hardware throttle from force conversion function")
            self.MASS = 1.287

        self.GRAVITY = 9.79136
        self.T_lookahead = 0.

        quad = Quadrotor(sim=self.sim)
        generate_c_code = True
        horizon = 3.0
        num_steps = 20
        self.mpc_solver = QuadrotorMPC2(generate_c_code, quad, horizon, num_steps)
        self.num_steps = num_steps
        self.horizon = horizon

        self.metadata = np.array([
            'Sim' if self.sim else 'Hardware',
            '2x Speed' if self.double_speed else '1x Speed',
            'Horizon:' + str(horizon),
            'Num Steps:' + str(num_steps),
            'Pyjoules' if self.pyjoules_on else 'No Pyjoules',
        ])

        # 10 Hz: 持续发 setpoint，保证 OFFBOARD 有效
        self.offboard_timer_period = 0.1
        self.offboard_timer = self.create_timer(
            self.offboard_timer_period,
            self.offboard_mode_timer_callback
        )

        # 100 Hz: 控制器
        self.controller_timer_period = 0.01
        self.vision_rate_hz = 100
        self.controller_timer = self.create_timer(
            self.controller_timer_period,
            self.controller_timer_callback
        )
        self.vision_timer = self.create_timer(1.0 / self.vision_rate_hz, self.vision_timer_cb)

    def vins_odom_cb(self, msg: Odometry):
        # 1) pose
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        self.last_vins_pose = ps

        # 2) twist（线速度 + 角速度）
        ts = TwistStamped()
        ts.header = msg.header
        ts.twist = msg.twist.twist
        self.last_twist = ts

    def vision_timer_cb(self):

        now = self.get_clock().now().to_msg()
        frame_id = self.last_vins_pose.header.frame_id if self.last_vins_pose.header.frame_id else "map"

        # 发布 pose（用当前时间戳，frame_id 保持一致）
        out_pose = PoseStamped()
        out_pose.header.stamp = now
        out_pose.header.frame_id = frame_id
        out_pose.pose = self.last_vins_pose.pose
        self.vision_pose_pub.publish(out_pose)

        # 发布 twist（线速度+角速度）
        out_twist = TwistStamped()
        out_twist.header.stamp = now
        out_twist.header.frame_id = frame_id
        out_twist.twist = self.last_twist.twist
        self.vision_twist_pub.publish(out_twist)


    # =========================
    # MAVROS service wrappers
    # =========================
    def arm(self):
        req = CommandBool.Request()
        req.value = True
        future = self.arming_client.call_async(req)
        self.get_logger().info('Arm command sent')
        return future

    def disarm(self):
        req = CommandBool.Request()
        req.value = False
        future = self.arming_client.call_async(req)
        self.get_logger().info('Disarm command sent')
        return future

    def engage_offboard_mode(self):
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = 'OFFBOARD'
        future = self.set_mode_client.call_async(req)
        self.get_logger().info('Switching to OFFBOARD mode')
        return future

    def land(self):
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = 'AUTO.LAND'
        future = self.set_mode_client.call_async(req)
        self.get_logger().info('Switching to AUTO.LAND mode')
        return future

    # =========================
    # callbacks
    # =========================
    def state_callback(self, msg: State):
        self.current_state = msg

    def rc_channel_callback(self, rc_msg: RCIn):
        print('rc channel callback')
        self.mode_channel = 5
        if len(rc_msg.channels) >= self.mode_channel:
            pwm = rc_msg.channels[self.mode_channel - 1]
            # 这里按常见 1000~2000 PWM 判断
            self.offboard_mode_rc_switch_on = True if 1600 >= pwm >= 1200 else False

    def normalize_angle(self, angle):
        return m.atan2(m.sin(angle), m.cos(angle))

    def adjust_yaw(self, yaw):
        mocap_psi = yaw
        self.mocap_k += 1
        psi = None

        if self.mocap_k == 0:
            self.prev_mocap_psi = mocap_psi
            psi = mocap_psi
        else:
            if self.prev_mocap_psi > np.pi * 0.9 and mocap_psi < -np.pi * 0.9:
                self.full_rotations += 1
            elif self.prev_mocap_psi < -np.pi * 0.9 and mocap_psi > np.pi * 0.9:
                self.full_rotations -= 1

            psi = mocap_psi + 2 * np.pi * self.full_rotations
            self.prev_mocap_psi = mocap_psi

        return psi

    def mavros_pos(self, msg: PoseStamped):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z
        q_msg = msg.pose.orientation
        q = [q_msg.w, q_msg.x, q_msg.y, q_msg.z]
        self.roll, self.pitch, self.yaw = quat2euler(q)

        self.state_vector = np.array(
            [[self.x, self.y, self.z, self.vx, self.vy, self.vz,
              self.roll, self.pitch, self.yaw]]
        ).T
        self.nr_state = np.array([[self.x, self.y, self.z, self.yaw]]).T

    def mavros_vel(self, msg: TwistStamped):
        self.vx = msg.twist.linear.x
        self.vy = msg.twist.linear.y
        self.vz = msg.twist.linear.z
        self.p = msg.twist.angular.x
        self.q = msg.twist.angular.y
        self.r = msg.twist.angular.z

        self.state_vector = np.array(
            [[self.x, self.y, self.z, self.vx, self.vy, self.vz,
              self.roll, self.pitch, self.yaw]]
        ).T
        self.nr_state = np.array([[self.x, self.y, self.z, self.yaw]]).T

        

    # =========================
    # setpoint publishers
    # =========================
    def publish_rates_setpoint(self, thrust: float, roll_rate: float, pitch_rate: float, yaw_rate: float):
        """
        MAVROS body rate + thrust:
        topic: /mavros/setpoint_raw/attitude
        msg: mavros_msgs/AttitudeTarget

        原 PX4 代码中 thrust 往往是负值表示向上推力，
        到 MAVROS 这里改成 0~1 的正值总推力。
        """
        msg = AttitudeTarget()

        # 忽略姿态，仅使用 body_rate + thrust
        msg.type_mask = (
            AttitudeTarget.IGNORE_ATTITUDE
        )

        msg.body_rate = Vector3(
            x=float(roll_rate),
            y=float(pitch_rate),
            z=float(yaw_rate)
        )

        thrust_cmd = float(np.clip(-thrust, 0.0, 1.0))
        msg.thrust = thrust_cmd

        self.attitude_target_pub.publish(msg)
        print(f"Publishing MAVROS attitude target [thrust, p, q, r]: {[thrust_cmd, roll_rate, pitch_rate, yaw_rate]}")

    def publish_position_setpoint(self, x: float, y: float, z: float):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)

        # yaw = 0
        R = euler2mat(0.0, 0.0, 0.0, axes='sxyz')
        q = mat2quat(R)  # [w, x, y, z]
        msg.pose.orientation.w = float(q[0])
        msg.pose.orientation.x = float(q[1])
        msg.pose.orientation.y = float(q[2])
        msg.pose.orientation.z = float(q[3])

        self.position_setpoint_pub.publish(msg)
        self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

    # =========================
    # force/throttle conversion
    # =========================
    def get_throttle_command_from_force(self, collective_thrust):
        collective_thrust = -collective_thrust
        if self.sim:
            motor_speed = np.sqrt(collective_thrust / (4.0 * self.MOTOR_CONSTANT))
            throttle_command = (motor_speed - self.MOTOR_VELOCITY_ARMED) / self.MOTOR_INPUT_SCALING
            return -throttle_command

        else:
            a = - 0.001063
            b = 0.087293
            c = 0.027268
            throttle_command = a * (collective_thrust/4.0) ** 2 + b * np.sqrt(max((collective_thrust/4.0), 0.0)) + c
            return -throttle_command

    def get_force_from_throttle_command(self, throttle_command):
        throttle_command = -throttle_command
        print(f"Conv2Force: throttle_command: {throttle_command}")
        if self.sim:
            motor_speed = (throttle_command * self.MOTOR_INPUT_SCALING) + self.MOTOR_VELOCITY_ARMED
            collective_thrust = 4.0 * self.MOTOR_CONSTANT * motor_speed ** 2
            return -collective_thrust
        else:
            a = 19.2463167420814
            b = 41.8467162352942
            c = -7.19353022443441
            collective_thrust = a * throttle_command ** 2 + b * throttle_command + c
            return -collective_thrust

    # =========================
    # main timer callbacks
    # =========================
    def offboard_mode_timer_callback(self):
        if not self.offboard_mode_rc_switch_on:
            self.offboard_setpoint_counter = 0
            return

        # 仅在进入 OFFBOARD 前做预热
        if not self.current_state.armed: 
            self.publish_rates_setpoint(-0.001, 0.0, 0.0, 0.0)

            if self.offboard_setpoint_counter == 10:
                self.engage_offboard_mode()
                self.arm()

            if self.offboard_setpoint_counter < 11:
                self.offboard_setpoint_counter += 1


    def controller_timer_callback(self):
        print("Controller Callback")

        if self.offboard_mode_rc_switch_on:
            print(f"--------------------------------------")

            if self.current_state.mode == "OFFBOARD":
                print("IN OFFBOARD MODE")
                print(f"Controller callback - timefromstart: {self.time_from_start}")
                self.mode_save = "OFFBOARD"
                if self.time_from_start <= self.time_before_land:
                    print(f"Entering MPC Control Loop for next: {self.time_before_land - self.time_from_start} seconds")
                    self.controller()
                    
                else:
                    print("BACK TO SPAWN")
                    self.publish_position_setpoint(0.0, 0.0, 0.5)
                    print(f"self.x: {self.x}, self.y: {self.y}, self.z: {self.z}")

                    if abs(self.x) < 0.1 and abs(self.y) < 0.1 and abs(self.z - 0.5) <= 0.2:
                        print("Switching to Land Mode")
                        self.land()

            if self.time_from_start > self.time_before_land:
                if self.current_state.mode == "AUTO.LAND":
                    print("IN LAND MODE")
                    if self.z <= 0.15:
                        print("\nDisarming and Exiting Program")
                        self.disarm()
                        print("\nSaving all data!")
                        raise SystemExit

            print(f"--------------------------------------")
            print("\n\n")
        else:
            print(f"Controller Callback: RC Flight Mode Channel {self.mode_channel} switch not set to Offboard")

    # =========================
    # control core
    # =========================
    def controller(self):
        t0 = time.time()
        print(f"NR States: {self.nr_state}")

        if self.first_iteration:
            print("First Iteration")
            self.T0 = time.time()
            self.first_iteration = False

        self.time_from_start = time.time() - self.T0

        if self.time_from_start <= self.cushion_time:
            reffunc = self.hover_ref_func(1)
        elif self.cushion_time < self.time_from_start < self.cushion_time + self.flight_time:
            reffunc = self.fig8_vert_ref_func_tall()
            # reffunc = self.hover_ref_func(1)
        elif self.cushion_time + self.flight_time <= self.time_from_start <= self.time_before_land:
            reffunc = self.hover_ref_func(1)
        else:
            reffunc = self.hover_ref_func(1)

        print(f"reffunc: {reffunc[:, 0]}")

        new_u = self.get_new_control_input(reffunc)
        print(f"new_u: {new_u}")
        print(f"self.z: {self.z}")

        new_force = -new_u[0]
        new_throttle = self.get_throttle_command_from_force(new_force)
        new_roll_rate = new_u[1]
        new_pitch_rate = new_u[2]
        new_yaw_rate = new_u[3]

        final = [new_throttle, new_roll_rate, new_pitch_rate, new_yaw_rate]
        current_input_save = np.array(final).reshape(-1, 1)
        print(f"newInput: \n{current_input_save}")
        self.u0 = current_input_save

        self.publish_rates_setpoint(final[0], final[1], final[2], final[3])

        print(f"reffunc:({reffunc[0][-1], reffunc[1][-1], reffunc[2][-1], reffunc[-1][-1]})\n")
        controller_callback_time = time.time() - t0
        state_input_ref_log_info = [
            float(self.x), float(self.y), float(self.z), float(self.yaw),
            float(final[0]), float(final[1]), float(final[2]), float(final[3]),
            float(reffunc[0][-1]), float(reffunc[1][-1]), float(reffunc[2][-1]), float(reffunc[-1][-1]),
            self.time_from_start, controller_callback_time, float(self.vx), float(self.vy), float(self.vz), 
            float(self.roll), float(self.pitch), float(self.p), float(self.q), float(self.r)
        ]
        self.update_logged_data(state_input_ref_log_info)

    # =========================
    # logs
    # =========================
    def update_logged_data(self, data):
        self.x_log.append(data[0])
        self.y_log.append(data[1])
        self.z_log.append(data[2])
        self.yaw_log.append(data[3])
        self.throttle_log.append(data[4])
        self.p_u_log.append(data[5])
        self.q_u_log.append(data[6])
        self.r_u_log.append(data[7])
        self.x_ref_log.append(data[8])
        self.y_ref_log.append(data[9])
        self.z_ref_log.append(data[10])
        self.yaw_ref_log.append(data[11])
        self.ctrl_loop_time_log.append(data[12])
        self.ctrl_callback_timel_log.append(data[13])
        self.vx_log.append(data[14])
        self.vy_log.append(data[15])
        self.vz_log.append(data[16])
        self.roll_log.append(data[17])
        self.pitch_log.append(data[18])
        self.p_log.append(data[19])
        self.q_log.append(data[20])
        self.r_log.append(data[21])


    def get_x_log(self): return np.array(self.x_log).reshape(-1, 1)
    def get_y_log(self): return np.array(self.y_log).reshape(-1, 1)
    def get_z_log(self): return np.array(self.z_log).reshape(-1, 1)
    def get_vx_log(self): return np.array(self.vx_log).reshape(-1, 1)
    def get_vy_log(self): return np.array(self.vy_log).reshape(-1, 1)
    def get_vz_log(self): return np.array(self.vz_log).reshape(-1, 1)
    def get_roll_log(self): return np.array(self.roll_log).reshape(-1, 1)
    def get_pitch_log(self): return np.array(self.pitch_log).reshape(-1, 1)
    def get_yaw_log(self): return np.array(self.yaw_log).reshape(-1, 1)
    def get_p_log(self): return np.array(self.p_log).reshape(-1, 1)
    def get_q_log(self): return np.array(self.q_log).reshape(-1, 1)
    def get_r_log(self): return np.array(self.r_log).reshape(-1, 1)
    def get_ctrl_loop_time_log(self): return np.array(self.ctrl_loop_time_log).reshape(-1, 1)
    def get_x_ref_log(self): return np.array(self.x_ref_log).reshape(-1, 1)
    def get_y_ref_log(self): return np.array(self.y_ref_log).reshape(-1, 1)
    def get_z_ref_log(self): return np.array(self.z_ref_log).reshape(-1, 1)
    def get_yaw_ref_log(self): return np.array(self.yaw_ref_log).reshape(-1, 1)

    def get_throttle_log(self): return np.array(self.throttle_log).reshape(-1, 1)
    def get_p_u_log(self): return np.array(self.p_u_log).reshape(-1, 1)
    def get_q_u_log(self): return np.array(self.q_u_log).reshape(-1, 1)
    def get_r_u_log(self): return np.array(self.r_u_log).reshape(-1, 1)
    def get_ctrl_callback_timel_log(self): return np.array(self.ctrl_callback_timel_log).reshape(-1, 1)
    def get_mpc_timel_log(self): return np.array(self.mpc_timel_array).reshape(-1, 1)


    def get_new_control_input(self, reffunc):
        if self.pyjoules_on:
            with EnergyContext(handler=self.csv_handler, domains=[RaplPackageDomain(0), RaplCoreDomain(0)]):
                return self.execute_control_input(reffunc)
        else:
            return self.execute_control_input(reffunc)

    def execute_control_input(self, reffunc):
        t0 = time.time()
        status, x_mpc, u_mpc = self.mpc_solver.solve_mpc_control(self.state_vector.flatten(), reffunc)
        mpc_timel = time.time() - t0
        print(f"Outside Acados Timel: {mpc_timel}sec, Good Enough for {1 / mpc_timel}Hz")
        self.mpc_timel_array.append(mpc_timel)
        print(f"u_mpc: {u_mpc}")
        return u_mpc

    # =========================
    # trajectories
    # 这里保留你原来的实现
    # =========================
    def hover_ref_func(self, num):
        hover_dict = {
            1: np.array([[0.0, 0.0, 0.5,     0.0, 0.0, 0.0,   0.0, 0.0, 0.0]]).T,
            2: np.array([[0.0, 0.8, 0.8,     0.0, 0.0, 0.0,   0.0, 0.0, 0.0]]).T,
            3: np.array([[0.8, 0.0, 0.8,     0.0, 0.0, 0.0,   0.0, 0.0, 0.0]]).T,
            4: np.array([[0.8, 0.8, 0.8,     0.0, 0.0, 0.0,   0.0, 0.0, 0.0]]).T,
        }

        if num > len(hover_dict) or num < 1:
            print(f"hover_dict #{num} not found")
            raise SystemExit

        print(f"hover_dict #{num}")
        r = hover_dict.get(num)
        r_final = np.tile(r, (1, self.num_steps))
        return r_final

    def fig8_vert_ref_func_tall(self):
        print("fig8_vert_ref_func_tall")
        t_start = self.time_from_start - self.cushion_time
        t = np.linspace(t_start, t_start + self.horizon, self.num_steps)

        PERIOD = 16
        if self.double_speed:
            PERIOD /= 2
        w = 2 * np.pi / PERIOD

        x = 1.0 * np.cos(w * t).reshape(-1)
        y = 1.0 * np.sin(w * t).reshape(-1)
        z = 0.5 * np.ones(self.num_steps).reshape(-1)
        vx = -1.0 * w * np.sin(w * t).reshape(-1)
        vy = 1.0 * w * np.cos(w * t).reshape(-1)
        vz = np.zeros(self.num_steps)
        roll = np.zeros(self.num_steps).reshape(-1)
        pitch = np.zeros(self.num_steps).reshape(-1)
        yaw = np.zeros(self.num_steps).reshape(-1)

        r = np.array([x, y, z, vx, vy, vz, roll, pitch, yaw])
        return r


def main(args=None):
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    logger = None

    def shutdown_logging():
        print("\nInterrupt/Error/Termination Detected, Triggering Logging Process and Shutting Down Node...")
        if logger:
            logger.log(offboard_control)
        offboard_control.destroy_node()
        rclpy.shutdown()

    try:
        print(f"\nInitializing ROS 2 node: '{__name__}' for offboard control")
        logger = Logger([sys.argv[1]])
        rclpy.spin(offboard_control)
    except KeyboardInterrupt:
        print("\nKeyboard interrupt detected (Ctrl+C), exiting...")
    except SystemExit:
        print("\nProgram requested exit.")
    except Exception as e:
        print(f"\nError in main: {e}")
        traceback.print_exc()
    finally:
        shutdown_logging()
        if offboard_control.pyjoules_on:
            offboard_control.csv_handler.save_data()
        print("\nNode has shut down.")


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f"\nError in __main__: {e}")
        traceback.print_exc()


