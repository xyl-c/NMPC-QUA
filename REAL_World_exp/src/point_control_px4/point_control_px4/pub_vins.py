
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, VehicleRatesSetpoint, VehicleCommand, VehicleStatus, VehicleOdometry, TrajectorySetpoint, RcChannels
from std_msgs.msg import Float64MultiArray
from px4_msgs.msg import ActuatorMotors, VehicleThrustSetpoint, VehicleTorqueSetpoint
import subprocess

import os
import sys
import traceback

from transforms3d.euler import quat2euler
import math as m
import numpy as np
import time

from geometry_msgs.msg import PoseStamped, Quaternion, Vector3, TwistStamped
from nav_msgs.msg import Odometry
from pyJoules.handler.csv_handler import CSVHandler
from pyJoules.device.rapl_device import RaplPackageDomain, RaplCoreDomain
from pyJoules.energy_meter import EnergyContext
from mavros_msgs.msg import State, AttitudeTarget, RCIn

from point_control_px4.script.quadModel_thrust_torque import QuadrotorModel
from point_control_px4.script.generate_nmpc_outp_thrust_tor import QuadrotorMPC2
from point_control_px4.script.Logger_pqr import Logger

from ament_index_python.packages import get_package_share_directory


class Pub_VINS(Node):
    """Node for controlling a vehicle in offboard mode."""
    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')


###############################################################################################################################################



        self.mode_channel = 5
        self.pyjoules_on = False # int(input("Use PyJoules? 1 for Yes 0 for No: ")) #False
        if self.pyjoules_on:
            self.csv_handler = CSVHandler('mpc_cpu_energy.csv')
###############################################################################################################################################

        # Configure QoS profile for publishing and subscribing
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


        # Create Publishers
        # Publishers for Setting to Offboard Mode and Arming/Diasarming/Landing/etc
        self.offboard_control_mode_publisher = self.create_publisher( #publishes offboard control heartbeat
            OffboardControlMode, '/fmu/in/offboard_control_mode', pub_qos)
        self.vehicle_command_publisher = self.create_publisher( #publishes vehicle commands (arm, offboard, disarm, etc)
            VehicleCommand, '/fmu/in/vehicle_command', pub_qos)
        
      
        self.vision_pose_pub = self.create_publisher(PoseStamped, "/mavros/vision_pose/pose", pub_qos)
        self.vision_twist_pub = self.create_publisher(TwistStamped, "/mavros/vision_speed/speed_twist", pub_qos)


        # Create subscribers
        self.vins_odom_sub = self.create_subscription(Odometry, '/odometry', self.vins_odom_cb, sensor_qos)


    


###############################################################################################################################################

        # Initialize variables:  
        self.cushion_time = 5.0
        self.flight_time = 20.0
        self.time_before_land = self.flight_time + 2.0*(self.cushion_time)
        print(f"time_before_land: {self.time_before_land}")
        self.offboard_setpoint_counter = 0 #helps us count 10 cycles of sending offboard heartbeat before switching to offboard mode and arming
        self.vehicle_status = VehicleStatus() #vehicle status variable to make sure we're in offboard mode before sending setpoints

        self.T0 = time.time() # initial time of program
        self.time_from_start = time.time() - self.T0 # time from start of program initialized and updated later to keep track of current time in program
        self.first_iteration = True #boolean to help us initialize the first iteration of the program

        
###############################################################################################################################################
###############################################################################################################################################
        
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
        self.state_vector = np.zeros((13, 1))
        self.nr_state = np.zeros((4, 1))

        self.last_vins_pose = PoseStamped()
        self.last_twist = TwistStamped()
        self.horizon = 3.0 #2, 3
        self.num_steps = 30 #10, 20


###############################################################################################################################################
       
###############################################################################################################################################


        # Create Function at 50Hz to Publish VINS Pose and Twist
        self.vision_rate_hz = 50.0
        self.vision_timer = self.create_timer(1.0 / self.vision_rate_hz, self.vision_timer_cb)


        


    # The following 4 functions all call publish_vehicle_command to arm/disarm/land/ and switch to offboard mode
    # The 5th function publishes the vehicle command
    # The 6th function checks if we're in offboard mode
    # The 7th function handles the safety RC control switches for hardware
    def arm(self): #1. Sends arm command to vehicle via publish_vehicle_command function
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self): #2. Sends disarm command to vehicle via publish_vehicle_command function
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self): #3. Sends offboard command to vehicle via publish_vehicle_command function
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self): #4. Sends land command to vehicle via publish_vehicle_command function
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

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

    def publish_vehicle_command(self, command, **params) -> None: #5. Called by the above 4 functions to send parameter/mode commands to the vehicle
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def vehicle_status_callback(self, vehicle_status): #6. This function helps us check if we're in offboard mode before we start sending setpoints
        """Callback function for vehicle_status topic subscriber."""
        # print('vehicle status callback')
        self.vehicle_status = vehicle_status
        





    def normalize_angle(self, angle):
        """ Normalize the angle to the range [-pi, pi]. """
        return m.atan2(m.sin(angle), m.cos(angle))


    def adjust_yaw(self, yaw):
        mocap_psi = yaw
        self.mocap_k += 1
        psi = None
        
        if self.mocap_k == 0:
            self.prev_mocap_psi = mocap_psi
            psi = mocap_psi

        elif self.mocap_k > 0:
            # mocap angles are from -pi to pi, whereas the angle state variable in the MPC is an absolute angle (i.e. no modulus)
            # I correct for this discrepancy here
            if self.prev_mocap_psi > np.pi*0.9 and mocap_psi < -np.pi*0.9:
                # Crossed 180 deg, CCW
                self.full_rotations += 1
            elif self.prev_mocap_psi < -np.pi*0.9 and mocap_psi > np.pi*0.9:
                # Crossed 180 deg, CW
                self.full_rotations -= 1

            psi = mocap_psi + 2*np.pi * self.full_rotations
            self.prev_mocap_psi = mocap_psi
        
        return psi


    def vehicle_odometry_callback(self, msg): # Odometry Callback Function Yields Position, Velocity, and Attitude Data
        """Callback function for vehicle_odometry topic subscriber."""
        # print('vehicle odometry callback')
        self.odom_seq += 1

        self.x = msg.position[0]
        self.y = msg.position[1]
        self.z = msg.position[2]

        self.vx = msg.velocity[0]
        self.vy = msg.velocity[1]
        self.vz = msg.velocity[2]

        # self.roll, self.pitch, yaw = self.euler_from_quaternion(msg.q)
        self.roll, self.pitch, self.yaw = quat2euler(msg.q)
        # self.yaw = self.adjust_yaw(yaw)

        self.p = msg.angular_velocity[0]
        self.q = msg.angular_velocity[1]
        self.r = msg.angular_velocity[2]

        self.state_vector = np.array([[self.x, self.y, self.z, self.roll, self.pitch, self.yaw, self.vx, self.vy, self.vz, self.p, self.q, self.r]]).T 
        # self.state_vector = np.array([[self.x, self.y, self.z, self.vx, self.vy, self.vz, self.roll, self.pitch, self.yaw, self.p, self.q, self.r]]).T
        self.nr_state = np.array([[self.x, self.y, self.z, self.yaw]]).T
        # print(f"State Vector: {self.state_vector}")
        # print(f"NR State: {self.nr_state}")




# ~~ Entry point of the code -> Initializes the node and spins it. Also handles exceptions and logging ~~
def main(args=None):
    rclpy.init(args=args)
    pub_vins = Pub_VINS()
    rclpy.spin(pub_vins)
    pub_vins.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f"\nError in __main__: {e}")
        traceback.print_exc()



