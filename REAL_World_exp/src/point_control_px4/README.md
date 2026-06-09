#  IMPC and NMPC for Quadrotor Real-World Experiments
This project includes 4 nodes. 
'nmpc_output_thrust_torque' is the quadrotor NMPC control node which control output are thrust and three-axis control moments.
'pub_vs' is the node publish VINS to PX4.
'nmpc_mavros' is quadrotor NMPC control node which control output are thrust and three-axis angular velocities.
'test_thrust' is the node for testing the throttle thrust curve.

This project partially utilizes code from project https://github.com/evannsmc/nmpc_acados_px4.git. 


## Prerequisites

Install the following: 
- **pyJoules** 
- **ACADOS** 
- **ACADOS Python interface** (`acados_template`) 
- **Micro XRCE-DDS Agent** 
- **MAVROS** 
- **px4_msgs** 
- **...**(Other commonly used packages)

The dependencies are all commonly used packages for ROS2-PX4-Quadrotors. 
For the installation of ACADOS, please refer to project https://github.com/evannsmc/nmpc_acados_px4.git. 

This project uses VINS-Fusion for navigation, and VINS-Fusion [https://github.com/zinuok/VINS-Fusion-ROS2.git] needs to be installed before use.
If other positioning sources are used, make corresponding modifications in the program.


## Clone Project
1. Clone directory (**point_control_px4**) into your ROS2 workspace's source directory and build:
```bash
cd <your_ros2_ws/src>
git init
git clone git@github.com/xxx.git
cd ..
colcon build --symlink-install
```


## How to  run:
```bash
1. ros2 launch realsense2_camera rs_launch.py enable_infra1:=true enable_infra2:=true
2. ros2 launch mavros px4.launch
3. MicroXRCEAgent serial -D /dev/ttyUSB0 -b 921600
4. ros2 run vins vins_node ~/vins_ws/src/VINS-Fusion-ROS2/config/realsense_d435i/realsense_stereo_imu_config.yaml
5. ros2 run point_control_px4 pub_vs
Printed fused position:
6. ros2 topic echo /fmu/out/vehicle_odometry --field position
After confirming the convergence of positions, run the control node.
run NMPC control:
7. ros2 run point_control_px4 nmpc_output_thrust_torque log.log
8. run plotTest.py to plot results
```
Other usage methods can be found in the code comments.
## Note:
1. Before using NMPC, it is necessary to modify the quadrotor model parameters, and it is recommended to conduct GAZEBO simulation first.


## Address the issue that the initialization estimation of ROS2 VINS-FUSION sometimes fails to converge：
**Change the default FastDDS in ROS2 to CycloneDDS**








