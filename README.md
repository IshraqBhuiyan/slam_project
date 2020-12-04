# SLAM_Project
16831 SLAM project

## Initialization
Initialize git submodules and download Git Large File Storage. https://git-lfs.github.com/ 

## Start Robot and Run Localization Script in Simulator
Make sure to source each terminal window:

```bash
source devel/setup.bash
```


To start Gazebo and environment, also source the model path:

```bash
source src/slam_projects/rover_denso/set_gazebo_path.sh
```
then,
```bash
roslaunch rover_denso_gazebo rover_denso_world.launch
```

To move the robot around, you can use a joystick. If you need keyboard control, follow this tutorial and install relevant packages: http://wiki.ros.org/teleop_twist_keyboard

Run this line to fold down the manipulator and LiDAR horizontal:

```bash
rosrun rover_denso_control lidar_goal.py
```

Run this line to publish the transform between the LiDAR sensor to the base of robot:

```bash
rosrun rover_denso_control velodyne_to_base.py
```


To start using HDL to process LiDAR point cloud, do:

```bash
roslaunch proj_work hdl_localization.launch
```

Start the HDL localization by giving it an initial pose estimate, go inside rviz window, and click '2D Pose Estimate'. (Be aware the direction is in LiDAR frame)

## Record data

To record simulator data, do:

```bash
rosbag record -a
```

Point cloud data is published on topic: /velodyne_points
Localization result is published on topic: /tf (Map origin to center of the robot base)

Rosbag data can be accessed using Python/C++ API (see http://wiki.ros.org/rosbag/Code%20API) or MATLAB's ROS toolbox (https://www.mathworks.com/help/ros/ref/rosbag.html)