# Digital Twin Puzzlebot

## Pre-requisites and Ignition installation

It is assumed that any distro of ROS 2 is already installed.
To avoid possible errors, please update your system and install the following ROS 2 dependencies.

```bash
sudo apt-get update
sudo apt-get install ros-$ROS_DISTRO-joint-state-publisher ros-$ROS_DISTRO-xacro ros-$ROS_DISTRO-joint-state-publisher-gui ros-$ROS_DISTRO-tf2-* ros-$ROS_DISTRO-rviz-default-plugins
```

To install Ignition to work with ROS 2, run the following command:

```bash
sudo apt-get install ros-$ROS_DISTRO-ros-gz
```

The ros-gz package from source can be found here 
https://github.com/gazebosim/ros_gz/tree/humble

> [!IMPORTANT]
> Additionally, to be able to communicate our simulation with ROS 2, it is needed to use a package called 'ros_gz_bridge'. This package provides a network bridge which enables the exchange of messages between ROS 2 and Gazebo transport. You can install this package by typing:

```bash
sudo apt-get install ros-$ROS_DISTRO-ros-ign-bridge
```
## Steps for launching the simulation
1) Clone the "gemelo_digital_puzzlebot" package
2) Run the following comand: 
```bash
ros2 launch gemelo_digital_puzzlebot simulation_launcher.launch.py 
```
### Ros2 topic list:
* /cmd_vel <- Robots velocity
* /r1/ground_truth <- Exact pose of the robot in the simulation
* /r1/odom <- Odometry based on the wheel encoders
* /r1/logi_camera/image <- Logi's camera raw image (2D)
