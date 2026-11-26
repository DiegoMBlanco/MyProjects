# Multi Welding Arm Simulation

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
> Additionally, to be able to communicate our simulation with ROS 2, it is needed to use a package called 'ros_gz_bridge' and 'gz_ros2_control'. This packages provides a network bridge which enables the exchange of messages between ROS 2 and Gazebo transport, and provide each robot arm's trajectories. You can install this packages by typing:

```bash
sudo apt-get install ros-$ROS_DISTRO-ros-ign-bridge
```

```bash
sudo apt install ros-$ROS_DISTRO-gz-ros2-control
```
## Steps for launching the simulation
1) Clone the "mas_orchestra_digital_twin" package into your "src" directory (Inside a workspace)
2) Run the following comand in one terminal: 
```bash
colcon build --packages-select mas_orchestra_digital_twin
source install/setup.bash
ros2 launch mas_orchestra_digital_twin simulation_launcher.launch.py 
```
## In a second terminal, activate welding sequence:
```bash
ros2 run mas_orchestra_digital_twin welding_cell_manager.py
```

## In third terminal, activate or send robots to maintenance:
```bash
ros2 topic pub --once /welding_cell/command std_msgs/msg/String "{data: mantenimiento_r1}"
```
```bash
ros2 topic pub --once /welding_cell/command std_msgs/msg/String "{data: activar_r1}"
```

### Debug section:

If the welding secuence executable is not found, give all permitions to the file from /mas_orchestra_digital_twin/:
```bash
chmod +x src/mas_orchestra_digital_twin/scripts/welding_cell_manager.py
``




https://github.com/user-attachments/assets/528ee45a-0e9b-41e0-8419-a46de16cf53f
