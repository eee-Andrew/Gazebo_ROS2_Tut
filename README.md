
# Gazebo_ROS2_foxy_PX4.

## Installation Guide


## Table of Contents
1. [Introduction](#introduction)
2. [Prerequisites](#prerequisites)
3. [Setup Instructions](#setup-instructions-1)

### PX4 Autopilot Setup

```bash
cd
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
cd PX4-Autopilot/
make px4_sitl
```

### Install ROS 2 Foxy on Ubuntu 20.04

Follow the official installation guide for ROS 2 Foxy on Ubuntu 20.04: [Install ROS 2 Foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html).

You can install either the desktop (`ros-foxy-desktop`) or bare-bones versions (`ros-foxy-ros-base`), and the development tools (`ros-dev-tools`).

#### Python Dependencies

```bash
pip install --user -U empy==3.3.4 pyros-genmsg setuptools
```

### Set Up uXRCE-DDS Agent for ROS 2 and PX4 Communication

To install and start the uXRCE-DDS agent, follow these steps:

```bash
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

To start the agent:

```bash
MicroXRCEAgent udp4 -p 8888
```

This agent is now running, ready to connect with the PX4 simulator on UDP port 8888.

### Start PX4 Simulator with Gazebo

In a new terminal, from the PX4 Autopilot repo root:

```bash
make px4_sitl gazebo-classic
```

When the agent and PX4 are connected, you should see INFO messages showing data writers creation.

### Build ROS 2 Workspace

Create and build a ROS 2 workspace in your home directory.

1. Open a terminal and create a workspace:

```bash
mkdir -p ~/ws_sensor_combined/src/
cd ~/ws_sensor_combined/src/
```

2. Clone the `px4_msgs` and `px4_ros_com` repositories:

```bash
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/PX4/px4_ros_com.git
```

3. Build the workspace:

```bash
cd ..
source /opt/ros/foxy/setup.bash
colcon build
```

### Running the Example

To run the ROS 2 Listener example:

1. Open a new terminal and source the environment:

```bash
cd ~/ws_sensor_combined/
source /opt/ros/foxy/setup.bash
source install/local_setup.bash
```

2. Launch the example:

```bash
ros2 launch px4_ros_com sensor_combined_listener.launch.py
```

You should see data output from the Sensor Combined topic.

### Control a Vehicle with ROS 2

ROS 2 applications can subscribe to telemetry topics published by PX4 and publish to topics for vehicle control.

Refer to `dds_topics.yaml` and the [uORB Message Reference](https://docs.px4.io/main/en/msg_docs/) for available topics.

Example QoS settings for subscribing:

```cpp
rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

subscription_ = this->create_subscription<px4_msgs::msg::SensorCombined>("/fmu/out/sensor_combined", qos,
```

### Frame Conventions

Conversions may be required to align frames between ROS and PX4. Use `frame_transforms` from `PX4/px4_ros_com` for convenient transformations.

To rotate from ENU to NED:

1. Pi/2 rotation around Z-axis (up)
2. Pi rotation around X-axis

Refer to [Coordinate Frames for Mobile Platforms](https://www.ros.org/reps/rep-0105.html) for more on ROS frames.

### ROS, Gazebo, and PX4 Time Synchronization

To install Gazebo Garden for ROS 2 Foxy on Ubuntu 20.04:

```bash
sudo apt install ros-foxy-ros-gzgarden
```

This guide now provides instructions for ROS 2 example applications, including:
- ROS 2 Advertiser
- Offboard Control
- Using Flight Controller Hardware
- Custom uORB Topics

For more tools and commands, explore the ROS 2 CLI:
(some commands)
```bash
ros2 topic list
ros2 topic echo
ros2 topic hz
ros2 launch
```
