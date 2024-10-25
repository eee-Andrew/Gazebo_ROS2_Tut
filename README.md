
# Gazebo_ROS2_Foxy_PX4.

## Installation Guide

### YouTube Channels for Reference

- [Getting started with ROS 2 and PX4 - Video 1](https://www.youtube.com/watch?v=qhLATrkA_Gw)
- [Getting started with ROS 2 and PX4 - Video 2](https://www.youtube.com/watch?v=8gKIP0OqHdQ)
- [Getting started with ROS 2 and PX4 - Video 3](https://www.youtube.com/watch?v=Nbc7fzxFlYo)
- [Getting started with ROS 2 and PX4 - Video 4](https://www.youtube.com/watch?v=iRnLB31aQmA)

## Unity Drone System

### Table of Contents
1. [Introduction](#introduction)
2. [Prerequisites](#prerequisites)
3. [Setup Instructions](#setup-instructions-1)

---

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

---

## Tables

### ROS 2 & PX4 Frame Conventions

| Frame          | PX4                                  | ROS                       |
|----------------|--------------------------------------|---------------------------|
| Body           | FRD (X Forward, Y Right, Z Down)     | FLU (X Forward, Y Left, Z Up) |
| World          | FRD or NED (X North, Y East, Z Down) | FLU or ENU (X East, Y North, Z Up) |

**Coordinate Conversions**:

To rotate a vector from ENU to NED:
- Pi/2 rotation around the Z-axis (up)
- Pi rotation around the X-axis

To rotate a vector from NED to ENU:
- Pi/2 rotation around the Z-axis (down)
- Pi rotation around the X-axis

To rotate a vector from FLU to FRD:
- Pi rotation around the X-axis (front)

To rotate a vector from FRD to FLU:
- Pi rotation around the X-axis (front)

Examples of vectors requiring rotation include fields in the `TrajectorySetpoint` and `VehicleThrustSetpoint` messages.

### ROS, Gazebo, and PX4 Time Synchronization

For Gazebo time synchronization:

Install Gazebo Garden for ROS 2 Foxy on Ubuntu 20.04:

```bash
sudo apt install ros-foxy-ros-gzgarden
```

---

### ROS 2 Example Applications

This guide also includes commands for ROS 2 CLI:

```bash
ros2 topic list
ros2 topic echo
ros2 topic hz
ros2 launch
```
