
# Gazebo_ROS2_Foxy_PX4.

### Installation Guide
_____________________________________________________________________________________________________________________________________________________________________

### YouTube Channels for Reference
- [Getting started with ROS 2 and PX4 - Video 1](https://www.youtube.com/watch?v=qhLATrkA_Gw)
- [Getting started with ROS 2 and PX4 - Video 2](https://www.youtube.com/watch?v=8gKIP0OqHdQ)
- [Getting started with ROS 2 and PX4 - Video 3](https://www.youtube.com/watch?v=Nbc7fzxFlYo)
- [Getting started with ROS 2 and PX4 - Video 4](https://www.youtube.com/watch?v=iRnLB31aQmA)

---

# Gazebo_ROS2_foxy_PX4.

_____________________________________________________________________________________________________________________________________________________________________

## Setup Instructions

 [Setup Instructions](https://docs.px4.io/main/en/ros2/user_guide.html#foxy)
 -ROS 2 User Guide
- Install PX4 (to use the PX4 simulator)
- Install ROS 2
- Setup Micro XRCE-DDS Agent & Client
- Build & Run ROS 2 Workspace
- ROS 2 Example Applications
- ROS 2 Advertiser
- Offboard Control
- Using Flight Controller Hardware
- Custom uORB Topics
- Customizing the Topic Namespace
  _____________________________________________________________________________________________________________________________________________________________________
 ## ros2 CLI basic commands
- ros2 topic list
- ros2 topic echo
- ros2 topic hz
- ros2 launch
_____________________________________________________________________________________________________________________________________________________________________
## Troubleshooting
- Missing dependencies(sudo apt install python3-colcon-common-extensions)
- ros_gz_bridge not publishing on the \clock topic



# ROS 2 Commands Cheat Sheet

## 1. Environment Setup
- **Source ROS 2 workspace**: `source /opt/ros/<distro>/setup.bash`
- **Source a local workspace**: `source ~/dev_ws/install/setup.bash`
- **Check ROS 2 version**: `ros2 --version`

## 2. Creating and Managing Packages
- **Create a new ROS 2 package**:
  ```
  ros2 pkg create <package_name> --build-type ament_cmake
  ```
- **Build a workspace**:
  ```
  colcon build
  ```
- **Build a specific package**:
  ```
  colcon build --packages-select <package_name>
  ```
- **Clean workspace**:
  ```
  colcon build --cmake-clean-cache
  ```

## 3. Nodes
- **List all nodes**: `ros2 node list`
- **Get info about a node**: `ros2 node info /<node_name>`
- **Run a node**:
  ```
  ros2 run <package_name> <node_executable>
  ```
- **Kill a node**: `ros2 node kill /<node_name>`

## 4. Topics
- **List all topics**: `ros2 topic list`
- **Echo a topic**: `ros2 topic echo /<topic_name>`
- **Publish to a topic**:
  ```
  ros2 topic pub /<topic_name> <msg_type> <data>
  ```
- **Get info about a topic**: `ros2 topic info /<topic_name>`
- **List active publishers/subscribers**:
  ```
  ros2 topic list -t
  ```

## 5. Services
- **List all services**: `ros2 service list`
- **Call a service**:
  ```
  ros2 service call /<service_name> <srv_type> <data>
  ```
- **Get info about a service**: `ros2 service info /<service_name>`

## 6. Actions
- **List all actions**: `ros2 action list`
- **Send a goal to an action server**:
  ```
  ros2 action send_goal /<action_name> <action_type> <data>
  ```
- **Get info about an action**: `ros2 action info /<action_name>`

## 7. Parameters
- **List all parameters of a node**: `ros2 param list /<node_name>`
- **Get parameter value**: `ros2 param get /<node_name> <parameter_name>`
- **Set parameter value**:
  ```
  ros2 param set /<node_name> <parameter_name> <value>
  ```
- **Load parameters from YAML file**:
  ```
  ros2 param load /<node_name> <yaml_file_path>
  ```

## 8. Launch Files
- **Launch a file**:
  ```
  ros2 launch <package_name> <launch_file>
  ```
- **Launch with parameter overrides**:
  ```
  ros2 launch <package_name> <launch_file> param:=value
  ```

## 9. Introspection and Debugging
- **List all active topics, nodes, and services**:
  ```
  ros2 list
  ```
- **Check message definition**:
  ```
  ros2 interface show <msg_type>
  ```
- **Echo node logs**:
  ```
  ros2 run rclcpp_components component_container
  ```
- **Record topics to a rosbag**:
  ```
  ros2 bag record -o <bag_name> /<topic_name>
  ```
- **Play back a rosbag**:
  ```
  ros2 bag play <bag_file>
  ```

---

This ROS 2 cheat sheet covers essential commands for basic operations, package management, nodes, topics, services, actions, parameters, launch files, introspection, and debugging.

















