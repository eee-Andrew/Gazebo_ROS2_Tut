
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


Installation guide
_____________________________________________________________________________________________________________________________________________________________________


# Unity Drone System

## Table of Contents
1. [Introduction](#introduction)
2. [Prerequisites](#prerequisites)
3. [Setup Instructions](#setup-instructions-1)



cd
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
cd PX4-Autopilot/
make px4_sitl



To install ROS 2 and its dependencies:

 install ROS 2 "Foxy" on Ubuntu 20.04:
    Follow the official installation guide: Install ROS 2 Foxy.
    https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html

You can install either the desktop (ros-foxy-desktop) or bare-bones versions (ros-foxy-ros-base), and the development tools (ros-dev-tools).


Some Python dependencies must also be installed (using pip or apt):

pip install --user -U empy==3.3.4 pyros-genmsg setuptools


For ROS 2 to communicate with PX4, uXRCE-DDS client must be running on PX4, connected to a micro XRCE-DDS agent running on the companion computer.


The agent can be installed onto the companion computer in a number of ways. Below we show how to build the agent "standalone" from source and connect to a client running on the PX4 simulator.

To setup and start the agent:

    Open a terminal.

    Enter the following commands to fetch and build the agent from source:

    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/

Start the agent with settings for connecting to the uXRCE-DDS client running on the simulator: (uxrce_dds_client :https://docs.px4.io/main/en/modules/modules_system.html#uxrce-dds-client)
sh

MicroXRCEAgent udp4 -p 8888


The agent is now running, but you won't see much until we start PX4 (in the next step).

INFO

You can leave the agent running in this terminal! Note that only one agent is allowed per connection channel.

The PX4 simulator starts the uXRCE-DDS client automatically, connecting to UDP port 8888 on the local host.

To start the simulator (and client):

    Open a new terminal in the root of the PX4 Autopilot repo that was installed above.


Start a PX4 Gazebo Classic simulation using (only works for ros2 foxy)

make px4_sitl gazebo-classic


The agent and client are now running they should connect.

The PX4 terminal displays the NuttShell(https://docs.px4.io/main/en/debug/system_console.html)/PX4 system Console(https://docs.px4.io/main/en/debug/system_console.html) output as PX4 boots and runs. As soon as the agent connects the output should include INFO messages showing creation of data writers:


...
INFO  [uxrce_dds_client] synchronized with time offset 1675929429203524us
INFO  [uxrce_dds_client] successfully created rt/fmu/out/failsafe_flags data writer, topic id: 83
INFO  [uxrce_dds_client] successfully created rt/fmu/out/sensor_combined data writer, topic id: 168
INFO  [uxrce_dds_client] successfully created rt/fmu/out/timesync_status data writer, topic id: 188
...

The micro XRCE-DDS agent terminal should also start to show output, as equivalent topics are created in the DDS network:

...
[1675929445.268957] info     | ProxyClient.cpp    | create_publisher         | publisher created      | client_key: 0x00000001, publisher_id: 0x0DA(3), participant_id: 0x001(1)
[1675929445.269521] info     | ProxyClient.cpp    | create_datawriter        | datawriter created     | client_key: 0x00000001, datawriter_id: 0x0DA(5), publisher_id: 0x0DA(3)
[1675929445.270412] info     | ProxyClient.cpp    | create_topic             | topic created          | client_key: 0x00000001, topic_id: 0x0DF(2), participant_id: 0x001(1)
...

Build ROS 2 Workspace

This section shows how create a ROS 2 workspace hosted in your home directory (modify the commands as needed to put the source code elsewhere).

The px4_ros_com(https://github.com/PX4/px4_ros_com) and px4_msgs packages(https://github.com/PX4/px4_msgs) are cloned to a workspace folder, and then the colcon tool is used to build the workspace. 
The example is run using ros2 launch.

INFO

The example builds the ROS 2 Listener(https://docs.px4.io/main/en/ros2/user_guide.html#ros-2-listener) example application, located in px4_ros_com(https://github.com/PX4/px4_ros_com) px4_msgs(https://github.com/PX4/px4_msgs) is needed too so that the example can interpret PX4 ROS 2 topics.

Building the Workspace

To create and build the workspace:

    1.Open a new terminal.

    2.Create and navigate into a new workspace directory using:


mkdir -p ~/ws_sensor_combined/src/
cd ~/ws_sensor_combined/src/

INFO
A naming convention for workspace folders can make it easier to manage workspaces.

3. Clone the example repository and px4_msgs9https://github.com/PX4/px4_msgs) to the /src directory (the main branch is cloned by default, which corresponds to the version of PX4 we are running):
sh

git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/PX4/px4_ros_com.git

4.Source the ROS 2 development environment into the current terminal and compile the workspace using colcon (Only for foxy_Ros2):

    
cd ..
source /opt/ros/foxy/setup.bash
colcon build

    This builds all the folders under /src using the sourced toolchain.

Running the Example

To run the executables that you just built, you need to source local_setup.bash. This provides access to the "environment hooks" for the current workspace. 
In other words, it makes the executables that were just built available in the current terminal.

INFO

The ROS2 beginner tutorials(https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html#source-the-overlay) recommend that you open a new terminal for running your executables.

In a new terminal:

  1. Navigate into the top level of your workspace directory and source the ROS 2 environment (in this case "Foxy"):


cd ~/ws_sensor_combined/
source /opt/ros/foxy/setup.bash

2. Source the local_setup.bash.

source install/local_setup.bash

3. Now launch the example. Note here that we use ros2 launch, which is described below.


    ros2 launch px4_ros_com sensor_combined_listener.launch.py

If this is working you should see data being printed on the terminal/console where you launched the ROS listener:
sh

RECEIVED DATA FROM SENSOR COMBINED
================================
ts: 870938190
gyro_rad[0]: 0.00341645
gyro_rad[1]: 0.00626475
gyro_rad[2]: -0.000515705
gyro_integral_dt: 4739
accelerometer_timestamp_relative: 0
accelerometer_m_s2[0]: -0.273381
accelerometer_m_s2[1]: 0.0949186
accelerometer_m_s2[2]: -9.76044
accelerometer_integral_dt: 4739

Controlling a Vehicle

To control applications, ROS 2 applications:

 -   subscribe to (listen to) telemetry topics published by PX4
 -   publish to topics that cause PX4 to perform some action.

The topics that you can use are defined in dds_topics.yaml(https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/uxrce_dds_client/dds_topics.yaml), and you can get more information about their data in the uORB  Message Reference(https://docs.px4.io/main/en/msg_docs/). 
For example, VehicleGlobalPosition(https://docs.px4.io/main/en/msg_docs/VehicleGlobalPosition.html) can be used to get the vehicle global position, while VehicleCommand(https://docs.px4.io/main/en/msg_docs/VehicleCommand.html)can be used to command actions such as takeoff and land.

The ROS 2 Example applications examples(https://docs.px4.io/main/en/ros2/user_guide.html#ros-2-example-applications) below provide concrete examples of how to use these topics.

Compatibility Issues

This section contains information that may affect how you write your ROS code.
ROS 2 Subscriber QoS Settings
ROS 2 code that subscribes to topics published by PX4 must specify a appropriate (compatible) QoS setting in order to listen to topics. Specifically, nodes should subscribe using the ROS 2 predefined QoS sensor data (from the listener example source code):


...
rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

subscription_ = this->create_subscription<px4_msgs::msg::SensorCombined>("/fmu/out/sensor_combined", qos,
...

This is needed because the ROS 2 default Quality of Service (QoS)(https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html) settings are different from the settings used by PX4. Not all combinations of publisher-subscriber Qos settings(https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html) are possible, and it turns out that the default ROS 2 settings for subscribing are not! Note that ROS code does not have to set QoS settings when publishing (the PX4 settings are compatible with ROS defaults in this case)

ROS 2 & PX4 Frame Conventions

The local/world and body frames used by ROS and PX4 are different.
Frame	                 PX4	                                                  ROS
Body	          FRD (X Forward, Y Right, Z Down)	             FLU (X Forward, Y Left, Z Up)
World	          FRD or NED (X North, Y East, Z Down)	         FLU or ENU (X East, Y North, Z Up)


TIP

See REP105: Coordinate Frames(TIP

See REP105: Coordinate Frames for Mobile Platforms for more information about ROS frames.) for Mobile Platforms for more information about ROS frames.
Both frames are shown in the image below (FRD on the left/FLU on the right).
![image](https://github.com/user-attachments/assets/8f07f929-4273-49fe-bddf-56bdd3052aba)

The FRD (NED) conventions are adopted on all PX4 topics unless explicitly specified in the associated message definition. Therefore, ROS 2 nodes that want to interface with PX4 must take care of the frames conventions.

    To rotate a vector from ENU to NED two basic rotations must be performed:
        first a pi/2 rotation around the Z-axis (up),
        then a pi rotation around the X-axis (old East/new North).

    To rotate a vector from NED to ENU two basic rotations must be performed:
        first a pi/2 rotation around the Z-axis (down),
        then a pi rotation around the X-axis (old North/new East). Note that the two resulting operations are mathematically equivalent.

    To rotate a vector from FLU to FRD a pi rotation around the X-axis (front) is sufficient.

    To rotate a vector from FRD to FLU a pi rotation around the X-axis (front) is sufficient.

Examples of vectors that require rotation are:

    all fields in TrajectorySetpoint(https://docs.px4.io/main/en/msg_docs/TrajectorySetpoint.html)message; ENU to NED conversion is required before sending them.
    all fields in VehicleThrustSetpoint(https://docs.px4.io/main/en/msg_docs/VehicleThrustSetpoint.html)message; FLU to FRD conversion is required before sending them.

Similarly to vectors, also quaternions representing the attitude of the vehicle (body frame) w.r.t. the world frame require conversion.

PX4/px4_ros_com(https://github.com/PX4/px4_ros_com) provides the shared library frame_transforms(https://github.com/PX4/px4_ros_com/blob/main/include/px4_ros_com/frame_transforms.h) to easily perform such conversions.


ROS, Gazebo and PX4 time synchronization

By default, time synchronization between ROS 2 and PX4 is automatically managed by the uXRCE-DDS middleware(https://micro-xrce-dds.docs.eprosima.com/en/latest/time_sync.html) and time synchronization statistics are available listening to the bridged topic /fmu/out/timesync_status. When the uXRCE-DDS client runs on a flight controller and the agent runs on a companion computer this is the desired behavior as time offsets, time drift, and communication latency, are computed and automatically compensated.

For Gazebo simulations PX4 uses the Gazebo /clock topic as time source(https://docs.px4.io/main/en/sim_gazebo_gz/#px4-gazebo-time-synchronization)instead. This clock is always slightly off-sync w.r.t. the OS clock (the real time factor is never exactly one) and it can can even run much faster or much slower depending on the user preferences(http://sdformat.org/spec?elem=physics&ver=1.9). Note that this is different from the simulation lockstep(https://docs.px4.io/main/en/simulation/#lockstep-simulation) procedure adopted with Gazebo Classic.

ROS2 users have then two possibilities regarding the time source(https://docs.px4.io/main/en/sim_gazebo_gz/#px4-gazebo-time-synchronization) of their nodes.


ROS2 nodes use the OS clock as time source (https://docs.px4.io/main/en/ros2/user_guide.html#foxy)
ROS2 nodes use the Gazebo clock as time source (https://docs.px4.io/main/en/ros2/user_guide.html#foxy)
First you will need to install Gazebo Garden, as by default Foxy comes with Gazebo Classic 11.

Then to install the interface packages for use with ROS 2 "Foxy" and Gazebo (Ubuntu 20.04):

sudo apt install ros-foxy-ros-gzgarden

ROS 2 Example Applications 
ROS 2 Advertiser 
Offboard Control 
Using Flight Controller Hardware 
Custom uORB Topics 
Custom uORB Topics 
[
ros2 CLI 
ros2 topic list 
ros2 topic echo 
ros2 topic hz 
ros2 launch 



