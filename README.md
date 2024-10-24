# Gazebo_ROS2_Tutorial.


Installation guide
_____________________________________________________________________________________________________________________________________________________________________


# Unity Drone System

## Table of Contents
1. [Introduction](#introduction)
2. [Prerequisites](#prerequisites)
3. [Setup Instructions](#setup-instructions-1)



## Introduction
This project is a Gazebi-based drone simulation system designed for controlled navigation. Near future will be autonomous with swarm of drones (Master-slave)


## Prerequisites
Before you begin, ensure you have the following installed on your system:


- ## Setup Commands
  
Launch Gazebo by running: # Fortress and Citadel use "ign gazebo" instead of "gz sim"
```bash
gz sim shapes.sdf
```

This command will launch both the Sim server and Sim GUI with a world that contains three simple shapes..a
Add the -v 4 command line argument to generate error, warning, informational, and debugging messages on the console.
Fortress and Citadel use "ign gazebo" instead of "gz sim"
```bash
gz sim shapes.sdf -v 4 
```
Gazebo Sim can also be run headless, i.e. without the GUI, by using the -s (server only) flag.
```bash
gz sim -s shapes.sdf -v 4  # Fortress and Citadel use "ign gazebo" instead of "gz sim"
```
