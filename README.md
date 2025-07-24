This repository contains the complete design and simulation of a **4-Wheel Differential Drive Autonomous Mobile Robot (AMR)** using **ROS 2 Humble**, **Gazebo**, and **URDF/XACRO**. The project was developed as part of a robotics internship project and includes robot modeling, simulation, teleoperation, and sensor integration (IMU and LiDAR).

---

## 🔧 Requirements

- Ubuntu 22.04
- [ROS 2 Humble Hawksbill](https://docs.ros.org/en/humble/index.html)
- Gazebo (Fortress or compatible version)

---

## 🛠️ Setup

After installing the required tools, open an ubuntu terminal and follow these steps:

```bash 
# Source the ROS 2 environment
source /opt/ros/humble/setup.bash

# Creating ROS workspace
mkdir -p ~/ws/src

# Build the workspace
colcon build

# Create the ROS 2 package
cd src
ros2 pkg create –build-type ament_cmake pkgname

# Create model and launch files according to the specific design of the robot
cd pkgname
mkdir launch model
cd model
gedit robot.xacro and gedit robot.gazebo
cd launch
gedit launch.py

# Edit package.xml and cmakelists.txt
cd pkg
gedit package.xml and cmakelists.txt

# Build the workspace
colcon build

# Source your workspace
source install/setup.bash

# Launch the simulation
ros2 launch pkgname launchfile.py
```

---

## 🎮 Teleoperation

To control the robot using keyboard:

```bash

source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# Ensure /cmd_vel topic is published
```


## 🧠 Future Work

- Add navigation stack (Nav2) for path planning
- Implement SLAM using LiDAR data
- Add sensor fusion (IMU + wheel encoders)
- Integrate with real robot hardware

![default_gzclient_camera(1)-2025-06-24T12_40_08 778891](https://github.com/user-attachments/assets/3ff26aad-ee29-4923-9f40-d72224f4d8de)

https://github.com/user-attachments/assets/ba3ce3af-f669-4a56-a548-e21b47649179
