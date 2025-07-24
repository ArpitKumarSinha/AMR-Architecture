This repository contains the complete design and simulation of a **4-Wheel Differential Drive Autonomous Mobile Robot (AMR)** using **ROS 2 Humble**, **Gazebo**, and **URDF/XACRO**. The project was developed as part of a robotics internship project and includes robot modeling, simulation, teleoperation, and sensor integration (IMU and LiDAR).

---

## 🔧 Requirements

- Ubuntu 22.04
- [ROS 2 Humble Hawksbill](https://docs.ros.org/en/humble/index.html)
- Gazebo (Fortress or compatible version)
- colcon build tools

---

## 🛠️ Installation and Setup

```bash
# Clone this repository
git clone https://github.com/<your-username>/amr_4wd.git
cd amr_4wd/

# Source your ROS 2 environment
source /opt/ros/humble/setup.bash

# Build the workspace
colcon build

# Source your workspace
source install/setup.bash

# Launch the simulation
ros2 launch amr_4wd amr_sim.launch.py
