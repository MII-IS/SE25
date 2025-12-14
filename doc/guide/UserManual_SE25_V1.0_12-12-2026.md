# **User Manual – Robot Simulation Control Software**

## **1. Introduction & System Requirements**

### **1.1 Purpose**
SE25 is a modular software system designed for **robot simulation, visualization, and control**. It enables users to:
- Simulate 6-DOF robotic manipulators, mobile robots, and custom mechanisms
- Perform path planning, inverse kinematics, and collision checking
- Build virtual environments with obstacles, fixtures, and workpieces
- Execute motion scripts and test robot behavior safely before deployment
- Iintegrate with ROS2, RViz, and Gazebo
- Import robot models in URDF, SDF, or custom formats.

### **1.2 Prerequisites**

#### **Operating System Compatibility** (TBC)
- **Linux (Ubuntu 22.04 recommended)**: full and easiest support 
- **Windows**: not supported
- **macOS** : not supported

#### **Hardware Requirements** (TBC)
- **CPU:** Dual-core or better  
- **RAM:** *8 GB minimum* (16 GB recommended)  
- **GPU:** OpenGL 3.3+ compatible GPU (dedicated GPU recommended)  
- **Storage:** ~**1 GB** free disk space  

#### **Software Dependencies**
- ROS 2 Jazzy Jalisco
- RViz 2
- CMake
- colcon
- rosdep
- Git

All required dependencies are installed during the setup process described below.


## **2. Installation and setup**  
### **2.1 Installation Steps**
Installation and build instructions are maintained in the main repository to avoid duplication.
Please follow: ```WinOnUbuntu_SE25_V1.0_2025-11-01.md``` (repository root) and ```SystemManual_SE25_V1.0_13-12-2026.md``` that you can find in ```doc/guide```. 
Official ROS 2 Jazzy documentation
Ensure that:
- ROS 2 is correctly sourced
- The workspace builds successfully using COLCON
### **2.2 First Launch**

## **3. User Interface (UI) Overview** 
The system interface consists of:
- 3D Viewport (RViz): Displays the robot and environment
- Control Panel (GUI): Used to control the robot
- Status Area: Displays warnings, errors, and collision events
