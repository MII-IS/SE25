# **User Manual – Robot Simulation Control Software**

## **1. Introduction & System Requirements**

### **Purpose**
SE25 is a modular software system designed for **robot simulation, visualization, and control**. It enables users to:
- Simulate 6-DOF robotic manipulators, mobile robots, and custom mechanisms
- Perform path planning, inverse kinematics, and collision checking
- Build virtual environments with obstacles, fixtures, and workpieces
- Execute motion scripts and test robot behavior safely before deployment
- Iintegrate with ROS2, RViz, and Gazebo
- Import robot models in URDF, SDF, or custom formats.

### **Prerequisites**

#### **Operating System Compatibility** (TBC)
- **Linux (Ubuntu 22.04 recommended)**: full and easiest support 
- **Windows 10/11**
- **macOS** — limited support (no full ROS2 toolchain)

#### **Hardware Requirements** (TBC)
- **CPU:** Dual-core or better  
- **RAM:** *8 GB minimum* (16 GB recommended)  
- **GPU:** OpenGL 3.3+ compatible GPU (dedicated GPU recommended)  
- **Storage:** ~**1 GB** free disk space  

#### **Software Dependencies**
- Python 3.8+
- ROS2 Humble / Iron
- RViz2
- Gazebo / Ignition
- OpenGL-compatible graphics drivers
- Python packages listed **TBU**

## **2. Installation and setup**  
### **Installation Steps**
Installation and build instructions are maintained in the main repository README to avoid duplication.
Please follow:
README.md (repository root)
Official ROS 2 Jazzy documentation
Ensure that:
ROS 2 is correctly sourced
The workspace builds successfully using colcon
