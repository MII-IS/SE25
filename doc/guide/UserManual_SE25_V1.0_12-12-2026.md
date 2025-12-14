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
This section explains the main elements of the graphical interface and how users interact with the system during normal operation.


### 3.1 Main Interface Layout
When the system is running, the user interacts with two main visual components:

- **3D Simulation View (RViz)** – shows the robot, its motion, and the environment
- **Control Interface (GUI)** – used to command and monitor the robot

The following image shows the **extended robot model** as visualized during execution:
 <img width="2335" height="1240" alt="Scenario 1" src="https://github.com/MII-IS/SE25/blob/develop/images/Robot_extended.png" />  

### 3.2 3D Simulation View (RViz)

The 3D view is the main visual feedback area and behaves similarly to a game camera.

From this view, users can:
- Observe the robot in real time
- See joint movements and base translation
- Detect collisions and abnormal behavior
- Zoom, rotate, and pan the camera freely

The robot cannot be controlled directly from this view. All commands are issued through the control panel.

---

### 3.3 Control Interface (GUI)

The control interface acts as the **command center** of the system.

It provides:
- **Joint sliders** for each robot joint
- **Base movement slider** (extended robot only)
- **Enable / Disable button** to activate motion
- **Emergency Stop** for immediate halt
- **Path Manager** for automated motion execution

Changes applied in the GUI are reflected instantly in the 3D view, allowing users to clearly see the effect of each action.

---

### 3.4 Status and Feedback

The interface continuously provides feedback to the user, including:
- Collision warnings
- Execution status of automated paths
- Error or safety notifications

If a collision or unsafe condition is detected, the system immediately stops the robot and displays a warning message.

This feedback loop ensures that users always understand the current state of the simulation.

## 4. Setting Up the Simulation Scene

This section describes how the simulation scene is initialized and how the user can restore it to a known state during execution.

---

### 4.1 Loading a Robot

When the system is launched, a robot model is loaded automatically as part of the startup process.

The available robot configurations are:
- `robot_initial`
- `robot_extended`

The robot model is selected through the system launch configuration. The user is not required to perform any additional actions to load the robot, and no model selection is exposed through the graphical user interface.

---

### 4.2 Simulation Environment

Once the system starts, the simulation environment is initialized automatically.

The environment includes:
- A fixed floor used as a reference surface
- Coordinate reference frames used for visualization and motion tracking

The environment configuration is predefined. Interaction with or modification of the environment is not performed through the user interface.

---

### 4.3 Resetting the Scene

At any time during operation, the user can restore the system to a known initial state.

This can be done by:
- Pressing the **Reset** button in the graphical user interface

Resetting the scene returns the robot to its predefined **Home position** and stops any ongoing motion, allowing the user to safely resume operation from a stable configuration.
