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


## **2. Installation and Setup**

This section explains how to install, build, and run the robot simulation as described in the project source documentation.

---

### **2.1 Installation Steps**
Installation and build instructions are maintained in the repository source folder to avoid duplication and keep the User Manual aligned with the latest setup procedure.

Please follow the README located in:
- [src README](../../src/README.md) (the README inside the `src/` folder)

That README includes the full step-by-step procedure to:
- Install the required packages (ROS 2 Jazzy, RViz 2, colcon, rosdep, etc.)
- Resolve dependencies using `rosdep`
- Build the workspace using `colcon`
- Source the workspace environment before running the system

Before proceeding, ensure that:
- ROS 2 is correctly installed and sourced in your terminal session
- The workspace builds successfully without errors

---
### **2.2 First Launch**

After completing the installation and build process, the system can be launched from a terminal session.

1. **Source the workspace**  
   Open a terminal in the workspace root directory and source the environment:
   ```bash
   source install/setup.bash
   ```
2. **Start the simulation**
Launch the robot simulation using the command specified in the `src` README:    
 ```bash
   ros2 launch ros2_control r6bot.launch.py
 ```

3. **Expected behavior on first launch**
- RViz 2 opens automatically.
- The 3D visualization displays the robot model loaded into the scene.
- The robot appears in its predefined Home position.
- ROS 2 controllers and required nodes are initialized automatically.

4. **Initial verification**
The first launch is successful if:
- The robot model is visible and correctly rendered in RViz.
- The robot responds to user commands through the control interface.
- No critical error messages are shown in the terminal.
 
## **3. User Interface (UI) Overview** 
This section explains the main elements of the graphical interface and how users interact with the system during normal operation.


### 3.1 Main Interface Layout
When the system is running, the user interacts with two main visual components:

- **3D Simulation View (RViz)** – shows the robot, its motion, and the environment
- **Control Interface (GUI)** – used to command and monitor the robot

The following image shows the **extended robot model** as visualized during execution:
 <img width="2335" height="1240" alt="Scenario 1" src="https://github.com/MII-IS/SE25/blob/develop/images/Robot_extended.png" />  
 - **Center button**: For RESET
 - **Randomize**: Gives a random position. 

### 3.2 3D Simulation View (RViz)

The 3D view is the main visual feedback area and behaves similarly to a game camera.

From this view, users can:
- Observe the robot in real time
- See joint movements and base translation
- Zoom, rotate, and pan the camera freely

The robot cannot be controlled directly from this view. All commands are issued through the control panel.

---

### 3.3 Control Interface (GUI)

The control interface acts as the **command center** of the system.

It provides:
- **Joint sliders** for each robot joint
- **Base movement slider** (extended robot only)
- **Path Manager** for automated motion execution

Changes applied in the GUI are reflected instantly in the 3D view, allowing users to clearly see the effect of each action.

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

## 5. Controlling the Robot
Manual control allows the user to move the robot joints directly in real time using the graphical user interface.

**Steps to perform manual control:**
1. Adjust the **joint sliders** to move individual robot joints.  
2. Observe the robot updating in real time in the 3D simulation view (RViz).  
3. Press the **Reset** button to return the robot to its predefined **Home position** if needed.

Manual control is useful for testing individual joint movements, understanding robot behavior, and verifying safety during operation.


## 6. Troubleshooting & FAQ

This section provides guidance for common issues that users may encounter and information on how to obtain support.

---

### 6.1 Common Errors

**Simulation runs slowly**  
- Ensure that your GPU drivers are correctly installed.  
- Close unnecessary background applications to free system resources.

**Robot model fails to load**  
- Verify that the workspace was built successfully.  
- Ensure that the ROS 2 environment is correctly sourced in the current terminal session.

**Controllers fail to start**  
- Confirm that all required ROS 2 nodes are running.  
- Use ROS 2 tools to inspect controller status if necessary.

---
### 6.2 Support

For any issues not covered above, users can report bugs or request help via the project repository:

- **GitHub Issues:** [https://github.com/MII-IS/SE25/issues](https://github.com/MII-IS/SE25/issues)

Include the following information when submitting an issue:
- Operating system and version  
- Steps to reproduce the problem  
- Any error messages or screenshots

---

## 7. Appendix

- **DOF (Degree of Freedom):** An independent axis of motion for the robot.  
- **RViz:** ROS 2 visualization tool used to display the robot and its environment.  
- **Trajectory:** A sequence of robot poses executed over time.  
- **Emergency Stop:** A control that immediately halts all robot motion.  
- **Path Manager:** GUI component used to define and execute automated motion sequences.
