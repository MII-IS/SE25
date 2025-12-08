# Robot Model Evolution: From Initial 6-DOF Arm to Extended Mobile-Base Version

## 1. Introduction

This document describes the evolution of the 6-DOF robotic arm model used in the project, including: 
- Evolution of the robot model  
- Differences between the initial and extended versions  
- ros2_control description and demo packages  
- Repository structure  
- Full instructions to build and run the simulation in RViz  
- Proposed enhancements for future development  

Originally, the system implemented a standard position-controlled 6-joint manipulator from the **ROS 2 Control Demos (Example 7)**. The robot could be manipulated through the **Joint State Publisher GUI**, which provided sliders to adjust each of the six revolute joints.

While this allowed full control of the arm configuration, the robot’s base was fixed to the environment, making it impossible to reposition the entire robot during simulation. This limitation motivated the introduction of an enhancement: enabling horizontal translation of the robot base through an additional GUI slider.

The goal of this enhancement was to make the model more flexible, realistic, and easier to test in different spatial configurations.

---

## 2. Initial Robot Model (Before Enhancement)

The initial version of the robot included:

- Six revolute joints (`joint_1` to `joint_6`)
- A fixed joint connecting the robot base to the world frame
- Only angular control through GUI sliders
- No possibility of translating the robot in the environment
<img width="2335" height="1240" alt="Robot ROS2Jazzy" src="https://github.com/MII-IS/SE25/blob/develop/images/RobotROS2Jazzy.jpeg" />
<img width="2335" height="1240" alt="Robot ROS2Jazzy 2" src="https://github.com/MII-IS/SE25/blob/develop/images/RobotROS2Jazzy2.jpeg" />

### Limitations

- The robot could not be repositioned in the workspace.
- Any change in the robot’s global position required manually editing the URDF.
- No support for testing scenarios requiring variable robot placement.
---

## 3. Extended Robot Model (After Enhancement)

To improve usability and flexibility during simulation, the base joint was modified:

- The original fixed joint  
  ```bash
  <joint name="base_joint" type="fixed">
  ```
  was replaced by a prismatic joint:
  ```bash
  <joint name="base_slide_x" type="prismatic">
  ```
This change enables the robot base to translate along the X-axis, which adds a new degree of freedom and introduces a new slider in the GUI.

### Effects of the Enhancement

- A new slider appears in the Joint State Publisher GUI.
- Users can now translate the robot horizontally within RViz.
- The model becomes more suitable for testing scenarios involving positional variation.
- The modification is fully backward compatible with the original arm structure.
<img width="2335" height="1240" alt="Robot_extended" src="https://github.com/MII-IS/SE25/blob/develop/images/Robot_extended.png" />


### Motivation

- More realistic and flexible testing.
- Ability to adjust the robot’s placement without editing URDF files.
- Supports extended system requirements.
- Enables new test cases in the validation pipeline.

---

## 4. Repository Organization

For clarity, the repository stores both robot models:
```
├── doc/
│ ├── deliverable/
│ │ └── archives/
│       ├── obot_extended_archived_2025-12-08/
│       └── robot_initial_archived_2025-12-08/
│
└── src/
  ├── ros2_control_demo_description/
  └── ros2_control_demo_example_7/
```

The versions located in archives are **obsoleted** versions of the Robot (Initial Robot Model). For the active development version, please refer to the root src/ directory ( Extended Robot Model ).

---

## 5. Running the Extended Robot Model

The extended model is the active version located inside `src/`.  
Below are the full setup and launch instructions.

---

### 6-DOF Robot Arm Simulation with ROS 2 Jazzy 
This setup runs a 6-joint robotic arm using the official ROS 2 Control demos. 
The robot is visualized in RViz 2, and each joint can be moved individually.
Everything works on Ubuntu 24.04 LTS (also inside WSL 2 on Windows). 


#### Source 
The example comes from the official ROS 2 Control Demos repository on GitHub: 
- **ros-controls/ros2_control_demos**: This repository provides examples to illustrate ros2_control and ros2_controllers. - **Example 7** is used here, which contains a simple 6-DOF robot model with position control.
---
- #### Requirements
- Install these packages first:
```bash
sudo apt update
sudo apt install -y \
  build-essential git python3-colcon-common-extensions python3-rosdep \
  ros-jazzy-desktop ros-jazzy-rviz2

sudo rosdep init
rosdep update
```
#### Steps to run the project 
**1. Clone the repository(develop branch)**
```bash
cd ~
git clone -b develop https://github.com/MII-IS/SE25.git
cd SE25
```

**2. Load ROS 2 environment**
```bash
source /opt/ros/jazzy/setup.bash
```
**3. Install dependencies**
```bash
rosdep install --from-paths src --ignore-src -r -y
```
**4. Build the workspace**
```bash
colcon build --symlink-install
```
**5. Activate the workspace**
```bash
source install/setup.bash
```
### Launch the Robot in RViz 2
```bash
ros2 launch ros2_control_demo_example_7 view_r6bot.launch.py
```
RViz 2 will open showing the robot with its six joints. 

### Quick Start (If You Already Installed Everything Before) 
If you have already cloned the repository, installed dependencies, and built the workspace at least once, you do not need to repeat all the steps. Just run these commands in a new terminal to launch the robot directly:
```bash
source /opt/ros/jazzy/setup.bash
cd ~/SE25
source install/setup.bash
ros2 launch ros2_control_demo_example_7 view_r6bot.launch.py
```
This will open RViz 2 and the Joint State Publisher GUI so you can move each joint manually, as well as adjust the position of the robot base.

### Moving the Joints Manually
Once the robot appears in RViz 2, you can control each joint manually from the *Joint State Publisher GUI* window.  
This tool lets you:

- Change the position of each of the six revolute joints using sliders or numeric input  
- Translate the robot base along the X-axis using the new base slider

As you move any slider, the robot model updates immediately in RViz to reflect the new configuration.  
This is the simplest and most reliable way to test the robot’s movement—including base translation—without needing to publish commands from the terminal.


