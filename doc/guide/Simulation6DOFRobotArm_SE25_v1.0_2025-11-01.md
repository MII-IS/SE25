# 6-DOF Robot Arm Simulation with ROS 2 Jazzy

This setup runs a 6-joint robotic arm using the official ROS 2 Control demos.  
The robot is visualized in RViz 2, and each joint can be moved individually.  
Everything works on Ubuntu 24.04 LTS (also inside WSL 2 on Windows).

---

## Source

The example comes from the official ROS 2 Control Demos repository on GitHub:

- **ros-controls/ros2_control_demos**: This repository provides examples to illustrate `ros2_control` and `ros2_controllers`.  
- **Example 7** is used here, which contains a simple 6-DOF robot model with position control.

---

## Requirements

Install these packages first:

```bash
sudo apt update
sudo apt install -y \
  build-essential git python3-colcon-common-extensions python3-rosdep \
  ros-jazzy-desktop ros-jazzy-rviz2

sudo rosdep init
rosdep update
```

## Steps to run the project
**1. Load the ROS 2 environment**
source /opt/ros/jazzy/setup.bash

**2. Create a workspace**
mkdir -p ~/ros2_robot_ws/src
cd ~/ros2_robot_ws/src

**3. Clone the demos (Jazzy branch)**
git clone -b jazzy https://github.com/ros-controls/ros2_control_demos.git

**4. Install dependencies and build only what is needed**
cd ~/ros2_robot_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select ros2_control_demo_description ros2_control_demo_example_7

**5. Activate the workspace**
source install/setup.bash

## Launch the Robot in RViz 2
ros2 launch ros2_control_demo_example_7 view_r6bot.launch.py
RViz 2 will open showing the robot with its six joints.

## Moving the Joints Manually
Once the robot appears in RViz 2, you can control each joint manually from the “Joint State
Publisher GUI” window.
This tool lets you change the position of each of the six joints using sliders or by entering
numeric values.
As you move each slider, the robot model updates immediately in RViz to show the new joint
configuration.
This is the simplest and most reliable way to test the robot’s movement without needing to
publish commands from the terminal.

<img width="2335" height="1240" alt="End-User Requirements Diagram" src="https://github.com/MII-IS/SE25/blob/main/images/end-user.png" />
<img width="2335" height="1240" alt="End-User Requirements Diagram" src="https://github.com/MII-IS/SE25/blob/main/images/end-user.png" />



