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
**1. Clone the repository(develop branch)**
```bash
cd ~
git clone -b develop https://github.com/MII-IS/SE25.git
cd SE25
````
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

## Launch the Robot in RViz 2
```bash
ros2 launch ros2_control_demo_example_7 view_r6bot.launch.py
```
RViz 2 will open showing the robot with its six joints.

## Quick Start (If You Already Installed Everything Before)

If you have already cloned the repository, installed dependencies, and built the workspace at least once, you do not need to repeat all the steps.

Just run these commands in a new terminal to launch the robot directly:
```bash
source /opt/ros/jazzy/setup.bash
cd ~/SE25
source install/setup.bash
ros2 launch ros2_control_demo_example_7 view_r6bot.launch.py
```
This will open RViz 2 and the Joint State Publisher GUI so you can move each joint manually.

## Moving the Joints Manually
Once the robot appears in RViz 2, you can control each joint manually from the “Joint State
Publisher GUI” window.
This tool lets you change the position of each of the six joints using sliders or by entering
numeric values.
As you move each slider, the robot model updates immediately in RViz to show the new joint
configuration.
This is the simplest and most reliable way to test the robot’s movement without needing to
publish commands from the terminal.

<img width="2335" height="1240" alt="Robot ROS2Jazzy" src="https://github.com/MII-IS/SE25/blob/develop/images/RobotROS2Jazzy.jpeg" />
<img width="2335" height="1240" alt="Robot ROS2Jazzy 2" src="https://github.com/MII-IS/SE25/blob/develop/images/RobotROS2Jazzy2.jpeg" />

