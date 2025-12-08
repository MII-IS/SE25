# 6-DOF Robot Arm Simulation with ROS 2 Jazzy

This workspace contains the **extended version** of the 6-DOF robotic arm based on the official ROS 2 Control Demos (Example 7).  
The robot is visualized in RViz 2, and its joints can be moved individually through the Joint State Publisher GUI.  
In this enhanced model, the **robot base can also be translated along the X-axis** using an additional slider.

Everything works on Ubuntu 24.04 LTS (including WSL 2).

---

## Source

The robot implementation is based on:

- **ros-controls/ros2_control_demos** – Official demos for `ros2_control`  
- **Example 7** – Simple 6-DOF robot model with position control  

This workspace extends the original model by introducing a prismatic base joint.

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

<img width="2335" height="1240" alt="Robot_extended" src="https://github.com/MII-IS/SE25/blob/develop/images/Robot_extended.png" />


