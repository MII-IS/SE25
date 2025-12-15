# Robot Model Evolution: From Initial 6-DOF Arm to Extended Mobile-Base Version

## 1. Introduction

This document describes the evolution of the 6-DOF robotic arm model used in the project, including: 
- Evolution of the robot model  
- Differences between the initial and extended versions  
- ros2_control description and demo packages  
- Repository structure   

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

To improve usability and flexibility during simulation, the URDF description file was modified. The base joint configuration was updated as follow:

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
│ │ └── archive/
│       ├── robot_extended_archived_2025-12-08/
│       └── robot_initial_archived_2025-12-08/
│
└── src/
  ├── ros2_control_demo_description/
  └── ros2_control_demo_example_7/
```

The versions located in `archive/` are obsolete iterations of the Robot. They are preserved as a historical record for documentation purposes and to ensure the original configuration remains accessible, should it be necessary to revert to the initial version in the future. For the active development version, please refer to the root `src/` directory (Extended Robot Model).


