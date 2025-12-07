# SE25 — Robot Simulation & Control System

## Overview

SE25 is a modular ROS2-based robot simulation and control system.  
This repository contains:  
- the *initial robot* version (`robot_initial/`) — baseline robot configuration and control,  
- an *extended robot* version (`robot_extended/`) — with enhanced configurations, demos, and `ros2_control` integration examples,  
- demo applications illustrating motion control, hardware interface usage, controller management and example trajectories.  

The “extended” branch supports experiments, demos, and enhancements over the base robot.

---

## Demos & Example Applications

This section describes the available demos in the project, how to run them, and what to expect.

| Demo / Example | Location (package / folder) | Description | How to run |
|---------------|-----------------------------|-------------|------------|
| **Basic Extended Robot** | `robot_initial/` | Baseline robot configuration, simulation and control. | Follow instructions in `robot_initial/README.md`. |
| **Extended Robot + ros2_control Demo Description** | `robot_extended/ros2_control_demo_description/` | Description files (URDF / XACRO / robot description) for using `ros2_control` with the extended robot. Useful for hardware-interface experiments or control-framework testing. | Build the workspace; then launch using the relevant launch file (e.g. `<demo_description_launch>.launch.py`). |
| **Example 7 — ros2_control Demo Example** | `robot_extended/ros2_control_demo_example_7/` | A complete demo showing a configured robot with controllers, suitable for testing joint control, controllers switching, or hardware abstraction — inspired by standard `ros2_control` demos. | See `ros2_control_demo_example_7/README.md` for build and run instructions. Typically: `colcon build`, then `ros2 launch …` or `ros2 run …`. |


