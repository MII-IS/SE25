# SE25 System Developer Manual

## Introduction

This document constitutes the **System Developer Manual** for the SE25 Robot Simulation and Control System. Its purpose is to provide technical guidance for developers who wish to understand, build, extend, or maintain the system.

Unlike a user manual, this document is targeted at **software developers and integrators**. It focuses on the internal structure of the codebase, the build and execution process, and the core architectural and implementation concepts that underpin the system.

The SE25 system is implemented as a ROS 2–based simulation and control environment for a 6-DOF robotic manipulator. The software follows a modular architecture and adheres to established software engineering practices.

This document is intended to evolve alongside the codebase and should be updated as new features, components, or integration mechanisms are introduced.

## 1. Getting Started (Build & Run)

This section details the steps to set up the development environment, install dependencies, and build the project from source.

### 1.1. Repository Cloning

Clone the repository into your ROS 2 workspace `src` directory. We strictly use the `develop` branch for integration.

```bash
cd ~
git clone -b develop https://github.com/MII-IS/SE25.git
cd SE25
```
### 1.2 Dependencies

#### Required Languages and Toolchain
- **C++17** (via GCC, installed through `build-essential`)
- **Python 3** (used by ROS 2 tooling such as `colcon` and `rosdep`)

#### Frameworks and Libraries
- **ROS 2 Jazzy Jalisco** – core middleware and runtime
- **RViz 2** – 3D visualization and debugging tool
- **CMake** – build system for C++ projects
- **colcon** – ROS 2 workspace build tool
- **rosdep** – ROS dependency management tool

#### Installation (Ubuntu 24.04 LTS)

All required dependencies can be installed using the following commands:

```bash
sudo apt update
sudo apt install -y \
  build-essential git \
  python3-colcon-common-extensions python3-rosdep \
  ros-jazzy-desktop ros-jazzy-rviz2

sudo rosdep init
rosdep update
```

### 1.3 Build Instructions

This repository is organized as a **ROS 2 workspace**, with all custom packages located in the `src/` directory. The project is built using `colcon`, the standard build tool for ROS 2.

Before building, ensure that the ROS 2 environment is sourced:

```bash
source /opt/ros/jazzy/setup.bash
```

Install any missing ROS package dependencies and build the workspace:
```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```
#### 1.3.1 Developer / Debug Build
If low-level debugging is required (e.g., using GDB or analyzing crashes), you can build with debug symbols enabled:
```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

### 1.4 Running the System 
After a successful build, source the local workspace:
```bash
source install/setup.bash
```
This step must be repeated in every new terminal session before running or developing the system.

####Running the Simulation
Once the workspace is built and sourced, the simulation can be launched using the provided ROS 2 launch files. For example, to start the 6-DOF robot visualization in RViz:
```bash
ros2 launch ros2_control_demo_example_7 view_r6bot.launch.py
```
This launch file initializes the robot model, controllers, and RViz visualization.

## 2. High-Level Architecture

This section describes the high-level organization of the repository and the main execution flow of the simulation and control system.

### 2.1 Directory Structure

The SE25 repository follows a ROS 2 workspace layout, extended with dedicated directories for documentation, system modeling, and design artifacts:

```
SE25/
├── doc/                            # Project documentation
│   ├── deliverable/                # Formal deliverables
│   │   ├── archive/                # Archived versions of deliverables
│   │   │   ├── robot_extended_archived_2025-12-08 
│   │   │   └── robot_initial_archived_2025-12-08
│   │   ├── SDD_SE25_*.md           # Software Design Document
│   │   ├── SRS_SE25_*.md           # Software Requirements Specification
│   │   └── UMLToolEvaluation*.md   # UML tool evaluation report
│   └── guide/                      # Internal development guides
│   |   ├── CodeStyleGuide_*.md
│   |   ├── ConfigurationManagementPlan_*.md
│   |   ├── InverseEngineering_*.md│       
│   |   ├── SimulationRobotRviz_*.md
│   |   ├── SoftwareDevelopmentPlan_ *.md
│   |   ├── UserManual_  *.md
│   |   └── WinOnUbuntu *.md
|   |
|   |── tests/
│         ├── test_bailarin.cpp
|         ├── test_path_manager.cpp
|
│
├── images/                         # Diagrams and visual assets (UML, SysML, architecture)
│
├── models/                         # System and software models
│   ├── Requisitos/                 # Requirements and SysML models
│   ├── Reverse_model/              # Reverse-engineered UML models
│   └── README.md
│
├── src/                            # Core ROS 2 source code
│   ├── ros2_control_demo_description/
│   ├── ros2_control_demo_example_7/
│   └── README.md
│
├── .gitignore
└── README.md
```

###Directory Responsibilities
- **doc/**  
  Contains all project documentation. The `deliverable` subdirectory stores formal documents such as the SRS and SDD, while `guide` includes internal development and configuration guides intended for the project team.

- **images/**  
  Stores diagrams and visual artifacts used throughout the documentation, including UML, SysML, and architectural figures.

- **models/**  
  Contains UML and SysML models developed using Modelio. These models support requirements traceability, architectural design, and reverse engineering activities.

- **src/**  
  Hosts the core implementation of the system, organized as ROS 2 packages. This includes robot descriptions (URDF/XACRO), controller configurations, launch files, and runtime logic.


### 2.2 The Simulation Loop

The system follows a repetitive simulation and control cycle based on ROS 2 communication and the `ros2_control` framework. This loop ensures that the robot responds to user commands and that its state is continuously updated and visualized.

The loop consists of the following steps:

1. **Apply control commands**  
   User inputs (for example, from GUI sliders) are converted into joint position commands and published through ROS 2 topics.

2. **Update controller state**  
   The `ros2_control` framework processes the incoming commands and updates the internal state of the robot joints according to the active controllers.

3. **Publish robot state**  
   The updated joint positions are published to the ROS 2 network (e.g., via the `/joint_states` topic), making the current robot configuration available to other components.

4. **Render the robot state**  
   RViz 2 subscribes to the published state information and renders the updated robot model in real time.

This loop runs continuously while the system is active, enabling real-time control and visualization of the robotic arm.


## 3. Core Concepts & Implementation

This section details how the robot is kinematically defined, the kinematic structure derived from the source code, and how the software interfaces with the simulation backend via `ros2_control`.

### 3.1 Robot Representation
The robot's physical structure and kinematics are defined using **URDF (Unified Robot Description Format)** augmented with **Xacro** macros for modularity.

* **Definition Source:**
    The main kinematic chain is defined in the following file:
    `src/ros2_control_demo_description/r6bot/urdf/r6bot_description.xacro`

* **Kinematic Structure:**
    The robot consists of a 6-DOF manipulator mounted on a prismatic base:
    1.  **Mobile Base (`base_slider_joint`):** A **prismatic joint** connecting the `world` to the `base_link`. It allows linear translation along the X-axis.
        * *Limits:* -1.0m to 1.0m.
    2.  **Manipulator (`joint_1` to `joint_6`):** Six **revolute joints** forming the arm structure.
    3.  **End Effector:** The chain terminates at the `tool0` frame, attached via a fixed joint to `ft_frame` (Force-Torque frame).

### 3.2 Kinematics
* **Forward Kinematics (FK):**
    The system utilizes the standard **`robot_state_publisher`**. This node parses the URDF tree and subscribes to joint angles (from the simulation) to broadcast the full **TF2 (Transform)** tree. This allows for the real-time computation of every link's position in 3D space relative to the `world` frame.

* **Inverse Kinematics (IK):**
    Currently, the system operates primarily in **Joint Space Control**. Target poses are achieved by driving individual joint angles.

### 3.3 Physics & Hardware Interface
The system does not rely on an external physics engine like Gazebo for this specific configuration. Instead, it uses a **System Interface** plugin integrated directly into the `ros2_control` loop.

* **Hardware Configuration File:**
    `src/ros2_control_demo_example_7/description/ros2_control/r6bot.ros2_control.xacro`

* **The Backend Plugin:**
    The system loads a custom C++ plugin defined in the configuration:
    ```xml
    <hardware>
        <plugin>ros2_control_demo_example_7/RobotSystem</plugin>
    </hardware>
    ```
    The `RobotSystem` class acts as the simulation backend (Mock Hardware). It receives read/write calls from the controller manager and simulates the motor response (perfect execution) and sensor feedback.

* **Exposed Interfaces:**
    The hardware exposes the following interfaces to the controllers:
    * **Command Interfaces:** `position` and `velocity` (available for all 7 joints).
    * **State Interfaces:** `position` and `velocity` (feedback from the simulation).
    * **Sensors:** A Force-Torque Sensor (`tcp_fts_sensor`) is defined at the tool center point, exposing 6 state interfaces: `force.x`, `force.y`, `force.z`, `torque.x`, `torque.y`, `torque.z`.

### 3.4 Controller Interface
Control algorithms inject commands through the **ROS 2 Controller Manager**, which abstracts the hardware details.

* **Command Injection:**
    Controllers (such as the `ForwardCommandController` used in the demo) publish to the standard hardware interfaces defined in the URDF.
* **Modifying Control Logic:**
    To add a new control algorithm (e.g., PID or Impedance Control), developers must create a class inheriting from `controller_interface::ControllerInterface`. This class must calculate the error between the desired state and the feedback from the **State Interfaces** and write the result to the **Command Interfaces**.

## 4. API & External Communication

This section describes how the SE25 system communicates with external processes and how it can be executed without graphical interfaces for development or automation purposes.

### 4.1 Inter-Process Communication (IPC)

Inter-process communication in the SE25 system is handled entirely through **ROS 2 middleware**. All components of the system communicate using ROS 2 nodes, topics, and standardized message types.

External applications or scripts (e.g., Python-based tools, analysis scripts, or additional ROS 2 nodes) can interact with the simulation by subscribing to or publishing ROS 2 topics.

Key characteristics of the communication model include:

- **Communication mechanism:**  
  ROS 2 publish/subscribe model built on top of DDS (Data Distribution Service).

- **Data exchange:**  
  Joint commands, joint states, and other system information are exchanged through ROS 2 topics such as `/joint_states`.

- **Language interoperability:**  
  ROS 2 enables communication between nodes written in different languages (e.g., C++ and Python) without requiring custom bindings or adapters.

No custom IPC mechanisms (such as shared memory segments, raw TCP/IP sockets, or ZeroMQ) are implemented at the application level. ROS 2 abstracts these details and provides a standardized and extensible communication layer.

---

### 4.2 Headless Mode (Non-Graphical Execution)

The SE25 system can be executed in **headless mode**, meaning without launching any graphical user interface or visualization tools. This is useful for automated testing, batch execution, or integration in CI/CD pipelines.

Headless execution is achieved by launching only the required control and simulation nodes, while omitting visualization components such as **RViz 2**.

For example, developers can avoid launching RViz by:

- Running launch files that do not include visualization nodes, or
- Modifying existing launch files to disable RViz-related nodes.

As long as the ROS 2 nodes responsible for control and state publishing are running, the system remains fully operational without graphical output.

---

## 5. Testing & Validation

This section outlines the testing strategy and procedures to verify the system requirements. The project employs a dual approach: **Validation Testing** (verifying the system meets requirements) and **Defect Testing** (stressing the system with edge cases).

### 5.1 Testing Strategy

We utilize **Equivalence Partitioning** to define test cases that cover the "space of possibilities" for robot trajectories (REQ-F-006). Inputs are categorized into three partitions to ensure robust coverage:

1.  **Valid Sequences (Validation):** Standard multi-point trajectories to confirm the "Happy Path" and functional compliance.
2.  **Boundary Sequences (Robustness):** Edge cases, such as single-point trajectories, to verify logical limits.
3.  **Invalid Sequences (Defect Testing):** Abnormal inputs (e.g., empty sequences or unreachable targets) to verify error handling and system stability without crashing.

### 5.2 Running Automated Verification

A dedicated Python script (`automated_test.py`) has been developed to perform automated functional verification of the robot's trajectory logic without requiring physical hardware.

**Prerequisites:**
The simulation environment must be running before executing the tests.

**Execution Steps:**

1.  **Launch the Simulation:**
    Open a terminal and launch the robot in RViz:
    ```bash
    ros2 launch ros2_control_demo_example_7 view_r6bot.launch.py
    ```
    *(Note: Ensure the "Joint State Publisher GUI" window is closed or untouched to avoid interference).*

2.  **Run the Test Script:**
    In a separate terminal, navigate to the source directory and execute the test node:
    ```bash
    python3 src/ros2_control_demo_example_7/test/automated_test.py
    ```

**Expected Output:**
The script will sequentially execute three specific test cases (`TC-TRAJ-01`, `TC-TRAJ-02`, `TC-TRAJ-04`). The terminal will display pass/fail logs, and the robot in RViz will perform the corresponding movements:
* **Validation:** Execution of a smooth, multi-point path.
* **Boundary:** Execution of a single-point movement.
* **Defect:** Rejection of an empty sequence (logs will show error detection) and the robot will return to Home.

### 5.3 Standard Unit Testing

For internal logic and package integrity, standard ROS 2 unit tests are maintained. These can be executed using the build tool:

```bash
# Run all tests in the workspace
colcon test --event-handlers console_direct+

# Check detailed results
colcon test-result --all
```


## 6. Contribution Guidelines

This section defines the contribution workflow and quality requirements for developers contributing to the SE25 project. All contributions must follow these guidelines to ensure code consistency, maintainability, and system stability.

By contributing to the SE25 project, developers agree to follow these guidelines in order to maintain a consistent and collaborative development process.



This approach allows the simulation and control logic to be executed in environments where a display server is not available.

---

### 6.1 Branching Strategy

The SE25 repository follows a lightweight branching strategy aligned with GitHub’s patch-based workflow:

- **main**  
  Stable branch containing reviewed and validated code only.  
  Direct commits to this branch are not allowed.

- **patch branches**  
  All contributions must be developed in patch branches created directly from `main`.

**Branch naming convention:**    
username-patch-number


**Workflow summary:**
1. Create a new patch branch from `main`
2. Implement the required changes
3. Validate the changes locally
4. Open a Pull Request targeting `main`
5. Address review comments if required
6. Merge after approval

---

### 6.2 Code Style and Formatting

To maintain readability and consistency across the codebase, contributors must adhere to the following code style rules:

#### C++ Code
- **Standard:** C++17
- **Formatter:** `clang-format`
- All C++ source files (`.cpp`, `.hpp`) must be formatted using the project’s `.clang-format` configuration (if provided).


---

### 6.3 Pull Request Checklist

Before submitting a Pull Request, ensure that all the following requirements are met:

- Code builds successfully using `colcon build`
- No existing functionality is broken
- Changes are limited to the scope of the patch
- New or modified behavior is documented where applicable
- Code adheres to the defined formatting and style guidelines
- Commit messages are clear and descriptive
- The Pull Request description clearly explains what was changed and why

Pull Requests that do not meet these criteria may be requested for revision or rejected.

---

## 7. Documentation of "Gotchas" / Known Limitations
This section documents known limitations, quirks, and common pitfalls encountered during the development of the SE25 system. These "gotchas" are inherent to the current architectural choices (Mock Hardware and no active IK).
### 7.1 Performance Bottlenecks & Concurrency Issues
#### The "Joint State" Race Condition: 
The most critical performance issue in the current setup arises when multiple nodes attempt to publish to the /joint_states topic simultaneously.
- **Scenario:** Running the joint_state_publisher_gui (the window with sliders) while simultaneously running an automated control node (e.g., move_robot_node).
- **Symptom:** The robot visualization in RViz will jitter, flicker, or jump rapidly between two positions. This happens because RViz receives conflicting state messages: one from the sliders (static) and one from the controller (moving).
- **Workaround:** Always terminate the GUI slider application before launching any automated control node.
#### RViz Rendering Load:
 While the control logic is lightweight, the visualization of the high-fidelity meshes in RViz 2 can be GPU-intensive. If the simulation lags, consider disabling "Shadows" in the RViz Global Options or reducing the update rate of the robot_state_publisher.

### 7.2 Physics Quirks & Simulation Limitations
Since the system runs on a Mock Hardware Interface (RobotSystem plugin) rather than a full physics engine (like Gazebo or Ignition), the following physical behaviors are not simulated:
#### Perfect Actuation (No Dynamics):
The mock hardware assumes "perfect execution." It does not simulate gravity, friction, inertia, or motor torque limits. If a controller commands a joint to move from 0° to 90° in 0.01 seconds, the visualization will show it happening instantly, even if physically impossible.
#### No Collision Detection:
The current mock backend acts purely as a kinematic state echo. It does not perform collision checking.
- **Quirk:** The robot arm can pass through the floor, the base, or itself without stopping or generating an error. Collision avoidance must be handled at the planning layer (e.g., MoveIt), which is not active in the base configuration.
#### Lack of Inverse Kinematics (IK): 
As discovered during the implementation of REQ-F-006, the base robot description does not include a pre-configured IK solver.
- **Limitations:** The robot cannot inherently accept Cartesian commands (e.g., "Go to X=0.5, Y=0.2"). It only accepts Joint Space commands (angles).
- **Implication of Developers:** Any test or feature involving Cartesian paths requires either implementing a custom IK solver or integrating the MoveIt 2 framework.
### 7.3 Environment & Build Gotchas
#### Sourcing the Overlay:
A frequent source of "Command not found" or "Package not found" errors is the failure to source the local workspace overlay.
- **Rule:** You must run source install/setup.bash in every new terminal window, even if you have already sourced the global ROS 2 installation.
#### Colcon Symlink Install:
When modifying Python launch files or descriptions, changes might not appear unless the project was built with symbolic links.
- **Recommendation:** Always build using colcon build --symlink-install. This allows you to modify launch files and Python scripts without recompiling every time.




