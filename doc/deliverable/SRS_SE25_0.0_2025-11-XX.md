
# Software Requirements Specification (SRS)

## Table of Contents
1. [Introduction](#1-introduction)  
   1.1 [Purpose](#11-purpose)  
   1.2 [Scope](#12-scope)  
   1.3 [Glossary](#13-glossary)  
   1.4 [References](#14-references)  
   1.5 [Overview](#15-overview)  
   1.6 [Definitions, Acronyms, and Abbreviations](#16-definitions-acronyms-and-abbreviations)

2. [Overall Description](#2-overall-description)  
   2.1 [Product Perspective](#21-product-perspective)  
   2.2 [Product Functions](#22-product-functions)  
   2.3 [User Characteristics](#23-user-characteristics)  
   2.4 [Operational Scenarios](#24-operational-scenarios)  
   2.5 [Constraints](#25-constraints)  

3. [System Requirements](#3-system-requirements)  
    3.1 [External Interface Requirements](#31-external-interface-requirements)  
    3.2 [Functional Requirements (REQ-F-XXX)](#32-functional-requirements)  
    3.3 [Non-Functional Requirements (REQ-N-XXX)](#33-non-functional-requirements)   

4. [Requirements Traceability Matrix (RTM)](#4-requirements-traceability-matrix-rtm)  

---

## 1. Introduction

### 1.1 Purpose
The purpose of this Software Requirements Specification (SRS) is to formally describe the requirements of the **SE25 – Robot Simulation and Control System**. This system aims to provide a realistic and interactive simulation of a 6-axis robotic arm, enabling both manual and automated control in a virtual 3D environment.  

The SRS document serves several purposes:
- **For developers (students):** It provides a clear specification to guide system design, implementation, and testing.  
- **For the instructor (customer):** It establishes a baseline to evaluate the team’s progress and ensure alignment with the educational objectives of the course.  
- **For end-users (hypothetical researchers):** It communicates what the system is expected to do and the constraints under which it must operate.  

This document will serve as a reference throughout the entire software life cycle, ensuring traceability from requirements definition to implementation and validation.

### 1.2 Scope
The **System of Interest (SoI)** is a distributed robotic simulation and control platform. It is composed of two main subsystems:  
1. **Simulation Environment:** A physics-based 3D environment capable of rendering a robotic arm and simulating its movement under user-defined commands.  
2. **Control Workstation:** A human–machine interface (HMI) that enables operators to issue manual control commands, define automated trajectories, and monitor the robot’s state.  

The communication between these components will be managed using **ROS 2 Jazzy Jalisco**, ensuring real-time data exchange and modularity.  

The primary objectives of the system are:
- Provide a robust 3D environment for robotic arm simulation.  
- Allow users to manually manipulate the robot’s joints or execute predefined motion trajectories.  
- Detect collisions with obstacles and respond with immediate safety measures.  
- Visualize the robot’s state and motion in real-time through a user-friendly GUI.  

### 1.3 Glossary
- **SoI (System of Interest):** The robotic simulation and control software being developed.  
- **OpsCon (Operational Concept):** A user-oriented description of how the system is expected to operate.  
- **URDF (Unified Robot Description Format):** A standard XML format used to describe the structure of a robot model.  
- **ROS 2 (Robot Operating System 2):** Middleware framework for distributed robotics applications.  
- **HMI (Human-Machine Interface):** The graphical interface through which the user interacts with the system.  
- **REQ-F:** Functional Requirement.  
- **REQ-N:** Non-Functional Requirement.  
- **RTM (Requirements Traceability Matrix):** A matrix mapping requirements to scenarios and stakeholders.  


### 1.4 References
- SE25 Project Description Document (SE25-01 v1.0 Draft, September 2025).  
- ISO/IEC/IEEE 12207:2017, *Systems and software engineering — Software life cycle processes*.  
- ISO/IEC/IEEE 29148:2018, *Systems and software engineering — Requirements engineering*.  
- IEEE Computer Society, *SWEBOK Guide v4.0*, 2025. 

### 1.5 Overview
This document is structured as follows:
- **Section 1 (Introduction):** Defines the purpose, scope, terminology, and references.  
- **Section 2 (Overall Description):** Provides a high-level description of the system, including perspective, functions, users, constraints, and assumptions.  
- **Section 3 (System Requirements):** Lists and details functional and non-functional requirements with unique IDs and acceptance criteria.  
- **Section 4 (Traceability Matrix):** Maps requirements to operational scenarios and stakeholders to ensure coverage and alignment.  

### 1.6 Definitions, Acronyms, and Abbreviations
URDF: Unified Robot Description Format used to describe the robot model.
ROS 2: Robot Operating System version 2, used for communication between system components.
GUI: Graphical User Interface for user interaction with the robot.
SRS: Software Requirements Specification.

## 2. Overall Description

### 2.1 Product Perspective
The system will act as a standalone 3D simulation platform. The SE25 system is designed as a **distributed real-time robotic simulation and control platform**. It consists of two interconnected subsystems:  
1. **Simulation Environment:**  
   - Built with a physics engine capable of simulating realistic robotic arm kinematics and dynamics.  
   - Provides a 3D rendering of the robot and its environment.  
   - Responsible for collision detection and state publishing.  
2. **Control Workstation:**  
   - Provides a GUI for user interaction.  
   - Sends manual and automated commands to the simulation.  
   - Receives and displays the robot’s current state in real time.  

The communication between these subsystems is implemented using **ROS 2 Jazzy Jalisco**, enabling a modular publish–subscribe architecture. For example:  
- `/RobotState` topic: simulation publishes joint positions and end-effector pose.  
- `/ControlCommands` topic: control interface sends joint commands to the simulation.  

Thus, the system integrates **simulation accuracy** with **user interactivity** while adhering to real-time performance constraints.
<img width="2335" height="1240" alt="image" src=https://github.com/MII-IS/SE25/blob/develop/images/sysml_structure.png />



### 2.2 Product Functions
At a high level, the SE25 system will provide the following functions:
- **System Initialization:** Load the robotic arm model (URDF) into the simulation environment.  
- **Manual Control:** Allow users to jog individual joints of the robotic arm via the GUI.  
- **Automated Path Execution:** Accept a sequence of target poses and execute them as trajectories in the simulation.  
- **Collision Detection:** Identify collisions between the robotic arm and environment obstacles.  
- **Emergency Response:** Halt motion immediately upon collision or detected fault.  
- **Visualization:** Display real-time robot pose and environment state to the user.   

### 2.3 User Characteristics
The system is intended for three primary categories of stakeholders:  
- **Students (Developers):**  
  - Background: Undergraduate software engineering students.  
  - Need: A clear, structured project that allows them to apply concepts from ISO 12207, real-time programming, and software engineering best practices.  
- **Instructor (Customer):**  
  - Role: Defines the project requirements and evaluates deliverables.  
  - Need: A project that fulfills course objectives, ensures traceability of requirements, and demonstrates applied knowledge.  
- **End-User (Hypothetical Researcher):**  
  - Background: Robotics researcher.  
  - Need: A reliable, flexible, and user-friendly tool for testing control algorithms without requiring physical robots.  

### 2.4 Operational Scenarios
The scenarios to take into consideration are the following:
- **Scenario 1: System Initialization and Robot/Environment Loading**. The user starts the control interface and the simulation environment. The simulation loads a 3D model of a robotic arm into a virtual world. The control interface establishes a connection with the simulation and displays the initial state of the robot.
- **Scenario 2: Manual Control of the Robotic Arm**. The user utilizes the control interface to manually jog the joints of the robotic arm. The control commands are sent to the simulation in real-time, and the user observes the updated robot pose in the simulation.
- **Scenario 3: Automated Path Execution**. The user defines a sequence of target poses for the robotic arm. The control system calculates a trajectory and sends a stream of commands to the simulation to execute the path. The simulation visualizes the robot moving along the planned trajectory.
- **Scenario 4: Fault Detection and Response**. The simulation detects a collision between the robotic arm and an obstacle in the environment. It sends a fault signal to the control system, which halts the robot’s motion and alerts the user.

### 2.5 Constraints
The development and execution of the system are subject to the following constraints:  
- **Platform Constraint:** The system must run on **Ubuntu 24.04 LTS**.  
- **Programming Languages:** The implementation shall use **C/C++**.  
- **Concurrency:** Multithreading must rely on **POSIX threads (pthreads)**.  
- **Middleware:** All communication between components must use **ROS 2 Jazzy Jalisco**.  
- **Version Control:** All source code shall be version controlled using **Git&GitHub**.  
- **Performance:** The system shall achieve a minimum of 30 FPS rendering rate. 


---

## 3. System Requirements
This section defines the functional and non-functional requirements of the SE25 Robot Simulation and Control System.

### 3.1 External Interface Requirements
#### User Interface
The GUI shall include:
- Manual control buttons for each joint.
- A panel to select and execute predefined trajectories.
- Real-time feedback display of joint positions and status.
- A clean layout with labeled controls and tooltips.

#### Hardware Interfaces
The system simulates a robotic arm and does not interface with physical hardware. However, the architecture allows future integration with real robots via ROS 2.

#### Software Interfaces
- The system uses ROS 2 topics and services for communication.
- The robot model is loaded from a URDF file.
- The GUI interacts with the simulation via ROS 2 nodes.

#### Communication Interfaces
All communication is local and handled via ROS 2. No external network protocols or internet connectivity are required.

### 3.2 Functional Requirements

#### **Requirement ID: REQ-F-001**
| Attribute | Details |
| :--- | :--- |
| **Requirement Name** | 3D Simulation Environment |
| **Description** | The system shall provide a 3D simulation environment for a 6-axis robotic arm. |
| **Type** | Functional |
| **Source** | Instructor (Customer) |
| **Rationale** |A 3D simulation environment allows users to visualize and test robotic arm operations without the risk of damaging physical hardware. It provides a safe and cost-effective platform for prototyping, debugging motion algorithms, and training personnel. Additionally, it ensures that system behaviors can be evaluated under controlled scenarios, improving design reliability and reducing development time |
| **Priority** |Must have |
| **Verification Method**|1. Launch the simulation application. <br>2. Verify that a 3D environment is displayed correctly. <br>3. Confirm that the 6-axis robotic arm is fully rendered. <br>4. Operate each joint and validate correct kinematic behavior. <br>5. Test collision detection and ensure visual feedback responds accurately. <br>6. Confirm simulation performance is smooth under typical scenarios.|
| **Dependencies**|None|
| **Status** | Proposed |

---

#### **Requirement ID: REQ-F-002**
| Attribute | Details |
| :--- | :--- |
| **Requirement Name** | URDF Model Loading |
| **Description** | The system shall allow a user to load a URDF (Unified Robot Description Format) model of a robotic arm into the simulation.  |
| **Type** | Functional |
| **Source** | Instructor (Customer) |
| **Rationale** | Allowing URDF model loading ensures that the simulation system is flexible and can accommodate various robotic arms without requiring hard-coded models. This capability enables users to experiment with different robotic designs, verify kinematics, and validate control strategies before deploying them on real hardware, thereby increasing versatility and reducing risk.  |
| **Priority** |Must have |
| **Verification Method**| 1. Open the model loading feature feature. <br>2. Select multiple URDF files representing different robotic arms. <br>3. Verify that each robot’s visual appearance, joints, and links are correctly displayed. <br>4. Check that joint movements correspond to defined kinematics. <br>5. Validate collision models respond correctly during simulation. <br>6. Ensure no errors or crashes occur during model loading. <br>7. Inspect logs to confirm proper parsing of XML structure. |
| **Dependencies** |REQ-F-001 |
| **Status** | Proposed |

---

#### **Requirement ID: REQ-F-003**
| Attribute | Details |
| :--- | :--- |
| **Requirement Name** | Graphical User Interface (GUI) |
| **Description** | The system shall provide a graphical user interface (GUI) for controlling the robotic arm. |
| **Type** | Functional |
| **Source** | Instructor (Customer) |
| **Rationale** | A graphical user interface simplifies interaction with the robotic arm by providing intuitive controls for movement, configuration, and monitoring. It reduces the learning curve for new users, increases operational efficiency, and minimizes the risk of errors that could occur with manual command entry. A well-designed GUI also improves accessibility and supports rapid prototyping and testing.  |
| **Priority** |Must have |
| **Verification Method**|1. Launch the GUI and confirm all interface elements are visible and correctly labeled. <br>2. Operate each control (sliders, buttons, input fields) and verify robotic arm movement corresponds accurately. <br>3. Test saving and loading of configurations. <br>4. Verify emergency stop/reset functionality works. <br>5. Check that the GUI provides feedback for invalid commands or system errors. |
| **Dependencies** |REQ-F-001|
| **Status** | Proposed |

---

#### **Requirement ID: REQ-F-004**
| Attribute | Details |
| :--- | :--- |
| **Requirement Name** | Real-time Pose Display |
| **Description** | The GUI shall display the real-time position and orientation of the robotic arm. |
| **Type** | Functional |
| **Source** | Project Statement |
| **Rationale** | Displaying real-time telemetry is crucial for providing the operator with immediate feedback on the robot's status. This allows for precise control, facilitates trajectory debugging, and helps in instantly detecting anomalous behaviors, significantly improving the system's safety and operational efficiency. |
| **Priority** | Must have |
| **Verification Method**| 1. Launch the simulation and the GUI. <br>2. Move a joint of the robotic arm using the GUI controls. <br>3. Verify that the position and orientation values (Cartesian and angular coordinates) in the GUI update in real-time. <br>4. Check that the displayed values correspond to the robot's actual position in the 3D environment. <br>5. Move the end-effector to a known point and confirm that the GUI accurately reflects those coordinates. <br>6. Ensure the data update rate is smooth and does not exhibit a noticeable lag. |
| **Dependencies** | REQ-F-003 |
| **Status** | Proposed |

---

#### **Requirement ID: REQ-F-005**
| Attribute | Details |
| :--- | :--- |
| **Requirement Name** | Individual Joint Control |
| **Description** | The user shall be able to control each joint of the robotic arm individually. |
| **Type** | Functional |
| **Source** | Project Statement |
| **Rationale** | Individual control of each joint is a fundamental feature for robot operation and debugging. It allows the user to make fine adjustments to the pose, verify the range of motion for each axis, and recover the arm from singular or complex positions. Furthermore, it is an indispensable control method for calibration and maintenance tasks. |
| **Priority** | Must have |
| **Verification Method**| 1. Select a specific joint (e.g., Joint 2) in the GUI. <br>2. Use the associated control (e.g., a slider) to command movement in one direction. <br>3. Visually verify in the 3D environment that only the selected joint moves in the expected direction. <br>4. Check that the joint's movement stops when it reaches its defined software limits. <br>5. Repeat steps 1 through 4 for each of the other joints of the robotic arm. <br>6. Confirm that the telemetry displayed in the GUI (REQ-F-004) accurately reflects the current angle of the moved joint. |
| **Dependencies** | REQ-F-003, REQ-F-004 |
| **Status** | Proposed |
---

#### **Requirement ID: REQ-F-006**
| Attribute | Details |
| :--- | :--- |
| **Requirement Name** | Pose Sequence Definition and Execution |
| **Description** | The user shall be able to define and execute a sequence of target poses for the robot's end-effector. |
| **Type** | Functional |
| **Source** | Project Statement |
| **Rationale** | This functionality is essential for automating tasks and transitioning from manual control to programming complex operations. Allowing the definition and execution of sequences enables the robot to perform useful and repetitive tasks (such as pick-and-place or path following) consistently, laying the groundwork for saving and validating complete robot programs. |
| **Priority** | Must have |
| **Verification Method**| 1. Use the GUI to define a sequence of at least three distinct target poses (position and orientation) for the end-effector. <br>2. Save the created pose sequence. <br>3. Press the 'Execute Sequence' control in the GUI. <br>4. Observe in the 3D environment and verify that the robot's end-effector reaches each of the defined poses in the correct order. <br>5. Check that the trajectory between poses is smooth and controlled. <br>6. Verify that the GUI displays feedback on the execution status (e.g., 'Sequence in progress,' 'Sequence completed'). |
| **Dependencies** | REQ-F-003, REQ-F-005|
| **Status** | Proposed |

---

#### **Requirement ID: REQ-F-007**
| Attribute | Details |
| :--- | :--- |
| **Requirement Name** | Collision Detection |
| **Description** | The system shall be capable of detecting collisions between the robotic arm and predefined obstacles within the simulation environment. |
| **Type** | Functional |
| **Source** | Project Statement |
| **Rationale** | Collision detection ensures safe and reliable operation of the robotic arm during simulation by preventing physical interference between components or obstacles. This contributes to reducing risks, costs, and debugging time during development.|
| **Priority** | Must have |
| **Verification Method**| 1. Launch the simulation application. <br>2. Initialize the robotic arm and load predefined obstacles. <br>3. Move the arm to positions that intersect with obstacles. <br>4. Verify that collision detection is triggered accurately and visual feedback is displayed. |
| **Dependencies** |REQ-F-001, REQ-F-004, REQ-F-006 |
| **Status** | Proposed |

---

#### **Requirement ID:REQ-F-008**
| Attribute | Details |
| :--- | :--- |
| **Requirement Name** | Collision Stop |
| **Description** | After a collision is detected, the system shall immediately stop the movement of the robotic arm. |
| **Type** | Functional |
| **Source** | Project Statement |
| **Rationale** | Implementing an automatic stop after collision detection ensures the safety of the robotic arm and surrounding environment. It prevents hardware damage, protects users, and maintains system integrity. This functionality is essential for testing safety protocols and validating the system’s response to unexpected contact events. |
| **Priority** | Must have |
| **Verification Method**| 1. Launch the simulation environment. <br>2. Initiate robotic arm movement toward a predefined obstacle. <br>3. Trigger a collision event and verify that all arm movements stop immediately. <br>4. Confirm that no further motion commands are executed after the stop signal. <br>5. Observe the system’s feedback to ensure correct detection and response.|
| **Dependencies** | REQ-F-007 |
| **Status** | Proposed |

---

#### **Requirement ID: REQ-F-009**
| Attribute | Details |
| :--- | :--- |
| **Requirement Name** | Teach-In Programming |
| **Description** | The system shall allow the user to manually move the robot to a desired pose and then record the current position and orientation of the end-effector to add it as a waypoint to a trajectory sequence. |
| **Type** | Functional |
| **Source** | End-User (Hypothetical) |
| **Rationale** | This programming method is highly intuitive and reduces the complexity of sequence creation, as it eliminates the need to manually enter numerical coordinates. It allows the user to visually "teach" the robot a trajectory, which speeds up task development and minimizes input errors. |
| **Priority** | Should have |
| **Verification Method**| 1. Use the manual controls to move the end-effector to a Pose A. <br>2. Activate a "Record Pose" control in the GUI. <br>3. Verify that Pose A has been added to the current sequence list. <br>4. Move the robot to a Pose B and record the pose again. <br>5. Verify that Pose B has been added to the list after Pose A. <br>6. Execute the sequence and confirm that the robot moves first to Pose A and then to Pose B. |
| **Dependencies** | REQ-F-005, REQ-F-006 |
| **Status** | Proposed |

---

#### **Requirement ID: REQ-F-010**
| Attribute | Details |
| :--- | :--- |
| **Requirement Name** | Manual Emergency Stop and Recovery |
| **Description** | The system shall provide an emergency stop (E-Stop) control in the GUI. When activated, it shall immediately halt all robot motion and cancel any executing sequence. After the stop, the system shall enter a recovery mode that allows the user to manually move the robot to a safe position. |
| **Type** | Functional |
| **Source** | Instructor (Customer) |
| **Rationale** | A manual emergency stop is a critical safety mechanism that provides the operator with a total override control. It allows for the instantaneous halting of the system to prevent damage or unintended behaviors that may not be detected automatically. The subsequent recovery mode is essential for safely repositioning the robot without needing to restart the entire simulation. |
| **Priority** | Must have |
| **Verification Method**| 1. Start the execution of an automated motion sequence. <br>2. During the movement, activate the "Emergency Stop" button in the GUI. <br>3. Verify that all robot motion halts immediately. <br>4. Confirm that the status of the executing sequence is displayed as "Cancelled" or "Aborted". <br>5. Check that controls for initiating automated sequences are disabled. <br>6. Verify that manual motion controls (REQ-F-005) are enabled and allow the robot to be moved. |
| **Dependencies** | REQ-F-003, REQ-F-005, REQ-F-006 |
| **Status** | Proposed |

---

#### **Requirement ID: REQ-F-011**
| Attribute | Details |
| :--- | :--- |
| **Requirement Name** | Sequence Pause and Resume |
| **Description** | The system shall allow the user to pause the execution of an ongoing motion sequence. Once paused, the user shall be able to resume the sequence from the exact point where it was stopped. |
| **Type** | Functional |
| **Source** | End-User (Hypothetical) |
| **Rationale** | This functionality provides non-destructive flow control over automated tasks. It allows the operator to temporarily halt the robot for inspection or to analyze the system's state, and then continue the task without losing progress, improving efficiency and operational flexibility. |
| **Priority** | Should have |
| **Verification Method**| 1. Start the execution of a multi-point sequence. <br>2. In the middle of a movement between two points, activate the "Pause" button. <br>3. Verify that the robot stops in a controlled manner. <br>4. Activate the "Resume" button. <br>5. Confirm that the robot continues its movement to the original target point and correctly completes the rest of the sequence. |
| **Dependencies** | REQ-F-006 |
| **Status** | Proposed |

---

#### **Requirement ID: REQ-F-012**
| Attribute | Details |
| :--- | :--- |
| **Requirement Name** | User Status and Notification Panel |
| **Description** | The GUI shall include a text panel (log or status bar) to display informational messages to the user. These messages shall include the current system status (e.g., "Ready", "Executing sequence..."), action confirmations (e.g., "Pose saved"), warnings, and errors (e.g., "Inverse kinematics solution not found"). |
| **Type** | Functional |
| **Source** | End-User (Hypothetical), Students (Developers) |
| **Rationale** | A centralized notification system is vital for usability and debugging. It keeps the user informed about the system's state, confirms that their actions have taken effect, and provides clear diagnostics when errors occur, reducing confusion and improving the overall interaction. |
| **Priority** | Must have |
| **Verification Method**| 1. Start the system and verify that the panel displays the "Ready" status. <br>2. Record a new pose and confirm that a message like "Pose X saved" appears. <br>3. Execute a sequence and verify that the status changes to "Executing sequence...". <br>4. Command an impossible action (e.g., moving to an unreachable point) and verify that a clear error message is displayed in the panel. |
| **Dependencies** | REQ-F-003 |
| **Status** | Proposed |

---

### 3.3 Non-Functional Requirements

---

#### **Requirement ID: REQ-N-001**
| Attribute | Details |
| :--- | :--- |
| **Requirement Name** | Communication Latency |
| **Description** | The communication between the operator interface and the simulation shall have a latency of less than 50 milliseconds. |
| **Type** | Non-Functional (Performance) |
| **Source** | Project Statement |
| **Rationale** | Maintaining low-latency communication between the GUI and the simulation backend is critical for real-time control and feedback. Latencies above 50 ms can cause noticeable delays, reducing the precision of manual operations and potentially affecting user confidence and control responsiveness. |
| **Priority** | Must have |
| **Verification Method**| 1. Establish a test setup with both GUI and simulation components running. <br> 2. Use a network analyzer or system log timestamps to measure round-trip message delay. <br> 3. Perform 20 random interaction samples (e.g., joint movement commands) and record latency values. <br> 4. Calculate the average latency and confirm it remains below 50 ms.|
| **Dependencies** | REQ-F-003, REQ-F-004 |
| **Status** | Proposed |

---

#### **Requirement ID: REQ-N-002**
| Attribute | Details |
| :--- | :--- |
| **Requirement Name** | Framerate |
| **Description** | The simulation environment shall maintain a frame rate of at least 30 frames per second. |
| **Type** | Non-Functional (Performance, Usability) |
| **Source** | Project Statement |
| **Rationale** | A minimum framerate of 30 FPS ensures smooth visual feedback and accurate representation of motion dynamics, which is essential for realistic simulation, intuitive user experience, and precise monitoring of robotic operations. |
| **Priority** | Must have |
| **Verification Method**| 1. Launch the simulation on the reference machine. <br> 2. Run a standard 3D scene with a moving robotic arm. <br> 3. Measure the framerate using the built-in performance monitor or an external profiler. <br> 4. Verify that the average framerate remains above 30 FPS during a one-minute test session. |
| **Dependencies** | REQ-F-001 |
| **Status** | Proposed |

---

#### **Requirement ID: REQ-N-003**
| Attribute | Details |
| :--- | :--- |
| **Requirement Name** | Development Platform and Programming Language |
| **Description** | The system shall be developed using C/C++ on a Linux operating system. |
| **Type** | Non-Functional (Technical Constraint) |
| **Source** | Project Statement |
| **Rationale** | Using C/C++ on Linux ensures compatibility with ROS 2, provides real-time performance, and aligns with the target deployment and testing environment. It also enables integration with existing robotics frameworks and tools commonly used in academic and industrial applications. |
| **Priority** | Must have |
| **Verification Method**| 1. Review build configuration files (CMakeLists, package.xml). <br> 2. Confirm compilation using GCC or Clang on a Linux system. <br> 3. Execute binaries and verify runtime environment variables. |
| **Dependencies** | None |
| **Status** | Proposed |

---

#### **Requirement ID: REQ-N-004**
| Attribute | Details |
| :--- | :--- |
| **Requirement Name** | Communication Framework |
| **Description** | The system shall use ROS 2 for all inter-component communication. |
| **Type** | Non-Functional (Technical Constraint, Integration) |
| **Source** | Project Statement |
| **Rationale** | ROS 2 provides a robust and scalable publish-subscribe communication model that supports distributed systems and real-time data exchange. Adopting ROS 2 ensures interoperability with other robotic modules, simplifies testing, and enables future system extensibility. |
| **Priority** | Must have |
| **Verification Method**| 1. Inspect project dependencies to confirm ROS 2 packages are included. <br> 2. Launch ROS 2 nodes and verify successful topic and service registration. <br> 3. Use ros2 topic list and ros2 node info to confirm data exchange among components. |
| **Dependencies** | REQ-N-003 |
| **Status** | Proposed |

---

#### **Requirement ID: REQ-N-005**
| Attribute | Details |
| :--- | :--- |
| **Requirement Name** | Concurring Programming Interfaces |
| **Description** | The system shall utilize POSIX interfaces for concurrent programming tasks. |
| **Type** | Non-Functional (Technical Constraint, Performance) |
| **Source** | Project Statement |
| **Rationale** | Implementing concurrency using POSIX threads guarantees portability, efficiency, and deterministic task scheduling within Linux environments. It supports real-time data handling, allowing multiple processes (such as GUI updates, kinematic computation, and sensor simulation) to execute simultaneously without interference. |
| **Priority** | Must have |
| **Verification Method**| 1. Inspect the source code for the inclusion of <pthread.h> or equivalent POSIX threading APIs. <br> 2. Run a concurrency stress test to verify correct parallel execution and synchronization.<br> 3. Confirm the absence of deadlocks or race conditions.|
| **Dependencies** | REQ-N-003, REQ-N-004 |
| **Status** | Proposed |

---

#### **Requirement ID: REQ-N-006**
| Attribute | Details |
| :--- | :--- |
| **Requirement Name** | Version Control with Git |
| **Description** | All source code shall be version controlled using Git. |
| **Type** | Non-Functional (Project Constraint) |
| **Source** | Instructor (Customer) |
| **Rationale** | Using Git for version control ensures collaborative, traceable, and reversible development. It allows multiple developers to work on the same project concurrently, supports code reviews through pull requests, and maintains a complete history of all changes. This promotes accountability, reduces integration issues, and provides a foundation for continuous integration and quality assurance processes. |
| **Priority** | Must have |
| **Verification Method** | 1. Inspect the GitHub repository to confirm that all source code is tracked under Git. <br>2. Verify that branching and merging strategies follow the project's configuration management plan. <br>3. Check that commits are properly documented with meaningful messages and author information. <br>4. Ensure that pull requests are used for code review before merging to the main branch. <br>5. Confirm that version tags or releases are defined for major milestones. |
| **Dependencies** | REQ-F-001, REQ-F-002, REQ-F-003, REQ-F-004, REQ-F-005, REQ-F-006, REQ-F-007, REQ-F-008, REQ-N-007 |
| **Status** | Proposed |

---

#### **Requirement ID: REQ-N-007**
| Attribute | Details |
| :--- | :--- |
| **Requirement Name** | Project Documentation Compliance |
| **Description** | The system shall be documented in accordance with the project’s documentation plan. |
| **Type** | Non-Functional |
| **Source** | Project Statement |
| **Rationale** | Comprehensive documentation ensures consistency, maintainability, and traceability across the project lifecycle. It provides a clear reference for future development, testing, and validation activities, while supporting quality assurance and compliance with ISO/IEC/IEEE 12207 and 29148 standards. Proper documentation also facilitates collaboration among team members and transparency in project deliverables. |
| **Priority** | Must have |
| **Verification Method** | 1. Review all project documents (SRS, ADD, SDD, QAP, SCMP, etc.). <br>2. Confirm that each document follows the format, structure, and content defined in the documentation plan. <br>3. Verify that version control and authorship are maintained through Git. <br>4. Ensure that documentation updates are synchronized with system changes and reviewed by the QA lead. <br>5. Check that all mandatory deliverables are submitted and accessible via the GitHub repository. |
| **Dependencies** | REQ-N-006, REQ-N-003, REQ-N-004, REQ-N-005, REQ-F-001, REQ-F-002, REQ-F-003, REQ-F-004, REQ-F-005, REQ-F-006, REQ-F-007, REQ-F-008 |
| **Status** | Proposed |

---

#### **Requirement ID: REQ-N-008**
| Attribute | Details |
| :--- | :--- |
| **Requirement Name** | System Startup Time |
| **Description** | The system shall be fully initialized and ready to receive user commands in less than 10 seconds from its execution on the reference machine. |
| **Type** | Non-Functional (Usability, Performance) |
| **Source** | End-User (Hypothetical) |
| **Rationale** | A fast startup time is a key factor in an application's usability. Limiting this time to under 10 seconds ensures that the user can begin working efficiently without frustration from prolonged waits, improving the overall user experience. |
| **Priority** | Should have |
| **Verification Method**| 1. From a terminal, execute the command to launch the application, recording the start time. <br>2. Record the time at which the GUI becomes interactive and the connection to the simulation is established. <br>3. Repeat the test 5 times. <br>4. Calculate the average startup time and verify that it is less than 10 seconds. |
| **Dependencies** | REQ-F-001, REQ-F-003 |
| **Status** | Proposed |

---

#### **Requirement ID: REQ-N-009**
| Attribute | Details |
| :--- | :--- |
| **Requirement Name** | Responsive system |
| **Description** | The GUI shall remain responsive during robot execution and simulation updates. |
| **Type** | Non-Functional (Usability, Performance) |
| **Source** | End-User (Hypothetical) |
| **Rationale** | A responsive interface is essential for usability and safety. It ensures that users can interact with the system in real time, even during intensive simulation or control operations. |
| **Priority** | Must have |
| **Verification Method**| 1. Launch the system and begin a trajectory execution. 2. Attempt GUI interactions (e.g., move sliders, click buttons) during execution. 3. Confirm that inputs are registered without delay and GUI updates occur every 100 ms |
| **Dependencies** | REQ-F-003, REQ-F-004, REQ-F-006 |
| **Status** | Proposed |


---

## 4. Requirements Traceability Matrix (RTM)

The Requirements Traceability Matrix (RTM) provides a clear correspondence between each system requirement and the operational scenarios in which it is applied. This ensures full coverage, consistency, and validation of system functionality from a user and stakeholder perspective. Each requirement is mapped to one or more operational scenarios that describe the context of its use, as well as to the stakeholder(s) responsible for or affected by its implementation.

| Requirement ID | Operational Scenario(s) | Stakeholder(s) |
| :--- | :--- | :--- |
| **REQ-F-001** | Scenario 1: System Initialization and Robot/Environment Loading | End-User (Hypothetical) |
| **REQ-F-002** | Scenario 1: System Initialization and Robot/Environment Loading | End-User (Hypothetical) |
| **REQ-F-003** | Scenario 2: Manual Control of the Robotic Arm | End-User (Hypothetical) |
| **REQ-F-004** | Scenario 2: Manual Control of the Robotic Arm | End-User (Hypothetical) |
| **REQ-F-005** | Scenario 2: Manual Control of the Robotic Arm | End-User (Hypothetical) |
| **REQ-F-006** | Scenario 3: Automated Path Execution | End-User (Hypothetical) |
| **REQ-F-007** | Scenario 4: Fault Detection and Response | Instructor (Customer) |
| **REQ-F-008** | Scenario 4: Fault Detection and Response | Instructor (Customer) |
| **REQ-F-009** | Scenario 3: Automated Path Execution | End-User (Hypothetical) |
| **REQ-F-010** | Scenario 4: Fault Detection and Response | Instructor (Customer) |
| **REQ-F-011** | Scenario 3: Automated Path Execution | End-User (Hypothetical) |
| **REQ-F-012** | Scenario 1, 2, 3, 4 | End-User (Hypothetical) |
| **REQ-N-001** | Scenario 2, Scenario 3 | End-User (Hypothetical) |
| **REQ-N-002** | Scenario 1, 2, 3, 4 | End-User (Hypothetical) |
| **REQ-N-003** | N/A (Course Constraint) | Instructor (Customer) |
| **REQ-N-004** | N/A (Course Constraint) | Instructor (Customer) |
| **REQ-N-005** | N/A (Course Constraint) | Instructor (Customer) |
| **REQ-N-006** | N/A (Development Process) | Students (Developers) |
| **REQ-N-007** | N/A (Course Deliverable) | Instructor (Customer) |
| **REQ-N-008** | Scenario 1: System Initialization and Robot/Environment Loading | End-User (Hypothetical) |
| **REQ-N-009** | Scenario 1, 2, 3, 4 | End-User (Hypothetical) |

---
