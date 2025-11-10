# Software Design Document (SDD)

## Table of Contents

1. [Introduction](#1-introduction)  
   1.1 [Purpose](#11-purpose)  
   1.2 [System Scope](#12-system-scope)  
   1.3 [Definitions, Acronyms, and Abbreviations](#13-definitions-acronyms-and-abbreviations)  
   1.4 [References](#14-references)  

2. [System Overview](#2-system-overview)  
   2.1 [System Perspective](#21-system-perspective)  
   2.2 [Main Functionalities](#22-main-functionalities)  
   2.3 [Design Constraints](#23-design-constraints)  
   2.4 [Assumptions and Dependencies](#24-assumptions-and-dependencies)  

3. [System Architecture](#3-system-architecture)  
   3.1 [High-Level Architecture](#31-high-level-architecture)  
   3.2 [Main Components](#32-main-components)  
   3.3 [Component Relationships and Interactions](#33-component-relationships-and-interactions)  
   3.4 [Design Rationale](#34-design-rationale)  

4. [Detailed Design Specification](#4-detailed-design-specification)  
   4.1 [Class and Data Structure Design](#41-class-and-data-structure-design)  
   4.2 [Key Algorithms](#42-key-algorithms)  
   4.3 [User Interface (UI)](#43-user-interface-ui)  
   4.4 [Error and Exception Handling](#44-error-and-exception-handling)  

5. [Non-Functional Characteristics](#5-non-functional-characteristics)  
   5.1 [Security](#51-security)  
   5.2 [Performance](#52-performance)  
   5.3 [Maintainability](#53-maintainability)  
   5.4 [Portability](#54-portability)  

6. [Requirements Traceability](#6-requirements-traceability)  
   6.1 [Requirements-to-Design Traceability Matrix](#61-requirements-to-design-traceability-matrix)  

7. [Appendices](#7-appendices)  
   7.1 [Additional UML Diagrams](#71-additional-uml-diagrams)  
   7.2 [Technical Glossary](#72-technical-glossary)  

---

## 1. Introduction

### 1.1 Purpose
The purpose of this document is to define the software design for the SE25 project: a robotic arm simulation and control system. This Software Design Document (SDD) translates the system requirements into a structured technical solution, providing a comprehensive description of the system architecture, components, interfaces, and design rationale. It serves as a reference for developers, testers, and stakeholders involved in the implementation and validation of the system.

### 1.2 System Scope
The scope of the SE25 project includes the design, development, and testing of a distributed software system that simulates a robotic arm in a 3D environment and enables its real-time control via a graphical user interface. The system comprises two main components:  
- A physics-based simulation environment;
- A control workstation for human operators.

Communication between components is handled via ROS 2 middleware. The project covers the full software life cycle, from requirements elicitation to system validation, but does not include the development of physical robotic hardware.

### 1.3 Definitions, Acronyms, and Abbreviations
|Term |Definition |
| ----- | ----- |
|SoISystem of Interest | the specific system being designed and implemented |
|ROS 2 | Robot Operating System 2 – middleware used for real-time communication between components |
|GUI | Graphical User Interface – the user-facing control interface |
|URDF | Unified Robot Description Format – XML format for representing robot models |
|OpsCon | Operational Concept – user-oriented description of system behavior and purpose |
|ISO/IEC/IEEE 12207 | International standard for software life cycle processes |
|SWEBOK | Software Engineering Body of Knowledge – IEEE guide to software engineering practices |


### 1.4 References
**ISO/IEC/IEEE 12207:2017** – Systems and software engineering — Software life cycle processes  
**ISO/IEC/IEEE 29148:2018** – Requirements engineering  
**IEEE SWEBOK Guide**, Version 4.0 (2025)  
**SE25 Project Description Document** – Ricardo Sanz, SE25-01 v1.0, September 28, 2025  
**ROS 2 Jazzy Jalisco Documentation** – https://docs.ros.org.

---

## 2. System Overview

### 2.1 System Perspective
The SE25 system is a distributed software solution composed of two primary components:
- Simulation Environment: A physics-based 3D virtual world that models the behavior of a robotic arm.
- Control Workstation: A user-facing interface that allows manual and automated control of the robot.

These components communicate in real-time using ROS 2 middleware, enabling synchronized command execution and state visualization. The system is designed to run entirely on standard computing hardware and does not involve physical robotic devices.

> **Note:** The System of Interest (SoI) is the software-only simulation and control system, not the physical robot.

> **Suggested Diagram:** Context diagram or system block diagram.

### 2.2 Main Functionalities
The system provides the following core functionalities:
- Load and visualize a robotic arm model in a 3D simulation environment.
- Allow manual control of individual joints via a graphical user interface.
- Enable automated execution of predefined motion trajectories.
- Display real-time feedback of the robot’s pose and state.
- Detect collisions with virtual obstacles and trigger emergency stop procedures.
- Maintain low-latency communication between control and simulation components.

### 2.3 Design Constraints
The design of the SE25 system is subject to the following constraints:
- Programming Language: C++ (using GCC compiler)
- Operating System: Ubuntu 24.04 LTS
- Middleware: ROS 2 Jazzy Jalisco
- Concurrency Model: POSIX threads
- Build System: CMake
- Development Environment: Visual Studio Code with ROS and CMake extensions
- Version Control: Git with GitHub integration
- Performance: Communication latency < 50 ms; simulation frame rate ≥ 30 FPS.

### 2.4 Assumptions and Dependencies
The system design assumes the following:
- All team members use a standardized development environment based on Ubuntu.
- ROS 2 is correctly installed and configured on all machines.
- The robotic arm model is provided in URDF format.
- Users have basic familiarity with robotic control concepts.
- No hardware integration is required; the system operates entirely in software.
- GitHub is used for collaboration, issue tracking, and code reviews.

---

## 3. System Architecture

### 3.1 High-Level Architecture
The SE25 system follows a distributed architecture composed of two main subsystems:
- **Simulation Subsystem:** Implements a physics-based 3D environment where the robotic arm is modeled and animated.
- **Control Subsystem:** Provides a graphical user interface (GUI) for manual and automated control of the robot.

These subsystems communicate via ROS 2 middleware using a publish/subscribe model. The architecture supports real-time data exchange and modular development, enabling parallel work by simulation and control teams.

> **Suggested Diagram:** Component diagram showing ROS 2 nodes and their interactions.  
> **Suggested Diagram:** Component or package UML diagram.

### 3.2 Main Components

|  | Simulation Engine | Control GUI | Middleware Layer (ROS) |
|---|---|---|---|
| **Description** | Simulates the robotic arm in a virtual 3D environment | User interface for controlling the robot and visualizing its state | Communication backbone between simulation and control components |
| **Inputs** | Joint commands, URDF model | User commands, robot state |  |
| **Outputs** | Robot state (pose, joint angles), collision events | Joint commands, trajectory plans |  |
| **Responsibilities** | Physics simulation, collision detection, state publishing | Manual control, trajectory planning, emergency stop | Real-time message passing, topic management |
| **Interactions** | Receives commands from Control GUI; publishes state to ROS 2 topics | Sends commands to Simulation Engine; receives state updates | Connects all ROS 2 nodes |


### 3.3 Component Relationships and Interactions
The system components interact through ROS 2 topics:

- */joint_commands:* Control GUI → Simulation Engine
- */robot_state:* Simulation Engine → Control GUI
- */collision_event:* Simulation Engine → Control GUI (for emergency stop)

Each component is implemented as a ROS 2 node, enabling modular deployment and testing. The publish/subscribe model ensures decoupling and scalability.

> **Suggested Diagram:** Sequence diagram showing message flow during a control cycle.  
> **Suggested Diagram:** Sequence or communication diagram.

### 3.4 Design Rationale
The architecture was chosen to support:
- **Modularity:** Clear separation between simulation and control logic
- **Real-time performance:** ROS 2 ensures low-latency communication
- **Scalability:** Components can be extended or replaced independently
- **Educational value:** Aligns with ISO 12207 processes and SWEBOK principles for software engineering education

The use of ROS 2, C++, and POSIX threads reflects industry standards for robotic systems and supports concurrency and performance requirements.

---

## 4. Detailed Design Specification

### 4.1 Class and Data Structure Design
Provide class diagrams and data structure definitions.

### 4.2 Key Algorithms
Describe important algorithms (e.g., control logic, path planning, collision detection).

### 4.3 User Interface (UI)
Describe the UI design and user interaction flow.

> **Suggested Elements:** Wireframes, flowcharts, screen mockups.

### 4.4 Error and Exception Handling
Explain how the system handles errors, faults, and exceptions.

---

## 5. Non-Functional Characteristics

### 5.1 Security
Describe security mechanisms (e.g., authentication, data validation, access control).

### 5.2 Performance
Define performance requirements (e.g., latency, throughput, frame rate).

### 5.3 Maintainability
Explain how the system is designed for ease of maintenance and future updates.

### 5.4 Portability
Describe supported platforms and portability considerations.

---

## 6. Requirements Traceability

### 6.1 Requirements-to-Design Traceability Matrix

| Requirement ID | Design Element | Associated Component |

---

## 7. Appendices

### 7.1 Additional UML Diagrams

### 7.2 Technical Glossary
