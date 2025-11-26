# SE25 — Software Engineering Project  
README for the structure of the branch. This repository contains the documentation, models, images, and source code for the SE25 project. The structure follows software engineering best practices to support maintainability, traceability, and collaborative work.

---
```
## Repository Structure
SE25/
├── doc/
│ ├── deliverable/
│ └── guide/
├── images/
├── models/
│ ├── Requisitos/
│ └── Reverse_model/
└── src/
  └── ros2_control/
The purpose of each directory is detailed below.
```
---

## 1. `doc` — Project Documentation

This directory contains all written documentation related to the project. It includes formal deliverables, internal guides, evaluation reports, and planning documents.

### 1.1 `doc/deliverable` — Formal Deliverables
This folder contains documents that are formally delivered to stakeholders, instructors, or clients.  
Files follow the naming convention:  
`[DocumentType]_[ProjectCode]_[Version]_[YYYY-MM-DD].md`

Typical documents include:
- SRS — Software Requirements Specification  
- SDD — Software Design Document  
- Technology and tool evaluation reports  
- Other required deliverables for project milestones

### 1.2 `doc/guide` — Internal Guides
This folder contains internal documents intended for the development team.  
These documents are not formal deliverables.

Examples:
- Code Style Guide  
- Change Management Plan  
- Software Development Plan  
- Reverse Engineering Guide  
- Simulation setup instructions for the 6-DOF robotic arm  
- Instructions for running the project in mixed Windows/Ubuntu environments

Purpose: to provide instructions and standards for development, configuration, and collaboration.

---

## 2. `images` — Image Assets

This directory contains images used throughout the documentation, such as:
- UML diagrams  
- Architecture diagrams  
- Simulation snapshots  
- Any other graphical assets referenced in Markdown documents

---

## 3. `models` — System Models

This directory contains all system modeling artifacts, typically produced with UML or similar methodologies.

### 3.1 `models/requirements`
Contains requirement-related models, including:
- Use case diagrams  
- Activity diagrams  
- Conceptual models  
- Other functional analysis models

### 3.2 `models/reversemodel`
Contains models generated through reverse engineering, such as:
- Class diagrams derived from source code  
- Dependency diagrams  
- Automatically generated structural representations

The `README.md` within the `models` directory provides a detailed description of the UML and SysML models, including their structure, purpose, and the modeling approach used in the project.

---

## 4. `src` — Source Code

This directory contains the executable implementation of the project.

### 4.1 `src/ros2_control`
Contains the ROS 2 codebase related to controlling the robotic system, including:
- ROS 2 nodes  
- Hardware interface configurations  
- Parameter and launch files  
- Code for controlling sensors and actuators

The `README.md` within the `src` directory provides a detailed explanation of how the source code is organized, its relationship with the UML/SysML models, and how generated, copied, and manually written code coexist within the implementation.


---


