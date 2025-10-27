# SE25 System Models

This directory contains the UML (Unified Modeling Language) and SysML (Systems Modeling Language) models that form the foundation of our robotics project. These models are the primary artifacts of our Model-Based Systems and Software Engineering (MBSSE) approach and serve as the single source of truth for the system's requirements, architecture, and behavior.

## Model Overview

The models are organized hierarchically, starting from high-level system specifications and progressively detailing into finer-grained design elements. The core of our design is captured in these models before any code is written.

### Model Structure

The models are structured as follows:

*   **Requirements/`: ** Contains the SysML Requirement Diagrams that capture the functional and non-functional requirements of the robotic system.
*   **Use Case/`: ** Includes UML Use Case Diagrams illustrating the interactions between the robot and external actors.
*   **Structure/`: ** This directory holds the structural design of the system, including:
    *   **Block Definition Diagrams (BDD):** Defines the system hierarchy and the relationships between different system blocks.
    *   **Internal Block Diagrams (IBD):** Describes the internal structure of the blocks, including their parts, ports, and connectors.
*   **Behavior/`: ** This section details the dynamic behavior of the system through:
    *   **Activity Diagrams:** Model the flow of control and data between different activities.
    *   **State Machine Diagrams:** Describe the different states an object can be in and the transitions between those states.
    *   **Sequence Diagrams:** Show the interaction between objects in a time-ordered sequence.

## Tooling

The models are primarily developed and maintained using Modelio. To view and edit the models, you will need a compatible version of this software.

## Contribution

For guidelines on how to contribute to the models, please refer to the main `CONTRIBUTING.md` file in the root of the project.