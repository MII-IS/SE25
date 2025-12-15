# SE25 System Models

This directory contains the UML (Unified Modeling Language) and SysML (Systems Modeling Language) models that form the foundation of our robotics project. These models are the primary artifacts of our Model-Based Systems and Software Engineering (MBSSE) approach and serve as the single source of truth for the system's requirements, architecture, and behavior.

The tool **Modelio** has been used to represent the requirements in each scenario.

---

## Model Overview

The models are organized hierarchically, starting from high-level system specifications and progressively detailing into finer-grained design elements. The core of our design is captured in these models before any code is written.

### Model Structure

The models are structured as follows:

* **Requirements:** Contains the SysML Requirement Diagrams that capture the functional and non-functional requirements of the robotic system.
* **Use Case:** Includes UML Use Case Diagrams illustrating the interactions between the robot and external actors.
* **Structure:** This directory holds the structural design of the system, including:
    * **Block Definition Diagrams (BDD):** Defines the system hierarchy and the relationships between different system blocks.
    * **Internal Block Diagrams (IBD):** Describes the internal structure of the blocks, including their parts, ports, and connectors.
* **Behavior:** This section details the dynamic behavior of the system through:
    * **Activity Diagrams:** Model the flow of control and data between different activities.
    * **State Machine Diagrams:** Describe the different states an object can be in and the transitions between those states.
    * **Sequence Diagrams:** Show the interaction between objects in a time-ordered sequence.

---

## Project Usage and Visualization (Modelio)

To explore the requirements traceability, modify diagrams, or generate new documentation, you must use the modeling tool with which the project was created: **Modelio**.

### Prerequisites

* **Modeling Tool:** **Modelio 5.4** is required. You can download the corresponding version from the official Modelio website.
* **Git Manager:** It is recommended to use **GitHub Desktop** to clone the repository and manage updates.

### Opening the Project (Using the ZIP File)

The Modelio projects are stored compressed in **ZIP files** within their respective folders in the repository (e.g., inside `Requisitos/` or `Reverse_model/`). Modelio can import these files directly.

1.  **Clone the Repository:** Use GitHub Desktop to clone this repository onto your local machine.
2.  **Locate the File:** Once cloned, navigate to the specific model folder you want to work on.
    * *Example:* `SE25/models/Requisitos/`
3.  **Import in Modelio:**
    * Open the **Modelio 5.4** application.
    * Go to the `File` menu and select **`Import a project...`**.
    * Navigate to the target folder (e.g., `SE25/models/Requisitos/`) and **select the Modelio project ZIP file**. Modelio will handle the decompression and import process.

---

### Updating the Models (ZIP Workflow)

If you have made modifications or added new requirements in Modelio and wish to update the GitHub repository, you must follow this specific process:

1.  **Compress the Project:** Once changes are saved in Modelio, navigate to the folder of your local Modelio project (the *unzipped folder* where the project files reside). **Compress all content** of the Modelio project back into a single **ZIP file**.
2.  **Replace the ZIP:** Place this new ZIP file into the corresponding repository folder. It must **overwrite** the previous ZIP file.
    * *Example:* If you modified requirements, overwrite the ZIP inside `SE25/models/Requisitos/`.
3.  **Synchronize with GitHub:**
    * Open GitHub Desktop.
    * You will see that the new ZIP file appears as modified.
    * Add a summary of the changes (Commit), and then **push** the changes to the remote repository.

---

## Requirements Modeling Diagrams

The following sections visually present the architecture and traceability of the project requirements, separated by the primary user perspective: **Developers** and **End-Users**.

### Developers View
This diagram corresponds to the requirements that affect the developers. In this case, these are all the established requirements.

<img width="100%" alt="Class Diagram" src="https://github.com/MII-IS/SE25/blob/main/images/REQ_Dev_Diagram.png" />

#### Detail: Requirement REQ-F-004
Since it is a complex diagram to interpret, detailed information can be obtained by clicking on both the requirements and the scenarios.

<img width="100%" alt="Req Detail" src="https://github.com/MII-IS/SE25/blob/main/images/REQ_ReqN004.png" />

#### Detail: Scenario 1

<img width="100%" alt="Scenario 1" src="https://github.com/MII-IS/SE25/blob/main/images/REQ_Scenario1.png" />

### End-User View
This diagram corresponds to the requirements that affect the **End-User**. This view shows all the operational scenarios and the specific functional and non-functional requirements they satisfy.

<img width="100%" alt="End-User Requirements Diagram" src="https://github.com/MII-IS/SE25/blob/main/images/end-user.png" />

Since the full diagram can be complex, here are detailed views from the Modelio `Links Editor` showing the traceability for a specific scenario and a specific requirement.

#### Detail: Scenario 2 - Manual Control of the Robotic Arm
This view from the `Links Editor` shows all the requirements that are satisfied by `Scenario 2`.

<img width="100%" alt="Scenario 2 Links" src="https://github.com/MII-IS/SE25/blob/main/images/scenario2.png" />

#### Detail: Requirement REQ-F-005 - Individual Joint Control
This view from the `Links Editor` shows which scenarios are linked to the requirement `REQ-F-005`.

<img width="100%" alt="REQ-F-005 Links" src="https://github.com/MII-IS/SE25/blob/main/images/req5.png" />
