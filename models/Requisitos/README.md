The tool **Modelio** has been used to represent the requirements in each scenario.

---

## Project Usage and Visualization (Modelio)

To explore the requirements traceability, modify diagrams, or generate new documentation, you must use the modeling tool with which the project was created: **Modelio**.

### Prerequisites

* **Modeling Tool:** **Modelio 5.4** is required. You can download the corresponding version from the official Modelio website.
* **Git Manager:** It is recommended to use **GitHub Desktop** to clone the repository and manage updates.

### Opening the Project (Using the ZIP File)

The Modelio project is stored compressed in a **ZIP file** within the repository. Modelio can open this file directly.

1.  **Clone the Repository:** Use GitHub Desktop to clone this repository onto your local machine.
2.  **Locate the File:** Once cloned, navigate to the following path in your file system:
    `SE25/models/Requisitos/`
3.  **Open in Modelio:**
    * Open the **Modelio 5.4** application.
    * Go to `File` and select `Open an existing project...`.
    * Navigate to the folder located at `SE25/models/Requisitos/` and **select the Modelio project ZIP file** (e.g., `ProjectName.zip`). Modelio will handle the decompression upon opening.

### Contribution and Model Updates

If you have made modifications or added new requirements in Modelio and wish to update the GitHub repository, you must follow the inverse process:

1.  **Compress the Project:** Once changes are saved in Modelio, navigate to the folder of your local Modelio project (the *unzipped folder* where the project files reside). **Compress all content** of the Modelio project back into a single **ZIP file**.
2.  **Replace the ZIP:** Place this new ZIP file into the repository folder `SE25/models/Requisitos/`. It must **overwrite** the previous ZIP file.
3.  **Synchronize with GitHub:**
    * Open GitHub Desktop.
    * You will see that the new ZIP file appears as modified.
    * Add a summary of the changes (Commit), and then **push** the changes to the remote repository.

---

## Requirements Modeling Diagrams

The following sections visually present the architecture and traceability of the project requirements, separated by the primary user perspective: **Developers** and **End-Users**.

# Developers
This is the diagram corresponding to the requirements that affect the developers. In this case, these are all the established requirements.  
<img width="2335" height="1240" alt="Class Diagram" src="https://github.com/MII-IS/SE25/blob/main/images/REQ_Dev_Diagram.png" />  

### Detail: Requirement REQ-F-004  

Since it is a complex diagram to interpret, detailed information can be obtained by clicking on both the requirements and the scenarios.  
<img width="2335" height="1240" alt="Class Diagram" src="https://github.com/MII-IS/SE25/blob/main/images/REQ_ReqN004.png" />  
### Detail:Scenario 1

<img width="2335" height="1240" alt="Class Diagram" src="https://github.com/MII-IS/SE25/blob/main/images/REQ_Scenario1.png" />  

# End-User
This is the diagram corresponding to the requirements that affect the **End-User**. This view shows all the operational scenarios and the specific functional and non-functional requirements they satisfy.

<img width="2335" height="1240" alt="End-User Requirements Diagram" src="https://github.com/MII-IS/SE25/blob/main/images/end-user.png" />

Since the full diagram can be complex, here are detailed views from the Modelio `Links Editor` showing the traceability for a specific scenario and a specific requirement.

### Detail: Scenario 2 - Manual Control of the Robotic Arm
This view from the `Links Editor` shows all the requirements that are satisfied by `Scenario 2`.

<img width="2335" height="1240" alt="Scenario 2 Links" src="https://github.com/MII-IS/SE25/blob/main/images/scenario2.png" />

### Detail: Requirement REQ-F-005 - Individual Joint Control
This view from the `Links Editor` shows which scenarios are linked to the requirement `REQ-F-005`.

<img width="2335" height="1240" alt="REQ-F-005 Links" src="https://github.com/MII-IS/SE25/blob/main/images/req5.png" />
