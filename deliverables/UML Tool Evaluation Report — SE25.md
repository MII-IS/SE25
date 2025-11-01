# UML Tool Evaluation Report

---

## Table of Contents

- [1. Introduction](#1-introduction)
  - [1.1 Scope & Objectives](#11-scope--objectives)
  - [1.2 Tools Under Evaluation](#12-tools-under-evaluation)
  - [1.3 Evaluation Methodology](#13-evaluation-methodology)
- [2. Detailed Evaluation](#2-detailed-evaluation)
  - [2.1 Eclipse Papyrus](#21-eclipse-papyrus)
    - [2.1.1 Eclipse Papyrus Overview](#211-eclipse-papyrus-overview)
    - [2.1.2 Eclipse Papyrus Diagrams](#212-eclipse-papyrus-diagrams)
  - [2.2 Modelio](#22-modelio)
    - [2.2.1 Modelio Overview](#221-modelio-overview)
    - [2.2.2 Modelio Diagrams](#222-modelio-diagrams)
  - [2.3 IBM Rhapsody](#23-ibm-rhapsody)
    - [2.3.1 IBM Rhapsody Overview](#231-ibm-rhapsody-overview)
    - [2.3.2 IBM Rhapsody Diagrams](#232-ibm-rhapsody-diagrams)
- [3. Comparison Matrix](#3-comparison-matrix)
- [4. Recommended Tool](#4-recommended-tool)

---

## 1. Introduction

### 1.1 Scope & Objectives

**Purpose:** Select a single standardized UML modeling tool for the SE25 project as the team transitions from architecture to detailed design (Weeks 3–4).

**Artifacts covered:** Class, Sequence, and State Machine diagrams.  
**Deliverable:** This evaluation report and example diagrams exported from the selected tool.

---

### 1.2 Tools Under Evaluation

1. **Eclipse Papyrus** — Open-source, standards-compliant UML tool integrated into the Eclipse IDE.  
2. **Modelio** — Open-source UML modeling environment supporting a wide range of diagrams and models.  
3. **IBM Rhapsody** — Professional Model-Based Systems Engineering (MBSE) tool. Academic licenses are available.

---

### 1.3 Evaluation Methodology

Each tool is assessed according to the following criteria:

1. **Ease of Installation & Setup** (on Ubuntu 24.04 LTS)  
2. **Learning Curve & Documentation** (user-friendliness and learning resources)  
3. **UML Diagram Support & Standard Compliance** (Class, Sequence, State Machine)  
4. **Integration with Project Workflow**  
5. **Performance & Stability**  
6. **Licensing Terms**

Each criterion is scored on a **1–5 scale**.

---

## 2. Detailed Evaluation

### 2.1 Eclipse Papyrus

#### 2.1.1 Eclipse Papyrus Overview

- **Overview:** _Eclipse Papyrus is an open-source modeling tool based on the Eclipse platform. It supports full UML 2.x and SysML modeling and allows the use of custom profiles. It’s suitable for academic and engineering projects, especially when working with complex systems. The tool is extensible through plug-ins and follows OMG standards._
- **Installation (Ubuntu 24.04):** _Installing Papyrus on Ubuntu 24.04 LTS wasn’t exactly quick or easy. It took quite some time to get everything running, especially since the setup required first installing Ubuntu through WSL and then configuring Eclipse Modeling with the Papyrus plug-in. The process involves several manual steps and can feel technical at first — you need to handle dependencies and use the correct Eclipse package. Once set up, Papyrus launched correctly and worked without issues. It just requires patience and careful setup the first time._
- **User Experience:** _The interface is detailed and feature-rich but has a learning curve. Users must understand how Eclipse handles modeling files (`.uml`, `.notation`, `.di`). Once familiar, managing diagrams becomes intuitive._
- **UML Support:** _Supports all major UML 2.x diagrams:_
  - _Class Diagrams_  
  - _Sequence Diagrams_
  - _State Machine Diagrams_
  - _Use Case, Component, Deployment, and Activity Diagrams_
- **Performance & Stability:** _Runs stably on standard hardware. Large models may slow down the interface, but no major crashes occurred during testing._
- **Licensing:** _Fully open-source under the Eclipse Public License (EPL). All features are available without restrictions._
- **Integration with Workflow:**
  - **C++ / Ubuntu integration:** _Does not generate C++ code directly but works well within Eclipse-based environments. Suitable for design and documentation._
  - **Git file management:** _Models are XML-based — versionable in Git, though diffs are not human-readable._
  - **Image export:** _Supports PNG, SVG, and PDF export with high quality suitable for documentation._

#### 2.1.2 Eclipse Papyrus Diagrams

- **Class Diagram:**  
  <img width="2335" height="1240" alt="Class Diagram" src="https://github.com/MII-IS/SE25/blob/main/images/ClassDiagram.png" />

- **Sequence Diagrams (4 Scenarios):**  
  <img width="2335" height="1240" alt="Scenario 1" src="https://github.com/elisa109/SE25Alumnos/blob/main/docs/images/Scenario1-SD.png](https://github.com/MII-IS/SE25/blob/main/images/Scenario1-SD.png" />  
  <img width="2335" height="1240" alt="Scenario 2" src="https://github.com/elisa109/SE25Alumnos/blob/main/docs/images/Scenario2-SD.png](https://github.com/MII-IS/SE25/blob/main/images/Scenario2-SD.png" />  
  <img width="2335" height="1240" alt="Scenario 3" src="https://github.com/elisa109/SE25Alumnos/blob/main/docs/images/Scenario3-SD.png](https://github.com/MII-IS/SE25/blob/main/images/Scenario3-SD.png" />  
  <img width="2335" height="1240" alt="Scenario 4" src="https://github.com/elisa109/SE25Alumnos/blob/main/docs/images/Scenario4-SD.png](https://github.com/MII-IS/SE25/blob/main/images/Scenario4-SD.png" />

- **State Machine Diagram:**  
  <img width="2335" height="1240" alt="State Machine Diagram" src="https://github.com/elisa109/SE25Alumnos/blob/main/docs/images/StateMach.png" />

---

### 2.2 Modelio

#### 2.2.1 Modelio Overview

- **Overview:** _Modelio is a versatile open-source modeling tool supporting UML, BPMN, ArchiMate, and TOGAF standards. It offers enterprise-grade features like requirements management, code generation (Java, C++), and documentation export. Its modular and scriptable architecture makes it highly extensible._
- **Installation (Ubuntu 24.04):** _Easily installed via Snap (`sudo snap install modelio`) with no extra configuration. Runs smoothly out of the box with automatic updates, ideal for low-maintenance environments._
- **User Experience:** _Though technical at first glance, Modelio’s interface is well-documented. Tutorials and community support ease the learning curve._
- **UML Support:** _Full UML 2.x compliance, including Class, Sequence, and State Machine diagrams. Enforces modeling standards with built-in validation._
- **Performance & Stability:** _Lightweight, reliable, and stable for large models._
- **Licensing:** _Open-source under GPLv3 (core) and Apache 2.0 (modules). No functional limitations._
- **Integration with Workflow:**
  - **C++ / Ubuntu integration:** _Supports C++ code generation and runs natively on Ubuntu. Generated code integrates easily with existing workflows._
  - **Git file management:** _XML-based models are Git-friendly — clean diffs, efficient tracking, and collaborative versioning._
  - **Image export:** _Exports to PNG, SVG, and PDF with documentation-ready quality._

#### 2.2.2 Modelio Diagrams

- **Class Diagram:**  
  <img width="2335" height="1240" alt="Modelio Class Diagram" src="https://github.com/elisa109/SE25Alumnos/blob/main/docs/images/Modelio_Class.png" />

- **Sequence Diagram:**  
  <img width="2335" height="1240" alt="Modelio Sequence Diagram" src="https://github.com/elisa109/SE25Alumnos/blob/main/docs/images/Modelio_Seq.png" />

- **State Machine Diagram:**  
  <img width="2335" height="1240" alt="Modelio State Machine" src="https://github.com/elisa109/SE25Alumnos/blob/main/docs/images/Modelio_StateMachine.png" />

---

### 2.3 IBM Rhapsody

#### 2.3.1 IBM Rhapsody Overview

- **Overview:** _IBM Engineering Systems Design Rhapsody is an advanced UML/SysML modeling tool designed for the development of embedded and real-time systems. It facilitates design, simulation, validation, and automatic code generation in multiple programming languages, including C, C++, Java, and Ada. The tool is specifically conceived for enterprise and industrial environments that demand high levels of traceability, integration with software lifecycle management tools (such as IBM DOORS and IBM Engineering Workflow Management), and strict compliance with international standards such as UML 2.x and SysML 1.x._  
- **Installation (Ubuntu 24.04):** _Installation on Ubuntu 24.04 presents a high level of complexity, as the official installer is optimized for Red Hat Enterprise Linux and Windows systems. For this reason, it is recommended to execute the software within a Docker container or a virtual machine running Windows. Additionally, the download process can be confusing due to the existence of multiple software versions with similar names. License activation posed an additional challenge, as the program initially failed to recognize the provided license file, preventing validation until several troubleshooting attempts were made._  
- **User Experience:** _The user interface, developed in Java, exhibits a dense visual layout typical of high-capacity systems engineering tools. While it offers an extensive range of functionalities, its learning curve is steep. The arrangement of menus, panels, and tools is clearly aimed at users with prior experience in UML/SysML modeling and model-driven development (MDD) methodologies._  
- **UML Support:** _IBM Rhapsody fully complies with UML 2.x and SysML 1.x specifications, providing a broad variety of structural diagrams (classes, components, blocks) and behavioral diagrams (activities, states, sequences). Its capabilities include state machine simulation, automatic code generation from models, and bidirectional synchronization between the model and source code, thereby facilitating traceability and design maintainability._  
- **Performance & Stability:** _During testing, long loading times and occasional unexpected program closures were observed. According to user community reports, this behavior is relatively common and may be related to an incorrect license configuration. Once the license environment is properly set up, the software demonstrates stable operation and satisfactory performance on hardware configurations with sufficient resources_  
- **Licensing:** _IBM provides free academic licenses through the IBM Academic Initiative program, valid for one year and renewable annually. These licenses offer full access to the professional version’s features, although they require institutional registration and manual renewal each academic cycle. Activation is performed through the IBM License Manager, or alternatively via a local license provided by the educational institution._  
- **Integration with Workflow:**
  - **C++ / Ubuntu integration:** _Rhapsody generates C/C++ code that can be compiled in Linux environments. It is compatible with GCC toolchains and Makefiles, allowing seamless integration into standard Linux development workflows. The tool can be integrated with Eclipse CDT or executed from the terminal to automate build processes. Models are synchronized with the source code through both forward and reverse engineering, ensuring consistency between design and implementation._  
  - **Git file management:** _Rhapsody projects include several binary files (.rpy, .sbs, .cat), which complicates version control using Git. It is therefore recommended to manage versioning through exported files (in XMI format) or synchronization scripts to prevent merge conflicts and repository bloat._  
  - **Image export:** _The tool supports exporting diagrams in multiple graphic formats, including PNG, JPEG, EMF, and BMP. The export quality is high and configurable, making it suitable for inclusion in technical documentation. Additionally, Rhapsody allows direct printing to PDF or PostScript using system-level tools._

#### 2.3.2 IBM Rhapsody Diagrams

- **Class Diagram:**  
  <img width="2335" height="1240" alt="Modelio Class Diagram" src="https://github.com/elisa109/SE25Alumnos/blob/main/docs/images/ClassDiagram_IBM.png" />

- **Sequence Diagram:**
  <img width="2335" height="1240" alt="Modelio Class Diagram" src="https://github.com/elisa109/SE25Alumnos/blob/main/docs/images/SequenceDiagramIBM.png" /> 
- **State Machine Diagram:**  
  <img width="2335" height="1240" alt="Modelio State Machine" src="https://github.com/elisa109/SE25Alumnos/blob/main/docs/images/Statechart_IBM.png" />

---

## 3. Comparison Matrix

| Criterion | Papyrus | Modelio | IBM Rhapsody |
|------------|:-------:|:-------:|:-------------:|
| Ease of Installation & Setup | 3 | 5 | 2 |
| Learning Curve & Documentation | 2 | 4 | 3 |
| UML Diagram Support & Compliance | 5 | 5 | 5 |
| Integration: C++ / Ubuntu | 4 | 4 | 3 |
| Integration: Git Friendliness | 3 | 4 | 2 |
| Integration: Export PNG / SVG | 4 | 5 | 4 |
| Performance & Stability | 4 | 5 | 3 |
| Licensing | 5 | 5 | 3 |
| **Total (sum)** | **30** | **37** | **25** |

---

## 4. Recommended Tool



**Selected Tool:** **Modelio**

**Justification:**

Modelio was selected as the best UML tool for the SE25 project. It offers a good balance between usability, features, and integration with our development workflow.

- **Ease of Use:** Quick installation (`5/5`) and intuitive learning curve (`4/5`).
- **Workflow Synergy:** Excellent Git compatibility (`4/5`) and high-quality exports (`5/5`).
- **Licensing & Support:** 100% open-source with professional-grade features (`5/5`).
- **Supports all required UML diagrams** (Class, Sequence, State Machine) and follows UML standards.

Overall, Modelio is the most practical and efficient choice for our team. It helps us work faster and collaborate better while meeting all technical needs.
