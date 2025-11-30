# Configuration Management Plan

This document defines how configuration changes are managed in this project.  
The goal is to keep the process **simple, transparent, and efficient**, while ensuring control over important items and avoiding unnecessary bottlenecks.

---

## 1. General Principles

- All changes must be traceable through Git history or Pull Requests (PRs).
- Small or isolated changes that do not affect others can be done directly.
- Any change that could affect other team members, deliverables, or configuration items (CIs) must go through a Pull Request.
- The **Configuration Manager (CM)** is responsible for approving or coordinating changes when required.

---

## 2. Change Rules

| Type of Change | Description | Approval Required |
|----------------|--------------|-------------------|
| **Direct commit (no PR)** | Changes that clearly do **not affect anyone else**. Example: personal scripts, fixing typos, or local configuration. | None |
| **New documents or CIs** | Adding new deliverables or configuration items to the repository. | CM approval (via PR) |
| **Uncertain impact** | Changes where you are **not sure if they might affect others**. | CM approval (via PR) |
| **Minor changes affecting others** | Small changes that affect one or more teammates. | Approval from affected teammates (via PR) |
| **Major changes, no external impact** | Large changes that do not affect others’ work. | CM approval (via PR) |
| **Major changes affecting others** | Large or significant changes that impact one or more teammates. | CM and affected teammates’ approval (via PR) |

---

## 3. Change Classification

To simplify decision-making, changes are classified as **minor** or **major**.

### 3.1 Minor Changes (Do *not* require CM approval)
Minor changes are small, low-risk updates that do **not alter the project’s structure, logic, or deliverables**.

Examples include:
- Fixing typos, grammar, or formatting in documentation.  
- Updating links or metadata.  
- Adjusting comments in code without changing logic.  
- Minor visual or style tweaks that don’t affect functionality.  
- Updating a personal or local configuration file.  
- Adding or updating non-critical notes or examples.  

If the change could slightly affect a teammate’s work, a PR is required **only for visibility and their approval**, not CM approval.

### 3.2 Major Changes (Require CM approval)
Major changes are significant updates that can impact **project integrity, deliverables, configuration items, or team workflows**.

Examples include:
- Introducing new features or modules.  
- Modifying the structure of deliverables or key directories.  
- Changing interfaces, dependencies, or configuration items.  
- Updating versioning, naming conventions, or project build settings.  
- Replacing or removing files that are part of official deliverables.  
- Any change that alters how the team interacts with or builds the project.  

When in doubt, treat the change as **major** and involve the CM.

---

## 4. Additional Guidelines

- Always communicate upcoming changes in advance (e.g., via issue, chat, or PR comment).
- Use **clear commit messages** describing *what* changed and *why*.
- Keep PRs **focused and small** when possible — avoid mixing unrelated changes.
- The **CM** has final authority to decide if a change requires additional approval.
- If in doubt, **open a PR and ask for review** — it’s safer and more transparent.

---

## 5. Roles

- **Configuration Manager (CM):**  
  - Oversees configuration control.  
  - Reviews and approves changes requiring CM involvement.  
  - Ensures consistency and traceability of configuration items.

- **Contributors / Team Members:**  
  - Follow this plan when committing or proposing changes.  
  - Review and approve PRs when affected by a change.  
  - Communicate proactively about potential impacts.

---
## 6.  Management of Models and Special CIs 

The Modelio files are critical **Configuration Items (CI)** representing the project's requirements, architecture, and traceability.

### 6.1 Specific Change Rules for Models (Modelio)

Any modification to the Modelio model (adding/modifying requirements, diagrams, traceability, or generated documentation) is automatically considered a **Major Change** and must meet the following requirements:

1.  **Required Approval:** **Configuration Manager (CM) approval** via a Pull Request (PR) is always required.
2.  **Contribution Process:** Detailed instructions on how to open, modify, compress, and update the project's ZIP file on GitHub are found in the reference document.

**Instruction Reference:** To learn **how** to open, modify, and upload the Modelio project, consult the document **doc/guide/SoftwareDevelopmentPlan_SE25_V0.0_202-11-01.md**.

## 7. Change Process Summary

1. Determine the **type of change** (see Section 2 and 3).  
2. If a PR is required, assign reviewers according to the rules.  
3. Once approved, merge following the repository’s merge policy.  
4. Tag or version releases as needed for configuration traceability.

---

*This plan is meant to be lightweight and adaptable. The goal is effective collaboration with minimal bureaucracy.*
e collaboration with minimal bureaucracy.*

