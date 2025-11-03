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

## 3. Additional Guidelines

- Always communicate upcoming changes in advance (e.g., via issue, chat, or PR comment).
- Use **clear commit messages** describing *what* changed and *why*.
- Keep PRs **focused and small** when possible — avoid mixing unrelated changes.
- The **CM** has final authority to decide if a change requires additional approval.
- If in doubt, **open a PR and ask for review** — it’s safer and more transparent.

---

## 4. Roles

- **Configuration Manager (CM):**  
  - Oversees configuration control.  
  - Reviews and approves changes requiring CM involvement.  
  - Ensures consistency and traceability of configuration items.

- **Contributors / Team Members:**  
  - Follow this plan when committing or proposing changes.  
  - Review and approve PRs when affected by a change.  
  - Communicate proactively about potential impacts.

---

## 5. Change Process Summary

1. Determine the **type of change** (see table above).  
2. If a PR is required, assign reviewers according to the rules.  
3. Once approved, merge following the repository’s merge policy.  
4. Tag or version releases as needed for configuration traceability.

---

*This plan is meant to be lightweight and adaptable. The goal is effective collaboration with minimal bureaucracy.*
