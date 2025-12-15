
# SE25 System Verification Report

**Project:** SE25 – Robot Simulation and Control System
**Date:** December 15, 2025
**Version:** 1.0 (Develop Branch)

## 1. Functional Requirements (REQ-F)

| Requirement | Verified? | Method | Result |
| :--- | :---: | :--- | :---: |
| **REQ-F-001** (3D Environment) | YES | Visual Inspection | **PASS** |
| **REQ-F-002** (URDF Loading) | YES | Visual Inspection | **PASS** |
| **REQ-F-003** (GUI Interface) | YES | Manual Test | **PASS** |
| **REQ-F-004** (Real-time Pose) | YES | Visual Inspection | **PASS** |
| **REQ-F-005** (Joint Control) | YES | Manual Test | **PASS** |
| **REQ-F-006** (Path Execution) | YES | **Automated Unit Test** | **PASS** |
| **REQ-F-007** (Collision Detection) | NO | --- | **NO PASS** |
| **REQ-F-008** (Collision Stop) | NO | --- | **NO PASS** |
| **REQ-F-009** (Teach-In) | NO | N/A | **NO PASS** |
| **REQ-F-010** (Emergency Stop) | NO | Manual Test | **NO PASS** |
| **REQ-F-011** (Pause/Resume) | NO | N/A | **NO PASS** |
| **REQ-F-012** (User Status Panel) | YES | Automated Unit Test | **PASS** |

## 2. Non-Functional Requirements (REQ-N)

| Requirement | Verified? | Method | Result |
| :--- | :---: | :--- | :---: |
| **REQ-N-001** (Latency < 50ms) | YES | Performance Check | **PASS** |
| **REQ-N-002** (Framerate > 30FPS) | YES | Performance Check | **PASS** |
| **REQ-N-003** (Dev Platform Linux/C++) | YES | Inspection | **PASS** |
| **REQ-N-004** (ROS 2 Framework) | YES | Inspection | **PASS** |
| **REQ-N-005** (Concurrency POSIX) | YES | Code Inspection | **PASS** |
| **REQ-N-006** (Git Version Control) | YES | Repo Inspection | **PASS** |
| **REQ-N-007** (Documentation) | YES | File Inspection | **PASS** |
| **REQ-N-008** (Startup Time < 10s) | YES | Manual Timing | **PASS** |
| **REQ-N-009** (Responsive System) | YES | Usability Test | **PASS** |

---

## 3. Evidence of Automated Testing (REQ-F-006)

**Command:**
`colcon test --packages-select ros2_control_demo_example_7 --ctest-args -R test_req_f_006`

**Output:**
```text
[ RUN      ] PathManagerTest.TCTRAJ01_ValidSequence
[       OK ] PathManagerTest.TCTRAJ01_ValidSequence
[ RUN      ] PathManagerTest.TCTRAJ04_EmptySequence
[Validation Fail] La lista de puntos está vacía.
[       OK ] PathManagerTest.TCTRAJ04_EmptySequence
[ RUN      ] PathManagerTest.TCTRAJ05_UnreachableTarget
[Validation Fail] Punto fuera del alcance (> 2.0m).
[       OK ] PathManagerTest.TCTRAJ05_UnreachableTarget
[ PASSED   ] 4 tests.
