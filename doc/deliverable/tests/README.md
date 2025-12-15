# Test Plan: Automated Path Execution (REQ-F-006)

This directory contains the test definitions and procedures designed to verify the **Automated Path Execution** functionality of the SE25 Robot Simulation System.

## 1. Scope and Objective
The primary objective of this test suite is to verify **Requirement REQ-F-006**: *The user shall be able to define and execute a sequence of target poses for the robot's end-effector.*

Following the project's Quality Assurance guidelines, we have adopted a dual testing strategy:
1.  **Validation Testing:** To demonstrate to the stakeholder that the software meets the requirements under normal operation conditions.
2.  **Defect Testing:** To discover faults by exposing the system to edge cases and abnormal inputs.

## 2. Testing Strategy: Partitioning the Space
To ensure comprehensive coverage of the "space of possibilities" for a trajectory, we have applied **Equivalence Partitioning**. We categorize the input sequences into three distinct partitions:

| Partition Type | Description | Goal |
| :--- | :--- | :--- |
| **Valid Sequences** | Standard trajectories with reachable points within the workspace. | **Validation:** Confirm the robot executes the path smoothly. |
| **Boundary Sequences** | Sequences with minimal (1) or varying lengths. | **Robustness:** Ensure the system handles edge cases defined in testing guidelines. |
| **Invalid Sequences** | Sequences that are empty, unreachable, or cause collisions. | **Defect Finding:** Verify the system rejects invalid inputs or halts safely (Safety). |

## 3. Test Cases Definition
Based on the partitions above, the following specific test cases have been defined.

### 3.1 Partition A: Valid Sequences (Validation)
*Goal: Demonstrate successful operation (Happy Path).*

* **Test Case ID:** `TC-TRAJ-01`
* **Description:** Execution of a standard multi-point trajectory.
* **Input:** Sequence of 3 distinct, reachable poses (Start -> A -> B -> End).
* **Expected Result:**
    * Robot reaches all points in order.
    * Status panel updates to "Executing sequence".
    * Status panel updates to "Sequence completed" upon finish.

### 3.2 Partition B: Boundary Sequences
*Goal: Verify system behavior at the limits of logical input.*

* **Test Case ID:** `TC-TRAJ-02` (Single Point)
    * **Description:** Sequence containing exactly **one** target pose.
    * **Rationale:** Testing sequences of size 1 ensures the loop logic handles single iterations correctly.
    * **Expected Result:** Robot moves to the single point and stops.

* **Test Case ID:** `TC-TRAJ-03` (Overwrite/Stress)
    * **Description:** Define a sequence, run it, and immediately define a new sequence while the robot is returning to home.
    * **Expected Result:** System should either queue the new command or reject it with a warning, but **must not crash**.

### 3.3 Partition C: Invalid Sequences (Defect Testing)
*Goal: Try to "break" the system or trigger safety mechanisms.*

* **Test Case ID:** `TC-TRAJ-04` (Empty Sequence)
    * **Description:** Attempt to execute a sequence with **zero** points.
    * **Rationale:** Testing sequences of zero length is a standard defect testing guideline.
    * **Expected Result:** System ignores the command or displays "Error: Sequence is empty". Robot does not move.

* **Test Case ID:** `TC-TRAJ-05` (Unreachable Target)
    * **Description:** Include a target pose that is outside the robot's maximum reach (workspace).
    * **Expected Result:** Inverse Kinematics (IK) solver returns an error. The system displays "Target unreachable" and prevents execution.

* **Test Case ID:** `TC-TRAJ-06` (Collision Path)
    * **Description:** Define a trajectory that passes through a known obstacle (Wall/Cube).
    * **Expected Result:**
        * **If pre-computation exists:** System warns "Path blocked".
        * **If runtime detection:** Robot stops immediately upon contact (triggers `REQ-F-008 Collision Stop`).

## 4. Execution Procedure

The tests are executed using the **GUI Manual Test Mode** (Scenario-based testing).

1.  Launch the full system: `ros2 launch control_pkg full_system_launch.py`
2.  Open the **Path Manager** tab in the GUI.
3.  For each Test Case above:
    * Clear previous points.
    * Input the specific poses defined in the detailed test sheet.
    * Click "Execute Path".
    * Record the result (Pass/Fail) and any observations.

## 5. Traceability Matrix (Subset)

| Test Case | Related Requirement | Type |
| :--- | :--- | :--- |
| `TC-TRAJ-01` | REQ-F-006 | Validation |
| `TC-TRAJ-02` | REQ-F-006 | Validation/Boundary |
| `TC-TRAJ-04` | REQ-F-006 | Defect |
| `TC-TRAJ-05` | REQ-F-006, REQ-F-012 | Defect |
| `TC-TRAJ-06` | REQ-F-007, REQ-F-008 | Defect/Safety |
