# Test Plan: Automated Path Execution (REQ-F-006)

This document describes the verification plan and procedures for **Requirement REQ-F-006**: *The user shall be able to define and execute a sequence of target poses.*

## 1. Testing Strategy: Partitioning the Space

To ensure comprehensive coverage of the "space of possibilities", we adhere to the **Equivalence Partitioning** method. The input space is categorized into three distinct partitions to validate both the correct behavior and the robustness of the system logic.

### 1.1 The Dual Approach
The requirement is addressed using two complementary testing levels:
1.  **Visual Validation (Integration):** Manual verification of the robot's kinematics moving smoothly in RViz (via `robot_bailarin` node).
2.  **Automated Defect Testing (Unit):** Rigorous verification of the trajectory logic using a **Mocking Strategy**. This allows testing edge cases and invalid inputs in isolation without the full physics engine.

## 2. Test Cases Definition

The following test cases are implemented in the automated suite (`PathManagerTest`) to cover the defined partitions.

### Partition A: Valid Sequences (Validation)
*Goal: Demonstrate that the system accepts correct inputs (Happy Path).*

* **Test Case ID:** `TCTRAJ01_ValidSequence`
* **Description:** Input of a standard sequence of 3D poses within the workspace.
* **Expected Result:** The `PathManager` logic validates the points and returns a success status.

### Partition B: Boundary/Edge Cases
*Goal: Verify behavior at the limits of the logic.*

* *Note:* Boundary testing (e.g., single-point trajectories) is handled inherently within the validation logic of `TCTRAJ01`. The system is designed to handle sequences of $N \ge 1$.

### Partition C: Invalid Sequences (Defect Testing)
[cite_start]*Goal: Discover faults by providing abnormal inputs [cite: 55-59].*

* **Test Case ID:** `TCTRAJ04_EmptySequence`
    * **Description:** Input of an initialized but empty trajectory container (Size = 0).
    * **Expected Result:** The system must reject the input and return the specific error: *"La lista de puntos está vacía"*.
    * **Rationale:** Verifies robustness against null/empty data handling.

* **Test Case ID:** `TCTRAJ05_UnreachableTarget`
    * **Description:** Input of a target pose defined outside the robot's maximum reach (> 2.0m).
    * **Expected Result:** The system must detect the safety violation and reject the input with error: *"Punto fuera del alcance"*.
    * **Rationale:** Verifies the safety layer logic in isolation.

## 3. Execution Procedure

The automated tests are implemented using **GoogleTest (GTest)** and integrated into the ROS 2 build system.

**Command to Run Tests:**
To execute the suite, run the following command in the workspace root:
```bash
colcon test --packages-select ros2_control_demo_example_7 \
  --ctest-args -R test_req_f_006 \
  --event-handlers console_direct+
