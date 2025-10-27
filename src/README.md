# SE25 Source Code

This directory contains the source code for the robotics project, which is copied from somewhere, generated and/or manually written based on the UML models located in the `../models` directory. The code in this folder is the implementation of the design specified in our Model-Based Systems and Software Engineering (MBSSE) approach.

## Copied Code

Most code comes from the ROS2 demo ...

## Code Generation

A significant portion of the code in this directory may be automatically generated from the UML models. This includes:

*   Class and data structures from the Block Definition Diagrams.
*   State machine implementations from the State Machine Diagrams.
*   Communication interfaces from the Internal Block Diagrams.

The code generation process is managed by [Name of Code Generation Tool/Framework] and can be triggered by running the scripts in the `scripts/` directory.

## Manually Written Code

While we strive for maximum code generation, some parts of the codebase require manual implementation. This typically includes:

*   Complex algorithmic logic within methods.
*   Hardware-specific driver implementations.
*   Integration with third-party libraries.

Manually written code is clearly marked and is designed to integrate seamlessly with the generated code.

## Directory Structure

The source code is organized by functionality, which mirrors the structure of the UML models:

*   **`<component_name>/`**: Each directory corresponds to a major component or block defined in the system's structural model.
    *   **`include/`**: Header files (`.h`, `.hpp`).
    *   **`src/`**: Source files (`.cpp`, `.py`).
*   **`main/`**: The main entry point of the application.
*   **`generated/`**: Contains the auto-generated code from the models. It is recommended not to modify files in this directory directly.

## Building the Code

To build the code, please follow the instructions in the main `README.md` file located at the root of the project. Ensure you have all the required dependencies and build tools installed.

## Relationship to Models

The traceability between the code and the models is crucial. Each component and major class in the source code should correspond to an element in the UML models. For any questions regarding the implementation of a specific model element, please refer to the relevant diagram in the `../models` directory.