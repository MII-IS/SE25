# SE25 Development Methodology

## A Simple Methodology for Software System Modeling and Development with Eclipse

A simple and straightforward methodology for modeling and developing software systems can be achieved by 
leveraging the powerful capabilities of Eclipse Papyrus for UML modeling and Eclipse CDT for C/C++ development. 

This approach facilitates a model-driven development process, enabling developers to design and visualize 
the system architecture before diving into coding, and then generating a code skeleton that can be further 
developed within the same integrated environment.

### **Phase 1: System Modeling with Eclipse Papyrus**

The initial phase focuses on creating a comprehensive and well-defined model of the software system using 
UML diagrams in Eclipse Papyrus.

**1. Project Setup and Perspective:**

*   Begin by creating a new "Papyrus Project" within Eclipse.
*   Switch to the "Papyrus" perspective to access the necessary modeling tools and views. This perspective
*   provides the "Model Explorer" for navigating the model structure and the diagram editor.

**2. UML Modeling:**

*   **Create UML Diagrams:** Papyrus supports a wide range of UML diagrams. For a basic methodology, start
  with the most relevant ones:
    *   **Class Diagrams:** To define the static structure of the system, including classes, their attributes,
      operations, and relationships (associations, generalizations, dependencies).
    *   **Sequence Diagrams:** To model the dynamic behavior and interactions between objects over time.
    *   **State Machine Diagrams:** To describe the behavior of individual objects in response to events.
*   **Apply the C/C++ Profile:** To enable C++ code generation, it is crucial to apply the "C/C++" profile
  to your UML model. This can be done through the "Profile" tab in the properties of the root model element.
*   **Use Stereotypes for Language-Specific Constructs:** After applying the C++ profile, you can use
  stereotypes to specify language-specific details. For instance, the `«Create»` and `«Destroy»` stereotypes
  from the UML standard profile can be applied to operations to designate them as constructors and destructors, respectively.



What diagrams are necesary? UML offers 14 types of diagrama, but, in our small course we will onaly address the 
minimal set of UML diagrams needed to understand a system's design.

> ###  Bare Essentials: A Minimal Set of UML Diagrams for System Design
>
> To gain a foundational understanding of a software system's design without getting lost in excessive detail,
> a minimal set of three UML diagrams is often sufficient. Each diagram answers a fundamental question about the system.
>
> ---
>
> #### 1. Use Case Diagram
>
> *   **What it answers:** *What does the system do and for whom?*
> *   **Purpose:** This diagram provides a high-level, external view of the system. It identifies the "actors"
>     (users or other systems) and the main "use cases" (the functionalities the system offers to those actors).
>     It's the best starting point for understanding the system's purpose and scope from a user's perspective.
> *   **Minimal content:**
>     *   All primary actors.
>     *   The most critical use cases that define the core functionality of the system.
>
> ---
>
> #### 2. Class Diagram
>
> *   **What it answers:** *What is the static structure of the system?*
> *   **Purpose:** This is the blueprint of the system's architecture. It shows the key classes, their
>   attributes (data), and their methods (behavior). Crucially, it visualizes the relationships between
>   classes, such as associations (how they are connected) and inheritance (how they are specialized).
> *   **Minimal content:**
>     *   The main classes or entities involved in the core logic.
>     *   Essential attributes and methods for each class (not necessarily all of them).
>     *   The primary relationships (associations, generalizations) between these classes.
>
> ---
>
> #### 3. Sequence Diagram
>
> *   **What it answers:** *How do the parts of the system collaborate to achieve a specific task?*
> *   **Purpose:** While a Class Diagram shows the static structure, a Sequence Diagram illustrates
>   the dynamic behavior. It models the interactions between objects over time for a *single, specific scenario*
>   (often corresponding to one of the use cases). It is invaluable for understanding how different parts of the system work together.
> *   **Minimal content:**
>     *   A diagram for one or two of the most important use cases, showing the sequence of method calls between the participating objects.
>
> By creating and analyzing just these three diagrams, a developer or stakeholder can quickly grasp the system's
> purpose, its main components, and how those components interact to deliver key functionality.



### **Phase 2: Code Generation from the UML Model**

Once the UML model is sufficiently detailed, the next step is to automatically generate C++ source code from it.

**3. Generating C++ Code:**

*   In the "Model Explorer," right-click on the model element (e.g., a package or the root element) from which you want to generate code.
*   Select the option to "Generate C++ Code." Papyrus will then translate the UML constructs into corresponding C++ header and source files.


> ### 💡 Alternative Approach: Reverse Engineering from Existing Code
>
> While the primary methodology described is a "model-first" approach (designing the model and then generating code), a powerful alternative is to start with an existing code implementation. This "reuse-oriented" workflow is particularly effective when you have a library, a previous project, or an open-source component that provides a solid foundation.
>
> The process is as follows:
>
> 1.  **Import Existing Code:** Begin by finding a suitable C++ codebase and importing it into Eclipse as a CDT project.
> 2.  **Generate the UML Model (Reverse Engineering):** Instead of creating a model from scratch, use a reverse engineering tool to generate the UML model directly from the imported source code. Eclipse Papyrus has capabilities for this, allowing you to create class diagrams and other structural representations automatically.
> 3.  **Analyze and Understand:** The generated model serves as a high-level blueprint of the existing code. Use it to quickly understand the architecture, key classes, and their relationships without having to read every line of the implementation.
> 4.  **Adapt and Complete:** With a clear understanding of the starting point, you can now modify the UML model to reflect your new requirements. Subsequently, adapt and complete the reused code in Eclipse CDT to match the updated design. This iterative process of refining the model and the code helps ensure they remain synchronized.
>
> This approach can significantly accelerate development by leveraging proven code and focusing effort on adaptation and new feature implementation rather than building everything from the ground up.

### **Phase 3: C++ Development with Eclipse CDT**

With the initial code structure in place (or the imported code if doing reuse), the development focus shifts to implementing the business logic 
within the generated files using Eclipse CDT.

**4. Switch to the CDT Perspective:**

*   Open the "C/C++" perspective in Eclipse to access the powerful features of the C/C++ Development Tooling (CDT),
  which provides a fully functional C and C++ IDE.

**5. Import and Develop the Code:**

*   If the generated code is not already in a CDT project, create a new "C++ Project" and import the generated
  files.
*   **Implement Business Logic:** The generated code will contain class declarations, method stubs, and other
  structural elements based on the UML model. The developer's primary task is to fill in the implementation
  details and business logic within these generated functions.
*   **Utilize CDT Features:** Leverage the advanced features of CDT, such as:
    *   **Code completion and syntax highlighting.**
    *   **Debugging tools.**
    *   **Integration with build systems like Make and GCC.**


---

### **Phase 4: Collaboration Using Git and GitHub**

For team-based projects, integrating a robust version control system is essential. Eclipse provides excellent integration with Git via the EGit plugin.

**6. Repository Setup and Structure:**
*   **Single Repository:** It is recommended to manage both the model and the code in a single Git repository to ensure they stay synchronized.
*   **Directory Structure:** Organize your repository with distinct folders for clarity:
    ```
    /project-name/
    |-- /model      # Contains all Papyrus files (.uml, .notation, .di)
    |-- /src        # Contains all C++ source code (.h, .cpp)
    |-- .gitignore  # To exclude build artifacts and IDE metadata
    |-- README.md
    ```*   **Configure `.gitignore`:** Create a `.gitignore` file in the root of your repository to prevent committing user-specific IDE files and build outputs.
    ```gitignore
    # Eclipse workspace files
    .metadata
    .recommenders

    # CDT build artifacts
    /Debug/
    /Release/
    *.o
    *.a
    *.so
    *.exe
    ```

**7. Handling Model and Code in Git:**
*   **The Challenge of Merging Models:** Papyrus model files are XML-based. While Git can track their history, merging branches with conflicting changes to the same model file is extremely difficult to do manually.
*   **Best Practices for Model Collaboration:**
    *   **Modularize Your Model:** Break down the system model into smaller, discrete packages. This allows different developers to work on separate parts of the model simultaneously with a lower risk of conflicts.
    *   **Communicate Clearly:** Before modifying a shared part of the model, communicate with your team to avoid conflicts. A common strategy is a "social lock," where a developer announces they are actively working on a specific model package.
    *   **Pull Frequently, Commit Often:** Keep your local branch updated by pulling changes from the remote repository frequently. Make small, logical commits to both the model and the code.

**8. Collaborative Workflow Example:**
1.  **Create a Feature Branch:** Before starting work, create a new branch from the main development branch (e.g., `git checkout -b feature/user-login`).
2.  **Model the Changes:** Open the project in the Papyrus perspective. Modify the UML diagrams and model elements as required for the new feature.
3.  **Commit Model Changes:** Commit the updated model files (`.uml`, `.notation`, etc.) with a clear message: `git commit -m "feat(model): Add LoginManager class and sequence diagram"`.
4.  **Generate and Implement Code:** Regenerate the C++ code from the model. Switch to the CDT perspective and implement the necessary business logic in the C++ files.
5.  **Commit Code Changes:** Commit the implemented source code: `git commit -m "feat(code): Implement login logic in LoginManager"`.
6.  **Push and Create a Pull Request:** Push your branch to GitHub (`git push origin feature/user-login`) and open a Pull Request (PR). In the PR description, clearly explain the changes made to both the model and the code.
7.  **Review and Merge:** Team members review the code. While the model changes are not easily visualized in a standard diff view, the commit messages and PR description provide the necessary context. Once approved, the PR is merged into the main branch.

---

### **Phase 5: Iteration and Synchronization**

Software development is an iterative process. This methodology supports iterating between the model and the code.

**9. Model-Code Synchronization:**

*   **Model-First Approach:** For significant structural changes, it is recommended to update the Papyrus UML
  model first. This includes adding new classes, changing relationships, or modifying public method signatures.
  After updating the model, regenerate the code. Be mindful that this may overwrite manual changes in the generated
  files, so a version control system is highly recommended.
*   **Manual Code Updates:** For smaller implementation changes or bug fixes, directly edit the code in CDT.
*   **Reverse Engineering (Advanced):** While Papyrus has some capabilities for reverse engineering, a disciplined
  model-first approach for architectural changes is generally simpler and less error-prone for this methodology.

---

By following this streamlined methodology, development teams can benefit from the clear architectural vision
provided by UML modeling in Papyrus while leveraging the robust C++ development environment of Eclipse CDT. 
This process promotes a better understanding of the system design, improves consistency between design and 
implementation, and can significantly accelerate the initial development phases.



