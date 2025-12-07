# Instructor Guide: Physical AI & Humanoid Robotics

This guide provides instructors with resources, teaching notes, and assessment rubrics for effectively delivering the "Physical AI & Humanoid Robotics: From Simulation to Reality" course.

## Course Overview

**Target Audience**: Students, AI practitioners, and enthusiasts possessing foundational Python and AI/ML knowledge.

**Course Goal**: Provide comprehensive, hands-on guidance for building, simulating, and controlling intelligent embodied systems interacting with physical environments.

**Key Learning Outcomes**:
*   Master ROS 2 for robotic control.
*   Construct digital twins using Gazebo, Unity, and NVIDIA Isaac Sim.
*   Integrate AI with robotics, including perception and navigation.
*   Develop Vision-Language-Action (VLA) systems using LLMs and voice control.
*   Implement capstone autonomous humanoid robot.

## Teaching Notes by Module

### Module 1: The Robotic Nervous System
*   **Focus**: ROS 2 foundations and robot modeling (URDF). Essential for all subsequent modules.
*   **Key Concepts**: ROS 2 nodes, topics, services, actions, parameters. URDF links, joints, visuals, collisions.
*   **Discussion Points**:
    *   Why middleware like ROS 2 is necessary for robotics?
    *   Distinctions between ROS 1 and ROS 2 (emphasize real-time, security, DDS).
    *   Importance of accurate robot modeling.
*   **Common Pitfalls**: Environment setup issues (sourcing, dependencies), XML syntax errors in URDF.
*   **Activity Ideas**:
    *   Live demonstration of `rqt_graph`.
    *   Challenge students modeling simple objects (e.g., furniture) in URDF.

### Module 2: The Digital Twin
*   **Focus**: Simulation environments (Gazebo, Unity, Isaac Sim). Bridging URDF to simulation.
*   **Key Concepts**: Gazebo components, physics engines, sensor simulation. Unity `ArticulationBody`, ROS-TCP-Connector.
*   **Discussion Points**:
    *   The "sim-to-real" gap and mitigation strategies.
    *   Trade-offs between different simulators (Gazebo for physics, Unity for graphics, Isaac Sim for AI/synthetic data).
*   **Common Pitfalls**: Physics parameters leading to unstable simulations, ROS 2-simulator communication issues.
*   **Activity Ideas**:
    *   Challenge students creating simple obstacle courses in Gazebo.
    *   Demonstrate simple control loops where ROS 2 nodes control Unity-simulated joints.

### Module 3: The AI Robot Brain
*   **Focus**: Advanced perception (Isaac ROS VSLAM) and autonomous navigation (Nav2).
*   **Key Concepts**: VSLAM principles, hardware acceleration, costmaps, global/local planning.
*   **Discussion Points**:
    *   How GPUs accelerate perception tasks.
    *   Docker's role in managing complex dependencies for AI robotics.
    *   Challenges in real-time mapping and localization.
*   **Common Pitfalls**: Docker setup issues, Nav2 parameter tuning, sensor data misconfiguration.
*   **Activity Ideas**:
    *   Guide students through tuning Nav2 parameters and observing effects.
    *   Discuss different VSLAM algorithms and their suitability for various environments.

### Module 4: Vision-Language-Action
*   **Focus**: Integrating voice control (Whisper) and cognitive planning (LLMs) with robotics.
*   **Key Concepts**: ASR, NLU, prompt engineering, LLM-based planning, action orchestration.
*   **Discussion Points**:
    *   Ethical implications of LLM-controlled robots.
    *   Future of human-robot natural language interaction.
    *   Limitations of current LLM planners.
*   **Common Pitfalls**: LLM prompt engineering, API key management, latency issues.
*   **Activity Ideas**:
    *   Brainstorm new robot skills that LLMs could orchestrate.
    *   Have students critique robot responses to ambiguous voice commands.

## Assessment Rubrics

### General Criteria
*   **Code Quality**: Readability, comments, adherence to ROS 2 best practices.
*   **Functionality**: Code executes without errors, meets requirements.
*   **Understanding**: Ability to explain concepts, troubleshoot issues.
*   **Problem Solving**: Approach to debugging, creative solutions.

### Project-Specific Rubrics

**Module Project 1 (Voice-Controlled Robot Arm - Conceptual):**
-   **Environment Setup**: ROS 2 correctly installed and sourced.
-   **URDF Model**: Robot model is valid and visualized in Rviz2.
-   **ROS 2 Nodes**: Basic publisher/subscriber nodes are functional.

**Module Project 2 (Simulated Robotic Arm in Gazebo):**
-   **Gazebo World**: Custom Gazebo world created and functional.
-   **Robot Spawning**: URDF arm correctly spawned and visible in Gazebo.
-   **ROS 2 Control**: Arm controllable via ROS 2 commands in Gazebo.

**Module Project 3 (Autonomous Robot Navigation in Isaac Sim):**
-   **Isaac Sim Setup**: Isaac Sim environment and robot configured.
-   **VSLAM Implementation**: VSLAM running and providing localization.
-   **Nav2 Integration**: Robot navigates autonomously to goals.

**Module Project 4 (Autonomous Humanoid with Voice Control - Capstone):**
-   **VLA Pipeline**: All components (Whisper, LLM, Nav2, Isaac Sim) integrated.
-   **Command Execution**: Humanoid executes multi-step voice commands.
-   **Error Handling**: Basic error handling implemented.

## Additional Instructor Resources

*(This section will be populated with links to external teaching materials, additional exercises, and advanced topics as content evolves.)*
