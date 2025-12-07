# Chapter 5: Gazebo Fundamentals

:::info Chapter Info
**Module**: The Digital Twin | **Duration**: 3 hours | **Difficulty**: Intermediate
:::

## Learning Objectives
After completing this chapter, you'll be able to:
1. Navigate the core components and interface of Gazebo, the predominant ROS 2 simulator.
2. Construct simple simulation worlds (`.world` files) containing various objects.
3. Import and spawn URDF robot models into Gazebo environments.
4. Master interaction with the Gazebo simulation platform.

## Prerequisites
- Completion of Module 1 (Chapters 1-4), particularly Chapter 4: URDF Robot Modeling.
- Functional ROS 2 Humble installation.
- Basic XML syntax understanding.

## What You'll Build
This chapter guides you through constructing your first custom Gazebo simulation environment, including:
- A custom `.world` file featuring a simple ground plane and static objects.
- Spawning your two-link robotic arm (from Chapter 4) into this world.
- Controlling the robotic arm within Gazebo simulation using ROS 2 commands.

---

## Introduction: Your Virtual Robotics Laboratory

The physical world presents challenges for robotics experimentation—it's unpredictable, messy, and expensive. Consider testing a new navigation algorithm by repeatedly colliding a physical robot with obstacles, or investing hours configuring complex environments for manipulation tasks. This is where **robot simulators** become invaluable. Robot simulators enable creation of virtual environments complete with physics, sensors, and dynamic objects, allowing safe, repeatable, cost-effective testing and debugging of robot software.

**Gazebo** stands as one of the most powerful and extensively used 3D robot simulators within the ROS ecosystem. It delivers a robust physics engine, high-quality graphics, and convenient interfaces for creating intricate virtual worlds that integrate seamlessly with ROS. Gazebo is an open-source platform enabling accurate simulation of robot populations in complex indoor and outdoor settings. It generates realistic sensor feedback and physically plausible interactions, making it an indispensable tool for roboticists.

This chapter establishes the foundation for effective robot simulation by exploring Gazebo fundamentals. You'll discover its core components, navigate its graphical user interface (GUI), and crucially, create custom simulation worlds. We'll animate the URDF model of the two-link robotic arm you constructed in Chapter 4, bringing it to life within a Gazebo environment, demonstrating how to spawn and interact with your robot in virtual settings. By mastering these Gazebo fundamentals, you'll acquire the capability to rapidly prototype, test, and iterate on robot designs and control algorithms, accelerating your Physical AI journey.

## Core Concepts: Understanding Gazebo Architecture

Gazebo is a sophisticated simulator comprising several key components working together to create realistic virtual worlds.

### 1. Gazebo Server (`gzserver`)

The **Gazebo Server** represents the core physics engine and simulation logic. It operates in the background, managing all physics calculations, sensor data generation, and world state updates. It lacks a graphical interface.

### 2. Gazebo Client (`gzclient`)

The **Gazebo Client** provides the graphical user interface (GUI) enabling visualization of simulations, model interaction, and sensor data inspection. It connects to the Gazebo Server to display current world state. You can launch Gazebo in "headless" mode (server only) when visualization isn't needed, common for large-scale data generation or reinforcement learning training.

### 3. Worlds (`.world` files)

A **World file** is an XML-based file (`.world` extension) defining the entire simulation environment. It specifies:
*   **Models**: Robots, static objects (tables, walls), dynamic objects (balls, boxes).
*   **Physics Engine**: Physics engine type (e.g., ODE, Bullet, Simbody, DART) and its parameters.
*   **Lights**: Ambient, directional, point, or spot lights illuminating the scene.
*   **Ground Plane**: A floor for objects to rest upon.
*   **Plugins**: Extend Gazebo functionality (e.g., `libgazebo_ros_factory.so` for model spawning).

World files utilize **SDF (Simulation Description Format)**, a more comprehensive XML format than URDF. While URDF describes only single robots, SDF can describe complete environments, including multiple robots, static objects, terrain, and sensor configurations.

### 4. Models (SDF Models)

Models in Gazebo represent objects within simulations. They can be robots, furniture, buildings, or any physical entity. Gazebo models are typically defined using SDF files (`.sdf` extension).
An SDF model can contain:
*   **`<link>`**: Similar to URDF links, defining rigid bodies.
*   **`<joint>`**: Connecting links.
*   **`<visual>`**: Visual appearance.
*   **`<collision>`**: Collision properties.
*   **`<sensor>`**: Virtual sensor definitions (camera, LiDAR, IMU).
*   **`<plugin>`**: Extend model functionality (e.g., `libgazebo_ros_diff_drive.so` for differential drive control).

### 5. Gazebo ROS Packages and Plugins

To integrate Gazebo with ROS 2, you utilize specific ROS 2 packages and Gazebo plugins:
*   **`gazebo_ros_pkgs`**: Contains packages bridging ROS 2 and Gazebo (e.g., `gazebo_ros`, `gazebo_plugins`).
*   **`gazebo_ros_factory`**: A plugin (often loaded via `spawn_entity.py` script or directly in launch files) allowing ROS 2 nodes to request Gazebo to spawn new models into simulations.
*   **`gazebo_ros_control`**: Enables ROS 2 controllers (from `ros2_control`) to interact with simulated robot joints in Gazebo.
*   **`gazebo_ros_diff_drive` / `gazebo_ros_planar_move`**: Plugins providing standard ROS 2 interfaces for common mobile robot base types.

## Hands-On Tutorial: Constructing Your First Custom Gazebo World

We'll create a simple Gazebo world and spawn your two-link robotic arm (from Chapter 4) into it.

### Step 1: Create a ROS 2 Package for Gazebo Worlds

Navigate to your ROS 2 workspace `src` directory (e.g., `~/ros2_ws/src`).
Create a new package named `my_gazebo_worlds`:

```bash
ros2 pkg create --build-type ament_cmake my_gazebo_worlds --dependencies gazebo_ros launch_ros
```
*   `gazebo_ros`: Provides necessary bridge between ROS 2 and Gazebo.
*   `launch_ros`: For writing ROS 2 Python launch files.

### Step 2: Define a Simple `.world` File

Create a `worlds` directory inside your `my_gazebo_worlds` package:
```bash
mkdir my_gazebo_worlds/worlds
```
Create the file `my_gazebo_worlds/worlds/empty_arm_world.world` with this content:

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <world name="empty_arm_world">
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Global physics parameters -->
    <physics name="default_physics" default="true" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Your robot will be spawned here -->
  </world>
</sdf>
```
This simple world includes a `sun` for lighting and a `ground_plane`.

### Step 3: Create a Launch File to Start Gazebo and Spawn Your Robot

Create a `launch` directory inside your `my_gazebo_worlds` package:
```bash
mkdir my_gazebo_worlds/launch
```
Create the file `my_gazebo_worlds/launch/spawn_arm.launch.py` with this content:

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get the share directory of your robot description package
    my_robot_description_share_dir = get_package_share_directory('my_robot_description')
    
    # Path to your URDF file
    urdf_path = os.path.join(my_robot_description_share_dir, 'urdf', 'two_link_arm.urdf')

    # Start Gazebo server and client
    gazebo_ros_launch_dir = get_package_share_directory('gazebo_ros')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_launch_dir, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': os.path.join(get_package_share_directory('my_gazebo_worlds'), 'worlds', 'empty_arm_world.world')}.items()
    )

    # Node to publish the robot model to /robot_description
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_path, 'r').read()}]
    )

    # Node to spawn the robot entity in Gazebo
    # Uses the 'spawn_entity.py' script from gazebo_ros
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'two_link_arm'],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher_node,
        spawn_entity_node,
    ])
```

### Step 4: Configure `CMakeLists.txt` to Install Worlds and Launch Files

Open `my_gazebo_worlds/CMakeLists.txt`.
Add these lines to install the `worlds` and `launch` directories:

```cmake
install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
```
Place these after `ament_export_dependencies(gazebo_ros launch_ros)`.

### Step 5: Build and Launch

1.  **Build the package**: Navigate to your workspace root (`~/ros2_ws`) and execute:
    ```bash
    colcon build --packages-select my_gazebo_worlds my_robot_description # Build both packages
    source install/setup.bash
    ```
    *(Ensure `my_robot_description` is also built and sourced, as `my_gazebo_worlds` depends on its URDF.)*
2.  **Launch Gazebo and spawn your arm**:
    ```bash
    ros2 launch my_gazebo_worlds spawn_arm.launch.py
    ```
    Gazebo GUI should open, displaying your two-link robotic arm spawned in the empty world.

### Step 6: Interact with the Robot in Gazebo

Once your robot is spawned, you can interact with it. Currently, it will appear static, as we haven't integrated controllers.

You can use the Gazebo GUI to:
*   **Rotate/Pan/Zoom**: Navigate the view.
*   **Select Model**: Click on the robot to select it.
*   **Move Model**: Use translate/rotate tools in the toolbar to manually move your robot. (This overrides physics, so use carefully).

## Deep Dive: Gazebo Model Database and Standard Models

Gazebo includes a rich online model database (`model://`) providing ready-to-use models of robots, sensors, environments, and objects. You can include these models directly in your `.world` files using the `<include>` tag with a `uri` pointing to the model name.

Examples:
*   `model://sun`
*   `model://ground_plane`
*   `model://husky` (Husky mobile robot)
*   `model://pr2` (PR2 humanoid robot)

You can also create custom SDF models for static or dynamic objects and place them in a Gazebo model path (set by the `GAZEBO_MODEL_PATH` environment variable) for easy inclusion in your worlds.

## Troubleshooting: Gazebo Fundamentals Issues

1.  **Issue**: Gazebo GUI doesn't open, or Gazebo crashes on launch.
    *   **Cause**: Graphics driver issues, incompatible Gazebo version, or conflicting ROS 2 installations.
    *   **Solution**: Ensure graphics drivers are current. Verify Gazebo installation. Try launching `gazebo` directly from terminal (`gazebo --verbose`).
2.  **Issue**: Robot doesn't appear in Gazebo, or appears distorted.
    *   **Cause**: URDF parsing error, incorrect paths to meshes/textures, or issues with `robot_state_publisher` or `spawn_entity.py`.
    *   **Solution**: Check terminal output for errors from `robot_state_publisher` or `spawn_entity.py`. Validate URDF file using `check_urdf two_link_arm.urdf`. Ensure all package dependencies are met and sourced.
3.  **Issue**: Gazebo runs very slowly.
    *   **Cause**: High-fidelity models, many objects, complex textures, or slow hardware.
    *   **Solution**: Simplify models (use simpler collision geometries), reduce sensor update rates, disable shadows if unnecessary, use headless mode for `gzserver` (if only data generation required). Check system hardware resources.
4.  **Issue**: `ros2 launch` cannot find `gazebo_ros` or `my_gazebo_worlds`.
    *   **Cause**: Packages not built or not sourced.
    *   **Solution**: Execute `colcon build --packages-select gazebo_ros my_gazebo_worlds` and `source install/setup.bash`.

## Practice Exercises

1.  **Customize Your World**:
    *   Modify `empty_arm_world.world` to add a simple `box` model (`model://box`) or `table` (`model://table`) from Gazebo model database.
    *   Adjust its `pose` to place it next to your robotic arm.
    *   Relaunch Gazebo and verify the new object appears.
2.  **Spawn Multiple Robots**:
    *   Modify `spawn_arm.launch.py` to spawn a second instance of your `two_link_arm` at a different position.
    *   Ensure the `entity` name is unique for each spawned robot.
    *   Relaunch and verify two arms appear in your world.
3.  **Explore Gazebo Plugins**:
    *   Research a simple Gazebo plugin (e.g., `Camera` plugin or `IMU` plugin) and attempt to add it to your URDF (or separate SDF model for a sensor).
    *   Spawn the model in Gazebo and use `ros2 topic list` to see if a new ROS 2 topic is published by the sensor.

## Summary

This chapter introduced you to robot simulation with Gazebo:
- You learned about Gazebo's core components (server, client, worlds, models).
- You created a custom Gazebo world and successfully spawned your two-link robotic arm.
- You used ROS 2 launch files to orchestrate Gazebo and your robot model.
- You began interacting with the simulated environment and explored Gazebo's model database.

This foundational knowledge is crucial for testing and refining robot behavior in controlled virtual environments.

## Next Steps

The next chapter, "Physics Simulation," delves deeper into configuring realistic physics for robot models and generating meaningful sensor data within Gazebo.

➡️ Continue to [Chapter 6: Physics Simulation](./06-physics-simulation.md)

## Additional Resources
-   [Gazebo Tutorials](http://classic.gazebosim.org/tutorials)
-   [SDF Format Specification](http://sdformat.org/spec)
-   [ROS 2 Gazebo Integration Tutorials](https://classic.gazebosim.org/tutorials?cat=connect_ros)
