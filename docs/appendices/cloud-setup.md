# Cloud Setup Guide

This guide provides instructions for establishing cloud-based development environments to run tutorials and projects in this course. This is an alternative if you lack access to recommended local hardware.

## Supported Cloud Platforms

*(This section will be populated with specific cloud platform recommendations, instance types, and setup instructions as content evolves.)*

## General Cloud Setup Steps

1.  **Choose Cloud Provider**: Select provider offering GPU-accelerated instances (e.g., AWS, Google Cloud, Azure).
2.  **Select Instance Type**: Choose instance type with adequate CPU, RAM, and GPU resources. Refer to [Hardware Guide](./hardware-guide.md) for minimum recommendations.
3.  **Operating System**: Launch instance with Ubuntu 22.04 LTS.
4.  **SSH Access**: Configure SSH access to your instance.
5.  **Install NVIDIA Drivers**: Install appropriate NVIDIA GPU drivers and CUDA toolkit for your instance.
6.  **Clone Repository**: Clone course's companion code repository.
7.  **Install Dependencies**: Follow environment setup instructions in Module 1 installing ROS 2, Gazebo, and other required software.

## Running Docker Containers in Cloud

Docker containers provide consistent and isolated environments, making them ideal for cloud-based development. The course provides Dockerfiles setting up ROS 2 and Isaac ROS environments.

### 1. Build Docker Image (Example: ROS 2 Humble)

First, build Docker image for desired environment. Navigate to `docker/` directory of course's code examples on cloud instance.

```bash
cd /path/to/course-code/docker
docker build -t ros2_humble_dev -f Dockerfile.ros2_humble .
```
This command builds image named `ros2_humble_dev` using `Dockerfile.ros2_humble`.

### 2. Run Docker Container

Once image is built, run container from it. This command launches interactive container with necessary privileges and mounts for graphical applications if needed.

```bash
docker run -it --rm --privileged \
    --network host \
    --runtime nvidia \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v /path/to/your/ros2_ws:/ros2_ws:rw \
    --name ros2_dev_container \
    ros2_humble_dev bash
```
**Parameter explanations:**
-   `-it`: Interactive and pseudo-TTY.
-   `--rm`: Remove container after exit.
-   `--privileged`: Gives extended privileges (often needed for robotics simulations).
-   `--network host`: Allows container using host's network stack (important for ROS 2 discovery).
-   `--runtime nvidia`: Enables GPU access within container (requires NVIDIA Container Toolkit on host).
-   `-e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw`: Enables graphical forwarding for GUI applications.
-   `-v /path/to/your/ros2_ws:/ros2_ws:rw`: Mounts local ROS 2 workspace into container.
-   `--name ros2_dev_container`: Assigns name to container.
-   `ros2_humble_dev`: Docker image name to use.
-   `bash`: Runs bash shell inside container.

### 3. Develop Inside Container

Once inside container, ROS 2 environment will be sourced, and mounted workspace (`/ros2_ws`) will be available. You can then proceed with building and running ROS 2 packages as if on native Ubuntu installation.

### 4. Isaac ROS Specific Containers

For Isaac ROS development, typically use NVIDIA's pre-built Isaac ROS Docker images. Process is similar: pull image and run with GPU and network configurations.

```bash
docker pull nvcr.io/nvidia/ros:humble-desktop-cuda-jammy
docker run -it --rm --network host --privileged \
    --runtime nvidia \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix/:/tmp/.X11-unix:rw \
    -v /path/to/your/ros2_ws:/ros2_ws:rw \
    --name isaac_ros_dev_container \
    nvcr.io/nvidia/ros:humble-desktop-cuda-jammy bash
```
