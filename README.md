# ROS-Navigation: Hybrid Architecture for Autonomous Robots 🤖🚀

**ROS-Navigation** is a robotics project focused on building a functional hybrid architecture for autonomous robot navigation. Developed using **ROS (Robot Operating System)** and **Python**, the system allows a Turtlebot3 to navigate through complex environments by combining deliberative planning and reactive control.

This project was developed for the **Robotics and Computer Vision** challenge at Universidad Pablo de Olavide (UPO).

## 🏗️ System Architecture

The project implements a hybrid navigation stack as described in the technical documentation:
1.  **Global Path Planner**: Responsible for calculating the optimal route using algorithms over a costmap (Deliberative).
2.  **Robot Controller**: A reactive controller that executes the path while performing obstacle avoidance using Potential Fields.
3.  **Localization**: Integration of **AMCL** and **Gmapping** for precise positioning within a known or unknown map.

## 📊 Simulation & Visualization
* **Gazebo**: Used as the 3D physics engine for simulating the robot and its environment (static and dynamic obstacles).
* **RViz**: Used for real-time data visualization, including laser scans, costmaps, planned paths, and estimated poses.

## 📁 Project Structure

The core of the project is a Catkin workspace located in `rva/rva_exchange/rva_ws/src`:

| Package | Description |
| :--- | :--- |
| **`path_planner`** | Implements global navigation and costmap management. |
| **`robot_controller`** | Reactive control logic and velocity command publishing. |
| **`localization`** | Configuration for AMCL, Gmapping, and map servers. |
| **`robotics_challenge`** | The integration layer and evaluation scripts for the final challenge. |
| **`epd1 / epd2`** | Early-stage development modules for fundamental robotic controls. |

## 🐳 Dockerized Environment
To avoid "it works on my machine" issues, the project includes a **Docker** configuration:
* `Dockerfile`: Sets up the ROS environment, dependencies, and workspace.
* `run_container.bash`: A script to launch the container with GUI support for Gazebo and RViz.

## 📜 Documentation
For a deep dive into the implementation details:
* **[Memoria_Robotica.pdf](Memoria_Robotica.pdf)**: The full technical report covering the implemented methods (Potential Fields, Controller logic, etc.).
* **[Robotics_challenge.pdf](Robotics_challenge.pdf)**: The original project requirements and evaluation metrics for the navigation challenge.

## 🚀 Getting Started

### Prerequisites
* Docker and NVIDIA Container Toolkit (if using GPU acceleration).
* ROS Noetic (if running locally).

### Running with Docker
1.  Navigate to the `rva` folder.
2.  Launch the environment:
    ```bash
    bash run_container.bash
    ```
3.  Inside the container, build the workspace:
    ```bash
    cd /rva_exchange/rva_ws
    catkin_make
    source devel/setup.bash
    ```
4.  Launch the challenge:
    ```bash
    roslaunch robotics_challenge robotics_challenge.launch
    ```

## 👤 Authors
* **Luis Carmona** - [eLeCe2611](https://github.com/eLeCe2611)

---
*Developed for academic purposes to explore Hybrid Architectures and Robot Navigation.*
