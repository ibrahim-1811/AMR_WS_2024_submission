# AMR Final Project

This project is designed to demonstrate an integrated robotics system using ROS2, structured around three core functionalities: Environment Exploration, Localization, and Path and Motion Planning. Below, you will find detailed instructions on setting up the environment and an explanation of each package's purpose and their interrelation within the project.

## Environment Setup

### Prerequisites

- ROS2 (Humble distribution) installed on your system. [Installation guide](https://gist.github.com/Elektra-V/74e241c97843efe6a5a0cc8e60067bca).
- Latest stable version of Python.

### Creating a ROS2 Workspace

1. Create a new workspace for the project:

```bash
mkdir -p ~/ros2_workspace/src
cd ~/ros2_workspace/
```

2. Clone this repository into the `src` directory of your workspace.

### Building the Workspace

In the root of your workspace (`~/ros2_workspace/`), run the following command to build all packages:

```bash
colcon build
```

### Sourcing the Workspace

To make ROS2 aware of the packages in your workspace, source the setup script:

```bash
source ~/ros2_workspace/install/setup.bash
```

## Packages Overview

### 1. Environment Exploration (`environment_exploration`)

This package is responsible for autonomously exploring the environment. It uses sensor data to map unknown areas and determine the next points of interest to explore.

- **Key Components:**
  - **Exploration Node:** Interacts with SLAM to receive map updates and determine exploration targets.
  - **Exploration Service:** Contains the logic for selecting exploration targets based on the current map and explored areas.

### 2. Localization (`localization`)

The localization package provides real-time positioning of the robot within the environment. It uses sensor data and the current map to estimate the robot's location accurately.

- **Key Components:**
  - **Localization Node:** Subscribes to sensor data and publishes the estimated position.
  - **Particle Filter Service:** Implements a particle filter algorithm to estimate the robot's position based on sensor readings and movements.

### 3. Path and Motion Planning (`path_and_motion_planning`)

This package handles navigating the robot towards a goal while avoiding obstacles. It combines global path planning with local motion strategies to ensure efficient and safe navigation.

- **Key Components:**
  - **Path Planning Node:** Receives goals and computes a path using both global and local planning algorithms.
  - **Planner Service:** Encapsulates the logic for path computation, integrating global path planning with a potential field method for obstacle avoidance.

## Inter-Component Relations

- **Environment Exploration and Localization:** The exploration component relies on accurate localization to update the map and choose new exploration targets. The map and robot's position, updated by the localization component, are essential for determining unexplored areas.
- **Path and Motion Planning:** Requires the map information from the environment exploration component for global path planning and the robot's current position from the localization component to plan and adjust the path in real-time.
- **Localization and Path and Motion Planning:** Localization provides the necessary position data for path planning, ensuring the robot can accurately follow the computed path and adjust its course based on real-time positioning.

## Running the System

To run each component, open a new terminal, ensure the workspace is sourced, and execute the corresponding `ros2 run` command for each node. For example:

```bash
ros2 run environment_exploration exploration_node
```

Repeat this step in separate terminals for the `localization` and `path_and_motion_planning` nodes.

## License

Project is open-sourced under the MIT license.
