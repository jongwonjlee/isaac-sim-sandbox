# Isaac Sim Sandbox with ROS2 Integration

Author: Jongwon Lee (jongwonjlee@gmail.com)

This repository shows how to load a custom world and robot model in Isaac Sim while integrating with ROS2 for robot control, sensor measurement, and state estimation visualization.

Specifically, this sandbox:
- Opens a custom scene in a simple indoor room with several objects of different colors and light properties (e.g., opacity, reflectivity, metallic properties, etc.) — a bunny, two Eiffel Towers, and two cubes — along with a custom robot model (TurtleBot3 Burger) equipped with an RGB-D camera, all placed in the scene.
- Commands the robot using the ROS2 `cmd_vel` topic.
- Publishes the robot's odometry (odom -> base_link, where the odom frame represents the robot's starting pose) along with tf (World -> base_link -> camera) and RGB-D camera data using ROS2 topics.

## Structure
- `assets/`: Contains asset files such as STL models and universal scene description (USD) files for the objects and robot used in the simulation. 
- `ros/`: Contains a ROS2 launch file and a RViz configuration file.
- `sim/`: Contains a USD scene file.
- `docs/`: Contains documentation files, including this README.
- `deprecated/`: Contains deprecated or obsolete files, including standalone Python scripts for launching simulations, ROS2 nodes, or WebRTC streaming commands.

## Getting Started

### Prerequisites

This sandbox has been tested with Isaac Sim 4.5.0 in a conda environment (Python 3.10) and ROS2 Humble on Ubuntu 22.04. The testing was conducted on a machine equipped with NVIDIA GPUs (two RTX 4090s), accessed remotely via SSH using the Isaac Sim WebRTC Client for live streaming the Isaac Sim interface locally.
- For Isaac Sim installation on a conda environment, please refer to the [official documentation](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/install_python.html#installation-using-pip).
- For ROS2 Humble installation, please refer to the [official documentation](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html).
- For setting up SSH with Isaac Sim WebRTC Client, please refer to the [official documentation](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/download.html#latest-release).

### Clone the code and set the local paths

```bash
git clone https://github.com/jongwonlee/isaac-sim-sandbox.git
cd isaac-sim-sandbox
export WORK_DIR=${PWD}
```

### Launch Isaac Sim

If the Isaac Sim is installed in a conda environment, please activate the conda environment first.
```bash
conda activate <your-isaac-sim-conda-env>
```

Then, run launch the Isaac Sim. 
```bash
isaacsim
```

If remoting into a host machine, please make sure to enable WebRTC streaming, by running the following command in the host machine terminal.
```bash
isaacsim isaacsim.exp.full.streaming --no-window
```
Then, open the WebRTC client in your local machine to view the Isaac Sim window.

### Open the custom scene and play the simulation

In the Isaac Sim application window, open the custom scene by navigating to `${WORK_DIR}/sim/scene_0.usd`.

Next, either (1) click the play button on the left panel or (2) press the spacebar to start the simulation. The simulation won't do anything until the ROS2 nodes are launched in the next step.

### Launch ROS2 nodes

In a terminal, please run the following command to launch the ROS2 nodes for robot control and sensor data publishing.
```bash
ros2 launch ${WORK_DIR}/ros/run.launch
```

This will launch
- The Rviz2 application with pre-configured settings to visualize the image and depth data from the robot's RGB-D camera,
- A velocity topic publisher to command the robot in a circular trajectory, and
- The output of the tf data being published from the Isaac Sim.

The TurtleBot3 Burger robot should begin moving in a circular trajectory while publishing RGB-D camera data and tf information to ROS2 topics.

![Demo](demo.gif)


## References
- TurtleBot3: https://github.com/ROBOTIS-GIT/turtlebot3
- STL files: https://ozeki.hu/p_1116-sample-stl-files-you-can-use-for-testing.html