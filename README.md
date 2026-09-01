# Differential Drive Robot (DDR) Simulation & Autonomous Exploration

A ROS 2 Humble project featuring a differential drive robot simulated in Gazebo, including autonomous mapping and exploration. The environment is fully containerized using Docker on WSL/Linux for seamless setup and reproducibility.

---

## Prerequisites

* [Docker Desktop](https://www.docker.com/) (with WSL backend enabled)
* [Visual Studio Code](https://code.visualstudio.com/)
* WSL configured on Windows (or a native Linux environment)
* An X-server setup (like VcXsrv) or WSLg for Gazebo/GUI forwarding

---

## Docker Environment Setup

Open your terminal in VS Code and ensure you are inside WSL:

```bash
wsl
``` 
1. Build the Docker Image:
build the container image with ROS 2 Humble and all required dependencies:
```bash
chmod +x build.sh run.sh exec.sh
./build.sh
```
2. Start and Enter the Container:
launch the container with GUI support enabled:
```bash
./run.sh
```
3. Open Additional Terminals (Multi-terminal Workflow):
whenever you need extra terminal instances inside the same running container (e.g., to run nodes in parallel), open a new terminal tab in VS Code and run:
```bash
./exec.sh
```
Build the Workspace
Once inside the running container, compile the ROS 2 packages and source the environment:
```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```
Tip: Remember to run source install/setup.bash in every new container terminal instance (opened via ./exec.sh).

Running the Simulation
1. Visualize the Robot and Environment
Inside the container, start the Gazebo simulation to inspect the robot model and world:
```bash
ros2 launch ddr_description gazebo.launch.py
```
2. Autonomous Navigation & Mapping
Before starting the autonomous pipeline, terminate the previous launch process (Ctrl + C).

Then, launch the autonomous mapping and exploration nodes:
```bash
ros2 launch ddr_exploration exploration.launch.py
```


