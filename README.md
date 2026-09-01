# Differential Drive Robot (DDR) Simulation & Autonomous Exploration

A ROS 2 Humble project featuring a differential drive robot simulated in Gazebo, including autonomous mapping and exploration. The environment is fully containerized using Docker on WSL/Linux for seamless setup and reproducibility.

---

## Prerequisites

* [Docker Desktop](https://www.docker.com/) (with WSL 2 backend enabled)
* [Visual Studio Code](https://code.visualstudio.com/)
* WSL 2 configured on Windows (or a native Linux environment)
* An X-server setup (like VcXsrv) or WSLg for Gazebo/GUI forwarding

---

## Docker Environment Setup

Open your terminal in VS Code and ensure you are inside WSL:

```bash
wsl
