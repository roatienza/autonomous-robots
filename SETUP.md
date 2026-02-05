# Autonomous Robots Project Setup Guide

## Prerequisites

### Install uv and Python 3.12.12

First, install `uv` which is a faster alternative to virtualenv and pip:

```bash
# Install uv
curl -LsSf https://astral.sh/uv/install.sh | sh

# Add uv to your PATH (if not automatically added)
export PATH="$HOME/.cargo/bin:$PATH"

# Install Python 3.12.12
uv python install 3.12.12
```

## Python Environment Setup

This project requires Python 3.12.12. Here's how to set up your development environment:

### 1. Create a virtual environment using uv

```bash
# Create virtual environment with Python 3.12.12
uv venv --python 3.12.12

# Activate the virtual environment
source .venv/bin/activate
```

### 2. Install Python dependencies using uv

```bash
# Install dependencies using uv (faster than pip)
uv pip install -r requirements.txt
```

### 3. Install ROS2 dependencies

The project requires ROS2 (Humble Hawksbill recommended). Install the following ROS2 packages:

```bash
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install ros-humble-rclpy
sudo apt install ros-humble-sensor-msgs
sudo apt install ros-humble-geometry-msgs
sudo apt install ros-humble-std-srvs
sudo apt install ros-humble-action-msgs
```

### 4. Install robot-specific packages

#### For Franka Robot:
```bash
pip install panda-py==0.1.0
```

#### For TurtleBot3 (actions):
```bash
sudo apt install ros-humble-turtlebot3-msgs
```

### 5. Install development tools

```bash
# Install Python development tools using uv
uv pip install pytest setuptools

# Install ROS2 build tools via apt
sudo apt install python3-colcon-common-extensions
```

## Project Structure

- `franka/`: Franka robot control code (C++ and Python)
- `ros2/`: ROS2 packages for actions, services, and sensors
  - `actions/`: ROS2 action servers and clients
  - `services/`: ROS2 service servers and clients  
  - `sensors/`: ROS2 sensor nodes and subscribers

## Key Python Packages

- **rclpy**: ROS2 Python client library
- **sensor_msgs, geometry_msgs, std_msgs**: ROS2 message types
- **numpy**: Numerical computing
- **opencv-python**: Computer vision
- **panda-py**: Franka robot control
- **fastmcp**: Motion control protocol

## Notes

1. ROS2 packages are typically installed via `apt` rather than pip
2. Some packages like `rclpy` may need to be installed from ROS2 repositories
3. The `panda-py` package is specific to Franka robots and may require additional setup
4. For full ROS2 functionality, you'll need to source the ROS2 environment:
   ```bash
   source /opt/ros/humble/setup.bash
   ```