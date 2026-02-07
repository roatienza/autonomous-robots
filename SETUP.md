# Autonomous Robots Project Setup Guide

## Prerequisites

### Install uv and Python 3.10.12

First, install `uv` which is a faster alternative to virtualenv and pip:

```bash
# Install uv
curl -LsSf https://astral.sh/uv/install.sh | sh

# Add uv to your PATH (if not automatically added)
export PATH="$HOME/.cargo/bin:$PATH"

# Install Python 3.10.12
uv python install 3.10.12
```

## Python Environment Setup

This project requires Python 3.10.12. Here's how to set up your development environment:

### 1. Create a virtual environment using uv

```bash
# Create virtual environment with Python 3.10.12
uv venv --python 3.10.12

# Activate the virtual environment
source .venv/bin/activate
```

### 2. Install Python dependencies using uv

```bash
# Install dependencies using uv (faster than pip)
uv pip install -r requirements.txt
```

### 3. ROS2 Environment Setup

This project assumes that ROS2 (Humble Hawksbill or later) is already installed on your system along with all necessary ROS2 packages and tools. This includes:

- ROS2 core packages (`rclpy`, `sensor_msgs`, `geometry_msgs`, `std_srvs`)
- ROS2 build tools (`colcon`, `ament_cmake`, etc.)
- TurtleBot3 specific packages (`turtlebot3_msgs`, etc.)
- Computer vision packages (`cv_bridge`, `image_transport`)

**Important**: Before running any ROS2 nodes, make sure to source your ROS2 environment:

```bash
source /opt/ros/humble/setup.bash
```

### 4. Install robot-specific packages

#### For Franka Robot:
```bash
pip install panda-py==0.1.0
```

### 5. Install development tools

```bash
# Install Python development tools using uv
uv pip install pytest setuptools
```

## Project Structure

```
autonomous-robots/
├── franka/
│   ├── cpp/
│   │   ├── CMakeLists.txt
│   │   ├── common.cpp
│   │   ├── common.h
│   │   ├── franka_hello_world.cpp
│   │   ├── grasp_object.cpp
│   │   ├── joint_impedance_control.cpp
│   │   ├── motion_server.cpp
│   │   ├── release_object.cpp
│   │   └── simple_motion.cpp
│   ├── python/
│   │   ├── basic.py
│   │   ├── mcp_client.py
│   │   ├── mcp_server.py
│   │   ├── rest.py
│   │   └── visual_control.ipynb
│   └── README.md
├── ros2/
│   ├── services/
│   │   ├── resource/
│   │   │   └── services
│   │   ├── services/
│   │   │   ├── __init__.py
│   │   │   ├── motion_server.py
│   │   │   └── sound_client.py
│   │   ├── test/
│   │   │   ├── test_copyright.py
│   │   │   ├── test_flake8.py
│   │   │   └── test_pep257.py
│   │   ├── package.xml
│   │   ├── setup.cfg
│   │   └── setup.py
│   └── sensors/
│       ├── resource/
│       │   └── sensors
│       ├── sensors/
│       │   ├── __init__.py
│       │   ├── sensors.py
│       │   └── timer_node.py
│       ├── test/
│       │   ├── test_copyright.py
│       │   ├── test_flake8.py
│       │   └── test_pep257.py
│       ├── package.xml
│       ├── setup.cfg
│       └── setup.py
├── .gitignore
├── LICENSE
├── README.md
├── requirements.txt
└── SETUP.md
```

### Key Directories:

- `franka/`: Franka robot control code (C++ and Python)
- `ros2/services/`: ROS2 service servers and clients
- `ros2/sensors/`: ROS2 sensor nodes and subscribers

## Key Python Packages

### ROS2 Core Packages:
- **rclpy**: ROS2 Python client library
- **sensor_msgs, geometry_msgs, std_msgs**: ROS2 message types
- **cv_bridge**: ROS2-OpenCV conversion bridge
- **image_transport**: ROS2 image transport plugins

### Computer Vision:
- **opencv-python**: Computer vision library
- **numpy**: Numerical computing

### Robot-Specific:
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
5. The video system requires OpenCV and cv_bridge for proper operation
6. Simulation mode is automatically enabled when no camera device is found
