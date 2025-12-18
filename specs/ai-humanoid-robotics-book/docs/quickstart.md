---
id: quickstart
title: Quickstart Guide - AI-Native Physical Humanoid Robotics
sidebar_label: Quickstart Guide
---

# Quickstart Guide: AI-Native Physical Humanoid Robotics

## Overview

This guide provides a step-by-step introduction to setting up and running the AI-Native Physical Humanoid Robotics system. It covers the essential components from ROS 2 setup to basic AI command execution.

## Prerequisites

### System Requirements
- Ubuntu 22.04 LTS (recommended)
- 16GB+ RAM, 8+ core processor
- NVIDIA GPU with CUDA support (for Isaac Sim and AI processing)
- Internet connection for package downloads

### Software Dependencies
- ROS 2 Humble Hawksbill
- NVIDIA Isaac Sim
- Python 3.11+
- Docker and Docker Compose
- Git and version control tools

## Installation Steps

### 1. Install ROS 2 Humble

```bash
# Setup locale
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup sources
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 packages
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-colcon-common-extensions
sudo apt install python3-rosdep
```

### 2. Install NVIDIA Isaac Sim

Follow the official NVIDIA Isaac Sim installation guide:
1. Download Isaac Sim from NVIDIA Developer website
2. Extract to desired location (e.g., `/opt/isaac-sim`)
3. Install dependencies using the provided installer
4. Set up environment variables

### 3. Set Up Workspace

```bash
# Create workspace directory
mkdir -p ~/humanoid_robot_ws/src
cd ~/humanoid_robot_ws

# Source ROS 2
source /opt/ros/humble/setup.bash

# Install Python dependencies
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
pip3 install openai  # For Whisper and LLM integration
pip3 install pinecone-client  # For RAG system
```

### 4. Clone and Build Robot Packages

```bash
cd ~/humanoid_robot_ws/src

# Clone essential repositories
git clone https://github.com/ros-planning/navigation2.git
git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git
git clone https://github.com/RobotLocomotion/drake.git  # For dynamics

# Build the workspace
cd ~/humanoid_robot_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

## Basic Simulation Setup

### 1. Launch Gazebo Simulation

```bash
# Source the workspace
source ~/humanoid_robot_ws/install/setup.bash

# Launch a basic humanoid robot simulation
ros2 launch example_robot_data example.launch.py
```

### 2. Verify ROS 2 Communication

```bash
# Check available topics
ros2 topic list

# Monitor robot state
ros2 topic echo /joint_states sensor_msgs/msg/JointState

# Check available services
ros2 service list
```

## AI Integration Setup

### 1. Configure API Keys

```bash
# Create environment file
cat << EOF > ~/humanoid_robot_ws/api_keys.env
OPENAI_API_KEY=your_openai_api_key_here
PINECONE_API_KEY=your_pinecone_api_key_here
EOF
```

### 2. Test AI Services

```bash
# Test speech-to-text with Whisper
python3 -c "
import openai
from openai import OpenAI
client = OpenAI(api_key='your_api_key_here')
print('AI services configured successfully')
"
```

## Running Your First AI Command

### 1. Launch the Complete System

```bash
# Terminal 1: Launch simulation
source ~/humanoid_robot_ws/install/setup.bash
ros2 launch humanoid_bringup simulation.launch.py

# Terminal 2: Launch AI services
source ~/humanoid_robot_ws/install/setup.bash
source ~/humanoid_robot_ws/api_keys.env
python3 ai_command_server.py

# Terminal 3: Send commands
source ~/humanoid_robot_ws/install/setup.bash
python3 voice_command_client.py
```

### 2. Basic Voice Command Example

```bash
# In the voice command client terminal
Say: "Move forward 1 meter"
Expected behavior: Robot navigates forward 1 meter in simulation
```

## Parameter Configuration

### Essential Configuration Files

1. **Navigation Parameters** (`config/nav2_params.yaml`)
   - Costmap inflation settings
   - Planner parameters
   - Controller configuration

2. **Robot Description** (`urdf/humanoid.urdf.xacro`)
   - Joint limits
   - Physical dimensions
   - Sensor placements

3. **AI Service Configuration** (`config/ai_services.yaml`)
   - API endpoints
   - Timeout values
   - Safety thresholds

### Sample Parameter Adjustment

```bash
# View current navigation parameters
ros2 param list

# Adjust a parameter (example)
ros2 param set /local_costmap/local_costmap inflation_radius 0.5
```

## Troubleshooting

### Common Issues

1. **ROS 2 Nodes Not Communicating**
   - Check that all terminals source the same workspace
   - Verify ROS_DOMAIN_ID is consistent across terminals
   - Ensure firewall is not blocking ROS communication

2. **Simulation Running Slowly**
   - Reduce physics update rate in Gazebo
   - Lower rendering quality settings
   - Check GPU drivers and CUDA installation

3. **AI Services Not Responding**
   - Verify API keys are correctly set
   - Check internet connectivity
   - Confirm AI service endpoints are accessible

### Verification Commands

```bash
# Check system status
ros2 component list  # List active components
ros2 doctor  # Check ROS 2 installation health
nvidia-smi  # Check GPU status
```

## Next Steps

After completing this quickstart:

1. Explore the four core modules detailed in the book:
   - Module 1: The Robotic Nervous System (ROS 2)
   - Module 2: The Digital Twin (Gazebo & Unity)
   - Module 3: The AI-Robot Brain (NVIDIA Isaac™)
   - Module 4: Vision-Language-Action (VLA)

2. Try the capstone project to integrate all components

3. Experiment with the RAG system for knowledge-enhanced robot behavior

This quickstart provides the foundation for exploring the full AI-Native Physical Humanoid Robotics system described in the book.