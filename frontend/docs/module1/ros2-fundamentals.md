---
sidebar_position: 2
---

# ROS 2 Fundamentals

## ROS 2 Architecture

ROS 2 follows a distributed, node-based architecture where:

- **Nodes** are executable processes that perform specific tasks
- **Topics** enable asynchronous publish-subscribe communication
- **Services** provide synchronous request-response communication
- **Actions** handle long-running tasks with feedback

## Core Components

### DDS (Data Distribution Service)

ROS 2 uses DDS (Data Distribution Service) as its middleware, which provides:

- Real-time performance
- Quality of Service (QoS) policies
- Security features
- Cross-platform compatibility

### Workspace Structure

A typical ROS 2 workspace looks like:

```
workspace/
├── src/
│   └── my_package/
│       ├── package.xml
│       ├── setup.py
│       └── my_package/
│           └── __init__.py
├── build/
├── install/
└── log/
```

## Installation

### Ubuntu/Debian

```bash
sudo apt update
sudo apt install ros-humble-desktop
source /opt/ros/humble/setup.bash
```

### macOS

```bash
brew tap ros/ros2
brew install ros-humble-desktop
source /opt/ros/humble/setup.bash
```

## Basic Commands

### Creating a Package

```bash
ros2 pkg create --build-type ament_python my_package
```

### Building a Workspace

```bash
cd workspace
colcon build
source install/setup.bash
```

### Running Nodes

```bash
ros2 run package_name node_name
```

### Listing Topics

```bash
ros2 topic list
ros2 topic echo /topic_name
```

## Quality of Service (QoS)

QoS policies control how messages are delivered:

- **Reliability**: RELIABLE vs BEST_EFFORT
- **Durability**: TRANSIENT_LOCAL vs VOLATILE
- **History**: KEEP_LAST vs KEEP_ALL
- **Depth**: Buffer size for KEEP_LAST

## Next Steps

Learn about [Nodes, Topics, and Services](/module1/nodes-topics-services) to understand ROS 2 communication patterns.

