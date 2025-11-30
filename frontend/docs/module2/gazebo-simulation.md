---
sidebar_position: 2
---

# Gazebo Simulation

## What is Gazebo?

**Gazebo** is a 3D physics simulator for robotics that provides:

- Realistic physics simulation
- Sensor simulation
- Environment modeling
- ROS 2 integration

## Installation

### Ubuntu/Debian

```bash
sudo apt-get update
sudo apt-get install gazebo11 libgazebo11-dev
```

### ROS 2 Integration

```bash
sudo apt-get install ros-humble-gazebo-ros-pkgs
```

## Basic Concepts

### World Files

World files (`.world`) define the simulation environment:

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>
    <model name="my_robot">
      <include>
        <uri>model://humanoid_robot</uri>
      </include>
    </model>
  </world>
</sdf>
```

### Launching Gazebo

```bash
# Launch empty world
gazebo

# Launch with world file
gazebo my_world.world

# Launch with ROS 2
ros2 launch gazebo_ros gazebo.launch.py world:=my_world.world
```

## Physics Simulation

Gazebo simulates:

- **Gravity**: Configurable gravitational forces
- **Collisions**: Realistic collision detection
- **Friction**: Surface friction properties
- **Dynamics**: Joint forces and torques

## Environment Building

### Adding Models

```xml
<model name="table">
  <pose>1 0 0.5 0 0 0</pose>
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry>
        <box>
          <size>1 1 0.1</size>
        </box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box>
          <size>1 1 0.1</size>
        </box>
      </geometry>
    </visual>
  </link>
</model>
```

## ROS 2 Integration

Gazebo plugins enable ROS 2 communication:

```xml
<plugin name="gazebo_ros" filename="libgazebo_ros_api_plugin.so">
  <ros>
    <namespace>/gazebo</namespace>
  </ros>
</plugin>
```

## Next Steps

Learn about [Unity Rendering](/module2/unity-rendering) for high-fidelity visualization.

