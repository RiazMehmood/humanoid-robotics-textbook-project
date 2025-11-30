---
sidebar_position: 5
---

# URDF for Humanoid Robots

## What is URDF?

**URDF (Unified Robot Description Format)** is an XML format for describing the physical structure of a robot, including:

- Links (rigid bodies)
- Joints (connections between links)
- Visual and collision geometries
- Inertial properties
- Materials and textures

## Basic URDF Structure

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">
  
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.3 0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.3 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>
  
  <!-- Left Leg -->
  <link name="left_leg">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  
  <!-- Joint connecting base to left leg -->
  <joint name="left_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_leg"/>
    <origin xyz="0.1 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="10"/>
  </joint>
  
</robot>
```

## Joint Types

### Revolute Joint
Rotational joint with limits:

```xml
<joint name="shoulder_joint" type="revolute">
  <axis xyz="0 0 1"/>
  <limit lower="-3.14" upper="3.14" effort="50" velocity="5"/>
</joint>
```

### Prismatic Joint
Linear sliding joint:

```xml
<joint name="slider_joint" type="prismatic">
  <axis xyz="1 0 0"/>
  <limit lower="0" upper="1.0" effort="100" velocity="0.5"/>
</joint>
```

### Fixed Joint
Rigid connection (no movement):

```xml
<joint name="fixed_joint" type="fixed"/>
```

## Humanoid Robot Structure

A typical humanoid robot includes:

- **Torso**: Central body
- **Head**: Vision and sensing
- **Arms**: Left and right with shoulder, elbow, wrist joints
- **Legs**: Left and right with hip, knee, ankle joints
- **Hands/Feet**: End effectors

## Xacro for Modular URDF

**Xacro** (XML Macros) allows you to create modular, reusable URDF files:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid">
  
  <xacro:include filename="$(find humanoid_description)/urdf/head.xacro"/>
  <xacro:include filename="$(find humanoid_description)/urdf/torso.xacro"/>
  <xacro:include filename="$(find humanoid_description)/urdf/arm.xacro"/>
  <xacro:include filename="$(find humanoid_description)/urdf/leg.xacro"/>
  
  <xacro:head name="head"/>
  <xacro:torso name="torso"/>
  <xacro:arm name="left_arm" side="left"/>
  <xacro:arm name="right_arm" side="right"/>
  <xacro:leg name="left_leg" side="left"/>
  <xacro:leg name="right_leg" side="right"/>
  
</robot>
```

## Visualizing URDF

### Using RViz2

```bash
ros2 run rviz2 rviz2
```

Then add a RobotModel display and set the description parameter.

### Using Gazebo

URDF files can be directly loaded into Gazebo for simulation.

## Best Practices

1. **Modular design**: Use xacro for complex robots
2. **Accurate inertial properties**: Important for realistic simulation
3. **Collision geometries**: Should be simpler than visual geometries
4. **Joint limits**: Set realistic limits based on hardware
5. **Naming conventions**: Use consistent, descriptive names

## Next Steps

Proceed to [Module 2: The Digital Twin (Gazebo & Unity)](/module2/introduction) to learn about simulation.

