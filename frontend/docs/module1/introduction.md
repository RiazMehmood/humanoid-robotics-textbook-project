---
sidebar_position: 1
---

# Module 1: Introduction to ROS 2

## Overview

**Module 1: The Robotic Nervous System (ROS 2)** introduces you to the Robot Operating System 2 (ROS 2), the middleware framework that serves as the "nervous system" for robotic applications. ROS 2 enables communication between different components of a robot, allowing sensors, actuators, and AI agents to work together seamlessly.

## Learning Objectives

By the end of this module, you will:

- Understand the architecture and core concepts of ROS 2
- Master ROS 2 nodes, topics, services, and actions
- Learn to bridge Python AI agents to ROS 2 controllers using `rclpy`
- Understand URDF (Unified Robot Description Format) for humanoid robots
- Build and deploy ROS 2 packages

## What is ROS 2?

ROS 2 is a set of software libraries and tools for building robot applications. It provides:

- **Distributed architecture**: Components can run on different machines
- **Language-agnostic**: Support for Python, C++, and other languages
- **Real-time capabilities**: Improved performance over ROS 1
- **Security**: Built-in security features for production deployments

## Key Concepts

### Nodes
ROS 2 nodes are processes that perform computation. Each node typically has a single, well-defined purpose (e.g., sensor reading, motor control, path planning).

### Topics
Topics are named buses over which nodes exchange messages. They enable publish-subscribe communication between nodes.

### Services
Services provide request-response communication, useful for synchronous operations like querying sensor state.

### Actions
Actions are long-running, goal-oriented operations (e.g., navigating to a location) with feedback.

## Module Structure

This module covers:

1. **ROS 2 Fundamentals** - Core architecture and concepts
2. **Nodes, Topics, and Services** - Communication patterns
3. **Python Agents to ROS** - Bridging AI agents with ROS controllers
4. **URDF for Humanoids** - Robot description and modeling

## Next Steps

Proceed to [ROS 2 Fundamentals](/module1/ros2-fundamentals) to begin learning the core concepts.

