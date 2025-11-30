---
sidebar_position: 4
---

# Bridging Python Agents to ROS Controllers

## Overview

This section covers how to integrate Python-based AI agents with ROS 2 controllers using `rclpy`. This enables AI systems to control physical robots through ROS 2 interfaces.

## Architecture Pattern

The typical architecture involves:

1. **AI Agent Layer**: Python code implementing AI logic (e.g., decision-making, planning)
2. **ROS 2 Bridge**: `rclpy` nodes that translate between AI agent actions and ROS 2 messages
3. **ROS 2 Controllers**: Standard ROS 2 nodes for robot control

## Creating a Bridge Node

### Basic Bridge Structure

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class AIAgentBridge(Node):
    def __init__(self):
        super().__init__('ai_agent_bridge')
        
        # Publisher for robot control
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Subscriber for sensor data
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )
        
        # AI agent instance
        self.ai_agent = MyAIAgent()
    
    def lidar_callback(self, msg):
        # Process sensor data with AI agent
        action = self.ai_agent.process_sensor_data(msg.ranges)
        
        # Convert AI action to ROS 2 command
        cmd = Twist()
        cmd.linear.x = action['linear_velocity']
        cmd.angular.z = action['angular_velocity']
        
        self.cmd_vel_pub.publish(cmd)
```

## Integrating with AI Frameworks

### OpenAI Integration Example

```python
import openai
from rclpy.node import Node
from std_msgs.msg import String

class LLMAgentBridge(Node):
    def __init__(self):
        super().__init__('llm_agent_bridge')
        self.openai_client = openai.OpenAI(api_key="your-api-key")
        
        self.command_sub = self.create_subscription(
            String,
            '/voice_command',
            self.process_command,
            10
        )
    
    def process_command(self, msg):
        # Use LLM to interpret command
        response = self.openai_client.chat.completions.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": "You are a robot control assistant."},
                {"role": "user", "content": f"Translate this command to robot actions: {msg.data}"}
            ]
        )
        
        # Parse LLM response and execute ROS 2 actions
        self.execute_actions(response.choices[0].message.content)
```

## Message Type Conversion

### Converting Between Python Types and ROS Messages

```python
from geometry_msgs.msg import Pose, Point, Quaternion

def create_pose(x, y, z, qx, qy, qz, qw):
    """Convert Python values to ROS 2 Pose message."""
    pose = Pose()
    pose.position = Point(x=x, y=y, z=z)
    pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
    return pose

def pose_to_dict(pose):
    """Convert ROS 2 Pose message to Python dict."""
    return {
        'position': {
            'x': pose.position.x,
            'y': pose.position.y,
            'z': pose.position.z
        },
        'orientation': {
            'x': pose.orientation.x,
            'y': pose.orientation.y,
            'z': pose.orientation.z,
            'w': pose.orientation.w
        }
    }
```

## Error Handling

Always implement robust error handling:

```python
def safe_publish(self, publisher, message):
    try:
        publisher.publish(message)
    except Exception as e:
        self.get_logger().error(f'Failed to publish: {e}')
```

## Next Steps

Learn about [URDF for Humanoids](/module1/urdf-humanoids) to understand robot description formats.

