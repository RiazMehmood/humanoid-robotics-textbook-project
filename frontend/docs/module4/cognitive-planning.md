---
sidebar_position: 3
---

# Cognitive Planning with LLMs

## Overview

**Cognitive Planning** uses Large Language Models (LLMs) to translate natural language commands into sequences of robot actions. For example, "Clean the room" becomes a series of ROS 2 actions.

## Architecture

```
Natural Language Command
    ↓
LLM Processing
    ↓
Action Sequence Generation
    ↓
ROS 2 Action Execution
```

## Implementation

### OpenAI Integration

```python
import openai
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class CognitivePlanner(Node):
    def __init__(self):
        super().__init__('cognitive_planner')
        self.openai_client = openai.OpenAI(api_key="your-api-key")
        self.command_sub = self.create_subscription(
            String, '/voice_command', self.process_command, 10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
    
    def process_command(self, msg):
        command = msg.data
        
        # Use LLM to generate action plan
        response = self.openai_client.chat.completions.create(
            model="gpt-4",
            messages=[
                {
                    "role": "system",
                    "content": """You are a robot control assistant. 
                    Translate natural language commands into JSON action sequences.
                    Available actions: move_forward, move_backward, turn_left, turn_right, 
                    pick_object, place_object, navigate_to.
                    Return JSON format: {"actions": [{"type": "action", "params": {}}]}"""
                },
                {
                    "role": "user",
                    "content": f"Command: {command}"
                }
            ]
        )
        
        # Parse and execute actions
        plan = self.parse_plan(response.choices[0].message.content)
        self.execute_plan(plan)
    
    def parse_plan(self, llm_response):
        import json
        try:
            plan = json.loads(llm_response)
            return plan["actions"]
        except json.JSONDecodeError:
            self.get_logger().error("Failed to parse LLM response")
            return []
    
    def execute_plan(self, actions):
        for action in actions:
            if action["type"] == "move_forward":
                self.move_forward(action["params"].get("distance", 1.0))
            elif action["type"] == "turn_left":
                self.turn_left(action["params"].get("angle", 90))
            # ... other actions
```

## Action Execution

### Movement Actions

```python
def move_forward(self, distance):
    cmd = Twist()
    cmd.linear.x = 0.5
    start_time = self.get_clock().now()
    
    while (self.get_clock().now() - start_time).nanoseconds < distance * 1e9:
        self.cmd_vel_pub.publish(cmd)
        rclpy.spin_once(self, timeout_sec=0.1)
    
    # Stop
    cmd.linear.x = 0.0
    self.cmd_vel_pub.publish(cmd)
```

## Context Awareness

Include robot state in LLM prompts:

```python
def get_robot_context(self):
    # Get current robot state
    position = self.get_current_position()
    objects = self.detect_objects()
    
    return f"""
    Robot State:
    - Position: {position}
    - Detected Objects: {objects}
    - Battery: {self.get_battery_level()}%
    """
```

## Next Steps

Proceed to the [Capstone Project](/module4/capstone-project) to build a complete autonomous system.

