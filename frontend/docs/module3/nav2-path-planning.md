---
sidebar_position: 4
---

# Nav2 Path Planning for Humanoids

## Overview

**Nav2** (Navigation2) is the ROS 2 navigation stack that provides path planning and obstacle avoidance for mobile robots, adapted here for bipedal humanoid movement.

## Installation

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

## Architecture

Nav2 consists of:

- **Planner**: Path planning algorithms
- **Controller**: Motion control
- **Recovery**: Recovery behaviors
- **BT Navigator**: Behavior tree navigation

## Configuration for Humanoids

### Costmap Configuration

```yaml
local_costmap:
  local_costmap:
    robot_base_frame: base_link
    update_frequency: 5.0
    publish_frequency: 2.0
    global_frame: odom
    robot_radius: 0.3
    inflation_radius: 0.5
```

### Planner Configuration

```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
```

## Bipedal Movement Considerations

### Gait Planning

For humanoid robots, consider:

- **Step height**: Clearance for obstacles
- **Step length**: Maximum stride
- **Balance**: Center of mass management
- **Terrain adaptation**: Uneven surface handling

### Implementation

```python
from nav2_simple_commander import BasicNavigator

navigator = BasicNavigator()

# Set initial pose
initial_pose = PoseStamped()
initial_pose.header.frame_id = 'map'
initial_pose.pose.position.x = 0.0
initial_pose.pose.position.y = 0.0
navigator.setInitialPose(initial_pose)

# Navigate to goal
goal_pose = PoseStamped()
goal_pose.header.frame_id = 'map'
goal_pose.pose.position.x = 5.0
goal_pose.pose.position.y = 3.0

navigator.goToPose(goal_pose)

# Wait for completion
while not navigator.isTaskComplete():
    feedback = navigator.getFeedback()
    # Process feedback
```

## Next Steps

Proceed to [Module 4: Vision-Language-Action (VLA)](/module4/introduction) to learn about LLM integration.

