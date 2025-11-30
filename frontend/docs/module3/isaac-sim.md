---
sidebar_position: 2
---

# NVIDIA Isaac Sim

## Overview

**Isaac Sim** is a scalable robotics simulation platform that provides photorealistic, physically accurate virtual environments for developing, testing, and training AI robots.

## Installation

### Docker (Recommended)

```bash
docker pull nvcr.io/nvidia/isaac-sim:2023.1.1
docker run --name isaac-sim --entrypoint bash -it -v ~/docker/isaac-sim:/workspace/workspace nvcr.io/nvidia/isaac-sim:2023.1.1
```

### Native Installation

Follow NVIDIA's installation guide for your platform.

## Key Features

### Photorealistic Rendering

- **RTX Ray Tracing**: Realistic lighting and shadows
- **Physically-Based Materials**: Accurate material properties
- **High-Fidelity Sensors**: Camera, LiDAR, IMU simulation

### Physics Simulation

- **PhysX Engine**: Accurate physics simulation
- **Collision Detection**: Realistic interactions
- **Joint Dynamics**: Accurate robot movement

## Creating Scenarios

### Python API

```python
from omni.isaac.kit import SimulationApp

# Initialize simulation
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.robots import Robot

# Create world
world = World(stage_units_in_meters=1.0)

# Add robot
robot = world.scene.add(
    Robot(
        prim_path="/robot",
        name="humanoid",
        position=np.array([0, 0, 1.0])
    )
)

# Run simulation
world.reset()
for i in range(1000):
    world.step(render=True)

simulation_app.close()
```

## Synthetic Data Generation

Isaac Sim can generate synthetic training data:

```python
from omni.isaac.synthetic_utils import SyntheticDataHelper

synthetic_data = SyntheticDataHelper()
synthetic_data.initialize()

# Generate dataset
for i in range(1000):
    # Randomize scene
    world.randomize()
    
    # Capture data
    rgb_image = synthetic_data.get_rgb()
    depth_image = synthetic_data.get_depth()
    segmentation = synthetic_data.get_segmentation()
    
    # Save data
    save_training_data(rgb_image, depth_image, segmentation)
```

## ROS 2 Integration

Isaac Sim includes ROS 2 bridge:

```bash
# Launch with ROS 2
./runheadless.native.sh --ros
```

## Next Steps

Learn about [Isaac ROS](/module3/isaac-ros) for hardware-accelerated perception.

