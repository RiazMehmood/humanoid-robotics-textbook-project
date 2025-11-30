---
sidebar_position: 3
---

# Isaac ROS

## Overview

**Isaac ROS** provides hardware-accelerated ROS 2 packages that leverage NVIDIA GPUs for high-performance perception and navigation.

## Installation

### Docker (Recommended)

```bash
docker pull nvcr.io/nvidia/isaac/isaac-ros:latest
docker run --rm -it --privileged \
  --network host \
  -v /dev:/dev \
  nvcr.io/nvidia/isaac/isaac-ros:latest
```

## Key Packages

### VSLAM (Visual SLAM)

Hardware-accelerated visual simultaneous localization and mapping:

```bash
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py
```

### Image Processing

GPU-accelerated image processing:

```bash
ros2 launch isaac_ros_image_proc image_proc.launch.py
```

### Object Detection

GPU-accelerated object detection using TensorRT:

```bash
ros2 launch isaac_ros_tensor_rt_ros isaac_ros_tensor_rt_ros.launch.py
```

## VSLAM Implementation

### Configuration

```yaml
visual_slam:
  ros__parameters:
    enable_rectified_pose: true
    enable_debug_mode: false
    enable_imu_fusion: true
    enable_slam_visualization: true
```

### Usage

```python
from isaac_ros_visual_slam import VisualSlamNode

vslam_node = VisualSlamNode()
vslam_node.set_parameters({
    'enable_rectified_pose': True,
    'enable_imu_fusion': True
})
```

## Performance Benefits

Isaac ROS provides:

- **10-100x speedup**: GPU acceleration vs CPU
- **Real-time processing**: High frame rates
- **Lower latency**: Reduced processing time
- **Energy efficiency**: GPU optimized algorithms

## Next Steps

Learn about [Nav2 Path Planning](/module3/nav2-path-planning) for bipedal humanoid navigation.

