---
sidebar_position: 4
---

# Sensor Simulation

## Overview

Simulating sensors is crucial for developing perception algorithms without physical hardware. This section covers simulation of LiDAR, depth cameras, and IMUs.

## LiDAR Simulation

### Gazebo LiDAR Plugin

```xml
<gazebo>
  <plugin name="gazebo_ros_laser_scan" filename="libgazebo_ros_laser_scan.so">
    <ros>
      <namespace>/</namespace>
    </ros>
    <output_type>sensor_msgs/LaserScan</output_type>
    <frame_name>laser_frame</frame_name>
  </plugin>
  <sensor name="laser" type="ray">
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>30.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
  </sensor>
</gazebo>
```

## Depth Camera Simulation

### RGB-D Camera Plugin

```xml
<gazebo>
  <plugin name="gazebo_ros_camera" filename="libgazebo_ros_camera.so">
    <ros>
      <namespace>/camera</namespace>
    </ros>
    <camera_name>camera</camera_name>
    <image_topic>image_raw</image_topic>
    <depth_image_topic>depth/image_raw</depth_image_topic>
  </plugin>
  <sensor name="camera" type="depth">
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
      </image>
    </camera>
  </sensor>
</gazebo>
```

## IMU Simulation

### IMU Plugin

```xml
<gazebo>
  <plugin name="gazebo_ros_imu" filename="libgazebo_ros_imu.so">
    <ros>
      <namespace>/imu</namespace>
    </ros>
    <topic_name>imu/data</topic_name>
    <body_name>base_link</body_name>
  </plugin>
  <sensor name="imu" type="imu">
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.01</stddev>
          </noise>
        </x>
      </angular_velocity>
    </imu>
  </sensor>
</gazebo>
```

## Sensor Data Processing

### ROS 2 Subscribers

```python
from sensor_msgs.msg import LaserScan, Image, Imu

class SensorProcessor(Node):
    def __init__(self):
        super().__init__('sensor_processor')
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10
        )
        self.camera_sub = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10
        )
    
    def lidar_callback(self, msg):
        # Process LiDAR data
        ranges = msg.ranges
        # ... processing logic
    
    def camera_callback(self, msg):
        # Process camera data
        # ... processing logic
    
    def imu_callback(self, msg):
        # Process IMU data
        orientation = msg.orientation
        # ... processing logic
```

## Next Steps

Proceed to [Module 3: The AI-Robot Brain (NVIDIA Isaac™)](/module3/introduction) to learn about advanced AI platforms.

