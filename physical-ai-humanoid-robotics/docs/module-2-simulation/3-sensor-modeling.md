---
id: sensor-modeling
title: 'Chapter 6: Advanced Sensor Simulation'
---

import Admonition from '@theme/Admonition';

## Simulating Senses: Giving Your Robot Eyes and Ears

A digital twin is incomplete without a simulated sensor suite. In Gazebo, we can add plugins to our URDF that replicate the data produced by real-world sensors like LiDARs, cameras, and IMUs. This allows us to develop and test perception and navigation algorithms entirely in simulation.

### The Sensor Simulation Data Flow

The core principle behind sensor simulation is to take the "ground truth" state from the physics engine, process it, add realistic noise, and publish the result as a standard ROS 2 message.

```mermaid
graph TD
    A[Ground Truth World State] --> B{Ideal Sensor Model};
    B --> C{Noise Model};
    C --> D[ROS 2 Message Publisher];
    D --> E[/sensor_topic];

    subgraph "Physics Engine (Gazebo)"
        A;
    end
    subgraph "Sensor Plugin"
        B;
        C;
        D;
    end
    subgraph "ROS 2 Network"
        E;
    end
```

---

## Noise Models: Embracing Imperfection

A perfect sensor doesn't exist. Real sensors have noise, and our simulations must model this to be useful.

*   **Gaussian (Normal) Noise:** This is the most common type of noise, affecting most sensor readings. It's modeled by a mean (usually 0.0) and a standard deviation. A larger standard deviation means more noise.
*   **IMU Drift & Bias:** An Inertial Measurement Unit (IMU) is particularly susceptible to two types of error:
    *   **Bias:** A constant offset in the readings.
    *   **Random Walk (Drift):** A slowly accumulating error over time. This is why an IMU alone isn't enough for long-term localization; its position estimate will drift.

The Gazebo sensor plugins allow you to specify the parameters for these noise models directly in your URDF/SDF.

---

## Simulating Key Sensors in Gazebo

Here are the SDF snippets required to add the three most common sensors to your robot's URDF. These `<gazebo>` blocks are added at the same level as your `<link>` and `<joint>` tags.

### 1. 3D LiDAR

This example uses a ray-based sensor to simulate a 360-degree LiDAR, publishing `sensor_msgs/LaserScan`.

```xml title="lidar.xacro"
<gazebo reference="base_link">
  <sensor type="ray" name="lidar_sensor">
    <pose>0 0 0.3 0 0 0</pose>
    <visualize>true</visualize>
    <update_rate>10</update_rate>
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
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.01</stddev>
      </noise>
    </ray>
    <plugin name="gazebo_ros_head_hokuyo_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/my_humanoid</namespace>
        <output_type>sensor_msgs/LaserScan</output_type>
        <topic_name>laser_scan</topic_name>
        <frame_name>lidar_link</frame_name>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

<Admonition type="info" title="Environmental Effects: Fog">
  In a Gazebo `.world` file, you can add a `<fog>` block. The LiDAR plugin will automatically interact with this, causing rays to terminate early and simulating the effect of reduced visibility.
</Admonition>

### 2. RGB-D Depth Camera

This simulates a depth camera like an Intel RealSense, providing a color image, a depth image, and a point cloud.

```xml title="depth_camera.xacro"
<gazebo reference="head_link">
  <sensor type="depth" name="depth_camera_sensor">
    <update_rate>20.0</update_rate>
    <camera name="head">
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.05</near>
        <far>8.0</far>
      </clip>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>
      </noise>
    </camera>
    <plugin name="depth_camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/my_humanoid</namespace>
        <camera_name>depth_cam</camera_name>
        <frame_name>depth_camera_link</frame_name>
        <hack_baseline>0.07</hack_baseline>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

### 3. Inertial Measurement Unit (IMU)

This plugin simulates an IMU, providing orientation, angular velocity, and linear acceleration. The noise parameters are critical here to simulate drift.

```xml title="imu.xacro"
<gazebo reference="torso_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <visualize>true</visualize>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
            <bias_mean>0.0000075</bias_mean>
            <bias_stddev>0.0000008</bias_stddev>
          </noise>
        </x>
        <!-- ... similar noise for y and z axes ... -->
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
            <bias_mean>0.1</bias_mean>
            <bias_stddev>0.001</bias_stddev>
          </noise>
        </x>
        <!-- ... similar noise for y and z axes ... -->
      </linear_acceleration>
    </imu>
    <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
      <ros>
        <namespace>/my_humanoid</namespace>
        <topic_name>imu/data</topic_name>
        <frame_name>imu_link</frame_name>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```
