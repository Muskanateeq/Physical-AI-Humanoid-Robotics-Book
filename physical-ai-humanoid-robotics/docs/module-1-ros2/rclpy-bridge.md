---
id: rclpy-bridge
title: 'Chapter 1: Bridging Worlds ROS 1 to ROS 2 Bridge'
---

import Admonition from '@theme/Admonition';

## Bridging the Gap: Connecting ROS 1 to ROS 2

While ROS 2 is the future, many legacy systems, drivers, and algorithms are still written for ROS 1. The `ros1_bridge` is a critical tool that allows for bidirectional communication between ROS 1 and ROS 2, enabling you to migrate your projects incrementally and leverage the vast ROS 1 ecosystem.

The bridge works by creating corresponding publishers and subscribers in the other ROS version for any topic, service, or action it discovers.

### How it Works

When you start the bridge, it scans for ROS 1 master and ROS 2 daemon activities. If a ROS 1 node starts publishing to `/ros1_topic`, the bridge will automatically create a ROS 2 publisher on `/ros1_topic` and pass the messages through. The same happens in reverse.

**ASCII Diagram: `ros1_bridge` in Action**
```
+------------------+      +---------------------+      +------------------+
|   ROS 1 Node     |      |                     |      |   ROS 2 Node     |
| (Pub on /data)   |----->|    ros1_bridge      |----->| (Sub on /data)   |
| (Sub on /cmd)    |<-----| (Handles translation) |<-----| (Pub on /cmd)    |
+------------------+      |                     |      +------------------+
                        +---------------------+
```

---

## Setting Up and Using the Bridge

To use the bridge, you need both a ROS 1 and a ROS 2 environment sourced.

### Installation

The bridge is a standard ROS 2 package.
```bash
# For ROS 2 Humble
sudo apt-get install ros-humble-ros1-bridge
```

### Running the Bridge

1.  **Terminal 1: Source ROS 1 and run roscore**
    ```bash
    source /opt/ros/noetic/setup.bash
    roscore
    ```

2.  **Terminal 2: Source both ROS versions and run the bridge**
    You must source both environments, with the version you are "bridging to" sourced last.
    ```bash
    source /opt/ros/noetic/setup.bash
    source /opt/ros/humble/setup.bash
    ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
    ```
    The `--bridge-all-topics` flag tells the bridge to handle every topic it finds. You can also bridge topics selectively.

---

## Practical Example: Bridging a ROS 1 Camera

Let's say you have a legacy camera driver that only publishes `sensor_msgs/Image` on ROS 1.

1.  **Terminal 1: Start `roscore`**
    ```bash
    source /opt/ros/noetic/setup.bash
    roscore
    ```

2.  **Terminal 2: Start the `ros1_bridge`**
    ```bash
    source /opt/ros/noetic/setup.bash
    source /opt/ros/humble/setup.bash
    ros2 run ros1_bridge dynamic_bridge
    ```

3.  **Terminal 3: Run the ROS 1 camera publisher**
    This command will publish a test video stream to the `/camera/image_raw` topic in ROS 1.
    ```bash
    source /opt/ros/noetic/setup.bash
    rosrun image_publisher image_publisher /path/to/your/test/video.mp4
    ```

4.  **Terminal 4: Verify in ROS 2**
    Now, in a ROS 2-sourced terminal, you can see the topic and even view the image stream.
    ```bash
    source /opt/ros/humble/setup.bash
    ros2 topic list
    # You should see /camera/image_raw in the list

    ros2 run image_tools showimage --ros-args -r image:=/camera/image_raw
    # An image window will appear showing the video stream
    ```

<Admonition type="warning" icon="️⚡️" title="Performance Considerations">
  The `ros1_bridge` is a powerful tool, but it's not free. Serializing and deserializing messages between ROS 1 and ROS 2 introduces latency and consumes CPU resources. For high-bandwidth topics like camera feeds or LiDAR scans, this overhead can become a bottleneck. If you're using a low-power embedded system like a **Raspberry Pi 4**, bridging more than one high-frequency data stream could saturate the CPU. In such cases, prioritize porting critical nodes directly to ROS 2 to avoid the bridge altogether.
</Admonition>
