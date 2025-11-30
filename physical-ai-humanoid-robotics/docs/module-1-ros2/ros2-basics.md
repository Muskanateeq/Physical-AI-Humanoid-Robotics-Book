---
id: ros2-basics
title: 'Chapter 2: The Core of ROS 2 Basics'
---

import Admonition from '@theme/Admonition';

## The Robotic Nervous System: An Introduction to ROS 2

Welcome to the foundational module of your journey into humanoid robotics. ROS 2 (Robot Operating System 2) is the backbone of our robot, analogous to a biological nervous system. It's a flexible framework for writing robot software, providing a standardized communication layer that allows different parts of your robot's software to discover each other, and send and receive data.

### Why ROS 2?

- **Decentralized:** No single point of failure (no ROS Master).
- **Real-time Friendly:** Improved support for real-time control loops.
- **Cross-platform:** Develop on Windows, macOS, and Linux.
- **Secure:** Built-in security features for authentication and encryption.

---

## Core Concepts: The Building Blocks of Communication

ROS 2 applications are a graph of interconnected nodes. Let's break down the fundamental components.

### Nodes

A **Node** is the smallest unit of computation in ROS 2. Think of it as a specialized worker in a factory. One node might be responsible for reading a sensor, another for controlling a motor, and a third for processing camera data. Each node is a standalone executable.

### Topics (The Asynchronous Megaphone)

**Topics** are named buses over which nodes exchange messages. They follow a publish-subscribe model.

- **Publisher:** A node that *pushes* messages onto a topic.
- **Subscriber:** A node that *receives* messages from a topic.

This is an asynchronous, one-to-many communication model. A publisher shouts out data, and any number of subscribers can listen.

**ASCII Diagram: ROS 2 Graph with a Topic**
```
+---------------------+           +----------------------+
|   /camera_driver    |           | /image_processor     |
|      (Publisher)    |---[ /image_raw ]-->|   (Subscriber)     |
+---------------------+           +----------------------+
```

### Services (The Synchronous Handshake)

**Services** are for request-response communication. This is a synchronous, one-to-one model. A **Client** node sends a request to a **Server** node and waits for a response. This is useful for tasks that need to be confirmed, like "did the arm successfully move to the target?"

### Actions (The Long-Term Task Manager)

**Actions** are for long-running, feedback-oriented tasks. Think of navigating to a goal. The process involves:
1. A **Client** sends a goal to an **Action Server**.
2. The **Server** provides continuous feedback (e.g., distance to target).
3. The **Server** sends a final result when the task is complete.
The client can also cancel the goal at any time.

---

## Your First ROS 2 Program: "Hello Universe" with `rclpy`

Let's build a classic publisher/subscriber system using `rclpy`, the Python client library for ROS 2.

### The Publisher Node

This node will publish a "Hello Universe" message to a topic named `/greeting`.

```python title="src/hello_publisher.py"
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class GreetingPublisher(Node):
    def __init__(self):
        super().__init__('greeting_publisher')
        self.publisher_ = self.create_publisher(String, 'greeting', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello Universe: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    greeting_publisher = GreetingPublisher()
    rclpy.spin(greeting_publisher)
    greeting_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### The Subscriber Node

This node will subscribe to the `/greeting` topic and print the messages it receives.

```python title="src/hello_subscriber.py"
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class GreetingSubscriber(Node):
    def __init__(self):
        super().__init__('greeting_subscriber')
        self.subscription = self.create_subscription(
            String,
            'greeting',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    greeting_subscriber = GreetingSubscriber()
    rclpy.spin(greeting_subscriber)
    greeting_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

<Admonition type="warning" icon="🤖" title="Hardware-Specific Note">
  If you are running perception-heavy ROS 2 nodes (e.g., those processing high-resolution camera streams or LiDAR data), your GPU choice matters. For instance, with an **NVIDIA RTX 4070 Ti** or higher, you can leverage the `isaac_ros_image_proc` package for hardware-accelerated image processing, significantly reducing CPU load and message latency. Without a capable GPU, you may need to reduce camera resolution or frame rates to achieve real-time performance.
</Admonition>
