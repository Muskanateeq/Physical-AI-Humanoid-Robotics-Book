---
id: urdf-humanoids
title: 'Chapter 3: Describing Your Robot URDF for Humanoids'
---

import Admonition from '@theme/Admonition';

## What is URDF? The Blueprint of a Robot

The **Unified Robot Description Format (URDF)** is an XML-based file format used in ROS to describe all physical aspects of a robot. It's the robot's digital blueprint, defining its structure, appearance, and kinematics.

For a humanoid robot, the URDF file is the source of truth for:
-   **Kinematic Chains:** The arrangement of arms, legs, and torso.
-   **Visual Appearance:** How the robot looks in simulators (e.g., RViz, Gazebo).
-   **Collision Properties:** The physical shapes used for collision detection.
-   **Inertial Properties:** The mass and inertia of each part, crucial for realistic physics simulation.

---

## The Core Components: Links and Joints

A URDF model is a tree of **links** connected by **joints**.

-   **`<link>`:** A link represents a rigid body part of the robot (e.g., a forearm, a thigh, a foot). It has visual, collision, and inertial properties.
-   **`<joint>`:** A joint connects two links and defines their relative motion. It specifies the axis of rotation or translation and may include limits on motion and dynamics (friction).

**ASCII Diagram: A Simple Arm Structure**
This shows a `shoulder_link` connected to a `torso_link` via a `shoulder_joint`, and an `upper_arm_link` connected to the `shoulder_link` via an `elbow_joint`.

```
[ torso_link ]----(shoulder_joint)----[ shoulder_link ]----(elbow_joint)----[ upper_arm_link ]
   (Parent)           (Child)           (Parent)          (Child)          (Parent)
```

---

## Example: A Simple Two-Link Arm

Let's define a basic arm with a shoulder and an elbow. This URDF describes two links and two revolute (rotating) joints.

```xml title="simple_arm.urdf"
<?xml version="1.0"?>
<robot name="simple_arm">

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </visual>
  </link>

  <!-- Shoulder Link -->
  <link name="shoulder_link">
    <visual>
      <geometry>
        <box size="0.6 0.1 0.1"/>
      </geometry>
      <origin xyz="0.3 0 0" rpy="0 0 0"/>
    </visual>
  </link>

  <!-- Shoulder Joint -->
  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="shoulder_link"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="300" velocity="1.0" lower="-1.57" upper="1.57" />
  </joint>

  <!-- Upper Arm Link -->
  <link name="upper_arm_link">
    <visual>
      <geometry>
        <cylinder length="1.0" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 -0.5" rpy="0 0 0"/>
    </visual>
  </link>

  <!-- Elbow Joint -->
  <joint name="elbow_joint" type="revolute">
    <parent link="shoulder_link"/>
    <child link="upper_arm_link"/>
    <origin xyz="0.6 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="300" velocity="1.0" lower="0" upper="1.57" />
  </joint>

</robot>
```

---

## Visualizing the URDF with RViz2

To check if your URDF is correct, you can visualize it in RViz2. The `robot_state_publisher` node reads the URDF and publishes the robot's transformations (`/tf` topic) based on joint states.

1.  **Launch File:** Create a launch file to start the necessary nodes.

    ```python title="display.launch.py"
    from launch import LaunchDescription
    from launch_ros.actions import Node
    from launch_ros.parameter_descriptions import ParameterValue
    from launch.substitutions import Command
    import os
    from ament_index_python.packages import get_package_share_directory

    def generate_launch_description():
        urdf_path = os.path.join(
            get_package_share_directory('your_package_name'),
            'urdf',
            'simple_arm.urdf'
        )

        robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': ParameterValue(Command(['xacro ', urdf_path]), value_type=str)}]
        )

        joint_state_publisher_gui_node = Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
        )

        rviz2_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        )

        return LaunchDescription([
            robot_state_publisher_node,
            joint_state_publisher_gui_node,
            rviz2_node
        ])
    ```

2.  **Run the Launch File:**
    ```bash
    ros2 launch your_package_name display.launch.py
    ```
    In RViz, add a "RobotModel" display, and you should see your arm. Use the `joint_state_publisher_gui` window to move the joints.

<Admonition type="info" icon="💡" title="Simulation Performance Note">
  The complexity of your URDF directly impacts simulation performance. A humanoid robot can have over 50 links. While visual meshes (`<visual>`) can be high-polygon (e.g., from a CAD model), the collision meshes (`<collision>`) should be simplified primitives (boxes, spheres, cylinders) or low-polygon convex hulls. A high-poly collision model will drastically slow down physics calculations in simulators like Gazebo. On a mainstream CPU like an **Intel Core i7** or **AMD Ryzen 7**, a full humanoid model with complex collision geometry can drop a simulation to well below real-time rates, making controller tuning nearly impossible.
</Admonition>
