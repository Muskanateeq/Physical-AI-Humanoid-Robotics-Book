---
id: urdf-humanoids
title: 'Chapter 3: Understanding URDF for Humanoids'
---

import Admonition from '@theme/Admonition';
import LanguageToggle from '@site/src/components/LanguageToggle/LanguageToggle';

<LanguageToggle />


<div className="english-content">

# Chapter 3: Understanding URDF for Humanoids

This chapter delves into the Unified Robot Description Format (URDF), a fundamental XML format in ROS for describing the kinematic and dynamic properties of a robot. We'll explore its structure, how it's used to define humanoid robots, and how these models are integrated and visualized within the ROS 2 ecosystem.

## 3.1 — Introduction to URDF

### What is URDF?

URDF (Unified Robot Description Format) is an XML file format used in ROS to describe all aspects of a robot. It's a powerful tool for defining the physical structure, kinematics, and dynamics of a robot, enabling various ROS tools to understand and interact with the robot model.

Key aspects of URDF:
- **XML-based:** Easy to read and write.
- **Hierarchical structure:** Defines a tree-like structure of links and joints.
- **Physical properties:** Specifies mass, inertia, and collision properties.
- **Visual properties:** Describes the visual appearance, including mesh files and colors.

### Why is URDF essential for Humanoids?

For humanoid robots, URDF is critical for:
- **Kinematic modeling:** Precisely defines the relationships between body parts (links) and their connections (joints), essential for motion planning and control.
- **Simulation:** Allows robots to be accurately represented and simulated in environments like Gazebo or Isaac Sim.
- **Visualization:** Tools like RViz use URDF to display the robot's structure, sensor data, and planned trajectories.
- **Collision detection:** Defines collision models for safe interaction with the environment and itself.

## 3.2 — URDF Structure: Links and Joints

A URDF file primarily consists of two main elements: `link` and `joint`.

### Links

A `link` represents a rigid body part of the robot. This could be a torso, an arm segment, a hand, or a foot. Each link has:
- **Visual properties:** How it looks (geometry, material, color, texture).
- **Collision properties:** The simplified shape used for collision detection.
- **Inertial properties:** Mass, center of mass, and inertia tensor, crucial for dynamic simulation.

Example of a simple link:
```xml
<link name="base_link">
  <visual>
    <geometry>
      <box size="0.6 0.4 0.2"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 0.8 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <box size="0.6 0.4 0.2"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="10"/>
    <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
  </inertial>
</link>
```

### Joints

A `joint` describes how two links are connected. Joints define the robot's degrees of freedom (DOF). For humanoids, common joint types include:
- **`revolute`:** A rotating joint with a single axis of rotation, like a shoulder or elbow.
- **`continuous`:** Similar to revolute but without joint limits, useful for wheels or continuous rotation.
- **`prismatic`:** A sliding joint along a single axis.
- **`fixed`:** Connects two links rigidly, removing any relative motion, often used for static base links.

Each joint has:
- **Parent and Child links:** Defines which links it connects.
- **Origin:** The transform from the parent link to the joint frame.
- **Axis:** The axis of rotation or translation.
- **Limits:** For revolute and prismatic joints, defines the range of motion.

Example of a simple joint connecting two links:
```xml
<joint name="shoulder_joint" type="revolute">
  <parent link="torso_link"/>
  <child link="upper_arm_link"/>
  <origin xyz="0 0 0.3" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="10"/>
</joint>
```

## 3.3 — Modeling Humanoids with URDF

Designing a URDF for a humanoid robot involves meticulously defining all its limbs, torso, head, and connecting them with appropriate joints.

### Key considerations for Humanoid URDFs:
- **Hierarchical structure:** Typically starts with a `base_link` (often the torso or pelvis) and branches out to arms, legs, and the head.
- **Joint limits:** Crucial for preventing self-collision and ensuring realistic movement.
- **Degrees of Freedom (DOF):** Humanoids typically have many DOFs, requiring careful joint placement and type selection.
- **Self-collision:** URDF can define collision geometries that are simpler than visual meshes to speed up collision checks during simulation and motion planning.

### Example: A Segment of a Humanoid Arm

Consider an arm with a shoulder, upper arm, elbow, and forearm.

```xml
<?xml version="1.0"?>
<robot name="humanoid_arm">

  <link name="torso_link"/>

  <link name="shoulder_link">
    <visual>
      <geometry><sphere radius="0.05"/></geometry>
      <material name="grey"><color rgba="0.5 0.5 0.5 1"/></material>
    </visual>
    <collision>
      <geometry><sphere radius="0.05"/></geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="shoulder_yaw_joint" type="revolute">
    <parent link="torso_link"/>
    <child link="shoulder_link"/>
    <origin xyz="0.1 0 0.5" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="10"/>
  </joint>

  <link name="upper_arm_link">
    <visual>
      <geometry><cylinder radius="0.03" length="0.2"/></geometry>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <material name="grey"/>
    </visual>
    <collision>
      <geometry><cylinder radius="0.03" length="0.2"/></geometry>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="shoulder_pitch_joint" type="revolute">
    <parent link="shoulder_link"/>
    <child link="upper_arm_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="0.5" effort="100" velocity="10"/>
  </joint>

  <link name="forearm_link">
    <visual>
      <geometry><cylinder radius="0.025" length="0.15"/></geometry>
      <origin xyz="0 0 0.075" rpy="0 0 0"/>
      <material name="grey"/>
    </visual>
    <collision>
      <geometry><cylinder radius="0.025" length="0.15"/></geometry>
      <origin xyz="0 0 0.075" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.0005"/>
    </inertial>
  </link>

  <joint name="elbow_joint" type="revolute">
    <parent link="upper_arm_link"/>
    <child link="forearm_link"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.5" upper="0.0" effort="100" velocity="10"/>
  </joint>

</robot>
```

## 3.4 — Visualization with RViz

RViz is a 3D visualization tool for ROS. It uses the URDF model to display the robot's current state, including joint positions, sensor data, and planned paths.

To visualize a URDF in RViz:
1. **Launch `robot_state_publisher`:** This ROS 2 node reads the URDF and publishes the robot's joint states as transformations.
2. **Launch `joint_state_publisher` (optional for manual control):** Publishes mock joint states for a static URDF, allowing you to manipulate the robot's joints manually.
3. **Open RViz:** Add a "RobotModel" display and configure it to subscribe to the `/robot_description` topic (which contains the URDF XML) and `/joint_states` topic.

<Admonition type="tip" title="Practical Tip">
  Always test your URDF in RViz to ensure all links and joints are correctly placed and articulated before moving to complex simulations or hardware integration.
</Admonition>

## 3.5 — Integrating URDF with ROS 2

URDF models are typically loaded and managed in ROS 2 through launch files and specific nodes.

- **`robot_description` parameter:** The entire URDF XML content is usually loaded into a ROS 2 parameter server under the name `robot_description`. This allows all ROS 2 nodes that need the robot's description to access it.
- **Launch files:** ROS 2 launch files are used to start the `robot_state_publisher` and `joint_state_publisher` nodes, and to load the URDF from an XML file.

Example of a simple launch file to display a URDF:
```xml
<launch>
  <arg name="model" default="$(find xacro_your_robot_description)/urdf/your_robot.urdf"/>
  <param name="robot_description" command="$(find xacro)/xacro --inorder $(arg model)"/>

  <node pkg="robot_state_publisher" exec="robot_state_publisher" name="robot_state_publisher">
    <param name="robot_description" value="$(param robot_description)"/>
  </node>

  <node pkg="joint_state_publisher_gui" exec="joint_state_publisher_gui" name="joint_state_publisher_gui"/>

  <node pkg="rviz2" exec="rviz2" name="rviz2" args="-d $(find xacro_your_robot_description)/rviz/urdf.rviz"/>
</launch>
```
<Admonition type="info" title="XACRO">
  For more complex robots, especially humanoids, XACRO (XML Macros for ROS) is often used. XACRO allows for more modular and readable URDF files by using macros, conditionals, and mathematical functions to generate the final URDF XML.
</Admonition>

Understanding URDF is foundational for working with humanoid robots in ROS 2, providing the blueprint for their physical and functional characteristics across simulation, visualization, and real-world deployment.

---

## 3.6 — Mini Project (Module 1)

**Goal:** Build and run a simple simulated humanoid that can move the arms and head. Control it via a Python agent node that sends high-level commands; the commands get executed by ROS action servers/trajectory controllers.

This mini-project ties together installation, `rclpy` code, URDF, RViz, and the agent bridge.

### Project structure
```
module1-mini/
├─ ros_ws/
│  ├─ src/
│  │  ├─ humanoid_description/   # URDF xacro files & meshes
│  │  ├─ humanoid_control/       # controllers, launch files
│  │  └─ humanoid_agent/         # agent node (rclpy) that sends commands
│  ├─ install/
│  └─ build/
├─ README.md
└─ launch/
```

### Step-by-step: implement and run

#### Step 1 — Clone scaffold
Create package structure in `~/ros_ws/src/` and commit to Git.

#### Step 2 — URDF
Place `humanoid.urdf.xacro` and parts under `humanoid_description`.

Use simple primitive shapes for visuals and collisions.

#### Step 3 — Controllers
Use `ros2_controllers` stack (`joint_state_controller`, `joint_trajectory_controller`).

Example controller config YAML (`controllers.yaml`):

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100

joint_state_broadcaster:
  ros__parameters:
    type: joint_state_broadcaster/JointStateBroadcaster

arm_controller:
  ros__parameters:
    type: joint_trajectory_controller/JointTrajectoryController
    joints:
      - left_shoulder_pitch
      - left_shoulder_roll
      - left_elbow
    gains:
      left_shoulder_pitch: {p: 100.0, d: 10.0}
      ...
```

Launch controller manager, load controllers, and start them via launch files.

#### Step 4 — Agent node (simple)
File: `agent_interface.py`

```python
import rclpy
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.action import ActionClient

class SimpleAgent(Node):
    def __init__(self):
        super().__init__('simple_agent')
        self._client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')

    def send_joint_goal(self, joints, positions, duration=2.0):
        goal_msg = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = joints
        p = JointTrajectoryPoint()
        p.positions = positions
        p.time_from_start.sec = int(duration)
        traj.points.append(p)
        goal_msg.trajectory = traj

        self._client.wait_for_server()
        send_goal_future = self._client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return None
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().result
```

#### Step 5 — Launch everything
Create a `launch.py` that starts:
- `robot_state_publisher` (publishes URDF),
- `joint_state_broadcaster`,
- `arm_controller`,
- `agent_interface` (optional, can run separately).

Run:
```bash
# Build
cd ~/ros_ws
colcon build --packages-select humanoid_description humanoid_control humanoid_agent
source install/setup.bash

# Launch
ros2 launch humanoid_control demo.launch.py
```

#### Step 6 — Use agent to move joints
Open a Python REPL to call `send_joint_goal` from `agent_interface` or run a demo script `move_arm_demo.py` to perform a preset motion.

#### Step 7 — Validate in RViz
Confirm the robot model appears and the arm moves to the commanded positions.
Observe topic `/joint_states` and controller feedback topics for errors.

## Tests & Evaluation

- **Functional test:** The arm should reach commanded joint positions within tolerance (±2°).
- **Timing test:** Controller responds to a goal within expected latency.
- **Safety test:** Send abort/cancel and ensure controller stops (test preemption).

## Troubleshooting tips

- **If joints don't move:** ensure controllers are active, the right joint names match URDF, and the `robot_state_publisher` is supplying TF.
- **If trajectory is rejected:** inspect goal joint names and controller gain values.
- **For physics mismatch in Gazebo:** tune mass/inertia or collision geometry.

## Appendix: Useful scripts & launch snippets

Example launch snippet (Python launch file)
```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode

def generate_launch_description():
    ld = LaunchDescription()

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': '<your xacro processed string maybe via xacro command>'}]
    )

    ld.add_action(robot_state_pub)
    # Add controller manager node and others similarly

    return ld
```

### Process for building and iterating
1. Modify URDF xacro → regenerate `robot_description`.
2. `colcon build --packages-select ...`
3. Source the workspace: `source install/setup.bash`
4. `ros2 launch ...`

## Best practices & production notes

- **Modular nodes:** keep AI logic separate from actuator drivers to allow simulation/hardware switching.
- **Typed plans:** use typed, validated messages or action goals; never send free-form text to controllers.
- **QoS tuning:** tune QoS profiles: `sensor_data` for camera and LIDAR; `reliable` for important commands.
- **Security:** if running across networks, configure ROS 2 security (SROS2 / DDS security) to avoid unauthorized control.
- **Logging & replay:** use `ros2 bag` to record experiments and replay for debugging.

## Checklist before moving to Module 2 (simulation & digital twin)

- [ ] ROS 2 core installed and `ros2` CLI works.
- [ ] `ros_ws` workspace builds and packages run.
- [ ] `talker` / `listener` demo successful.
- [ ] URDF loads in `robot_state_publisher` and RViz.
- [ ] Controllers load and can accept `FollowJointTrajectory` goals.
- [ ] Agent node can submit goals and receive feedback.
- [ ] Safety: ensure you can stop controllers and there is an emergency stop path.

</div>

<div className="urdu-content">


# باب 3: ہیومنوائڈز کے لیے URDF کو سمجھنا

یہ باب یونیفائیڈ روبوٹ ڈیسکرپشن فارمیٹ (URDF) میں گہرائی سے جاتا ہے، جو ROS میں روبوٹ کے کنیمیٹک اور ڈائنامک خصوصیات کی وضاحت کے لیے ایک بنیادی XML فارمیٹ ہے۔ ہم اس کی ساخت، اس کے استعمال کو ہیومنوائڈ روبوٹس کی وضاحت کرنے کے لیے، اور یہ کہ یہ ماڈلز ROS 2 ایکو سسٹم کے اندر کیسے انضمام اور ویژولائز کیے جاتے ہیں، کا جائزہ لیں گے۔

## 3.1 — URDF کا تعارف

### URDF کیا ہے؟

URDF (یونیفائیڈ روبوٹ ڈیسکرپشن فارمیٹ) ROS میں روبوٹ کے تمام پہلوؤں کو بیان کرنے کے لیے استعمال ہونے والا ایک XML فائل فارمیٹ ہے۔ یہ روبوٹ کی جسمانی ساخت، کنیمیٹکس، اور ڈائنامکس کی وضاحت کے لیے ایک طاقتور ٹول ہے، جو مختلف ROS ٹولز کو روبوٹ ماڈل کو سمجھنے اور اس کے ساتھ بات چیت کرنے کی اجازت دیتا ہے۔

URDF کے کلیدی پہلو:

- **XML-مبنی:** پڑھنے اور لکھنے میں آسان۔
- **سلسلہ ساخت:** ایک درخت نما ساخت کی وضاحت کرتا ہے جس میں لنکس اور جوائنٹس ہوتے ہیں۔
- **جسمانی خصوصیات:** ماس، انیشیا، اور کالیژن کی خصوصیات کی وضاحت کرتا ہے۔
- **بصری خصوصیات:** ویژول ایپیرنس کی وضاحت کرتا ہے، میش فائلز اور رنگوں سمیت۔

### ہیومنوائڈز کے لیے URDF کیوں ضروری ہے؟

ہیومنوائڈ روبوٹس کے لیے، URDF کے لیے اہم ہے:

- **کنیمیٹک ماڈلنگ:** جسم کے حصوں (لنکس) اور ان کے کنیکشنز (جوائنٹس) کے درمیان تعلقات کی درست وضاحت کرتا ہے، جو موشن منصوبہ بندی اور کنٹرول کے لیے ضروری ہے۔
- **سیمولیشن:** روبوٹس کو Gazebo یا Isaac Sim جیسے ماحول میں درست طریقے سے نمائندگی اور سیمولیٹ کرنے کی اجازت دیتا ہے۔
- **ویژولائزیشن:** RViz جیسے ٹولز URDF کا استعمال روبوٹ کی ساخت، سینسر ڈیٹا، اور منصوبہ بند کردہ ٹریجکٹریز کو دکھانے کے لیے کرتے ہیں۔
- **کالیژن ڈیٹیکشن:** ماحول اور خود کے ساتھ محفوظ تعامل کے لیے کالیژن ماڈلز کی وضاحت کرتا ہے۔

## 3.2 — URDF ساخت: لنکس اور جوائنٹس

ایک URDF فائل بنیادی طور پر دو اہم عناصر پر مشتمل ہوتی ہے: `link` اور `joint`۔

### لنکس

ایک `link` روبوٹ کے ایک سخت جسم کے حصے کی نمائندگی کرتا ہے۔ یہ ایک ٹورسو، ایک بازو کا حصہ، ایک ہاتھ، یا ایک پاؤں ہو سکتا ہے۔ ہر لنک کے پاس:

- **بصری خصوصیات:** یہ کیسے نظر آتا ہے (جیومیٹری، میٹریل، رنگ، ٹیکسچر)۔
- **کالیژن کی خصوصیات:** کالیژن ڈیٹیکشن کے لیے استعمال ہونے والی سادہ شکل۔
- **انیشیل کی خصوصیات:** ماس، مرکزِ ماس، اور انیشیا ٹینسر، ڈائنامک سیمولیشن کے لیے اہم۔

ایک سادہ لنک کی مثال:

```xml
<link name="base_link">
  <visual>
    <geometry>
      <box size="0.6 0.4 0.2"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 0.8 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <box size="0.6 0.4 0.2"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="10"/>
    <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
  </inertial>
</link>
```

### جوائنٹس

ایک `joint` بیان کرتا ہے کہ دو لنکس کیسے جڑے ہوئے ہیں۔ جوائنٹس روبوٹ کے ڈیگریز آف فریڈم (DOF) کی وضاحت کرتے ہیں۔ ہیومنوائڈز کے لیے، عام جوائنٹس کی اقسام میں شامل ہیں:

- **`revolute`:** ایک گھومنے والا جوائنٹ ایک گھوماؤ کے محور کے ساتھ، جیسے کندھا یا کوہنی۔
- **`continuous`:** revolute کی طرح لیکن بغیر جوائنٹ حدود کے، پہیوں یا مسلسل گھوماؤ کے لیے مفید۔
- **`prismatic`:** ایک اکسل پر سلائیڈنگ جوائنٹ۔
- **`fixed`:** دو لنکس کو سختی سے جوڑتا ہے، کوئی رشتہ دار حرکت کو ختم کر دیتا ہے، اکثر سٹیٹک بیس لنکس کے لیے استعمال ہوتا ہے۔

ہر جوائنٹ کے پاس:

- **پیرنٹ اور چائلڈ لنکس:** وضاحت کرتا ہے کہ یہ کون سے لنکس کو جوڑتا ہے۔
- **اوریجن:** والد لنک سے جوائنٹ فریم تک ٹرانسفارم۔
- **ایکسیس:** گھوماؤ یا ٹرانسلیشن کا محور۔
- **لیمٹس:** revolute اور prismatic جوائنٹس کے لیے، حرکت کی حد کی وضاحت کرتا ہے۔

دو لنکس کو جوڑنے والے ایک سادہ جوائنٹ کی مثال:

```xml
<joint name="shoulder_joint" type="revolute">
  <parent link="torso_link"/>
  <child link="upper_arm_link"/>
  <origin xyz="0 0 0.3" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="10"/>
</joint>
```

## 3.3 — URDF کے ساتھ ہیومنوائڈز کی ماڈلنگ

ہیومنوائڈ روبوٹ کے URDF کو ڈیزائن کرنا تمام اس کے اعضا، ٹورسو، سر، اور انہیں مناسب جوائنٹس کے ساتھ جوڑنے کی مکمل وضاحت کا متقاضی ہے۔

### ہیومنوائڈ URDFs کے لیے کلیدی امور:

- **سلسلہ ساخت:** عام طور پر ایک `base_link` (اکثر ٹورسو یا pelvis) سے شروع ہوتا ہے اور بازوؤں، ٹانگوں، اور سر کی طرف پھیل جاتا ہے۔
- **جوائنٹ لیمٹس:** خود کالیژن سے بچنے اور حقیقی حرکت کو یقینی بنانے کے لیے اہم ہے۔
- **ڈیگریز آف فریڈم (DOF):** ہیومنوائڈز کے پاس عام طور پر بہت DOFs ہوتے ہیں، جس کے لیے جوائنٹ کی جگہ اور قسم کے انتخاب کی احتیاط کی ضرورت ہوتی ہے۔
- **خود کالیژن:** URDF کالیژن جیومیٹریز کی وضاحت کر سکتا ہے جو وژول میشس سے سادہ ہوں تاکہ سیمولیشن اور موشن منصوبہ بندی کے دوران کالیژن چیکس کو تیز کیا جا سکے۔

### مثال: ایک ہیومنوائڈ بازو کا حصہ

ایک بازو کا تصور کریں جس میں کندھا، اوپری بازو، کوہنی، اور کلائی ہو۔

```xml
<?xml version="1.0"?>
<robot name="humanoid_arm">

  <link name="torso_link"/>

  <link name="shoulder_link">
    <visual>
      <geometry><sphere radius="0.05"/></geometry>
      <material name="grey"><color rgba="0.5 0.5 0.5 1"/></material>
    </visual>
    <collision>
      <geometry><sphere radius="0.05"/></geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="shoulder_yaw_joint" type="revolute">
    <parent link="torso_link"/>
    <child link="shoulder_link"/>
    <origin xyz="0.1 0 0.5" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="10"/>
  </joint>

  <link name="upper_arm_link">
    <visual>
      <geometry><cylinder radius="0.03" length="0.2"/></geometry>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <material name="grey"/>
    </visual>
    <collision>
      <geometry><cylinder radius="0.03" length="0.2"/></geometry>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="shoulder_pitch_joint" type="revolute">
    <parent link="shoulder_link"/>
    <child link="upper_arm_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="0.5" effort="100" velocity="10"/>
  </joint>

  <link name="forearm_link">
    <visual>
      <geometry><cylinder radius="0.025" length="0.15"/></geometry>
      <origin xyz="0 0 0.075" rpy="0 0 0"/>
      <material name="grey"/>
    </visual>
    <collision>
      <geometry><cylinder radius="0.025" length="0.15"/></geometry>
      <origin xyz="0 0 0.075" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.0005"/>
    </inertial>
  </link>

  <joint name="elbow_joint" type="revolute">
    <parent link="upper_arm_link"/>
    <child link="forearm_link"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.5" upper="0.0" effort="100" velocity="10"/>
  </joint>

</robot>
```

## 3.4 — RViz کے ساتھ ویژولائزیشن

RViz ROS کے لیے ایک 3D ویژولائزیشن ٹول ہے۔ یہ روبوٹ کی موجودہ حالت کو دکھانے کے لیے URDF ماڈل کا استعمال کرتا ہے، جس میں جوائنٹ کی پوزیشنز، سینسر ڈیٹا، اور منصوبہ بند کردہ راستے شامل ہیں۔

URDF کو RViz میں دیکھنے کے لیے:

1.  **`robot_state_publisher` لانچ کریں:** یہ ROS 2 نوڈ URDF کو پڑھتا ہے اور روبوٹ کی جوائنٹ اسٹیٹس کو ٹرانسفارمیشنز کے طور پر پبلش کرتا ہے۔
2.  **`joint_state_publisher` لانچ کریں (دستی کنٹرول کے لیے اختیاری):** ایک سٹیٹک URDF کے لیے نقلی جوائنٹ اسٹیٹس پبلش کرتا ہے، جو آپ کو روبوٹ کے جوائنٹس کو دستی طور پر مینیولیٹ کرنے کی اجازت دیتا ہے۔
3.  **RViz کھولیں:** ایک "RobotModel" ڈسپلے شامل کریں اور اسے `/robot_description` ٹاپک (جس میں URDF XML ہے) اور `/joint_states` ٹاپک کو سبسکرائب کرنے کے لیے کنفیگر کریں۔

<Admonition type="tip" title="عملی مشورہ">
  ہمیشہ اپنے URDF کو RViz میں ٹیسٹ کریں تاکہ یقینی بنایا جا سکے کہ تمام لنکس اور جوائنٹس درست طریقے سے جگہ اور مربوط ہیں پیچیدہ سیمولیشن یا ہارڈویئر انضمام سے پہلے۔
</Admonition>

## 3.5 — ROS 2 کے ساتھ URDF کا انضمام

URDF ماڈلز عام طور پر ROS 2 میں لانچ فائلوں اور مخصوص نوڈز کے ذریعے لوڈ اور منیج کیے جاتے ہیں۔

- **`robot_description` پیرامیٹر:** پورے URDF XML مواد کو عام طور پر ایک ROS 2 پیرامیٹر سرور میں `robot_description` کے نام سے لوڈ کیا جاتا ہے۔ یہ تمام ROS 2 نوڈز کو اس کی تفصیل تک رسائی دینے کی اجازت دیتا ہے جنہیں روبوٹ کی تفصیل کی ضرورت ہوتی ہے۔
- **لانچ فائلیں:** ROS 2 لانچ فائلز `robot_state_publisher` اور `joint_state_publisher` نوڈز شروع کرنے اور URDF کو XML فائل سے لوڈ کرنے کے لیے استعمال کی جاتی ہیں۔

URDF کو دکھانے کے لیے ایک سادہ لانچ فائل کی مثال:

```xml
<launch>
  <arg name="model" default="$(find xacro_your_robot_description)/urdf/your_robot.urdf"/>
  <param name="robot_description" command="$(find xacro)/xacro --inorder $(arg model)"/>

  <node pkg="robot_state_publisher" exec="robot_state_publisher" name="robot_state_publisher">
    <param name="robot_description" value="$(param robot_description)"/>
  </node>

  <node pkg="joint_state_publisher_gui" exec="joint_state_publisher_gui" name="joint_state_publisher_gui"/>

  <node pkg="rviz2" exec="rviz2" name="rviz2" args="-d $(find xacro_your_robot_description)/rviz/urdf.rviz"/>
</launch>
```

<Admonition type="info" title="XACRO">
  مزید پیچیدہ روبوٹس کے لیے، خاص طور پر ہیومنوائڈز کے لیے، XACRO (XML میکروز فار ROS) کا اکثر استعمال کیا جاتا ہے۔ XACRO URDF فائلز کو ماڈیولر اور پڑھنے کے قابل بناتا ہے جو میکروز، شرائط، اور ریاضیاتی فنکشنز کا استعمال کرتے ہوئے حتمی URDF XML تیار کرتے ہیں۔
</Admonition>

URDF کو سمجھنا ROS 2 میں ہیومنوائڈ روبوٹس کے ساتھ کام کرنے کے لیے بنیاد ہے، جو سیمولیشن، ویژولائزیشن، اور حقیقی دنیا کے ا deployments میں ان کی جسمانی اور فنکشنل خصوصیات کے لیے بلیو پرنٹ فراہم کرتا ہے۔

---

## 3.6 — منی پروجیکٹ (ماڈیول 1)

**ہدف:** ایک سادہ سیمولیٹڈ ہیومنوائڈ تیار کریں اور چلائیں جو بازو اور سر کو حرکت دے سکے۔ اسے ایک Python ایجنٹ نوڈ کے ذریعے کنٹرول کریں جو ہائی لیول کمانڈز بھیجتا ہے؛ کمانڈز ROS ایکشن سرورز/ٹریجکٹری کنٹرولرز کے ذریعے انجام پاتی ہیں۔

یہ منی پروجیکٹ انسٹالیشن، `rclpy` کوڈ، URDF، RViz، اور ایجنٹ برج کو ایک ساتھ جوڑتا ہے۔

### پروجیکٹ کی ساخت
```
module1-mini/
├─ ros_ws/
│  ├─ src/
│  │  ├─ humanoid_description/   # URDF xacro فائلز اور میشس
│  │  ├─ humanoid_control/       # کنٹرولرز، لانچ فائلز
│  │  └─ humanoid_agent/         # ایجنٹ نوڈ (rclpy) جو کمانڈز بھیجتا ہے
│  ├─ install/
│  └─ build/
├─ README.md
└─ launch/
```

### مرحلہ وار: نافذ کریں اور چلائیں

#### مرحلہ 1 — سکافولڈ کلون کریں
`~/ros_ws/src/` میں پیکج ساخت بنائیں اور Git میں کمیٹ کریں۔

#### مرحلہ 2 — URDF
`humanoid.urdf.xacro` اور اجزاء کو `humanoid_description` کے تحت رکھیں۔

بصریات اور کالیژن کے لیے سادہ ابتدائی شکلیں استعمال کریں۔

#### مرحلہ 3 — کنٹرولرز
`ros2_controllers` اسٹیک استعمال کریں (`joint_state_controller`, `joint_trajectory_controller`)۔

ایک مثالی کنٹرولر کنفیگ YAML (`controllers.yaml`):

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100

joint_state_broadcaster:
  ros__parameters:
    type: joint_state_broadcaster/JointStateBroadcaster

arm_controller:
  ros__parameters:
    type: joint_trajectory_controller/JointTrajectoryController
    joints:
      - left_shoulder_pitch
      - left_shoulder_roll
      - left_elbow
    gains:
      left_shoulder_pitch: {p: 100.0, d: 10.0}
      ...
```

کنٹرولر مینیجر لانچ کریں، کنٹرولرز لوڈ کریں، اور لانچ فائلز کے ذریعے انہیں شروع کریں۔

#### مرحلہ 4 — ایجنٹ نوڈ (سادہ)
فائل: `agent_interface.py`

```python
import rclpy
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.action import ActionClient

class SimpleAgent(Node):
    def __init__(self):
        super().__init__('simple_agent')
        self._client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')

    def send_joint_goal(self, joints, positions, duration=2.0):
        goal_msg = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = joints
        p = JointTrajectoryPoint()
        p.positions = positions
        p.time_from_start.sec = int(duration)
        traj.points.append(p)
        goal_msg.trajectory = traj

        self._client.wait_for_server()
        send_goal_future = self._client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return None
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().result
```

#### مرحلہ 5 — سب کچھ لانچ کریں
ایک `launch.py` بنائیں جو شروع کرے:
- `robot_state_publisher` (URDF پبلش کرتا ہے)،
- `joint_state_broadcaster`,
- `arm_controller`,
- `agent_interface` (اختیاری، الگ سے چلایا جا سکتا ہے)۔

چلائیں:
```bash
# بنائیں
cd ~/ros_ws
colcon build --packages-select humanoid_description humanoid_control humanoid_agent
source install/setup.bash

# لانچ کریں
ros2 launch humanoid_control demo.launch.py
```

#### مرحلہ 6 — جوائنٹس کو حرکت دینے کے لیے ایجنٹ کا استعمال کریں
ایک Python REPL کھولیں تاکہ `agent_interface` سے `send_joint_goal` کو کال کیا جا سکے یا ایک ڈیمو اسکرپٹ `move_arm_demo.py` چلائیں تاکہ ایک مقررہ حرکت انجام دی جا سکے۔

#### مرحلہ 7 — RViz میں تصدیق کریں
یقین کریں کہ روبوٹ ماڈل ظاہر ہوتا ہے اور بازو کمانڈ کردہ پوزیشنز پر جاتا ہے۔
ٹاپک `/joint_states` اور کنٹرولر فیڈ بیک ٹاپکس کو خامیوں کے لیے دیکھیں۔

## ٹیسٹس اور جائزہ

- **فنکشنل ٹیسٹ:** بازو کمانڈ کردہ جوائنٹ پوزیشنز کے اندر برداشت (±2°) تک پہنچنا چاہیے۔
- **ٹائمینگ ٹیسٹ:** کنٹرولر ایک گوئل کے جواب میں متوقع تاخیر کے اندر جواب دیتا ہے۔
- **سیفٹی ٹیسٹ:** ایبورٹ/کینسل بھیجیں اور یقین کریں کہ کنٹرولر رک جاتا ہے (پریمپشن ٹیسٹ کریں)۔

## ٹربولشونگ کے مشورے

- **اگر جوائنٹس حرکت نہیں کرتے:** یقین کریں کہ کنٹرولرز فعال ہیں، صحیح جوائنٹ نام URDF سے مماثل ہیں، اور `robot_state_publisher` TF فراہم کر رہا ہے۔
- **اگر ٹریجکٹری مسترد کر دی جاتی ہے:** گوئل جوائنٹ نامز اور کنٹرولر گین ویلیوز کو دیکھیں۔
- **Gazebo میں فزکس میچ کے لیے:** ماس/انیشیا یا کالیژن جیومیٹری ٹیون کریں۔

## ایپنڈکس: مفید اسکرپٹس اور لانچ اسنوپٹس

ایک مثالی لانچ اسنوپٹ (Python لانچ فائل)
```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode

def generate_launch_description():
    ld = LaunchDescription()

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': '<آپ کا xacro پروسیسڈ سٹرنگ شاید xacro کمانڈ کے ذریعے>'}]
    )

    ld.add_action(robot_state_pub)
    # کنٹرولر مینیجر نوڈ اور دیگر کو اسی طرح شامل کریں

    return ld
```

### بنانے اور دہرانے کا عمل
1. URDF xacro میں ترمیم → `robot_description` دوبارہ تیار کریں۔
2. `colcon build --packages-select ...`
3. ورک اسپیس کو سورس کریں: `source install/setup.bash`
4. `ros2 launch ...`

## بہترین مشقیں اور پروڈکشن نوٹس

- **ماڈیولر نوڈز:** AI منطق کو ایکٹوایٹر ڈرائیورز سے الگ رکھیں تاکہ سیمولیشن/ہارڈویئر سوئچنگ کی اجازت مل سکے۔
- **ٹائپڈ پلانز:** ٹائپڈ، والیڈیٹڈ میسجس یا ایکشن گوئلز استعمال کریں؛ کبھی فری فارم ٹیکسٹ کو کنٹرولرز کو نہ بھیجیں۔
- **QoS ٹیوننگ:** QoS پروفائلز ٹیون کریں: کیمرہ اور LIDAR کے لیے `sensor_data`؛ اہم کمانڈز کے لیے `reliable`۔
- **سیکورٹی:** اگر نیٹ ورکس کے مابین چلا رہے ہیں، تو ROS 2 سیکورٹی (SROS2 / DDS سیکورٹی) کنفیگر کریں تاکہ غیر مجاز کنٹرول سے بچا جا سکے۔
- **لاگنگ اور ری پلے:** `ros2 bag` کا استعمال تجربات ریکارڈ کرنے اور ڈیبگنگ کے لیے دوبارہ چلانے کے لیے کریں۔

## ماڈیول 2 (سیمولیشن اور ڈیجیٹل ٹوئن) پر جانے سے پہلے چیک لسٹ

- [ ] ROS 2 کور انسٹال ہے اور `ros2` CLI کام کرتا ہے۔
- [ ] `ros_ws` ورک اسپیس بن رہا ہے اور پیکجز چل رہے ہیں۔
- [ ] `talker` / `listener` ڈیمو کامیاب ہوا۔
- [ ] URDF `robot_state_publisher` میں لوڈ ہوتا ہے اور RViz میں۔
- [ ] کنٹرولرز لوڈ ہوتے ہیں اور `FollowJointTrajectory` گوئلز قبول کر سکتے ہیں۔
- [ ] ایجنٹ نوڈ گوئلز جمع کر سکتا ہے اور فیڈ بیک حاصل کر سکتا ہے۔
- [ ] سیفٹی: یقین کریں کہ آپ کنٹرولرز کو روک سکتے ہیں اور ایمرجنسی سٹاپ کا راستہ ہے۔

<Admonition type="note">
  rclpy آپ کو Python میں ROS 2 نوڈز لکھنے کی اجازت دیتا ہے، جو AI ایجنٹس کے لیے بہت اہم ہے کیونکہ زیادہ تر AI ٹولز Python میں لکھے جاتے ہیں۔
</Admonition>

</div>
