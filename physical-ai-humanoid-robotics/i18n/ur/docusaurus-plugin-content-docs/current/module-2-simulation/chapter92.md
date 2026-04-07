---
id: Unity-and-mini-project
title: 'Chapter 10: Unity for Robotics and Integration'
---

import LanguageToggle from '@site/src/components/LanguageToggle/LanguageToggle';

<LanguageToggle />
import Admonition from '@theme/Admonition';


<div className="english-content">

# Chapter 10: Unity for Robotics and Integration

Having explored the robust physics and robot simulation capabilities of Gazebo, we now turn our attention to Unity, a powerful game engine that offers unparalleled visual fidelity and interactive experiences. This chapter guides you through integrating Unity into your robotics workflow, focusing on its strengths for high-fidelity human-robot interaction and seamless communication with the ROS 2 ecosystem. Finally, we'll culminate our Module 2 journey with a mini-project that synthesizes all the concepts learned.

## 10.1 — Introduction to Unity for Robotics

While Gazebo excels at physics-accurate simulations and provides deep integration with ROS 2 for backend robot control, Unity shines in its ability to create visually stunning, highly interactive, and immersive environments. Its roots as a game engine make it a formidable tool for frontend applications in robotics, particularly where rich visualization, advanced human-robot interaction, or VR/AR experiences are crucial.

### Unity for Robotics

Unity is a cross-platform game engine developed by Unity Technologies, widely used for developing video games for PC, consoles, mobile devices, and websites. Its extensive feature set, intuitive editor, and large asset store make it increasingly popular in robotics for applications such as:

*   **Avatars:** Creating realistic human and robot avatars for advanced human-robot interaction studies.
*   **Human Simulation:** Simulating human behavior and presence within robotic environments.
*   **VR/AR Applications:** Developing virtual reality and augmented reality interfaces for robot teleoperation, training, and data visualization.
*   **Realistic Environments:** Building highly detailed and visually rich environments for simulating complex scenarios, especially where visual perception is key.
*   **Digital Twin Visualization:** Serving as a visually enhanced frontend for Digital Twins, rendering real-time robot data from a backend simulation (like Gazebo) or a physical robot.

### Getting Started with Unity

To begin, you'll need to install Unity.

**Install Unity Hub:**
The Unity Hub is a management tool that helps you manage your Unity projects and installations of the Unity Editor. Download it from the official Unity website.

**Install Unity Editor (LTS):**
Within the Unity Hub, install a **Long Term Support (LTS)** version of the Unity Editor. LTS versions are recommended for stability and long-term project support.

### Key Unity Concepts for Robotics

As you start working with Unity, you'll encounter some core concepts that are fundamental to its architecture:

*   **Scenes (Environments):** A Scene in Unity is where you design and build your game or simulation environment. It's essentially a container for your assets and GameObjects, defining the layout of your virtual world. In robotics, a scene might represent a factory floor, a home environment, or an outdoor landscape.
*   **GameObjects (Robots, Objects, Cameras):** GameObjects are the fundamental objects in Unity that represent characters, props, lights, cameras, or anything else you might see in your scene. In robotics, your robot model, environmental objects, sensors (virtual cameras in Unity), and user interface elements will all be GameObjects.
*   **Components (Sensors, Controllers, Behaviors):** Components are the functional pieces that you attach to GameObjects to give them capabilities. A GameObject is just an empty container; components define its behavior. Examples in robotics include:
    *   **Mesh Renderer Component:** Makes a 3D model visible.
    *   **Collider Component:** Enables physical interactions and collision detection.
    *   **Rigidbody Component:** Applies physics (gravity, forces) to a GameObject.
    *   **Script Components:** Custom C# scripts you write to define complex robot behaviors, sensor logic, or control interfaces.

By combining these elements, Unity provides a flexible and powerful platform for creating sophisticated robotic simulations and interactive interfaces.


## 10.2 — High-Fidelity Human-Robot Interaction

Unity's capabilities extend significantly into the realm of Human-Robot Interaction (HRI), offering tools to create rich and believable interactions that are crucial for social robots, collaborative robotics, and intuitive control systems.

### Enhancing Interaction with Unity

In Unity, you can add layers of realism and expressiveness to human-robot interactions:

*   **Human Avatars:** Import and animate realistic human character models. These avatars can serve as stand-ins for human users, enabling the simulation of human presence and interaction within a robot's operational space.
*   **NPCs (Non-Player Characters):** Create AI-driven virtual agents that can simulate specific human behaviors, reactions, and movements. This allows for testing how robots interact with dynamic human environments.
*   **AI-based Expressions:** Implement systems for avatars (both human and robot) to display facial expressions or body language that convey emotion or intent, making interactions more natural and understandable.
*   **Gesture Control:** Leverage Unity's input systems and potentially external devices (e.g., Leap Motion, VR controllers) to interpret human gestures. These gestures can then be translated into commands for the robot, allowing for intuitive, non-verbal communication.

### Simulating Complex Interactions

Unity becomes particularly powerful for simulating nuanced HRI scenarios:

*   **✅ Handshake:** Design and animate a robot's response to a human handshake, incorporating subtle movements and force feedback (if combined with a physics engine or haptic device).
*   **✅ Conversation:** Integrate speech recognition and synthesis with character animations to simulate dialogue between a human and a robot.
*   **✅ Following:** Program a robot to follow a human avatar, reacting to their movements and maintaining a safe distance.
*   **✅ Commands:** Develop natural language understanding (NLU) interfaces where spoken or gestural commands from a human avatar are processed by the robot.

These capabilities are great for social robotics, where the robot's ability to understand and appropriately respond to human cues is paramount. Unity provides the visual and interactive canvas to develop and refine these complex interactions.


## 10.3 — Connecting Unity with ROS 2

To truly integrate Unity into a robotics workflow, it must be able to communicate with the core robot intelligence typically managed by ROS 2. The `ROS-TCP-Connector` is the standard tool for bridging this gap.

### The ROS-TCP-Connector

The `ROS-TCP-Connector` is a Unity package developed by Unity Technologies in collaboration with the ROS community. It enables Unity applications to act as ROS 2 nodes, allowing them to publish messages, subscribe to topics, and interact with services and actions on a ROS 2 network.

**Install the ROS-TCP-Connector:**
You'll typically clone the repository into your Unity project's `Packages` folder or import it via Unity's Package Manager.

```bash
git clone https://github.com/Unity-Technologies/ROS-TCP-Connector
```
Follow the installation instructions provided in the repository to ensure proper setup within your Unity project. This usually involves installing the core package and then generating C# message classes from your ROS 2 `.msg` and `.srv` files.

### Why Connect Unity to ROS 2?

This connection allows Unity to become a powerful frontend or a rich simulation environment for your ROS 2-powered robots:

*   **Control Robot in Unity:** Develop intuitive UI elements or interactive scenarios within Unity that publish control commands (e.g., velocity, joint positions, high-level task goals) to ROS 2 topics. These commands are then executed by your robot (physical or Gazebo-simulated).
*   **See Sensor Output:** Subscribe to sensor data (LiDAR scans, camera images, IMU data) published by ROS 2 from your robot. Unity can then visualize this data in a highly graphical and informative way, overlaying it on the 3D scene.
*   **Visualize Movement and State:** Display the robot's real-time pose, joint states, and planned paths within Unity. This creates a visually accurate Digital Twin of your robot, providing valuable insight into its operation.

Industry companies like **Tesla Bot, Meta, and Amazon Robotics** are actively using similar integrations to develop and test their next-generation robotic systems, leveraging Unity's visual prowess with ROS 2's robust robotics framework.


## 10.4 — MINI PROJECT (MODULE 2): Digital Room and Humanoid Navigation

This mini-project serves as the culmination of everything we've learned in Module 2. It integrates the concepts of Digital Twins, Gazebo for physics simulation, robot modeling, sensor integration, and basic navigation principles.

**Goal:** Create a Digital Twin simulation room with a humanoid robot that can navigate and avoid dynamic obstacles.

### ✅ Tasks:

1.  **Create a digital room in Gazebo:** Design a simple indoor environment with static obstacles.
2.  **Place a humanoid robot inside:** Integrate a humanoid URDF/SDF model into your Gazebo world.
3.  **Enable navigation:** Set up a basic navigation system for the humanoid, potentially using ROS 2 Nav2.
4.  **Add moving obstacles:** Introduce dynamic elements into the environment to test reactive avoidance.
5.  **Add LiDAR + Camera:** Equip your humanoid with simulated LiDAR and camera sensors for perception.

### Step 1 — Make Digital Room

**✅ Completed in 2.2.3** (from "Foundations of Digital Twins and Gazebo" chapter)
You should already have a basic `room.world` file. Expand on this by adding more static objects (tables, chairs) using `<model>` tags with `<include>` elements for existing Gazebo models or by creating simple box/cylinder models.

### Step 2 — Add Humanoid

**Import URDF into Gazebo:**
As discussed in section 2.4, you need to find a suitable humanoid URDF/SDF model and ensure it's loaded into your Gazebo world. This will involve:
*   Finding a humanoid URDF (e.g., from `gazebo_ros_demos` or other open-source projects).
*   Ensuring the URDF includes necessary `<gazebo>` tags for physics, sensors (LiDAR, camera, IMU), and `ros2_control` interfaces.
*   Creating a ROS 2 launch file to spawn the humanoid into your custom Gazebo world.

### Step 3 — Add Navigation

This is where the power of ROS 2 comes to the forefront. You will typically integrate the **ROS 2 Navigation Stack (Nav2)**.

**Install Nav2:**
```bash
sudo apt install ros-humble-navigation2 # Replace humble with your ROS 2 distro
```

**Launch with Robot:**
You'll need a comprehensive ROS 2 launch file that:
*   Launches your custom Gazebo world.
*   Spawns your humanoid robot.
*   Launches the `ros_gz_sim` bridge nodes.
*   Launches your robot's `ros2_control` controllers (e.g., joint position controllers for the legs, torso, arms).
*   Launches the Nav2 stack, configured for your humanoid. This will be the most challenging part, as Nav2 is traditionally for wheeled robots. You will likely need a custom "cmd_vel to humanoid joint commands" translator node.

### Step 4 — Dynamic Obstacles

To truly test your navigation system's reactive capabilities, introduce moving obstacles.

**Add moving objects:**
You can define simple dynamic models directly in your `.world` file:

```xml
<model name="moving_box">
  <pose>0 2 0.5 0 0 0</pose> <!-- Initial pose -->
  <link name="box_link">
    <inertial><mass>1.0</mass></inertial>
    <visual name="visual">
      <geometry><box><size>0.5 0.5 0.5</size></box></geometry>
      <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Red</name></script></material>
    </visual>
    <collision name="collision">
      <geometry><box><size>0.5 0.5 0.5</size></box></geometry>
    </collision>
  </link>
  <!-- Give it velocity plugin -->
  <plugin name="moving_plugin" filename="libmoving_box_plugin.so">
    <target_pose>0 -2 0.5 0 0 0</target_pose>
    <speed>0.5</speed>
  </plugin>
</model>
```
You would need to write a simple custom Gazebo plugin (`libmoving_box_plugin.so` in the example) that moves the box between two points, or simply apply forces/velocities via a ROS 2 node.

This mini-project will challenge your understanding of model integration, control, perception, and navigation, providing a holistic experience in developing Digital Twins for humanoid robotics. Remember to iterate, test, and debug each component methodically. Good luck!

</div>

<div className="urdu-content">


# باب 10: روبوٹکس کے لیے یونٹی اور انضمام

Gazebo کی مضبوط فزکس اور روبوٹ سیمولیشن کی صلاحیتوں کو تلاش کرنے کے بعد، ہم اب یونٹی کی طرف توجہ دیتے ہیں، ایک طاقتور گیم انجن جو بے مثال بصری وفاداری اور تعاملی تجربات فراہم کرتا ہے۔ یہ باب آپ کو یونٹی کو اپنی روبوٹکس ورک فلو میں انضمام کے بارے میں رہنمائی فراہم کرتا ہے، اس کی طاقتوں پر توجہ مرکز کرتے ہوئے اعلیٰ وفاداری انسان-روبوٹ تعامل اور ROS 2 ایکو سسٹم کے ساتھ بے داغ رابطے کے لیے۔ آخر میں، ہم ایک منی پروجیکٹ کے ساتھ ہمارا ماڈیول 2 کا سفر مکمل کریں گے جو سب کچھ سیکھے گئے تصورات کو یکجا کرتا ہے۔

## 10.1 — روبوٹکس کے لیے یونٹی کا تعارف

جبکہ Gazebo فزکس-درست سیمولیشن میں بہترین کارکردگی دکھاتا ہے اور ROS 2 کے ساتھ بیک اینڈ روبوٹ کنٹرول کے لیے گہرا انضمام فراہم کرتا ہے، یونٹی اس کی صلاحیت میں چمکتا ہے کہ وہ نہایت دلکش، انتہائی تعاملی، اور محو کن ماحول تیار کرے۔ گیم انجن کے طور پر اس کے جڑ میں اسے روبوٹکس میں فرنٹ اینڈ ایپلیکیشنز کے لیے ایک قابلِ مقابلہ اوزار بنا دیتے ہیں، خاص طور پر جہاں غنی ویژولائزیشن، اعلیٰ درجے کا انسان-روبوٹ تعامل، یا VR/AR تجربات اہم ہوں۔

### روبوٹکس کے لیے یونٹی

یونٹی ایک کراس پلیٹ فارم گیم انجن ہے جو یونٹی ٹیکنالوجیز کے ذریعے تیار کیا گیا ہے، جس کا استعمال ویڈیو گیمز تیار کرنے کے لیے PC، کنسول، موبائل ڈیوائسز، اور ویب سائٹس کے لیے کیا جاتا ہے۔ اس کا وسیع فیچر سیٹ، سمجھدار ایڈیٹر، اور بڑا اثاثہ اسٹور روبوٹکس میں اسے انتہائی مقبول بنا رہا ہے ایپلیکیشنز کے لیے جیسے:

*   **اوتارز:** اعلیٰ درجے کے انسان-روبوٹ تعامل کے مطالعات کے لیے حقیقی انسان اور روبوٹ اوتارز تخلیق کرنا۔
*   **انسان کی سیمولیشن:** روبوٹکس کے ماحول میں انسانی رویے اور موجودگی کی سیمولیشن۔
*   **VR/AR ایپلیکیشنز:** روبوٹ ٹیلی آپریشن، تربیت، اور ڈیٹا ویژولائزیشن کے لیے ورچوئل ریلٹی اور ایگزیومنٹڈ ریلٹی انٹرفیس تیار کرنا۔
*   **حقیقی ماحول:** پیچیدہ منظر ناموں کے لیے انتہائی تفصیلی اور بصروی طور پر غنی ماحول تیار کرنا، خاص طور پر جہاں بصروی ادراک کلیدی ہو۔
*   **ڈیجیٹل ٹوئن ویژولائزیشن:** ڈیجیٹل ٹوئنز کے لیے بصروی طور پر بہتر فرنٹ اینڈ کے طور پر کام کرنا، Gazebo یا جسمانی روبوٹ جیسے بیک اینڈ سیمولیشن یا جسمانی روبوٹ سے حقیقی وقت کے روبوٹ ڈیٹا رینڈر کرنا۔

### یونٹی کے ساتھ شروع کرنا

شروع کرنے کے لیے، آپ کو یونٹی انسٹال کرنے کی ضرورت ہوگی۔

**یونٹی ہب انسٹال کریں:**
یونٹی ہب ایک انتظامی اوزار ہے جو آپ کو آپ کے یونٹی پروجیکٹس اور یونٹی ایڈیٹر کے انسٹالیشن کا انتظام کرنے میں مدد کرتا ہے۔ اسے سرکاری یونٹی ویب سائٹ سے ڈاؤن لوڈ کریں۔

**یونٹی ایڈیٹر (LTS) انسٹال کریں:**
یونٹی ہب کے اندر، یونٹی ایڈیٹر کا **لمبی مدت کے ساتھ سپورٹ (LTS)** ورژن انسٹال کریں۔ LTS ورژن کو مستحکم اور طویل مدتی پروجیکٹ سپورٹ کے لیے تجویز کیا جاتا ہے۔

### روبوٹکس کے لیے کلیدی یونٹی تصورات

جب آپ یونٹی کے ساتھ کام کرنا شروع کریں گے، تو آپ کچھ بنیادی تصورات کا سامنا کریں گے جو اس کے معماری کے لیے اساسی ہیں:

*   **مناظر (ماحول):** یونٹی میں ایک منظر وہ جگہ ہے جہاں آپ اپنی گیم یا سیمولیشن کا ماحول ڈیزائن اور تعمیر کرتے ہیں۔ یہ دراصل آپ کے اثاثوں اور گیم آبجیکٹس کا ایک کنٹینر ہے، جو آپ کی مجازی دنیا کا لے آؤٹ طے کرتا ہے۔ روبوٹکس میں، ایک منظر ایک فیکٹری کا کام، گھر کا ماحول، یا ایک بیرونی منظر کی نمائندگی کر سکتا ہے۔
*   **گیم آبجیکٹس (روبوٹس، اشیاء، کیمرے):** گیم آبجیکٹس یونٹی میں بنیادی اشیاء ہیں جو کرداروں، اشیاء، لائٹس، کیمرے، یا کسی اور چیز کی نمائندگی کرتے ہیں جو آپ اپنے منظر میں دیکھ سکتے ہیں۔ روبوٹکس میں، آپ کا روبوٹ ماڈل، ماحولیاتی اشیاء، سینسرز (یونٹی میں مجازی کیمرے)، اور صارف انٹرفیس کے عناصر سب گیم آبجیکٹس ہوں گے۔
*   **کمپوننٹس (سینسرز، کنٹرولرز، رویے):** کمپوننٹس وہ فنکشنل ٹکڑے ہیں جو آپ گیم آبجیکٹس سے منسلک کرتے ہیں تاکہ انہیں صلاحیتیں دی جا سکیں۔ ایک گیم آبجیکٹ صرف ایک خالی کنٹینر ہے؛ کمپوننٹس اس کے رویے کی وضاحت کرتے ہیں۔ روبوٹکس میں مثالیں شامل ہیں:
    *   **میش رینڈر کمپوننٹ:** 3D ماڈل کو نظر آنے کے قابل بناتا ہے۔
    *   **کالیڈر کمپوننٹ:** جسمانی تعاملات اور کالیژن ڈیٹیکشن کو فعال کرتا ہے۔
    *   **رگڈ بڈی کمپوننٹ:** فزکس (گریویٹی، فورسز) کو گیم آبجیکٹ پر لاگو کرتا ہے۔
    *   **سکرپٹ کمپوننٹس:** حسب ضرورت C# سکرپٹس جو آپ لکھتے ہیں تاکہ پیچیدہ روبوٹ رویے، سینسر منطق، یا کنٹرول انٹرفیسز کی وضاحت کی جا سکے۔

یہ عناصر کو ملا کر، یونٹی پیچیدہ روبوٹکس سیمولیشنز اور تعاملی انٹرفیسز تخلیق کرنے کے لیے ایک لچکدار اور طاقتور پلیٹ فارم فراہم کرتا ہے۔

## 10.2 — اعلیٰ وفاداری انسان-روبوٹ تعامل

یونٹی کی صلاحیتیں انسان-روبوٹ تعامل (HRI) کے شعبے میں کافی حد تک پھیل جاتی ہیں، ایسے ٹولز فراہم کرتے ہوئے جو امیر اور قابلِ یقین تعاملات تخلیق کرتے ہیں جو سوشل روبوٹس، تعاونی روبوٹکس، اور سمجھدار کنٹرول سسٹم کے لیے اہم ہیں۔

### یونٹی کے ساتھ تعامل کو بہتر بنانا

یونٹی میں، آپ انسان-روبوٹ کے تعاملات میں حقیقیت اور اظہار کی تہوں کو شامل کر سکتے ہیں:

*   **انسانی اوتارز:** حقیقی انسانی کردار کے ماڈلز کو درآمد اور اینیمیٹ کریں۔ یہ اوتار انسانی صارفین کے لیے اسٹینڈ ان کے طور پر کام کر سکتے ہیں، اس کی اجازت دیتے ہوئے کہ روبوٹ کے آپریشنل سپیس کے اندر انسانی موجودگی اور تعامل کی سیمولیشن ہو سکے۔
*   **NPCs (غیر-پلیئر کردار):** AI-ڈرائیون مجازی ایجنٹس تخلیق کریں جو مخصوص انسانی رویے، ردعمل، اور حرکات کی سیمولیشن کر سکتے ہیں۔ یہ روبوٹس کے تعاملات کو متحرک انسانی ماحول کے ساتھ ٹیسٹ کرنے کی اجازت دیتا ہے۔
*   **AI-مبنی اظہارات:** اوتارز (انسان اور روبوٹ دونوں) کے لیے اظہار کے نظام نافذ کریں جو جذبات یا ارادے کو ظاہر کرنے کے لیے چہرے کے اظہارات یا جسمانی زبان کا استعمال کر سکیں، تعاملات کو زیادہ قدرتی اور قابلِ سمجھنا بناتے ہیں۔
*   **اشاروں کا کنٹرول:** یونٹی کے ان پٹ سسٹم اور ممکنہ طور پر بیرونی ڈیوائسز (مثلاً لیپ موشن، VR کنٹرولرز) کو انسانی اشاروں کی تشریح کے لیے استعمال کریں۔ پھر یہ اشارے روبوٹ کے لیے کمانڈز میں تبدیل کیے جا سکتے ہیں، جو ذاتی، غیر الفاظ کے رابطے کی اجازت دیتے ہیں۔

### پیچیدہ تعاملات کی سیمولیشن

یونٹی خاص طور پر نازک HRI منظر ناموں کی سیمولیشن کے لیے انتہائی طاقتور بن جاتا ہے:

*   **✅ مصافحہ:** انسانی مصافحے کے جواب میں روبوٹ کا ردعمل ڈیزائن اور اینیمیٹ کریں، باریک حرکات اور فورس فیڈ بیک کو شامل کرتے ہوئے (اگر فزکس انجن یا ہیپٹک ڈیوائس کے ساتھ جوڑا جائے)۔
*   **✅ گفتگو:** تقریر کی پہچان اور ترجمانی کو کردار کے اینیمیشنز کے ساتھ انضمام کریں تاکہ انسان اور روبوٹ کے درمیان گفتگو کی سیمولیشن ہو سکے۔
*   **✅ پیچھے چلنا:** ایک روبوٹ کو انسانی اوتار کے پیچھے چلنے کے لیے پروگرام کریں، ان کی حرکات پر ردعمل ظاہر کرتے ہوئے اور محفوظ فاصلہ برقرار رکھتے ہوئے۔
*   **✅ کمانڈز:** قدرتی زبان کی سمجھ کے انٹرفیسز تیار کریں جہاں انسانی اوتار کے منہ سے کہے گئے یا اشاروں کے ذریعے کمانڈز کو روبوٹ کے ذریعے عمل میں لایا جاتا ہے۔

یہ صلاحیتیں سوشل روبوٹکس کے لیے بہترین ہیں، جہاں روبوٹ کی انسانی اشاروں کو سمجھنے اور مناسب طریقے سے جواب دینے کی صلاحیت انتہائی اہم ہے۔ یونٹی بصری اور تعاملی کینوس فراہم کرتا ہے جو ان پیچیدہ تعاملات کی ترقی اور بہتری کے لیے استعمال کیا جا سکتا ہے۔

## 10.3 — یونٹی کو ROS 2 کے ساتھ جوڑنا

روبوٹکس ورک فلو میں یونٹی کو واقعی انضمام دینے کے لیے، اسے ROS 2 کے ذریعے منظم کیے جانے والے بنیادی روبوٹ کی ذہانت سے رابطہ کرنا چاہیے۔ `ROS-TCP-Connector` اس خلا کو پُر کرنے کے لیے معیاری ٹول ہے۔

### ROS-TCP-Connector

`ROS-TCP-Connector` یونٹی ٹیکنالوجیز کے ذریعے تیار کیا گیا ایک یونٹی پیکج ہے جو ROS کمیونٹی کے تعاون سے تیار کیا گیا ہے۔ یہ یونٹی ایپلیکیشنز کو ROS 2 نوڈس کے طور پر کام کرنے کے قابل بناتا ہے، جو انہیں پیغامات پبلش کرنے، ٹاپکس کو سبسکرائب کرنے، اور سروسز اور ایکشنز کے ساتھ ROS 2 نیٹ ورک پر تعامل کرنے کی اجازت دیتا ہے۔

**ROS-TCP-Connector انسٹال کریں:**
آپ عام طور پر ریپوزٹری کو اپنے یونٹی پروجیکٹ کے `Packages` فولڈر میں کلون کریں گے یا یونٹی کے پیکج مینیجر کے ذریعے اسے درآمد کریں گے۔

```bash
git clone https://github.com/Unity-Technologies/ROS-TCP-Connector
```
ریپوزٹری میں فراہم کردہ انسٹالیشن ہدایات پر عمل کریں تاکہ آپ کے یونٹی پروجیکٹ کے اندر مناسب سیٹ اپ یقینی بنایا جا سکے۔ یہ عام طور پر کور پیکج انسٹال کرنے اور پھر اپنے ROS 2 `.msg` اور `.srv` فائلوں سے C# پیغام کلاسز جنریٹ کرنے میں شامل ہوتا ہے۔

### یونٹی کو ROS 2 سے کیوں جوڑیں؟

یہ رابطہ یونٹی کو آپ کے ROS 2-پاورڈ روبوٹس کے لیے ایک طاقتور فرنٹ اینڈ یا غنی سیمولیشن ماحول بننے کی اجازت دیتا ہے:

*   **یونٹی میں روبوٹ کو کنٹرول کریں:** یونٹی کے اندر سمجھدار UI عناصر یا تعاملی منظر نامے تیار کریں جو کنٹرول کمانڈز (مثلاً ویلوسٹی، جوائنٹ پوزیشنز، ہائی لیول ٹاسک گوئلز) کو ROS 2 ٹاپکس پر پبلش کرتے ہیں۔ پھر یہ کمانڈز آپ کے روبوٹ (جسمانی یا Gazebo-سیمولیٹڈ) کے ذریعے انجام دی جاتی ہیں۔
*   **سینسر آؤٹ پٹ دیکھیں:** اپنے روبوٹ سے ROS 2 کے ذریعے پبلش کیے گئے سینسر ڈیٹا (LiDAR سکینز، کیمرہ امیجز، IMU ڈیٹا) کو سبسکرائب کریں۔ پھر یونٹی یہ ڈیٹا انتہائی گریفکل اور معلوماتی طریقے سے ویژولائز کر سکتا ہے، اسے 3D منظر پر اوور لے کر کے۔
*   **حرکت اور حالت کی ویژولائزیشن:** یونٹی کے اندر روبوٹ کی حقیقی وقت کی پوز، جوائنٹ اسٹیٹس، اور منصوبہ بند کردہ راستے دکھائیں۔ یہ آپ کے روبوٹ کا بصروی طور پر درست ڈیجیٹل ٹوئن تخلیق کرتا ہے، جو اس کے کام کرنے کے بارے میں قیمتی بصروی فراہم کرتا ہے۔

صنعتی کمپنیاں جیسے **ٹیسلا بٹ، میٹا، اور ایمیزون روبوٹکس** اس طرح کے انضمامات کو اپنی اگلی نسل کے روبوٹکس سسٹم کی ترقی اور ٹیسٹ کرنے کے لیے فعال طور پر استعمال کر رہی ہیں، ROS 2 کے مضبوط روبوٹکس فریم ورک کے ساتھ یونٹی کی بصری صلاحیت کا فائدہ اٹھاتے ہوئے۔

## 10.4 — منی پروجیکٹ (ماڈیول 2): ڈیجیٹل کمرہ اور ہیومنوائڈ نیویگیشن

یہ منی پروجیکٹ ہمارے ماڈیول 2 میں سب کچھ سیکھنے کے اختتام کا کام کرتا ہے۔ یہ ڈیجیٹل ٹوئنز، Gazebo برائے فزکس سیمولیشن، روبوٹ ماڈلنگ، سینسر انضمام، اور بنیادی نیویگیشن کے اصولوں کو یکجا کرتا ہے۔

**گوئل:** ایک ڈیجیٹل ٹوئن سیمولیشن کمرہ تخلیق کریں جس میں ایک ہیومنوائڈ روبوٹ ہو جو نیویگیٹ کر سکے اور متحرک رکاوٹوں سے بچ سکے۔

### ✅ کام:

1.  **Gazebo میں ایک ڈیجیٹل کمرہ تخلیق کریں:** سٹیٹک رکاوٹوں کے ساتھ ایک سادہ اندر کا ماحول ڈیزائن کریں۔
2.  **ایک ہیومنوائڈ روبوٹ کو اندر رکھیں:** اپنی Gazebo دنیا میں ایک ہیومنوائڈ URDF/SDF ماڈل کو انضمام دیں۔
3.  **نیویگیشن کو فعال کریں:** ہیومنوائڈ کے لیے بنیادی نیویگیشن سسٹم سیٹ کریں، ممکنہ طور پر ROS 2 Nav2 کا استعمال کر کے۔
4.  **متحرک رکاوٹیں شامل کریں:** ماحول میں متحرک عناصر کو متعارف کرائیں تاکہ رد عمل کی رکاوٹ کو ٹیسٹ کیا جا سکے۔
5.  **LiDAR + کیمرہ شامل کریں:** اپنے ہیومنوائڈ کو ادراک کے لیے سیمولیٹڈ LiDAR اور کیمرہ سینسرز سے لیس کریں۔

### مرحلہ 1 — ڈیجیٹل کمرہ بنائیں

**✅ 2.2.3 میں مکمل** (ڈیجیٹل ٹوئنز اور Gazebo کی بنیادوں کے باب سے)
آپ کے پاس پہلے سے ہی ایک بنیادی `room.world` فائل ہونی چاہیے۔ اس میں مزید سٹیٹک اشیاء (میزیں، کرسیاں) کو شامل کر کے اس کو وسعت دیں `<model>` ٹیگز کا استعمال کر کے `<include>` عناصر کے ساتھ موجودہ Gazebo ماڈلز کے لیے یا سادہ باکس/سلنڈر ماڈلز تخلیق کر کے۔

### مرحلہ 2 — ہیومنوائڈ شامل کریں

**Gazebo میں URDF درآمد کریں:**
سیکشن 2.4 میں بحث کردہ کے مطابق، آپ کو ایک مناسب ہیومنوائڈ URDF/SDF ماڈل تلاش کرنے اور یقینی بنانا ہوگا کہ یہ آپ کی Gazebo دنیا میں لوڈ ہو گیا ہے۔ اس میں یہ شامل ہوگا:
*   ایک ہیومنوائڈ URDF تلاش کرنا (مثلاً `gazebo_ros_demos` یا دیگر اوپن سورس پروجیکٹس سے)۔
*   یقینی بنانا کہ URDF میں ضروری `<gazebo>` ٹیگز فزکس، سینسرز (LiDAR، کیمرہ، IMU)، اور `ros2_control` انٹرفیسز کے لیے شامل ہیں۔
*   ایک ROS 2 لانچ فائل تخلیق کرنا تاکہ ہیومنوائڈ کو آپ کی حسب ضرورت Gazebo دنیا میں اسپون کیا جا سکے۔

### مرحلہ 3 — نیویگیشن شامل کریں

یہیں Nav2 کی طاقت ROS 2 کے سامنے آتی ہے۔ آپ عام طور پر **ROS 2 نیویگیشن اسٹیک (Nav2)** کو انضمام دیں گے۔

**Nav2 انسٹال کریں:**
```bash
sudo apt install ros-humble-navigation2 # humble کو اپنے ROS 2 ڈسٹرو میں تبدیل کریں
```

**روبوٹ کے ساتھ لانچ کریں:**
آپ کو ایک جامع ROS 2 لانچ فائل کی ضرورت ہوگی جو:
*   آپ کی حسب ضرورت Gazebo دنیا لانچ کرے۔
*   آپ کے ہیومنوائڈ روبوٹ کو اسپون کرے۔
*   `ros_gz_sim` برج نوڈس لانچ کرے۔
*   آپ کے روبوٹ کے `ros2_control` کنٹرولرز لانچ کرے (مثلاً ٹانگوں، ٹورسو، بازوؤں کے لیے جوائنٹ پوزیشن کنٹرولرز)۔
*   Nav2 اسٹیک لانچ کرے، آپ کے ہیومنوائڈ کے لیے کنفیگر کیا گیا۔ یہ سب سے چیلنجنگ حصہ ہوگا، کیونکہ Nav2 کو روایتی طور پر چکروں والے روبوٹس کے لیے تیار کیا گیا ہے۔ آپ کو ایک حسب ضرورت "cmd_vel سے ہیومنوائڈ جوائنٹ کمانڈز" کے ترجمہ کرنے والا نوڈ درکار ہوگا۔

### مرحلہ 4 — متحرک رکاوٹیں

اپنے نیویگیشن سسٹم کی رد عمل کی صلاحیتوں کو واقعی ٹیسٹ کرنے کے لیے، متحرک رکاوٹیں متعارف کرائیں۔

**متحرک اشیاء شامل کریں:**
آپ اپنی `.world` فائل میں براہ راست سادہ متحرک ماڈلز کی وضاحت کر سکتے ہیں:

```xml
<model name="moving_box">
  <pose>0 2 0.5 0 0 0</pose> <!-- ابتدائی پوز -->
  <link name="box_link">
    <inertial><mass>1.0</mass></inertial>
    <visual name="visual">
      <geometry><box><size>0.5 0.5 0.5</size></box></geometry>
      <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Red</name></script></material>
    </visual>
    <collision name="collision">
      <geometry><box><size>0.5 0.5 0.5</size></box></geometry>
    </collision>
  </link>
  <!-- اسے ویلوسٹی پلگ ان دیں -->
  <plugin name="moving_plugin" filename="libmoving_box_plugin.so">
    <target_pose>0 -2 0.5 0 0 0</target_pose>
    <speed>0.5</speed>
  </plugin>
</model>
```
آپ کو ایک سادہ حسب ضرورت Gazebo پلگ ان لکھنے کی ضرورت ہوگی (`libmoving_box_plugin.so` مثال میں) جو ڈبے کو دو نقاط کے درمیان منتقل کرے، یا صرف ایک ROS 2 نوڈ کے ذریعے فورسز/ویلوسٹیز کو لاگو کریں۔

یہ منی پروجیکٹ ماڈل انضمام، کنٹرول، ادراک، اور نیویگیشن کی آپ کی سمجھ کو چیلنج کرے گا، ہیومنوائڈ روبوٹکس کے لیے ڈیجیٹل ٹوئنز کی ترقی میں ایک مکمل تجربہ فراہم کرے گا۔ ہر جزو کو نظام وار طور پر دہرائیں، ٹیسٹ کریں، اور ڈیبگ کریں۔ بہتر کامیابی!


</div>
