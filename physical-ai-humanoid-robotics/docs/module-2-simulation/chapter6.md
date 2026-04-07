---
id: simulation-and-gazebo
title: 'Chapter 6: Humanoid Simulation and Advanced Gazebo'
---

import LanguageToggle from '@site/src/components/LanguageToggle/LanguageToggle';

<LanguageToggle />


<div className="english-content">

# Chapter 6: Humanoid Simulation and Advanced Gazebo Features

Building upon our understanding of Digital Twins and Gazebo fundamentals, this chapter dives into the specifics of simulating complex humanoid robots within this powerful environment. We will cover how to integrate humanoid models and then explore the crucial role of sensor simulation, enabling your virtual robots to perceive their digital world just as their physical counterparts do.

## 6.1 — Simulating a Humanoid Robot in Gazebo

Simulating a humanoid robot effectively in Gazebo requires careful attention to its detailed structure, physical properties, and control interfaces. These robots, with their many degrees of freedom and the challenge of bipedal locomotion, provide an excellent testbed for advanced simulation techniques.

### Importing URDF or SDF

To bring a humanoid robot into Gazebo, you primarily use one of two descriptive file formats:

*   **URDF (Unified Robot Description Format):** An XML format for describing the robot's kinematic and dynamic properties, visual representation, and collision geometries. It's widely used in ROS.
*   **SDF (Simulation Description Format):** A more comprehensive XML format, native to Gazebo, which can describe not only robots but also entire environments, lights, and static objects. While URDF can be converted to SDF, SDF often provides more direct control over simulation-specific parameters.

**Step 1 — Get a Sample Humanoid Model:**
To begin, you can obtain a sample humanoid model. Many open-source robotics projects provide URDF/SDF files for their robots. A good starting point might be to clone a repository with Gazebo and ROS 2 demos, which often include example robot models. For instance, the `gazebo_ros_demos` repository often contains various robot models.

```bash
git clone https://github.com/ros-simulation/gazebo_ros_demos
```
Navigate through this (or similar) repository to find a humanoid robot description (e.g., in a `description` folder within a robot package).

**Step 2 — Launch Gazebo with the Robot:**
Once you have the URDF/SDF files, you need to launch Gazebo and instruct it to load your robot. This is typically done via ROS 2 launch files.

First, you might launch an empty Gazebo world (or one you created earlier):
```bash
ros2 launch gazebo_ros empty_world.launch.py
```
This command typically starts Gazebo. You would then usually have a separate ROS 2 node or launch file specifically designed to "spawn" your robot's URDF/SDF into this running Gazebo instance. The `gazebo_ros` package often provides tools for this.

For example, if your humanoid robot's package (let's say `my_humanoid_description`) contains a launch file to spawn it:
```bash
ros2 launch my_humanoid_description spawn_humanoid.launch.py
```
Or, you might manually insert the humanoid model directly into the Gazebo UI if it's placed in a directory known to Gazebo (e.g., `~/.gazebo/models`). In the Gazebo GUI, go to `Insert` and look for your model.

**You now see a humanoid ✅** within your Gazebo simulation, ready for interaction.

### Walking Simulation

Simulating stable and dynamic walking for a humanoid robot is one of the pinnacle challenges in robotics. It involves orchestrating hundreds of joint movements while maintaining balance against gravity.

Initially, your humanoid might just stand there or fall over, depending on its initial pose and controller setup. To achieve walking, you need to provide commands to its joints.

**You control joints via ROS 2 topics:**
The individual joints of your humanoid robot are controlled by publishing messages to specific ROS 2 topics. For example, to set the position of a joint, you might publish to a topic like `/my_humanoid/joint_controller/commands` using a `std_msgs/msg/Float64` or `trajectory_msgs/msg/JointTrajectory` message type, depending on your controller configuration.

To monitor the current state of the joints:
```bash
ros2 topic pub /joint_states # This is to publish, not monitor.
ros2 topic echo /joint_states # This is to monitor joint states.
```
The `/joint_states` topic (typically published by the `joint_state_publisher` and `gazebo_ros_pkgs`) provides the current positions, velocities, and efforts of all joints. This is crucial feedback for any walking controller.

**Later, AI will control walking:**
While you can manually publish joint commands for basic movements, true humanoid walking is governed by complex control algorithms. These often involve:
*   **Inverse Kinematics (IK):** Calculating the joint angles required to achieve a desired foot placement or body pose.
*   **Balance Control:** Using feedback from IMUs and force sensors to keep the robot upright (e.g., Zero Moment Point (ZMP) control, whole-body control).
*   **Gait Generation:** Algorithms that define the rhythmic pattern of leg and body movements for walking.
*   **Reinforcement Learning:** Training an AI agent to learn optimal walking policies through trial and error in simulation.

The Gazebo environment, coupled with ROS 2, provides the perfect platform to develop, test, and refine these advanced AI-driven walking algorithms for humanoid robots.


## 6.2 — Sensor Simulation

Just as real robots rely heavily on their sensors to perceive the world and make intelligent decisions, simulated robots depend on accurate sensor models to gather virtual data from their digital environments. Gazebo provides excellent capabilities for simulating a wide array of sensors.

### Gazebo Can Simulate:

*   **LiDAR (Light Detection and Ranging):** Generates 2D or 3D point cloud data for mapping and obstacle detection.
*   **Camera:** Provides realistic RGB images.
*   **Depth Camera:** Offers RGB and depth (distance to objects) information, similar to RealSense or Kinect.
*   **IMU (Inertial Measurement Unit):** Measures angular velocity and linear acceleration, crucial for odometry and balance.
*   **Encoders:** Provide joint position and velocity feedback for robot kinematics.

These sensor capabilities are implemented as Gazebo plugins, which are loaded as part of your robot's URDF/SDF definition.

### 6.2.1 LiDAR Simulation

LiDAR sensors are fundamental for autonomous navigation, mapping (SLAM), and collision avoidance. Gazebo's LiDAR simulation plugin mimics the behavior of real-world laser scanners.

**Add this to your model (within a `<link>` or as a separate `<sensor>` block in SDF):**
For URDF, you'd typically embed a `<gazebo>` tag that references a `libgazebo_ros_laser.so` plugin. In SDF, it might look like this:

```xml
<sensor name="laser_sensor" type="ray">
  <pose>0 0 0.1 0 0 0</pose> <!-- Position relative to its parent link -->
  <visualize>true</visualize>
  <update_rate>10</update_rate>
  <ray>
    <scan>
      <horizontal>
        <samples>640</samples>
        <resolution>1</resolution>
        <min_angle>-2.2</min_angle>
        <max_angle>2.2</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.08</min>
      <max>10.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <plugin name="gazebo_ros_laser_controller" filename="libgazebo_ros_laser.so">
    <topicName>/scan</topicName>
    <frameName>laser_frame</frameName>
  </plugin>
</sensor>
```

**Get data from the simulated LiDAR:**
Once Gazebo is running with your robot and its LiDAR, the plugin will publish laser scan messages to a ROS 2 topic (by default, `/scan` for many plugins). You can view these messages:

```bash
ros2 topic echo /scan
```

This data, typically `sensor_msgs/msg/LaserScan`, is then used for:

*   **Mapping:** Building 2D occupancy grid maps of the environment.
*   **Obstacle detection:** Identifying obstacles in the robot's immediate vicinity.
*   **Navigation:** Providing input to navigation algorithms for local path planning and collision avoidance.

### 6.2.2 Depth Camera Simulation

Depth cameras are invaluable for 3D perception, object recognition, and manipulation tasks, providing both color (RGB) and distance (depth) information. Gazebo can simulate these sensors effectively.

**Add this to your model (similar to LiDAR, using a camera type sensor with a depth plugin):**

```xml
<sensor name="depth_camera_sensor" type="depth">
  <pose>0 0 0.2 0 0 0</pose>
  <visualize>true</visualize>
  <update_rate>30</update_rate>
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.05</near>
      <far>3</far>
    </clip>
  </camera>
  <plugin name="camera_controller" filename="libgazebo_ros_depth_camera.so">
    <ros>
      <namespace>depth_camera</namespace>
      <argument>--ros-args -r image:=image_raw -r depth_image:=depth/image_raw</argument>
    </ros>
    <cameraName>camera</cameraName>
    <alwaysOn>true</alwaysOn>
    <updateRate>30.0</updateRate>
    <imageTopicName>image_raw</imageTopicName>
    <depthImageTopicName>depth/image_raw</depthImageTopicName>
    <cameraInfoTopicName>camera_info</cameraInfoTopicName>
    <frameName>camera_link_optical</frameName>
  </plugin>
</sensor>
```

**View the simulated camera feeds:**
The depth camera plugin will publish various topics, including `/camera/image_raw` (RGB image), `/camera/depth/image_raw` (depth image), and `/camera/points` (point cloud). You can view the image streams using `rqt_image_view` from ROS 2:

```bash
ros2 run rqt_image_view rqt_image_view
```
This tool opens a GUI where you can select the image topic (e.g., `/depth_camera/image_raw`) to view the live camera feed from your robot 👀.

### 6.2.3 IMU & Motion Sensors

IMU (Inertial Measurement Unit) sensors are vital for providing data on a robot's orientation, angular velocity, and linear acceleration. This information is critical for odometry, state estimation, and especially for balance control in humanoids.

**Add this to your model (using a `imu` type sensor and its plugin):**

```xml
<sensor name="imu_sensor" type="imu">
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
    <ros>
      <namespace></namespace>
      <argument>--ros-args -r imu:=imu_data</argument>
    </ros>
    <frameName>imu_link</frameName>
    <initialOrientationAsReference>false</initialOrientationAsReference>
  </plugin>
</sensor>
```

**The IMU provides:**

*   **Orientation:** Typically as a quaternion, representing the robot's attitude in space.
*   **Angular Velocity:** The rate of rotation around the robot's axes.
*   **Linear Acceleration:** The acceleration experienced by the sensor.

This data is usually published to a ROS 2 topic (e.g., `/imu_data` as `sensor_msgs/msg/Imu`). It is extensively used for:

*   **Odometry:** Fusing with wheel encoder data (for mobile bases) to get more accurate pose estimation.
*   **Balance Control:** Providing critical feedback for humanoid walking controllers to maintain upright posture and recover from perturbations.
*   **State Estimation:** Input for Kalman filters or other estimators to get a robust estimate of the robot's full state.

By accurately simulating these various sensors, Gazebo allows you to develop and test complex perception, navigation, and control algorithms for humanoid robots in a rich, virtual environment before deploying them to costly and delicate physical hardware. This significantly accelerates the development cycle and reduces risks.

</div>

<div className="urdu-content">

# باب 6: ہیومنوائڈ سیمولیشن اور اعلیٰ درجے کے گیزبو خصوصیات

ڈیجیٹل ٹوئنز اور گیزبو کی بنیادوں کو سمجھنے کی بنیاد پر، یہ باب اس طاقتور ماحول کے اندر پیچیدہ ہیومنوائڈ روبوٹس کی سیمولیشن کی تفصیلات میں گہرا جاتا ہے۔ ہم یہ دیکھیں گے کہ ہیومنوائڈ ماڈلز کو کیسے انضمام کیا جاتا ہے اور پھر سینسر سیمولیشن کے اہم کردار کو تلاش کریں گے، جو آپ کے مجازی روبوٹس کو اپنی ڈیجیٹل دنیا کا ادراک کرنے کے قابل بناتا ہے جیسے ان کے جسمانی ہم آہنگ کرتے ہیں۔

## 6.1 — گیزبو میں ہیومنوائڈ روبوٹ کی سیمولیشن

گیزبو میں بطورِ مؤثر ایک ہیومنوائڈ روبوٹ کی سیمولیشن کے لیے اس کی تفصیلی ساخت، جسمانی خصوصیات، اور کنٹرول انٹرفیس پر غور کرنا ضروری ہے۔ یہ روبوٹ، اپنی بہت سے ڈیگریز آف فریڈم اور بائی پیڈل لوکوموشن کی چیلنج کے ساتھ، اعلیٰ درجے کی سیمولیشن تکنیکوں کے لیے ایک عمدہ ٹیسٹ بیڈ فراہم کرتے ہیں۔

### URDF یا SDF کو امپورٹ کرنا

گیزبو میں ہیومنوائڈ روبوٹ کو لانے کے لیے، آپ بنیادی طور پر دو تفصیلی فائل فارمیٹس میں سے ایک کا استعمال کرتے ہیں:

*   **URDF (یونیفائیڈ روبوٹ ڈیسکرپشن فارمیٹ):** روبوٹ کے کنیمیٹک اور ڈائنامک خصوصیات، بصری نمائندگی، اور کالیژن جیومیٹریز کی وضاحت کے لیے ایک XML فارمیٹ۔ یہ ROS میں وسیع پیمانے پر استعمال ہوتا ہے۔
*   **SDF (سیمولیشن ڈیسکرپشن فارمیٹ):** ایک زیادہ جامع XML فارمیٹ، گیزبو کے لیے اصل، جو نہ صرف روبوٹس کی وضاحت کر سکتا ہے بلکہ پورے ماحول، لائٹس، اور سٹیٹک اشیاء کی بھی۔ جبکہ URDF کو SDF میں تبدیل کیا جا سکتا ہے، SDF اکثر سیمولیشن مخصوص پیرامیٹرز پر براہ راست کنٹرول فراہم کرتا ہے۔

**مرحلہ 1 — ایک نمونہ ہیومنوائڈ ماڈل حاصل کریں:**
شروع کرنے کے لیے، آپ ایک نمونہ ہیومنوائڈ ماڈل حاصل کر سکتے ہیں۔ بہت سے اوپن سورس روبوٹکس پروجیکٹس اپنے روبوٹس کے لیے URDF/SDF فائلوں کو فراہم کرتے ہیں۔ ایک اچھی شروعات کا نقطہ یہ ہو سکتا ہے کہ Gazebo اور ROS 2 ڈیمو کے ساتھ ایک ریپوزٹری کو کلون کریں، جو اکثر مثالی روبوٹ ماڈلز پر مشتمل ہوتا ہے۔ مثال کے طور پر، `gazebo_ros_demos` ریپوزٹری اکثر مختلف روبوٹ ماڈلز پر مشتمل ہوتا ہے۔

```bash
git clone https://github.com/ros-simulation/gazebo_ros_demos
```
اس (یا اس جیسی) ریپوزٹری کے ذریعے نیویگیٹ کریں تاکہ ایک ہیومنوائڈ روبوٹ کی تفصیل تلاش کریں (مثلاً روبوٹ پیکج کے اندر ایک `description` فولڈر میں)۔

**مرحلہ 2 — روبوٹ کے ساتھ گیزبو لانچ کریں:**
ایک بار جب آپ کے پاس URDF/SDF فائلیں ہوں، تو آپ کو گیزبو کو لانچ کرنا ہوگا اور اسے اپنے روبوٹ کو لوڈ کرنے کا حکم دینا ہوگا۔ یہ عام طور پر ROS 2 لانچ فائلوں کے ذریعے کیا جاتا ہے۔

سب سے پہلے، آپ ایک خالی گیزبو دنیا (یا ایک جو آپ نے پہلے بنائی تھی) کو لانچ کر سکتے ہیں:
```bash
ros2 launch gazebo_ros empty_world.launch.py
```
یہ کمانڈ عام طور پر گیزبو شروع کرتی ہے۔ پھر آپ کے پاس عام طور پر ایک علیحدہ ROS 2 نوڈ یا لانچ فائل ہوگی جو خاص طور پر "اسپون" کرنے کے لیے ڈیزائن کی گئی ہو گی آپ کے روبوٹ کا URDF/SDF اس چلتے ہوئے گیزبو انسٹانس میں۔ `gazebo_ros` پیکج اکثر اس کے لیے ٹولز فراہم کرتا ہے۔

مثال کے طور پر، اگر آپ کے ہیومنوائڈ روبوٹ کا پیکج (مثلاً `my_humanoid_description`) اسے اسپون کرنے کے لیے ایک لانچ فائل پر مشتمل ہے:
```bash
ros2 launch my_humanoid_description spawn_humanoid.launch.py
```
یا، آپ خود انسانی ماڈل کو براہ راست گیزبو UI میں داخل کر سکتے ہیں اگر اسے گیزبو کے لیے معلوم ڈائریکٹری میں رکھا گیا ہو (مثلاً `~/.gazebo/models`)۔ گیزبو GUI میں، `Insert` پر جائیں اور اپنے ماڈل کی تلاش کریں۔

**آپ اب اپنی گیزبو سیمولیشن کے اندر ایک ہیومنوائڈ دیکھ رہے ہیں ✅**، تعامل کے لیے تیار۔

### چلنے کی سیمولیشن

ہیومنوائڈ روبوٹ کے لیے مستحکم اور متحرک چلنے کی سیمولیشن روبوٹکس میں انتہا کے چیلنجوں میں سے ایک ہے۔ یہ گریویٹی کے خلاف توازن برقرار رکھتے ہوئے سینکڑوں جوائنٹ حرکات کو منظم کرنے میں شامل ہے۔

شروع میں، آپ کا ہیومنوائڈ صرف وہیں کھڑا ہو سکتا ہے یا گر سکتا ہے، اس کے ابتدائی پوز اور کنٹرولر سیٹ اپ پر منحصر ہے۔ چلنے کے لیے، آپ کو اس کے جوائنٹس کو کمانڈز فراہم کرنی ہوں گی۔

**آپ جوائنٹس کو ROS 2 ٹاپکس کے ذریعے کنٹرول کرتے ہیں:**
آپ کے ہیومنوائڈ روبوٹ کے انفرادی جوائنٹس کو مخصوص ROS 2 ٹاپکس پر میسجس پبلش کر کے کنٹرول کیا جاتا ہے۔ مثال کے طور پر، جوائنٹ کی پوزیشن سیٹ کرنے کے لیے، آپ `/my_humanoid/joint_controller/commands` جیسے ٹاپک پر ایک `std_msgs/msg/Float64` یا `trajectory_msgs/msg/JointTrajectory` میسج ٹائپ کا استعمال کر کے پبلش کر سکتے ہیں، آپ کے کنٹرولر کنفیگریشن پر منحصر ہے۔

جوائنٹس کی موجودہ حالت کو مانیٹر کرنے کے لیے:
```bash
ros2 topic pub /joint_states # یہ پبلش کرنے کے لیے ہے، نہ کہ مانیٹر کرنے کے لیے۔
ros2 topic echo /joint_states # یہ جوائنٹ اسٹیٹس کو مانیٹر کرنے کے لیے ہے۔
```
`/joint_states` ٹاپک (عام طور پر `joint_state_publisher` اور `gazebo_ros_pkgs` کے ذریعے پبلش کیا گیا) تمام جوائنٹس کی موجودہ پوزیشنز، ویلوسٹیز، اور ایفروٹس فراہم کرتا ہے۔ یہ کسی بھی چلنے والے کنٹرولر کے لیے انتہائی اہم فیڈ بیک ہے۔

**بعد میں، AI چلنے کو کنٹرول کرے گا:**
جبکہ آپ بنیادی حرکات کے لیے جوائنٹ کمانڈز دستی طور پر پبلش کر سکتے ہیں، حقیقی ہیومنوائڈ چلنے کو پیچیدہ کنٹرول الگورتھم کے ذریعے حکم دیا جاتا ہے۔ ان میں اکثر شامل ہوتا ہے:

*   **ان ورس کنیمیٹکس (IK):** مطلوبہ پاؤں کی جگہ یا جسم کے پوز کو حاصل کرنے کے لیے ضروری جوائنٹ اینگلز کا حساب لگانا۔
*   **توازن کنٹرول:** IMUs اور فورس سینسرز سے فیڈ بیک کا استعمال کرتے ہوئے روبوٹ کو کھڑا رکھنا (مثلاً زیرو مومنٹ پوائنٹ (ZMP) کنٹرول، وہول باڈی کنٹرول)۔
*   **گیٹ جنریشن:** الگورتھم جو چلنے کے لیے لیگ اور جسم کی حرکات کے ریتھمک پیٹرن کی وضاحت کرتے ہیں۔
*   **ریفورسمنٹ لرننگ:** سیمولیشن میں تجربہ اور غلطی کے ذریعے بہترین چلنے کی پالیسیز سیکھنے کے لیے ایک AI ایجنٹ کو تربیت دینا۔

گیزبو ماحول، ROS 2 کے ساتھ جڑا ہوا، ہیومنوائڈ روبوٹس کے لیے اعلیٰ درجے کے AI-ڈرائیون چلنے کے الگورتھم کی ترقی، ٹیسٹ، اور بہتری کے لیے مکمل پلیٹ فارم فراہم کرتا ہے۔

## 6.2 — سینسر سیمولیشن

جیسا کہ حقیقی روبوٹس دنیا کا ادراک کرنے اور ذہین فیصلے کرنے کے لیے اپنے سینسرز پر زبردست انحصار کرتے ہیں، سیمولیٹڈ روبوٹس اپنی ڈیجیٹل ماحول سے مجازی ڈیٹا جمع کرنے کے لیے درست سینسر ماڈلز پر منحصر ہیں۔ گیزبو سینسرز کی وسیع رینج کو سیمولیٹ کرنے کے لیے عمدہ صلاحیتیں فراہم کرتا ہے۔

### گیزبو سیمولیٹ کر سکتا ہے:

*   **LiDAR (لائٹ ڈیٹیکشن اینڈ رینجنگ):** نقشہ کشی اور رکاوٹ کے پتہ لگانے کے لیے 2D یا 3D پوائنٹ کلاؤڈ ڈیٹا تیار کرتا ہے۔
*   **کیمرا:** حقیقی RGB تصاویر فراہم کرتا ہے۔
*   **ڈیپتھ کیمرا:** RGB اور ڈیپتھ (اشیاء تک کا فاصلہ) کی معلومات فراہم کرتا ہے، RealSense یا Kinect کے مماثل۔
*   **IMU (انرٹیل میزورمنٹ یونٹ):** زاویہ ویلوسٹی اور لکیری تیزی کو پیمائش کرتا ہے، جو اوڈومیٹری اور توازن کے لیے اہم ہے۔
*   **اینکوڈرز:** روبوٹ کنیمیٹکس کے لیے جوائنٹ پوزیشن اور ویلوسٹی کا فیڈ بیک فراہم کرتے ہیں۔

یہ سینسر صلاحیتیں Gazebo پلگ انز کے طور پر نافذ کی گئی ہیں، جو آپ کے روبوٹ کی URDF/SDF کی تعریف کے حصے کے طور پر لوڈ کیے جاتے ہیں۔

### 6.2.1 LiDAR سیمولیشن

LiDAR سینسرز خودکار نیویگیشن، نقشہ کشی (SLAM)، اور کالیژن سے بچاؤ کے لیے بنیادی ہیں۔ گیزبو کی LiDAR سیمولیشن پلگ ان حقیقی دنیا کے لیزر سکینرز کے رویے کو نقل کرتا ہے۔

**اپنے ماڈل میں یہ شامل کریں (ایک `<link>` کے اندر یا SDF میں ایک الگ `<sensor>` بلاک کے طور پر):**
URDF کے لیے، آپ عام طور پر ایک `<gazebo>` ٹیگ ایمبیڈ کریں گے جو `libgazebo_ros_laser.so` پلگ ان کا حوالہ دیتا ہے۔ SDF میں، یہ اس طرح نظر آ سکتا ہے:

```xml
<sensor name="laser_sensor" type="ray">
  <pose>0 0 0.1 0 0 0</pose> <!-- اس کے والد لنک کے مقابلے میں پوزیشن -->
  <visualize>true</visualize>
  <update_rate>10</update_rate>
  <ray>
    <scan>
      <horizontal>
        <samples>640</samples>
        <resolution>1</resolution>
        <min_angle>-2.2</min_angle>
        <max_angle>2.2</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.08</min>
      <max>10.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <plugin name="gazebo_ros_laser_controller" filename="libgazebo_ros_laser.so">
    <topicName>/scan</topicName>
    <frameName>laser_frame</frameName>
  </plugin>
</sensor>
```

**سیمولیٹڈ LiDAR سے ڈیٹا حاصل کریں:**
ایک بار جب Gazebo آپ کے روبوٹ اور اس کے LiDAR کے ساتھ چل رہا ہو، پلگ ان لیزر سکین میسجس کو ایک ROS 2 ٹاپک پر پبلش کرے گا (ڈیفالٹ کے طور پر، بہت سے پلگ انز کے لیے `/scan`)۔ آپ ان میسجس کو دیکھ سکتے ہیں:

```bash
ros2 topic echo /scan
```

یہ ڈیٹا، عام طور پر `sensor_msgs/msg/LaserScan`، کا استعمال ہوتا ہے:

*   **نقشہ کشی:** ماحول کے 2D اکوپنسی گرڈ نقشے تیار کرنے کے لیے۔
*   **رکاوٹ کا پتہ چلنا:** روبوٹ کے فوری ارد گرد میں رکاوٹوں کی شناخت کرنے کے لیے۔
*   **نیویگیشن:** مقامی راستہ منصوبہ بندی اور کالیژن سے بچاؤ کے لیے نیویگیشن الگورتھم کے لیے ان پٹ فراہم کرنے کے لیے۔

### 6.2.2 ڈیپتھ کیمرا سیمولیشن

ڈیپتھ کیمرے 3D ادراک، اشیاء کی پہچان، اور مینیپولیشن کے کاموں کے لیے بے بہا ہیں، جو رنگ (RGB) اور فاصلہ (ڈیپتھ) کی معلومات فراہم کرتے ہیں۔ گیزبو ان سینسرز کو مؤثر طریقے سے سیمولیٹ کر سکتا ہے۔

**اپنے ماڈل میں یہ شامل کریں (LiDAR کی طرح، کیمرہ ٹائپ سینسر کا استعمال کرتے ہوئے ایک ڈیپتھ پلگ ان کے ساتھ):**

```xml
<sensor name="depth_camera_sensor" type="depth">
  <pose>0 0 0.2 0 0 0</pose>
  <visualize>true</visualize>
  <update_rate>30</update_rate>
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.05</near>
      <far>3</far>
    </clip>
  </camera>
  <plugin name="camera_controller" filename="libgazebo_ros_depth_camera.so">
    <ros>
      <namespace>depth_camera</namespace>
      <argument>--ros-args -r image:=image_raw -r depth_image:=depth/image_raw</argument>
    </ros>
    <cameraName>camera</cameraName>
    <alwaysOn>true</alwaysOn>
    <updateRate>30.0</updateRate>
    <imageTopicName>image_raw</imageTopicName>
    <depthImageTopicName>depth/image_raw</depthImageTopicName>
    <cameraInfoTopicName>camera_info</cameraInfoTopicName>
    <frameName>camera_link_optical</frameName>
  </plugin>
</sensor>
```

**سیمولیٹڈ کیمرہ فیڈ دیکھیں:**
ڈیپتھ کیمرہ پلگ ان مختلف ٹاپکس پبلش کرے گا، بشمول `/camera/image_raw` (RGB تصویر)، `/camera/depth/image_raw` (ڈیپتھ تصویر)، اور `/camera/points` (پوائنٹ کلاؤڈ)۔ آپ ROS 2 سے `rqt_image_view` کا استعمال کرتے ہوئے تصویر کے سٹریمز دیکھ سکتے ہیں:

```bash
ros2 run rqt_image_view rqt_image_view
```
یہ ٹول ایک GUI کھولتا ہے جہاں آپ تصویر کے ٹاپک (مثلاً `/depth_camera/image_raw`) کو منتخب کر سکتے ہیں تاکہ اپنے روبوٹ کا لائیو کیمرہ فیڈ دیکھ سکیں 👀۔

### 6.2.3 IMU & موشن سینسرز

IMU (انرٹیل میزورمنٹ یونٹ) سینسرز روبوٹ کے جہت، زاویہ ویلوسٹی، اور لکیری تیزی کے بارے میں ڈیٹا فراہم کرنے کے لیے اہم ہیں۔ یہ معلومات اوڈومیٹری، اسٹیٹ ایسٹیمیشن، اور خاص طور پر ہیومنوائڈز میں توازن کنٹرول کے لیے اہم ہے۔

**اپنے ماڈل میں یہ شامل کریں (ایک `imu` ٹائپ سینسر اور اس کے پلگ ان کا استعمال کرتے ہوئے):**

```xml
<sensor name="imu_sensor" type="imu">
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
    <ros>
      <namespace></namespace>
      <argument>--ros-args -r imu:=imu_data</argument>
    </ros>
    <frameName>imu_link</frameName>
    <initialOrientationAsReference>false</initialOrientationAsReference>
  </plugin>
</sensor>
```

**IMU فراہم کرتا ہے:**

*   **جہت:** عام طور پر ایک کوارٹر نین کے طور پر، جو خلائی میں روبوٹ کی جسامت کی نمائندگی کرتا ہے۔
*   **زاویہ ویلوسٹی:** روبوٹ کے محور کے گرد گھومنے کی شرح۔
*   **لکیری تیزی:** سینسر کے ذریعے تجربہ کی گئی تیزی۔

یہ ڈیٹا عام طور پر ایک ROS 2 ٹاپک پر پبلش کیا جاتا ہے (مثلاً `/imu_data` کے طور پر `sensor_msgs/msg/Imu`)۔ اس کا استعمال وسیع پیمانے پر ہوتا ہے:

*   **اوڈومیٹری:** پہیوں کے اینکوڈر ڈیٹا کے ساتھ فیوژن کرنے کے لیے (موبائل بیس کے لیے) تاکہ زیادہ درست پوز ایسٹیمیشن حاصل کی جا سکے۔
*   **توازن کنٹرول:** ہیومنوائڈ چلنے والے کنٹرولرز کو کھڑا رہنے اور متزلزل ہونے سے بچنے کے لیے اہم فیڈ بیک فراہم کرنے کے لیے۔
*   **اسٹیٹ ایسٹیمیشن:** کیلمن فلٹرز یا دیگر ایسٹیمیٹرز کے لیے ان پٹ تاکہ روبوٹ کی مکمل حالت کا ایک مضبوط اندازہ حاصل کیا جا سکے۔

مختلف سینسرز کی درست طریقے سے سیمولیشن کر کے، گیزبو آپ کو ہیومنوائڈ روبوٹس کے لیے پیچیدہ ادراک، نیویگیشن، اور کنٹرول الگورتھم کی ترقی اور ٹیسٹ کرنے کی اجازت دیتا ہے ایک محفوظ، مجازی ماحول میں پہلے انہیں مہنگے اور نازک جسمانی ہارڈویئر پر نفاذ کرنے سے پہلے۔ یہ ترقی کے چکر کو نمایاں طور پر تیز کرتا ہے اور خطرات کو کم کرتا ہے۔

</div>
