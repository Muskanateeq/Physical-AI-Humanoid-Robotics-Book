---
id: robotic-system
title: 'Chapter 1: The Robotic Nervous System (ROS 2)'
---

import Admonition from '@theme/Admonition';
import LanguageToggle from '@site/src/components/LanguageToggle/LanguageToggle';

<LanguageToggle />


<div className="english-content">

# Chapter 1: The Robotic Nervous System (ROS 2)

Full, production-grade chapter: theory + practical labs + runnable examples.

This module teaches the middleware that connects AI, perception, and hardware: ROS 2. You’ll learn the theory (communication models, QoS, lifecycle), how to install and configure a reproducible ROS 2 workspace, how to write real rclpy nodes (publishers, subscribers, services, actions), how to design a URDF humanoid, visualize in RViz, and—critically—how to bridge an AI agent (Python/LLM) into ROS 2 controllers safely and robustly. The module ends with a complete mini-project with all files, launch commands, and test/checklists so you can run a simple humanoid demo: move arms and head via a Python agent.

Assumes basic familiarity with Linux, Python, and Git. Prefer Ubuntu 22.04 LTS for compatibility with common ROS 2 distros (Humble/Ion). Adjust distribution names if you use a different ROS 2 release.

## 1.1 — Introduction to ROS 2

### What is ROS?

ROS (Robot Operating System) is a flexible framework for writing robot software. It provides standard ways to:

- write nodes (processes) that communicate,
- exchange typed messages,
- organize packages and launch systems,
- compose systems of multiple interacting components, and
- reuse community libraries for sensing, planning, control.

ROS is not an OS; it's middleware and tooling to compose distributed robotics applications.

### Why ROS 2 instead of ROS 1

- DDS-based transport: better real-time and cross-process communication, multiple vendors.
- Quality of Service (QoS): tailor reliability/latency for sensors vs diagnostics.
- Lifecycle nodes: explicit startup/stop states for safety-critical components.
- Cross-platform and modern: supports Linux (primary), Windows, macOS; improved security.
- Multi-threading & composition: safer for modern robotics.

### Real-world role in robotics

ROS 2 is the nervous system of complex robots:

- abstracts hardware drivers into messages,
- allows swapping simulation/hardware with minimal code changes,
- provides standard action/service patterns for tasks like navigation and manipulation.

### ROS 2 architecture overview

Key pieces:

- **Nodes** — processes handling single responsibilities.
- **Topics** — asynchronous pub/sub streams (sensor data, odometry).
- **Messages** — strongly typed structures for topics.
- **Services** — synchronous request/response (short RPC).
- **Actions** — long-running tasks with feedback and preemption.
- **Parameter server & parameters API** — configurable runtime values.
- **DDS (Data Distribution Service)** — under-the-hood transport with QoS policies.

## 1.2 — ROS 2 Core Concepts

### Nodes

- Encapsulate functionality (e.g., `vision_node`, `controller_node`).
- Lightweight processes; can be in Python (`rclpy`) or C++ (`rclcpp`).
- :::note Recommended
  One responsibility per node (single-responsibility principle).
  :::

### Topics

- Used for streaming data (camera images, joint states).
- Publishers and subscribers are decoupled: multiple publishers, multiple subscribers.
- Topics use message types (e.g., `sensor_msgs/msg/Image`, `geometry_msgs/msg/Twist`).

### Messages

- Strongly-typed definitions in `.msg` files.
- Keep messages small and predictable; large binary blobs should use shared storage with references.

### Services

- Synchronous request/response RPC for quick tasks (e.g., `CalibrateSensor`).
- Not suited for long-running tasks.

### Actions

- For long-running tasks requiring feedback and cancellation (e.g., `FollowPath`, `PickObject`).
- Provide goal, result, and feedback channels.

### Parameters

- Key/Value store associated with nodes.
- Ideally configured via launch files or parameter YAML for reproducibility.


### 1.2.1 Node Communication Model

- Graph-based: system is a graph of nodes connected by topics/services/actions.
- Discovery: DDS enables runtime discovery — nodes discover each other by topic/service names.
- Composition: multiple nodes may run in one process for performance or isolated for safety.
- Use composition for low-latency internal components; separate processes for fault isolation.
  

### 1.2.2 Pub/Sub System Explained

- **Publisher:** creates messages and publishes to a named topic.
- **Subscriber:** receives messages from that topic.
- **QoS:** pick `reliable` for critical messages, `best_effort` for lossy high-rate sensors.
- **Transient_local:** for late-joining subscribers to get last value.

### 1.2.3 Services vs Actions

| Aspect       | Service           | Action                       |
|-------------|-----------------|------------------------------|
| Duration     | short            | long-running                 |
| Use Case     | config, inquiry  | motion, manipulation         |
| Feedback     | none             | continuous feedback          |
| Preemption   | no               | supports cancel              |

Practice: Implement configuration via services and robotic tasks via actions.

### 1.2.4 Parameters & Configuration

- Place static config in YAML files loaded at launch.
- Dynamic reconfigure: change parameters at runtime with validation and safety checks.
- Namespacing: group parameters per node or per robot instance.

## 1.3 — ROS 2 Installation & Setup

This is a practical, reproducible walkthrough to get ROS 2 running, create a workspace, and test demo nodes.

The exact package names change with ROS distros. Below are reproducible commands for Ubuntu 22.04 and ROS 2 Humble as a concrete example. If you use a different ROS 2 version, replace distro-specific words accordingly.

### Install ROS 2 Humble (example)
- Setup locale
```bash
sudo locale-gen en_US en_US.UTF-8
export LANG=en_US.UTF-8
```

### Add ROS 2 apt repository
```bash
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list
```

### Install ROS 2 desktop
```bash
sudo apt update
sudo apt install -y ros-humble-desktop
```

### Environment
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Install development tools
```bash
sudo apt install -y python3-colcon-common-extensions python3-rosdep
sudo rosdep init
rosdep update
```

### Workspace setup (ros_ws)
```bash
mkdir -p ~/ros_ws/src
cd ~/ros_ws
colcon build
source install/setup.bash
```

### Creating your first package (Python)
```bash
cd ~/ros_ws/src
ros2 pkg create --build-type ament_python demo_comm
```
This creates package with setup.cfg and package.xml
Add a simple publisher node under demo_comm/demo_comm/talker.py
- Then build:
```bash
cd ~/ros_ws
colcon build --packages-select demo_comm
source install/setup.bash
```

### Running demo nodes

ROS 2 ships with demo nodes. Test pub/sub with the demo:

### in Terminal 1
ros2 run demo_nodes_py talker

### in Terminal 2
ros2 run demo_nodes_py listener


If you see messages printed, core ROS 2 comms are functional.

</div>

<div className="urdu-content">

# باب 1: روبوٹکس کا نروس سسٹم (ROS 2)

مکمل، پروڈکشن گریڈ باب: نظریہ + عملی لیبز + چلنے والے مثالیں۔

یہ ماڈیول وہ مڈل ویئر سیکھاتا ہے جو AI، ادراک، اور ہارڈویئر کو جوڑتا ہے: ROS 2. آپ نظریہ (مواصلات کے ماڈلز، QoS، لائف سائیکل) سیکھیں گے، ایک دوبارہ استعمال کے قابل ROS 2 ورک اسپیس کو انسٹال اور کنفیگر کریں گے، حقیقی rclpy نوڈز (پبلشرز، سبسکرائبرز، سروسز، ایکشنز) لکھیں گے، URDF ہیومنوائڈ ڈیزائن کریں گے، RViz میں ویژولائز کریں گے، اور—اہم بات—AI ایجنٹ (Python/LLM) کو ROS 2 کنٹرولرز میں محفوظ اور مضبوط طریقے سے جوڑنے کا طریقہ سیکھیں گے۔ ماڈیول کے اختتام پر، ایک مکمل منی پروجیکٹ ہے جس میں تمام فائلیں، لانچ کمانڈز، اور ٹیسٹ/چیک لسٹس شامل ہیں تاکہ آپ ایک سادہ ہیومنوائڈ ڈیمو چلا سکیں: ایک Python ایجنٹ کے ذریعے بازوؤں اور سر کو حرکت دیں۔

بنیادی Linux، Python، اور Git کے علم کی ضرورت ہے۔ ROS 2 کے ہم آہنگی کے لیے Ubuntu 22.04 LTS کو ترجیح دیں (Humble/Ion)۔ اگر آپ مختلف ROS 2 ریلیز استعمال کرتے ہیں تو تقسیم کے ناموں کو ایڈجسٹ کریں۔

## 1.1 — ROS 2 کا تعارف

### ROS کیا ہے؟

ROS (روبوٹ آپریٹنگ سسٹم) روبوٹ سافٹ ویئر لکھنے کے لیے ایک لچکدار فریم ورک ہے۔ یہ معیاری طریقے فراہم کرتا ہے:

- نوڈز (عمل) لکھنے کے لیے جو مواصلت کریں،
- ٹائپڈ پیغامات کا تبادلہ،
- پیکجز اور لانچ سسٹم کو منظم کرنا،
- متعدد ملوث اجزاء کے سسٹم تشکیل دینا، اور
- سینسنگ، منصوبہ بندی، کنٹرول کے لیے کمیونٹی لائبریریز دوبارہ استعمال کرنا۔

ROS OS نہیں ہے؛ یہ مڈل ویئر اور ٹولز ہے جو تقسیم شدہ روبوٹکس ایپلیکیشنز کو تشکیل دیتا ہے۔

### ROS 1 کے بجائے ROS 2 کیوں؟

- DDS-مبنی ٹرانسپورٹ: بہتر ریل ٹائم اور کراس-پروسیس مواصلات، متعدد وینڈرز۔
- کوالٹی آف سروس (QoS): سینسرز بمقابلہ ڈائیگنوسٹکس کے لیے قابلِ اعتماد/تاخیر کو ایڈجسٹ کرنا۔
- لائف سائیکل نوڈز: حفاظتی اجزاء کے لیے واضح اسٹارٹ/اسٹاپ اسٹیٹس۔
- کراس-پلیٹ فارم اور جدید: Linux (اولین)، Windows، macOS کی حمایت کرتا ہے؛ بہتر سیکورٹی۔
- ملٹی-تھریڈنگ اور کمپوزیشن: جدید روبوٹکس کے لیے محفوظ۔

### روبوٹکس میں حقیقی دنیا کا کردار

ROS 2 پیچیدہ روبوٹس کا نروس سسٹم ہے:

- ہارڈویئر ڈرائیورز کو پیغامات میں م abstract کرتا ہے،
- سیمولیشن/ہارڈویئر کو کم سے کم کوڈ تبدیلیوں کے ساتھ سویپ کرنے کی اجازت دیتا ہے،
- نیویگیشن اور مینیپولیشن جیسے کاموں کے لیے معیاری ایکشن/سروس پیٹرنز فراہم کرتا ہے۔

### ROS 2 معماری کا جائزہ

اہم اجزاء:

- **نوڈز** — واحد ذمہ داریوں کو ہینڈل کرنے والے عمل۔
- **ٹاپکس** — غیر ہم وقت ساز pub/sub سٹریمز (سینسر ڈیٹا، اودومیٹری)۔
- **میسجس** — ٹاپکس کے لیے مضبوط ٹائپ کردہ سٹرکچر۔
- **سروسز** — ہم وقت ساز درخواست/جواب (چھوٹا RPC)۔
- **ایکشنز** — فیڈ بیک اور پریمپشن کے ساتھ طویل چلنے والے کام۔
- **پیرامیٹر سرور اور پیرامیٹر API** — کنفیگریبل رن ٹائم ویلیوز۔
- **DDS (ڈیٹا ڈسٹری بیوشن سروس)** — QoS پالیسیز کے ساتھ ٹرانسپورٹ کے تحت۔

## 1.2 — ROS 2 کے بنیادی تصورات

### نوڈز

- فعالیت کو ایبسٹریکٹ کرتا ہے (مثلاً، `vision_node`، `controller_node`)۔
- ہلکے عمل؛ Python (`rclpy`) یا C++ (`rclcpp`) میں ہو سکتا ہے۔
- :::note تجویز کردہ
  ایک نوڈ پر ایک ذمہ داری (سینگل ریسپانسیبلٹی پرنسپل)۔
  :::

### ٹاپکس

- ڈیٹا اسٹریمنگ کے لیے استعمال ہوتا ہے (کیمرہ امیجز، جوائنٹ اسٹیٹس)۔
- پبلشرز اور سبسکرائبرز غیر منسلک ہیں: متعدد پبلشرز، متعدد سبسکرائبرز۔
- ٹاپکس میسج ٹائپس استعمال کرتے ہیں (مثلاً، `sensor_msgs/msg/Image`، `geometry_msgs/msg/Twist`)۔

### میسجس

- `.msg` فائلز میں مضبوط ٹائپ کردہ تعریفات۔
- میسجس چھوٹے اور قابلِ پیش گوئی رکھیں؛ بڑے بائنری بولبز کو ریفرنسز کے ساتھ مشترکہ اسٹوریج کا استعمال کرنا چاہیے۔

### سروسز

- جلدی کے کاموں کے لیے ہم وقت ساز درخواست/جواب RPC (مثلاً، `CalibrateSensor`)۔
- طویل چلنے والے کاموں کے لیے مناسب نہیں۔

### ایکشنز

- فیڈ بیک اور منسوخی کی ضرورت والے طویل چلنے والے کاموں کے لیے (مثلاً، `FollowPath`، `PickObject`)۔
- گوئل، ریزلٹ، اور فیڈ بیک چینلز فراہم کرتا ہے۔

### پیرامیٹرز

- نوڈز سے منسلک کلید/ویلیو اسٹور۔
- بہترین طور پر لانچ فائلز یا YAML پیرامیٹر کے ذریعے کنفیگر کیا جانا چاہیے تاکہ دوبارہ استعمال کیا جا سکے۔

### 1.2.1 نوڈ کمیونیکیشن ماڈل

- گراف-مبنی: سسٹم ٹاپکس/سروسز/ایکشنز کے ذریعے جڑے ہوئے نوڈز کا گراف ہے۔
- دریافت: DDS رن ٹائم کی دریافت کو فعال کرتا ہے — نوڈز ایک دوسرے کو ٹاپک/سروس ناموں کے ذریعے تلاش کرتے ہیں۔
- کمپوزیشن: متعدد نوڈز ایک عمل میں چل سکتے ہیں کارکردگی کے لیے یا حفاظت کے لیے علیحدہ۔
- کم تاخیر والے داخلی اجزاء کے لیے کمپوزیشن کا استعمال کریں؛ نقصان کے جزموں کے لیے الگ عمل۔

### 1.2.2 Pub/Sub سسٹم کی وضاحت

- **پبلشر:** پیغامات بناتا ہے اور ایک نامزد ٹاپک پر پبلش کرتا ہے۔
- **سبسکرائبر:** اس ٹاپک سے پیغامات وصول کرتا ہے۔
- **QoS:** `reliable` کو اہم پیغامات کے لیے، `best_effort` کو نقصان دہ زیادہ شرح والے سینسرز کے لیے چنیں۔
- **Transient_local:** دیر سے شامل ہونے والے سبسکرائبرز کو آخری ویلیو حاصل کرنے کے لیے۔

### 1.2.3 سروسز بمقابلہ ایکشنز

| پہلو | سروس | ایکشن |
| :------------- | :----------------- | :------------------------------ |
| مدت | مختصر | طویل چلنے والا |
| استعمال کا مقصد | کنفیگ، استفسار | موشن، مینیپولیشن |
| فیڈ بیک | کوئی نہیں | مسلسل فیڈ بیک |
| پریمپشن | نہیں | منسوخ کی حمایت کرتا ہے |

مشق: کنفیگریشن کو سروسز کے ذریعے اور روبوٹکس کے کاموں کو ایکشنز کے ذریعے نافذ کریں۔

### 1.2.4 پیرامیٹرز اور کنفیگریشن

- سٹیٹک کنفیگ کو YAML فائلز میں رکھیں جو لانچ پر لوڈ ہوں۔
- ڈائینامک ری کنفیگر: تصدیق اور حفاظتی چیکس کے ساتھ رن ٹائم پر پیرامیٹر تبدیل کریں۔
- نیم اسپیسنگ: ہر نوڈ یا ہر روبوٹ انسٹینس کے لیے پیرامیٹر گروپ کریں۔

## 1.3 — ROS 2 انسٹالیشن اور سیٹ اپ

یہ ایک عملی، دوبارہ استعمال کے قابل ورک فلو ہے ROS 2 کو چلانے، ایک ورک اسپیس تخلیق کرنے، اور ڈیمو نوڈز ٹیسٹ کرنے کے لیے۔

بالکل وہی پیکج نام ROS distros کے ساتھ تبدیل ہوتے ہیں۔ ذیل میں Ubuntu 22.04 اور ROS 2 Humble کے لیے دوبارہ استعمال کے قابل کمانڈز ہیں ایک مثال کے طور پر۔ اگر آپ مختلف ROS 2 ورژن استعمال کرتے ہیں تو distro مخصوص الفاظ کو مطابق کریں۔

### ROS 2 Humble انسٹال کریں (مثال)

- لوکل سیٹ اپ کریں
```bash
sudo locale-gen en_US en_US.UTF-8
export LANG=en_US.UTF-8
```

### ROS 2 apt ریپوزٹری شامل کریں
```bash
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list
```

### ROS 2 ڈیسک ٹاپ انسٹال کریں
```bash
sudo apt update
sudo apt install -y ros-humble-desktop
```

### ماحول
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### ترقیاتی اوزار انسٹال کریں
```bash
sudo apt install -y python3-colcon-common-extensions python3-rosdep
sudo rosdep init
rosdep update
```

### ورک اسپیس سیٹ اپ (ros_ws)
```bash
mkdir -p ~/ros_ws/src
cd ~/ros_ws
colcon build
source install/setup.bash
```

### اپنا پہلا پیکج تخلیق کریں (Python)
```bash
cd ~/ros_ws/src
ros2 pkg create --build-type ament_python demo_comm
```
یہ سیٹ اپ.cfg اور package.xml کے ساتھ پیکج تخلیق کرتا ہے
demo_comm/demo_comm/talker.py کے تحت ایک سادہ پبلشر نوڈ شامل کریں
- پھر بنائیں:
```bash
cd ~/ros_ws
colcon build --packages-select demo_comm
source install/setup.bash
```

### ڈیمو نوڈز چلائیں

ROS 2 ڈیمو نوڈز کے ساتھ آتا ہے۔ pub/sub کو ڈیمو کے ساتھ ٹیسٹ کریں:

### ٹرمنل 1 میں
ros2 run demo_nodes_py talker

### ٹرمنل 2 میں
ros2 run demo_nodes_py listener

اگر آپ پرنٹ کردہ پیغامات دیکھ رہے ہیں، تو ROS 2 کا کور کمیونیکیشن کام کر رہا ہے۔

</div>
