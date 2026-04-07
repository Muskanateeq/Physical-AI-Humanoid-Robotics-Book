---
id: intro-physical-ai
title: 'Introduction to Physical AI & Humanoid Robotics'
slug: /
---

import LanguageToggle from '@site/src/components/LanguageToggle/LanguageToggle';

<LanguageToggle />
import Admonition from '@theme/Admonition';


<div className="english-content">

# Chapter 0 — Introduction to Physical AI & Humanoid Robotics

## 0.1 — What is Physical AI?

Physical AI is the discipline that unites artificial intelligence with physical systems — robots that sense, reason, and act in the real world. Unlike purely virtual AI (large language models, recommendation systems), Physical AI must cope with real-world constraints: force, friction, latency, noisy sensors, and safety.

It requires integration across:

* Hardware
* Embedded systems
* Control theory
* Perception
* Planning
* High-level cognition

### Key Characteristics

* **Embodiment:** Intelligence expressed through a physical body that has dynamics and constraints.
* **Tight perception–action loop:** Sensing → Reasoning → Action (continuous and latency-aware)
* **Multi-disciplinary stack:** Mechanical design, control systems, middleware (e.g., ROS 2), simulation, and machine learning.

## 0.2 — What is a Humanoid Robot?

A humanoid robot is a robot whose kinematics and sensor layout mimic the human form (head, torso, two arms, two legs).

Humanoids are studied and built for:

* Working in **human environments** (stairs, tables, knobs)
* Interacting **naturally with people** (social robotics)
* Testing **embodied intelligence** in anthropomorphic settings

### Key challenges

* Dynamic balance and bipedal locomotion
* Multi-joint coordination
* Manipulation in cluttered scenes
* Integration of high-level reasoning with fast low-level control

## 0.3 — Why This Field is the Future

Physical AI and humanoids bring AI into the physical world where the biggest economic and social impact can occur:

* Automation of dangerous or monotonous tasks
* In-home assistive robots for aging populations
* Human-robot collaboration in manufacturing
* Robotics in exploration (underwater, space)
* New interfaces for telepresence and mixed reality

Advances in:

* Simulation
* GPU-accelerated perception
* Large Language Models (LLMs)

…are accelerating real-world applications.
**This book teaches you how to combine these advances into working humanoid systems.**


### Real-World Applications

* **Industry:** Human-assistive robots on assembly lines
* **Healthcare:** Robotic rehabilitation assistants
* **Space & Exploration:** Human-like manipulators in zero gravity
* **Defense & Emergency:** Robots in hazardous environments
* **Home & Service:** Domestic assistance & caregiving
* **Research & Education:** Testbeds for embodied cognition & HRI


## 0.4 — What You Will Learn in This Book

This course is project and simulation-driven. By the end, you will build a **voice-driven humanoid workflow** that:

* senses
* plans
* navigates
* manipulates

### Skills & Technologies Covered

* **Middleware & Control:** ROS 2 (nodes, topics, services, actions)
* **Modeling & Kinematics:** URDF/SDF robot description
* **Simulation:** Gazebo & NVIDIA Isaac
* **Perception:** Camera/LiDAR & object detection
* **SLAM & Navigation:** VSLAM and Nav2
* **AI Integration:** LLM + Voice → Action pipeline
* **Full Integration:** Deployment & hardware bridging

### Tools & Frameworks

* ROS 2
* Python (rclpy)
* Gazebo & Unity
* NVIDIA Isaac
* OpenAI / VLA components
* Datastores / RAG (later modules)

### Hardware + Software Synergy

You’ll learn optimal use of:

* **GPUs** for AI/vision
* **CPUs** for control loops
* **Simulation** for rapid iteration

## 0.5 — The Architecture of a Humanoid Robot

Think of the humanoid as **layered systems**. This architecture is intentionally modular so components can be simulated, swapped, or scaled independently.

## Layered View (Bottom → Top)

### Mechanical Layer (Body)
- Links, joints, actuators (motors/servos), mechanical constraints, gear reduction  
- Example artifacts: URDF files, CAD/mesh assets, mass/inertia tables  

### Low-Level Control & Firmware
- Motor drivers, real-time controllers (PID, feedforward), safety interlocks  
- Runs at high frequency (100–1000 Hz)  

### Middleware — ROS 2 (Nervous System)
- Message passing, action servers, lifecycle nodes, parameter server, composition  
- Synchronizes and abstracts hardware  

### Perception & Estimation
- Sensor drivers → preprocessing → state estimation (IMU fusion, joint encoders) → scene understanding (object detection)  
- Rates vary: IMU (100–1000 Hz), camera (15–60 Hz), LiDAR (5–20 Hz)  

### Motion Planning & Navigation
- Path planning (global), trajectory generation (local), balance controllers (bipedal), footstep planners  

### Cognitive Planning & Task Execution
- LLMs and symbolic planners → task decomposition → action primitives  
- Higher latency permitted (seconds), but must be robust to execution failures  

### User Interface & Monitoring
- Voice, dashboards, telemetry, failure reporting, human override  

## 0.6 — Key Interfaces and Data Flow (Decision Loop)


**Important considerations:**

- **Latency budgets:** Control loops require deterministic timings; heavy perception and planning must not block safety-critical loops  
- **Message QoS (ROS 2):** tuning reliability and latency for different topics (e.g., sensor streams vs. diagnostic messages)  
- **Separation of concerns:** keep safety-critical code in minimal, verifiable modules  

### The Brain (AI)

- Implements task reasoning, goal decomposition, and policy selection  
- Patterns:
  - **Reactive controllers:** quick responses (obstacle avoidance)  
  - **Deliberative planners:** long-term planning (sequence of tasks)  
  - **Learning components:** perception models, behavior policies learned from data  

**Design tip:** Use hybrid architectures — deterministic controllers for low-level stability, ML components where approximation is acceptable (perception)

### The Nervous System (ROS 2)

ROS 2 provides DDS-based communication, better real-time and cross-platform support.  

**Concepts you must master:** nodes, topics, services, actions, lifecycle nodes, composition, QoS  

**Practical Example — Python (rclpy skeleton):**

```python
# talker.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.pub = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.timer_cb)

    def timer_cb(self):
        msg = String()
        msg.data = 'hello'
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Talker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

# The Body, Sensors & Actuators, and Simulation

This section demonstrates the **ROS 2 publisher pattern** you’ll use as the **core glue between AI and actuators** and describes the physical and simulated components of a humanoid robot.

## The Body (Sensors & Actuators)

### Common Sensors

- **IMU:** orientation and acceleration (used for balance)  
- **Cameras:** monocular / stereo / RGB-D for perception  
- **LiDAR:** dense depth scans for mapping and obstacle detection  
- **Force/Torque sensors:** contact estimation, manipulation safety  
- **Encoders:** precise joint positions  

### Actuators

- **Types:** DC motors with encoders, brushless motors, servo motors, hydraulic actuators (less common for small humanoids)  
- **Control modes:** position, velocity, torque — choose appropriate mode for stability and safety  

## Environment (Simulation)

Simulators allow you to **iterate faster** and **safely test edge-cases**.

- **Gazebo / Ignition:** physics-based simulation, ROS integration  
- **Unity:** HRI and photorealistic scenarios for realistic visuals  
- **NVIDIA Isaac Sim:** GPU-accelerated photorealism and synthetic data pipelines  

### Simulation Uses

- Development and testing of algorithms (e.g., SLAM, controllers)  
- Synthetic dataset generation for training AI and perception models  
- Stress testing under extreme conditions without risking hardware  


## 0.7 — Technology Stack Overview

Below are the tools we use in this book, with recommended versions where relevant and concrete notes you’ll need to integrate them.

:::note Recommended
Base OS: **Ubuntu 22.04 LTS** (stable for ROS 2 Humble/Ion — verify exact ROS distro compatibility with Isaac Sim and your required Gazebo)
:::

### ROS 2 (Middleware)

**Why ROS 2:** DDS-based communication, QoS controls, better real-time and lifecycle management than ROS 1.

**Key components:** ros2 CLI, colcon build tool, rclpy (Python client), rclcpp (C++ client)

:::note Recommended
Distro: **ROS 2 Humble** or the latest LTS compatible with other tools
:::

**Examples / Best Practices:**

- Use **QoS profile** `sensor_data` for camera/LiDAR streams; `best_effort` for high-rate streams where occasional drop is acceptable  
- Use **actions** for long-running tasks (e.g., `move_to_pose`) to support preemption and feedback

### Python (rclpy + Ecosystem)

**Role:** orchestrate AI/ML components, integration glue, and prototyping

**Key packages:** rclpy, numpy, opencv-python, torch or tensorflow, scipy, pyyaml

:::note Recommended
Use **virtualenvs or conda** to isolate project environments
:::

### Gazebo / Ignition (Physics Simulation)

**Use cases:** mid-fidelity physics, multi-robot interaction, sensor emulation

**Integration:** `ros_gz` or `gazebo_ros_pkgs` for bridge to ROS 2 topics/services

**Caveat:** physics parameters (friction, restitution) must be tuned for realistic behavior

### Unity (Rendering & HRI)

**Use cases:** high-fidelity visuals, avatars, human behavior simulation

**Bridge:** `ROS-TCP-Connector` package for Unity to exchange messages with ROS 2

### NVIDIA Isaac (Isaac Sim & Isaac ROS)

**Why:** photorealistic sim and GPU-accelerated perception stacks (useful for synthetic datasets)

**Isaac Sim:** uses Omniverse, requires NVIDIA GPU + drivers and specific CUDA versions

**Isaac ROS:** optimized packages for perception, SLAM, and sensor simulation

### OpenAI + Vision-Language-Action (VLA)

**Components in this course:**

- **Speech recognition:** Whisper (local or hosted)  
- **LLMs for planning:** cognitive planners (local or via API)  
- **Prompt engineering:** structured prompts & Spec-Kit decomposition  
- **Safety:** never allow unfiltered LLM outputs to actuate hardware directly — always validate or simulate

### Computer Vision & SLAM

**VSLAM:** ORB-SLAM variants, RTAB-Map, or deep-learning based SLAM  

**Perception models:** RetinaNet / YOLO / Detectron2, or lightweight models for onboard deployment  

**Training:** synthetic data generation from Isaac Sim / Gazebo / Unity with domain randomization to close sim2real gap



## 0.8 — Learning Path Mastery Planner

Below is a reproducible **12-week plan** with daily tasks, artifacts, and measurable checkpoints.  
Assumes ~10–15 hours/week; adjust pace for full-time, part-time, or team learning.

### Overall Timeline (12 Weeks)

### Weeks 1–2: Foundations & ROS 2 Basics

**Goals:** Set up a working ROS 2 environment and implement first nodes.  

**Deliverables:**

- ROS 2 installed and `ros_ws` workspace created  
- Implemented `talker/listener` nodes in `rclpy`  
- Basic URDF (one-link manipulator) visualized in RViz  

**Daily Tasks (Example Week 1):**

| Day | Task |
|-----|------|
| 1   | Install Ubuntu, set up git, Python venv |
| 2   | Install ROS 2 (Humble or chosen distro), run tutorials |
| 3   | Create workspace, build sample `demo_nodes_py` |
| 4   | Write first rclpy talker/listener |
| 5   | Create basic URDF, visualize in RViz |
| Weekend | Document setup and test in a fresh VM |

### Weeks 3–4: URDF, Actuators & Control

**Goals:** Humanoid skeleton, kinematics + simulation-ready URDF  

**Deliverables:**

- URDF for torso + one arm (links/joints/inertias)  
- Joint state publisher + trajectory controller demo in Gazebo  
- Small report: inertia calculation and joint limits  

**Key Tasks:**

- Model link geometry and inertial properties  
- Add sensor placeholders (camera / IMU)  
- Launch Gazebo, spawn robot, test joint control  

### Weeks 5–6: Simulation & Sensor Pipelines

**Goals:** Simulate sensors in Gazebo + basic SLAM  

**Deliverables:**

- Gazebo world with obstacles  
- Simulated LiDAR + camera nodes publishing to ROS topics  
- Simple SLAM map generation  

**Tasks:**

- Implement and tune physics parameters  
- Record sensor data (bag files) for reference  
- Explore RViz visualization of sensor streams  

### Weeks 7–8: Perception & Nav2

**Goals:** Integrate perception pipelines with Nav2  

**Deliverables:**

- Object detection in simulation (pretrained model)  
- Nav2 configured for humanoid (footstep planner or simplified diff drive)  
- Autonomy demo: go-to-goal in a simulated room  

**Tasks:**

- Integrate detection model (GPU if available)  
- Create costmaps, tune inflation radius and local planner parameters  
- Evaluate success rate across multiple randomized starts  

### Weeks 9–10: Isaac Sim & Synthetic Data

**Goals:** Generate synthetic datasets and fine-tune models  

**Deliverables:**

- Isaac Sim scene producing annotated images  
- Trained or fine-tuned detection model evaluated on held-out sim set  

**Tasks:**

- Create scripted camera sweeps  
- Apply domain randomization: lighting, textures, object positions  
- Record dataset metadata and random seeds  

### Weeks 11–12: Vision-Language-Action (VLA) & Capstone

**Goals:** Integrate speech → LLM planning → ROS action execution  

**Deliverables:**

- Voice → plan pipeline producing validated ROS action sequences  
- Capstone demo: simulated humanoid executes “Bring the red cup” end-to-end  

**Tasks:**

- Build full pipeline: audio → transcription → entity extraction → plan → action execution  
- Introduce rehearsal/failsafe: plan validation, pre/postconditions  
- Run robustness tests (noisy audio, occluded object)  

### Milestones & Success Criteria

| Milestone | Success Criteria |
|-----------|----------------|
| 1 | ROS 2 nodes publish/subscribe success rate > 99% locally |
| 2 | URDF visualized without joint errors; joint control achieves ±2° accuracy |
| 3 | Nav2 path planning success ≥ 80% over random seeds (4×4 room) |
| 4 | Object detection precision@0.5 ≥ 0.7 on synthetic validation set |
| Capstone | End-to-end task success (voice → pick → place) ≥ 70% across 20 trials |

### Daily Learning Structure

* Monday: Study
* Tuesday: Code
* Wednesday: Lab
* Thursday: Debug
* Friday: Write notes
* Weekend: Experiments


## 0.9 — Installation & System Requirements

### Recommended Hardware

* **CPU:** i5 / Ryzen 5 +
* **GPU:** RTX 2060+ (8GB minimum)
* **RAM:** 16–32 GB
* **Storage:** 250GB–1TB SSD
* **OS:** Ubuntu 22.04 LTS (Recommended)

### Essential Software

* git, curl
* python3 + venv
* ROS 2 + colcon
* Gazebo / Isaac
* Docker
* NVIDIA drivers + CUDA

### Example Commands

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y git curl build-essential python3-pip python3-venv

python3 -m venv venv
source venv/bin/activate
pip install -U pip setuptools wheel
pip install -U colcon-common-extensions
```
## 0.10 — How to Use This Book Effectively

This section gives explicit patterns and rules to follow depending on your **role** and **goals**.

### For Students (Step-by-Step Path)

If you are learning this as a student or beginner, follow this structured approach:

- **Follow the mastery planner**  
  Complete the weekly milestones in the given order. Do not jump ahead.

- **Run every lab**  
  Never skip simulations or experiments. Labs are reproducibility checks.

- **Log outcomes**  
  Maintain a learning diary. Every experiment should include:
  - Commands used
  - Input data / seed values
  - Hyperparameters
  - Results (success / failure / notes)

- **Ask for feedback**  
  If you get stuck, share a **minimal reproducible example** in forums or your study group.

Goal: *Master concepts first, then optimize performance.*

### For Developers (Integration Mindset)

If you are building real systems or products, follow software engineering discipline:

- **Modularize everything**  
  Write ROS nodes that do one job and expose clean interfaces:
  - Topics
  - Services
  - Actions

- **Version control**  
  Use `git`, create release tags, and generate Docker images with:
  - Pinned versions
  - Reproducible builds

- **Continuous Integration (CI)**  
  Add GitHub Actions to:
  - Run `flake8` (Python linting)
  - Run ROS unit tests
  - Build Docker image automatically

Goal: *Build scalable, reusable, and maintainable robotics software.*

### For Researchers (Reproducibility Mindset)

If you are doing academic or experimental work, focus on reproducibility:

- **Preserve artifacts**
  - Datasets
  - Config files
  - Random seed lists
  - Raw logs

- **Define clear metrics**
  - Success rate
  - Latency
  - Energy usage
  - Precision / Accuracy (where applicable)

- **Promote open science**
  - Use Zenodo / GitHub Releases
  - Attach datasets and experiment configs

Goal: *Anyone should be able to reproduce your results.*

### For Self-Learners (Efficiency Mode)

If you are learning independently:

- **Start small**
  First master:
  - ROS talker / listener
  Then go to:
  - URDF
  - Simulation
  - AI / perception

- **Use prebuilt images**
  Use official ROS / Isaac / Docker images to save setup time.

- **Set strict constraints**
  Avoid scope creep:
  - One feature at a time
  - One week = one goal

Goal: *Learn fast, not perfect.*

### For Teams (Coordination & Process)

If you are working in a team environment:

- **Assign clear roles**
  - Mapping
  - Perception
  - Control
  - Integration
  - Documentation

- **Use an integration checklist**
  Before merging:
  - All launch files run
  - No topic conflicts
  - All modules pass smoke test

- **Keep documentation alive**
  Maintain:
  - Architecture diagrams
  - ROS API specs (topics / services / actions)

Goal: *Reduce confusion and increase team velocity.*

### Final Advice

No matter your role:

> **Start small → Build consistently → Test often → Document everything → Share your progress**

That is the true workflow of professionals in Physical AI and Humanoid Robotics.


### Pre-Start Checklist

* Ubuntu / WSL ready
* Python venv set
* ROS2 installed
* colcon workspace created
* Gazebo / Isaac working
* NVIDIA drivers installed
* Git repo initialized

### Before Capstone

* URDF loads in RViz
* Gazebo robot spawn
* SLAM + Perception working
* Voice → Plan → Execute pipeline functional

**Capstone Goal:**
Humanoid receives a **voice command** and performs a **pick & place task** autonomously in simulation.

</div>

<div className="urdu-content">



# باب 0 — فزیکل AI اور ہیومنوائڈ روبوٹکس کا تعارف

## 0.1 — فزیکل AI کیا ہے؟

فزیکل AI وہ میدان ہے جو مصنوعی ذہانت کو جسمانی نظاموں کے ساتھ جوڑتا ہے — روبوٹس جو حقیقی دنیا میں محسوس کرتے ہیں، سوچتے ہیں، اور کام کرتے ہیں۔ محض ورچوئل AI (بڑے لینگویج ماڈلز، سفارشی نظام) کے برعکس، فزیکل AI کو حقیقی دنیا کی پابندیوں کا سامنا کرنا پڑتا ہے: قوت، مٹان، تاخیر، شور میں محسوس کرنے والے سینسرز، اور حفاظت۔

اس کے لیے ضروری ہے:

* ہارڈویئر
* ایمبیڈڈ سسٹم
* کنٹرول تھیوری
* ادراک
* منصوبہ بندی
* اعلیٰ درجے کی سوچ

### کلیدی خصوصیات

* **ایمبوڈمنٹ:** جسمانی جسم کے ذریعے ظاہر ہونے والی ذہانت جس کے پاس ڈائنامکس اور پابندیاں ہیں۔
* **سخت ادراک-ایکشن لوپ:** محسوس کرنا → سوچنا → کام کرنا (مسلسل اور تاخیر-سے-آگاہ)
* **کثیر الجہتی اسٹیک:** میکانیکل ڈیزائن، کنٹرول سسٹم، مڈل ویئر (مثلاً ROS 2)، سیمولیشن، اور مشین لرننگ۔

## 0.2 — ہیومنوائڈ روبوٹ کیا ہے؟

ہیومنوائڈ روبوٹ ایک ایسا روبوٹ ہے جس کی کنیمیٹکس اور سینسر کی ترتیب انسانی شکل کی نقل کرتی ہے (سر، ٹورسو، دو بازو، دو ٹانگیں)۔

ہیومنوائڈز کا مطالعہ اور تعمیر کی جاتی ہے:

* **انسانی ماحول میں کام کرنے کے لیے** (سیڑھیاں، میزیں، نوبلز)
* **لوگوں کے ساتھ قدرتی طور پر بات چیت کرنے کے لیے** (سماجی روبوٹکس)
* **ایمبوڈیڈ انٹیلی جنس کی جانچ کے لیے** انسانی شکل والے ماحول میں

### کلیدی چیلنجز

* ڈائنا مک بیلنس اور بائی پیڈل لوکوموشن
* ملٹی-جوائنٹ کوآرڈی نیشن
* پُر چڑھی ہوئی منظروں میں مینیپولیشن
* اعلیٰ درجے کی منطق کو تیز کم درجے کے کنٹرول کے ساتھ انضمام دینا

## 0.3 — کیوں یہ شعبہ مستقبل کے لیے ہے

فزیکل AI اور ہیومنوائڈز AI کو جسمانی دنیا میں لا کرتے ہیں جہاں سب سے بڑا معاشی اور سماجی اثر ہو سکتا ہے:

* خطرناک یا اکیلے کاموں کی خودکاری
* بوڑھے آبادی کے لیے گھر میں مددگار روبوٹس
* مینوفیکچرنگ میں انسان-روبوٹ کولیبoration
* اسپیس اور ایکسپلوریشن میں روبوٹکس
* ٹیلی پریزنٹس اور مکسڈ ریلٹی کے لیے نئے انٹرفیسز

کے شعبوں میں پیشرفت:

* سیمولیشن
* GPU-تیز کردہ ادراک
* بڑے لینگویج ماڈلز (LLMs)

…حقیقی دنیا کی ایپلیکیشنز کو تیز کر رہے ہیں۔
**یہ کتاب آپ کو ان پیشروں کو کام کرنے والے ہیومنوائڈ سسٹم میں ضم کرنے کا طریقہ سکھاتی ہے۔**

### حقیقی دنیا کی ایپلی کیشنز

* **صنعت:** ایسیمبلي لائنز پر انسان-مددگار روبوٹس
* **صحت کی دیکھ بھال:** روبوٹک ریہیبیلی ٹیشن اسسٹنٹس
* **اسپیس اور ایکسپلوریشن:** صفر گریویٹی میں انسان نما مینیپولیٹرز
* **دفاع اور ہنگامی:** خطرناک ماحول میں روبوٹس
* **گھر اور سروس:** گھریلو مدد اور دیکھ بھال
* **تحقیق اور تعلیم:** ایمبوڈیڈ کوگنیشن اور HRI کے لیے ٹیسٹ بیڈز

## 0.4 — آپ اس کتاب میں کیا سیکھیں گے

یہ کورس پروجیکٹ اور سیمولیشن-پر مبنی ہے۔ ختم ہونے پر، آپ ایک **آواز-چلنے والا ہیومنوائڈ ورک فلو** تیار کریں گے جو:

*   ادراک کرتا ہے
*   منصوبہ بندی کرتا ہے
*   نیویگیٹ کرتا ہے
*   مینیپولیٹ کرتا ہے

### مہارتوں اور ٹیکنالوجیز کا احاطہ

* **مڈل ویئر اور کنٹرول:** ROS 2 (نوڈز، ٹاپکس، سروسز، ایکشنز)
* **ماڈلنگ اور کنیمیٹکس:** URDF/SDF روبوٹ کی تفصیل
* **سیمولیشن:** Gazebo اور NVIDIA Isaac
* **ادراک:** کیمرہ/لیڈار اور اوبجیکٹ ڈیٹیکشن
* **SLAM اور نیویگیشن:** VSLAM اور Nav2
* **AI انضمام:** LLM + آواز → ایکشن پائپ لائن
* **مکمل انضمام:** ڈیپلائمنٹ اور ہارڈویئر برجنگ

### ٹولز اور فریم ورکس

* ROS 2
* Python (rclpy)
* Gazebo اور Unity
* NVIDIA Isaac
* OpenAI / VLA اجزاء
* ڈیٹا سٹورز / RAG (بعد کے ماڈیولز)

### ہارڈویئر + سافٹ ویئر کا ہم آہنگ

آپ سیکھیں گے کہ کیا استعمال کرنا ہے:

* **GPUs** AI/ویژن کے لیے
* **CPUs** کنٹرول لوپس کے لیے
* **سیمولیشن** تیز تکرار کے لیے

## 0.5 — ہیومنوائڈ روبوٹ کی معماری

ہیومنوائڈ کو **تہوں والے سسٹم** کے طور پر سوچیں۔ یہ معماری منسلک طور پر ماڈیولر ہے تاکہ اجزاء کو سیمولیٹ، سویپ، یا سکیل کیا جا سکے۔

### تہوں کا نظارہ (نیچے سے اوپر)

#### میکانیکل تہ (جسم)

- لنکس، جوائنٹس، ایکچوایٹرز (موٹرز/سرووس)، میکانیکل پابندیاں، گیئر ریڈکشن
- مثال کے مظاہر: URDF فائلز، CAD/میش اثاثے، ماس/انیشیا ٹیبلز

#### کم درجے کا کنٹرول اور فرم ویئر

- موٹر ڈرائیورز، ریل ٹائم کنٹرولرز (PID، فیڈ فارورڈ)، حفاظتی انٹر لاکس
- 100–1000 Hz پر چلتا ہے

#### مڈل ویئر — ROS 2 (نروس سسٹم)

- میسج پاسنگ، ایکشن سرورز، لائف سائیکل نوڈز، پیرامیٹر سرور، کمپوزیشن
- ہارڈویئر کو مطابقت اور ایبسٹریکٹ کرتا ہے

#### ادراک اور اسٹیمیشن

- سینسر ڈرائیورز → پری-پروسیسنگ → اسٹیٹ اسٹیمیشن (IMU فیوژن، جوائنٹ انکوڈرز) → منظر کی سمجھ (اوبجیکٹ ڈیٹیکشن)
- شرحیں مختلف ہیں: IMU (100–1000 Hz)، کیمرہ (15–60 Hz)، LiDAR (5–20 Hz)

#### موشن پلاننگ اور نیویگیشن

- راستہ منصوبہ بندی (گلوبل)، ٹریجکٹری جنریشن (مقامی)، بیلنس کنٹرولرز (بائی پیڈل)، فوٹ سٹیپ پلانرز

#### کوگنیٹو پلاننگ اور ٹاسک ایگزیکیوشن

- LLMs اور علامتی پلانرز → ٹاسک ڈیکومپوزیشن → ایکشن پرائمریز
- زیادہ تاخیر کی اجازت ہے (سیکنڈز)، لیکن ایگزیکیوشن ناکامیوں کے خلاف مضبوط ہونا چاہیے

#### صارف انٹرفیس اور مانیٹرنگ

- آواز، ڈیش بورڈز، ٹیلی میٹری، ناکامی کی اطلاع، انسانی اوور رائیڈ

## 0.6 — کلیدی انٹرفیسز اور ڈیٹا فلو (فیصلہ لوپ)

**اہم امور:**

- **تاخیر کے بجٹ:** کنٹرول لوپس کو متعین طور پر ٹائم کی ضرورت ہوتی ہے؛ بھاری ادراک اور منصوبہ بندی کو حفاظتی طور پر لوپس کو بلاک نہیں کرنا چاہیے
- **میسج QoS (ROS 2):** مختلف ٹاپکس کے لیے قابلِ اعتمادی اور تاخیر کو ٹیون کریں (مثلاً، سینسر سٹریمز vs. ڈائیگنوسٹک میسجس)
- **مسئلے کے لحاظ سے الگ الگ:** حفاظتی طور پر کریکل کوڈ کو کم سے کم، تصدیق شدہ ماڈیولز میں رکھیں

### دماغ (AI)

- ٹاسک ریزننگ، گوئل ڈیکومپوزیشن، اور پالیسی سلیکشن کو نافذ کرتا ہے
- پیٹرنز:
  - **ری ایکٹیو کنٹرولرز:** تیز جوابات (رکاوٹ سے بچاؤ)
  - **ڈیلیبریٹیو پلینرز:** طویل مدتی منصوبہ بندی (کاموں کی ترتیب)
  - **سیکھنے والے اجزاء:** ڈیٹا سے سیکھے گئے ادراک ماڈلز، برتاؤ کی پالیسیز

**ڈیزائن کا مشورہ:** ہائبرڈ معماریاں استعمال کریں — کم درجے کی مستحکمیت کے لیے متعین کنٹرولرز، تقریب کے قابل تخمینہ کے لیے ML اجزاء (ادراک)

### نروس سسٹم (ROS 2)

ROS 2 DDS-مبنی مواصلات فراہم کرتا ہے، بہتر ریل ٹائم اور کراس-پلیٹ فارم حمایت۔

**تصورات جو آپ کو ماسٹر کرنا چاہیے:** نوڈز، ٹاپکس، سروسز، ایکشنز، لائف سائیکل نوڈز، کمپوزیشن، QoS

**عملی مثال — Python (rclpy سکلیٹن):**

```python
# talker.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.pub = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.timer_cb)

    def timer_cb(self):
        msg = String()
        msg.data = 'hello'
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Talker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

# جسم، حواس اور ایکچو ایٹرز، اور سیمولیشن

یہ حصہ **ROS 2 پبلشر پیٹرن** کو ظاہر کرتا ہے جسے آپ **AI اور ایکچو ایٹرز کے درمیان کور گلو کے طور پر استعمال کریں گے** اور ہیومنوائڈ روبوٹ کے جسمانی اور تصوراتی اجزاء کی وضاحت کرتا ہے۔

## جسم (سینسرز اور ایکچو ایٹرز)

### عام سینسرز

- **IMU:** جہت اور تسخیر (توازن کے لیے استعمال ہوتا ہے)
- **کیمرے:** مونوکل / سٹیریو / RGB-D ادراک کے لیے
- **LiDAR:** نقشہ کشی اور رکاوٹ ڈیٹیکشن کے لیے گھنے ڈیپتھ اسکینز
- **فورس/ٹورک سینسرز:** کانٹیکٹ کا تخمینہ، مینیپولیشن کی حفاظت
- **انکوڈرز:** درست جوائنٹ پوزیشنز

### ایکچو ایٹرز

- **اقسام:** DC موٹرز انکوڈرز کے ساتھ، برش لیس موٹرز، سرو موٹرز، ہائیڈرولک ایکچو ایٹرز (چھوٹے ہیومنوائڈز کے لیے کم عام)
- **کنٹرول موڈز:** پوزیشن، ویلوسٹی، ٹورک — استحکام اور حفاظت کے لیے مناسب موڈ منتخب کریں

## ماحول (سیمولیشن)

سیمولیٹرز آپ کو **تیزی سے دہرانے** اور **کنارے کے معاملات کو محفوظ طریقے سے ٹیسٹ کرنے** کی اجازت دیتے ہیں۔

- **Gazebo / Ignition:** فزکس-مبنی سیمولیشن، ROS انضمام
- **Unity:** HRI اور فوٹو ریل سٹک منظرناموں کے لیے حقیقی وضاحت
- **NVIDIA Isaac Sim:** GPU-تیز کردہ فوٹو ریل سٹم اور مصنوعی ڈیٹا پائپ لائنز

### سیمولیشن کے استعمالات

- الگورتھم کی ترقی اور ٹیسٹنگ (مثلاً SLAM، کنٹرولرز)
- AI اور ادراک ماڈلز کی تربیت کے لیے مصنوعی ڈیٹا سیٹ تیار کرنا
- ایکسٹریم حالت میں کام کرنے کا جائزہ جو ہارڈویئر کے خطرے کے بغیر

## 0.7 — ٹیکنالوجی اسٹیک کا جائزہ

ذیل میں ہم اس کتاب میں استعمال ہونے والے ٹولز ہیں، جہاں متعلقہ تجویز کردہ ورژن اور مکمل نوٹس ہیں جن کی آپ کو ضرورت ہوگی کہ انہیں انضمام دیں۔

:::note تجویز کردہ
بیس OS: **Ubuntu 22.04 LTS** (ROS 2 Humble/Ion کے لیے مستحکم — Isaac Sim اور آپ کے مطلوبہ Gazebo کے ساتھ مطابقت کی تصدیق کریں)
:::

### ROS 2 (مڈل ویئر)

**ROS 2 کیوں:** DDS-مبنی مواصلات، QoS کنٹرولز، ROS 1 کے مقابلے میں بہتر ریل ٹائم اور لائف سائیکل مینجمنٹ۔

**اہم اجزاء:** ros2 CLI، colcon بِلڈ ٹول، rclpy (Python کلائنٹ)، rclcpp (C++ کلائنٹ)

:::note تجویز کردہ
Distro: **ROS 2 Humble** یا دیگر ٹولز کے ساتھ مطابقت کا حالیہ LTS
:::

**مثالیں / بہترین مشقیں:**

- **QoS پروفائل** `sensor_data` کیمرہ/LiDAR سٹریمز کے لیے استعمال کریں؛ `best_effort` ایسے ہائی-ریٹ سٹریمز کے لیے جہاں جزوی ڈراپ قابلِ قبول ہے
- **ایکشنز** طویل چلنے والے کاموں (مثلاً `move_to_pose`) کے لیے استعمال کریں تاکہ پریمپشن اور فیڈ بیک کی حمایت ہو سکے

### Python (rclpy + ایکو سسٹم)

**کردار:** AI/ML اجزاء کو آرکیسٹریٹ کرنا، انضمام گلو، اور پروٹو ٹائپنگ

**اہم پیکجز:** rclpy، numpy، opencv-python، torch یا tensorflow، scipy، pyyaml

:::note تجویز کردہ
**virtualenvs یا conda** کا استعمال منصوبہ جاتی ماحول کو الگ کرنے کے لیے کریں
:::

### Gazebo / Ignition (فزکس سیمولیشن)

**استعمال کے مواقع:** درمیانی وفاداری والی فزکس، متعدد روبوٹ انٹر ایکشن، سینسر ایمولیشن

**انضمام:** `ros_gz` یا `gazebo_ros_pkgs` کا استعمال ROS 2 ٹاپکس/سروسز کے ساتھ برج کے لیے

**خبردار:** فزکس پیرامیٹرز (مٹان، ریسٹی ٹوشن) کو حقیقی رویے کے لیے ٹیون کرنا ضروری ہے

### Unity (رینڈرنگ اور HRI)

**استعمال کے مواقع:** بلند وفاداری والی وضاحت، اوتارز، انسانی رویہ کی سیمولیشن

**برج:** `ROS-TCP-Connector` پیکج یونٹی سے ROS 2 کے ساتھ میسجس کے تبادلے کے لیے

### NVIDIA Isaac (Isaac Sim اور Isaac ROS)

**کیوں:** فوٹو ریل سٹک سیمولیشن اور GPU-تیز کردہ ادراک سٹیکس (سینٹھیٹک ڈیٹا کے لیے مفید)

**Isaac Sim:** Omniverse استعمال کرتا ہے، NVIDIA GPU + ڈرائیورز اور مخصوص CUDA ورژن کی ضرورت ہے

**Isaac ROS:** ادراک، SLAM، اور سینسر سیمولیشن کے لیے بہتر کردہ پیکجز

### OpenAI + ویژن-لینگویج-ایکشن (VLA)

**اس کورس میں اجزاء:**

- **تقریر کی پہچان:** Whisper (مقامی یا میزبان)
- **LLMs منصوبہ بندی کے لیے:** کوگنیٹو پلینرز (مقامی یا API کے ذریعے)
- **پروموٹ انجینئرنگ:** ساختہ پروموٹس اور Spec-Kit ڈیکومپوزیشن
- **حفاظت:** کبھی بھی غیر فلٹرڈ LLM آؤٹ پٹس کو ہارڈویئر کو براہ راست ایکچو ایٹ کرنے کی اجازت نہ دیں — ہمیشہ تصدیق کریں یا سیمولیٹ کریں

### کمپیوٹر ویژن اور SLAM

**VSLAM:** ORB-SLAM ویرینٹس، RTAB-Map، یا ڈیپ لرننگ مبنی SLAM

**ادراک ماڈلز:** RetinaNet / YOLO / Detectron2، یا بورڈ پر ڈیپلائمنٹ کے لیے ہلکے ماڈلز

**تربیت:** Isaac Sim / Gazebo / Unity سے سینٹھیٹک ڈیٹا جنریشن، ڈومین رینڈمائزیشن کے ساتھ sim2real گیپ کو بند کرنے کے لیے

## 0.8 — سیکھنے کا راستہ ماسٹری پلانر

ذیل میں ایک قابلِ دہرائی **12 ہفتے کا منصوبہ** ہے جس میں روزانہ کے کام، مظاہر، اور قابلِ پیمائش چیک پوائنٹس ہیں۔
10–15 گھنٹے/ہفتہ کا فرض کرتا ہے؛ فل ٹائم، پارٹ ٹائم، یا ٹیم لرننگ کے لیے رفتار کو ایڈجسٹ کریں۔

### مجموعی ٹائم لائن (12 ہفتے)

### ہفتے 1–2: بنیادیں اور ROS 2 بنیادیات

**اہداف:** ایک کام کرتا ہوا ROS 2 ماحول سیٹ اپ کریں اور پہلے نوڈز نافذ کریں۔

** deliverables:**

- ROS 2 انسٹال ہے اور `ros_ws` ورک اسپیس بنائی گئی ہے
- `talker/listener` نوڈز `rclpy` میں نافذ کیے گئے ہیں
- بنیادی URDF (ایک لنک مینیپولیٹر) RViz میں دکھایا گیا ہے

**روزانہ کے کام (مثال ہفتہ 1):**

| دن | کام |
|-----|------|
| 1 | Ubuntu انسٹال کریں، git سیٹ اپ کریں، Python venv |
| 2 | ROS 2 (Humble یا منتخب کردہ ڈسٹرو) انسٹال کریں، ٹیوٹوریل چلائیں |
| 3 | ورک اسپیس بنائیں، نمونہ `demo_nodes_py` بنائیں |
| 4 | پہلا rclpy ٹاکر/لیسنر لکھیں |
| 5 | بنیادی URDF بنائیں، RViz میں ویژولائز کریں |
| آخر ہفتہ | سیٹ اپ کو دستاویز کریں اور تازہ VM میں ٹیسٹ کریں |

### ہفتے 3–4: URDF، ایکچو ایٹرز اور کنٹرول

**اہداف:** ہیومنوائڈ سکلٹن، کنیمیٹکس + سیمولیشن-تیار URDF

** deliverables:**

- ٹورسو + ایک بازو کے لیے URDF (لنکس/جوائنٹس/انیشیا)
- Gazebo میں جوائنٹ اسٹیٹ پبلشر + ٹریجکٹری کنٹرولر ڈیمو
- چھوٹی رپورٹ: انیشیا کا حساب اور جوائنٹ حدود

**اہم کام:**

- لنک جیومیٹری اور انیشیل خصوصیات کو ماڈل کریں
- سینسر پلیس ہولڈرز شامل کریں (کیمرہ / IMU)
- Gazebo لانچ کریں، روبوٹ اسپون کریں، جوائنٹ کنٹرول ٹیسٹ کریں

### ہفتے 5–6: سیمولیشن اور سینسر پائپ لائنز

**اہداف:** Gazebo میں سینسرز کی سیمولیشن + بنیادی SLAM

** deliverables:**

- رکاوٹوں کے ساتھ Gazebo دنیا
- ROS ٹاپکس پر پبلش کرنے والے سیمولیٹڈ LiDAR + کیمرہ نوڈز
- سادہ SLAM نقشہ تیار کرنا

**کام:**

- فزکس پیرامیٹرز کو نافذ کریں اور ٹیون کریں
- حوالہ کے لیے سینسر ڈیٹا (bag files) ریکارڈ کریں
- سینسر سٹریمز کی RViz ویژولائزیشن کا جائزہ لیں

### ہفتے 7–8: ادراک اور Nav2

**اہداف:** ادراک پائپ لائنز کو Nav2 کے ساتھ انضمام دینا

** deliverables:**

- سیمولیشن میں اوبجیکٹ ڈیٹیکشن (پری ٹرینڈ ماڈل)
- ہیومنوائڈ کے لیے Nav2 کنفیگر (فوٹ سٹیپ پلینر یا سادہ diff drive)
- خود مختار ڈیمو: ایک سیمولیٹڈ کمرے میں جانے کا ہدف

**کام:**

- ڈیٹیکشن ماڈل انضمام دیں (اگر دستیاب ہو تو GPU)
- کوسٹ میپس بنائیں، انفلیشن ریڈیس اور مقامی پلینر پیرامیٹرز ٹیون کریں
- متعدد رینڈمائز شروعاتوں پر کامیابی کی شرح کا جائزہ لیں

### ہفتے 9–10: Isaac Sim اور سینٹھیٹک ڈیٹا

**اہداف:** سینٹھیٹک ڈیٹا سیٹس تیار کریں اور ماڈلز کو فائن ٹیون کریں

** deliverables:**

- Isaac Sim منظر جو اینوٹیٹڈ امیجز پیدا کرتا ہے
- ہولڈ-آؤٹ سیم سیٹ پر جائزہ لیا گیا ٹرینڈ یا فائن ٹیونڈ ڈیٹیکشن ماڈل

**کام:**

- اسکرپٹڈ کیمرہ سویپس بنائیں
- ڈومین رینڈمائزیشن لاگو کریں: لائٹنگ، ٹیکسچرز، اشیاء کی پوزیشنز
- ڈیٹا سیٹ میٹا ڈیٹا اور رینڈم سیڈز ریکارڈ کریں

### ہفتے 11–12: ویژن-لینگویج-ایکشن (VLA) اور کیپسٹون

**اہداف:** گفتگو → LLM منصوبہ بندی → ROS ایکشن ایگزیکیوشن کو انضمام دینا

** deliverables:**

- وائس → منصوبہ پائپ لائن جو جائز ROS ایکشن ترتیبات پیدا کرتا ہے
- کیپسٹون ڈیمو: سیمولیٹڈ ہیومنوائڈ "لال کپ لائیں" کو آخر تک انجام دیتا ہے

**کام:**

- مکمل پائپ لائن بنائیں: آڈیو → ٹرانسکرپشن → ایںٹیٹی ایکسٹریکشن → منصوبہ → ایکشن ایگزیکیوشن
- ریہرسل/فیل سیف متعارف کرائیں: منصوبہ کی توثیق، پری/پوسٹ کنڈیشنز
- مزاحمت کے ٹیسٹ چلائیں (شوروی آڈیو، اوکلڈیڈ اوبجیکٹ)

### مائل سٹونز اور کامیابی کا معیار

| مائل سٹون | کامیابی کا معیار |
|-----------|----------------|
| 1 | ROS 2 نوڈز کا پبلش/سبسکرائب کامیابی کی شرح > 99% مقامی طور پر |
| 2 | URDF بغیر جوائنٹ خرابیوں کے ویژولائز ہوتا ہے؛ جوائنٹ کنٹرول ±2° درستی حاصل کرتا ہے |
| 3 | Nav2 راستہ منصوبہ بندی کامیابی ≥ 80% (4×4 کمرہ) رینڈم سیڈز پر |
| 4 | اوبجیکٹ ڈیٹیکشن پریسیژن@0.5 ≥ 0.7 سینٹھیٹک تصدیقی سیٹ پر |
| کیپسٹون | آخر سے آخر تک ٹاسک کامیابی (آواز → اٹھائیں → جگہ دیں) 20 ٹرائلز پر ≥ 70% |

### روزانہ کے سیکھنے کا ڈھانچہ

* سوموار: مطالعہ
* منگل: کوڈ
* بدھ: لیب
* جمعرات: ڈیبگ
* جمعہ: نوٹس لکھیں
* آخر ہفتہ: تجربات

## 0.9 — انسٹالیشن اور سسٹم کی ضروریات

### تجویز کردہ ہارڈویئر

* **CPU:** i5 / Ryzen 5 +
* **GPU:** RTX 2060+ (8GB کم از کم)
* **RAM:** 16–32 GB
* **اسٹوریج:** 250GB–1TB SSD
* **OS:** Ubuntu 22.04 LTS (تجویز کردہ)

### ضروری سافٹ ویئر

* git, curl
* python3 + venv
* ROS 2 + colcon
* Gazebo / Isaac
* Docker
* NVIDIA ڈرائیورز + CUDA

### مثالی کمانڈز

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y git curl build-essential python3-pip python3-venv

python3 -m venv venv
source venv/bin/activate
pip install -U pip setuptools wheel
pip install -U colcon-common-extensions
```

### 0.10 — اس کتاب کو مؤثر طریقے سے استعمال کیسے کریں

یہ حصہ آپ کے **کردار** اور **مقاصد** کے لحاظ سے عمل کرنے کے لیے واضح پیٹرنز اور قواعد فراہم کرتا ہے۔

### طلباء کے لیے (مرحلہ وار راستہ)

اگر آپ اسے طالب علم یا شروع کرنے والے کے طور پر سیکھ رہے ہیں، تو اس ساختہ نقطہ نظر پر عمل کریں:

- **ماسٹری پلینر کو فالو کریں**
  دیے گئے ترتیب میں ہفتہ وار مائل سٹونز مکمل کریں۔ آگے نہ کودیں۔

- **ہر لیب چلائیں**
  کبھی بھی سیمولیشنز یا تجربات کو چھوڑیں۔ لیب دہرائے جانے کے چیکس ہیں۔

- **نیتجے لاگ کریں**
  ایک سیکھنے والے کا ڈائری برقرار رکھیں۔ ہر تجربہ میں شامل ہونا چاہیے:
  - استعمال کردہ کمانڈز
  - ان پٹ ڈیٹا / سیڈ ویلیوز
  - ہائپر پیرامیٹرز
  - نتائج (کامیابی / ناکامی / نوٹس)

- **فیڈ بیک کے لیے کہیں**
  اگر آپ پھنس جائیں تو، فورمز یا آپ کے مطالعہ گروپ میں ایک **کم از کم قابلِ دہرائی مثال** شیئر کریں۔

هدف: *پہلے تصورات کو ماسٹر کریں، پھر کارکردگی کو بہتر بنائیں۔*

### ڈیولپرز کے لیے (انضمام کا ذہن)

اگر آپ حقیقی سسٹم یا مصنوعات تیار کر رہے ہیں، تو سافٹ ویئر انجینئرنگ کی纪律 کو فالو کریں:

- **ہر چیز کو ماڈیولرائز کریں**
  وہ ROS نوڈز لکھیں جو ایک کام کریں اور صاف انٹرفیسز کو بے نقاب کریں:
  - ٹاپکس
  - سروسز
  - ایکشنز

- **ورژن کنٹرول**
  `git` استعمال کریں، ریلیز ٹیگس بنائیں، اور ڈاکر امیجز بنائیں:
  - پن کردہ ورژن
  - قابلِ دہرائی بلڈس

- **مسلسل انضمام (CI)**
  GitHub ایکشنز شامل کریں تاکہ:
  - `flake8` چلائیں (Python لائنٹنگ)
  - ROS یونٹ ٹیسٹس چلائیں
  - ڈاکر امیج خود بخود بنائیں

مقصد: *.scalable, reusable, and maintainable robotics software* بنائیں۔

### تحقیق کاروں کے لیے (قابلِ دہرائی کا ذہن)

اگر آپ اکیڈمک یا تجرباتی کام کر رہے ہیں، تو قابلِ دہرائی پر توجہ مرکوز کریں:

- ** artefacts کو محفوظ کریں**
  - ڈیٹا سیٹس
  - کنفیگ فائلز
  - رینڈم سیڈ لسٹس
  - خام لاگز

- **واضح میٹرکس کی وضاحت کریں**
  - کامیابی کی شرح
  - تاخیر
  - توانائی کا استعمال
  - درستگی / درستی (جہاں قابلِ اطلاق)

- **کھلا سائنس فروغ دیں**
  - Zenodo / GitHub ریلیز استعمال کریں
  - ڈیٹا سیٹس اور تجربہ کنفیگس منسلک کریں

مقصد: *کوئی بھی شخص آپ کے نتائج کو دہرا سکے۔*

### خود سیکھنے والوں کے لیے (کارکردگی کا موڈ)

اگر آپ آزادانہ طور پر سیکھ رہے ہیں:

- **چھوٹے سے شروع کریں**
  پہلے ماسٹر کریں:
  - ROS ٹاکر / لیسنر
  پھر جائیں:
  - URDF
  - سیمولیشن
  - AI / ادراک

- **پری بلٹ امیجز استعمال کریں**
  سیٹ اپ کا وقت بچانے کے لیے سرکاری ROS / Isaac / Docker امیجز استعمال کریں۔

- **سخت پابندیاں مقرر کریں**
  دائرہ کار کو بڑھنے سے بچیں:
  - ایک فیچر ایک وقت میں
  - ایک ہفتہ = ایک ہدف

مقصد: *تیزی سے سیکھیں، مکمل نہیں۔*

### ٹیموں کے لیے ( coordination & Process)

اگر آپ ایک ٹیم کے ماحول میں کام کر رہے ہیں:

- **واضح کردار مقرر کریں**
  - میپنگ
  - ادراک
  - کنٹرول
  - انضمام
  - دستاویزات

- **ایک انضمام چیک لسٹ استعمال کریں**
  ضم کرنے سے پہلے:
  - تمام لانچ فائلز چل رہی ہیں
  - کوئی ٹاپک تنازعات نہیں ہیں
  - تمام ماڈیولز اسموک ٹیسٹ پاس کرتے ہیں

- **دستاویزات کو زندہ رکھیں**
  برقرار رکھیں:
  - معماری ڈائی گرامز
  - ROS API اسپیس (ٹاپکس / سروسز / ایکشنز)

مقصد: *الجھاؤ کو کم کریں اور ٹیم کی رفتار میں اضافہ کریں۔*

### آخری مشورہ

چاہے آپ کا کردار کیا ہو:

> **چھوٹا شروع کریں → مسلسل بنائیں → اکثر ٹیسٹ کریں → ہر چیز کو دستاویز کریں → اپنی پیشرفت کو شیئر کریں**

یہ جسمانی AI اور ہیومنوائڈ روبوٹکس میں پیشہ ورانہ افراد کا حقیقی ورک فلو ہے۔

### پری اسٹارٹ چیک لسٹ

* Ubuntu / WSL تیار
* Python venv سیٹ
* ROS2 انسٹال
* colcon ورک اسپیس بنائی
* Gazebo / Isaac کام کر رہا ہے
* NVIDIA ڈرائیورز انسٹال
* Git repo شروع کیا

### کیپسٹون سے پہلے

* URDF RViz میں لوڈ ہوتا ہے
* Gazebo روبوٹ اسپون
* SLAM + ادراک کام کر رہا ہے
* آواز → منصوبہ → ایگزیکیوٹ پائپ لائن کام کر رہی ہے

**کیپسٹون ہدف:**
ہیومنوائڈ ایک **آواز کمانڈ** وصول کرتا ہے اور **اٹھانے اور جگہ دینے کا کام** خود بخود سیمولیشن میں انجام دیتا ہے۔

</div>
