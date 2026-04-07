---
id: ros-controllers
title: 'Chapter 2: ROS controllers using rclpy'
---

import Admonition from '@theme/Admonition';
import LanguageToggle from '@site/src/components/LanguageToggle/LanguageToggle';

<LanguageToggle />


<div className="english-content">

# Chapter 2: ROS controllers using rclpy

This chapter is critical as it focuses on bridging the high-level AI logic, typically written in Python, to the low-level hardware control systems of a humanoid robot. In complex physical systems, merely sending simple messages is insufficient; one must manage a three-tiered architecture encompassing High-Level Planning (AI/LLMs), Mid-Level Execution (ROS 2 Actions/Services), and Low-Level Control (Firmware/Motors). ROS Controllers serve to abstract this Low-Level Control, providing a standardized interface that allows your Python AI agent to set precise joint targets (like elbow angle or knee position) without needing to worry about the specifics of the motor driver hardware. The rclpy library facilitates this essential communication bridge, enabling Python to securely interact with these controllers, primarily utilizing asynchronous communication patterns like Actions for long-running tasks such as bipedal locomotion, and Services for immediate, synchronous requests like reading the current joint state.

## 2.1 — Bridging Python Agents to ROS controllers using rclpy (Python in ROS 2)

`rclpy` is the recommended Python client for ROS 2. This chapter gives **real, runnable code** for:

- Publisher
- Subscriber
- Service Server & Client
- Action Server & Client (pattern)
- Package structure
- Agent → ROS Controller bridge

Place these files in:

```
~/ros_ws/src/ros_tutorials/ros_tutorials/
```

Then build them using `colcon`.

:::note Recommended
Keep your ROS 2 Python nodes inside a clean package structure so that they can be reused in simulation and real hardware with minimal changes.
:::

### Creating a Python Publisher Node

**File: `talker.py`**

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):

    def __init__(self):
        super().__init__('talker')

        # QoS: queue depth 10 (default)
        self.pub = self.create_publisher(String, 'chatter', 10)

        # Timer: 0.5 seconds
        self.timer = self.create_timer(0.5, self.timer_cb)

        self.count = 0

    def timer_cb(self):
        msg = String()
        msg.data = f'hello {self.count}'
        self.pub.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = Talker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
```

### Build & Run

```bash
# From ~/ros_ws
colcon build --packages-select ros_tutorials
source install/setup.bash

ros2 run ros_tutorials talker
```

:::note Recommended
Use publishers for sensor data, perception output, or system state spreading.  
This pattern is your **core communication channel** for AI → Robot messaging.
:::

### Creating a Python Subscriber Node

**File: `listener.py`**

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Listener(Node):

    def __init__(self):
        super().__init__('listener')
        self.sub = self.create_subscription(
            String,
            'chatter',
            self.cb,
            10
        )

    def cb(self, msg):
        self.get_logger().info(f'I heard: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = Listener()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
```

### Run in parallel

```bash
# Terminal 1
ros2 run ros_tutorials talker

# Terminal 2
ros2 run ros_tutorials listener
```

:::note Recommended
Subscribers should be light-weight and non-blocking.  
Never run heavy ML inference in subscriber callbacks.
:::

## 2.2 — Service Server & Client (AddTwoInts)

### Server: `add_two_ints_server.py`

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class AddTwoIntsServer(Node):

    def __init__(self):
        super().__init__('add_two_ints_server')

        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.callback
        )

    def callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(
            f'Received: {request.a} + {request.b} = {response.sum}'
        )
        return response


def main():
    rclpy.init()
    node = AddTwoIntsServer()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
```

### Client: `add_two_ints_client.py`

```python
import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class Client(Node):

    def __init__(self):
        super().__init__('add_two_ints_client')

        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting...')

        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b

        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)

        return future.result()


def main(args=None):
    rclpy.init(args=args)

    client = Client()
    result = client.send_request(int(sys.argv[1]), int(sys.argv[2]))

    print('Result:', result.sum)

    client.destroy_node()
    rclpy.shutdown()
```

### Run

```bash
# Terminal 1
ros2 run ros_tutorials add_two_ints_server

# Terminal 2
ros2 run ros_tutorials add_two_ints_client 2 3
```

:::note Recommended
Services are used for **quick request/response commands**, not streaming data.
:::

### Action Server & Client (Concept Pattern)

Create action file:

```
action_interfaces/action/Count.action
```

```text
# Goal
int32 order
---
# Result
int32[] sequence
---
# Feedback
int32[] partial_sequence
```

### Action Server – Pattern

- Accept goal
- Publish feedback
- Return final result
- Support cancellation

### Action Client – Pattern

- Send goal
- Receive feedback
- Wait for result

:::note Recommended
Actions must be used for **long-running robotic tasks** like:
- moving arms
- walking
- navigation
- grasping
:::

# 1.5 — Bridging AI Agents to ROS 2 Controllers

This is the **core brain-to-body bridge**.

### Architecture (Recommended)

```
[LLM / AI Agent] → [Agent Interface Node (rclpy)] → [ROS Controllers / Actions]
      Python               ROS Messages & Actions           Motors / Sim
```

Agent Responsibilities:

- Perception interpretation
- Planning
- Commanding using structured goals

:::note Recommended
**Never allow raw LLM text to directly control motors.**  
Always parse → validate → convert → execute.
:::

## 2.3 — Agent-to-ROS Detailed Flow

Example AI output:

```json
[
  {"action": "move_base", "params": {"pose": [1.2, 0.5, 1.57]}},
  {"action": "manipulator.move_to", "params": {"joint_positions":[0.1,0.5,0.3]}},
  {"action": "gripper.close", "params": {}}
]
```

Pipeline:

1. Input: speech / text / camera feed
2. Cognitive Module: LLM / planner
3. Validator: checks safety rules
4. Executor → ROS Action Client

:::note Recommended
Always test in simulation **before hardware**.
:::

## 2.4 — Command Mapping (YAML)

Create `action_map.yaml`

```yaml
move_base:
  type: action
  action_name: /nav2_actions/navigate_to_pose
  message_type: nav2_msgs/ActionNavigateToPose

manipulator.move_to:
  type: action
  action_name: /arm_controller/follow_joint_trajectory
  message_type: control_msgs/FollowJointTrajectory

gripper.close:
  type: service
  service_name: /gripper_controller/close
```

:::note Recommended
Configuration must stay outside code for flexibility and safety.
:::

## 2.5 — Real-Time Performance Rules

- Controllers: **100–1000 Hz**
- Agent nodes: should **not block executor**
- Use:
  - MultiThreadedExecutor
  - Async action calls
  - Worker processes for ML

## Example Flow (Full Concept)

**User:**  
"Pick the red cup on the table."

**Agent plan:**
```
[
  "navigate_to(table)",
  "detect(red cup)",
  "approach",
  "grasp"
]
```

System executes:

1. Convert each into ROS goal
2. Validate bounds + safety
3. Send action
4. Get feedback
5. Replan if fail

:::note Recommended
Use **Simulation-in-the-loop** before real-world action!
:::

</div>

<div className="urdu-content">

# باب 2: rclpy کا استعمال کرتے ہوئے ROS کنٹرولرز

یہ باب انتہائی اہم ہے کیونکہ یہ اعلیٰ سطح کے AI منطق، جو عام طور پر Python میں لکھا جاتا ہے، کو ہیومنوائڈ روبوٹ کے کم سطح کے ہارڈویئر کنٹرول سسٹم سے جوڑنے پر توجہ مرکوز کرتا ہے۔ پیچیدہ جسمانی نظاموں میں، صرف سادہ پیغامات بھیجنے سے کافی نہیں ہے؛ ایک کو تین ٹیئر آرکیٹیکچر کا انتظام کرنا ہوتا ہے جس میں اعلیٰ سطح کی منصوبہ بندی (AI/LLMs)، درمیانی سطح کی ایگزیکوشن (ROS 2 ایکشنز/سروسز)، اور کم سطح کنٹرول (فرومر/موٹرز) شامل ہیں۔ ROS کنٹرولرز اس کم سطح کنٹرول کو خفیہ کرتے ہیں، ایک معیاری انٹرفیس فراہم کرتے ہیں جو آپ کے Python AI ایجنٹ کو درست جوائنٹ ٹارگٹس (جیسے کوہنی کا زاویہ یا گھٹنے کی پوزیشن) سیٹ کرنے کی اجازت دیتا ہے بغیر یہ فکر کیے کہ موٹر ڈرائیور ہارڈویئر کی تفصیلات کیا ہیں۔ rclpy لائبریری یہ اہم رابطہ فراہم کرتی ہے، Python کو ان کنٹرولرز کے ساتھ محفوظ طریقے سے بات چیت کرنے کی اجازت دیتی ہے، زیادہ تر غیر ہم وقت ساز مواصلاتی پیٹرنز کا استعمال کرتے ہوئے جیسے ایکشنز طویل چلنے والے کاموں کے لیے جیسے بائی پیڈل لوکوموشن، اور سروسز فوری، ہم وقت ساز درخواستوں کے لیے جیسے موجودہ جوائنٹ کی حالت پڑھنا۔

## 2.1 — rclpy کا استعمال کرتے ہوئے Python ایجنٹس کو ROS کنٹرولرز سے جوڑنا (Python in ROS 2)

`rclpy` ROS 2 کے لیے تجویز کردہ Python کلائنٹ ہے۔ یہ باب **اصل، چلنے والے کوڈ** فراہم کرتا ہے:

- پبلشر
- سبسکرائبر
- سروس سرور اور کلائنٹ
- ایکشن سرور اور کلائنٹ (پیٹرن)
- پیکج کی ساخت
- ایجنٹ → ROS کنٹرولر برج

ان فائلز کو یہاں رکھیں:

```
~/ros_ws/src/ros_tutorials/ros_tutorials/
```

پھر انہیں `colcon` کا استعمال کرتے ہوئے بنائیں۔

:::note تجویز کردہ
اپنے ROS 2 Python نوڈز کو ایک صاف پیکج سٹرکچر کے اندر رکھیں تاکہ وہ سیمولیشن اور حقیقی ہارڈویئر میں کم تبدیلیوں کے ساتھ دوبارہ استعمال کیے جا سکیں۔
:::

### ایک Python پبلشر نوڈ تخلیق کرنا

**فائل: `talker.py`**

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):

    def __init__(self):
        super().__init__('talker')

        # QoS: قطار کی گہرائی 10 (ڈیفالٹ)
        self.pub = self.create_publisher(String, 'chatter', 10)

        # ٹائمر: 0.5 سیکنڈ
        self.timer = self.create_timer(0.5, self.timer_cb)

        self.count = 0

    def timer_cb(self):
        msg = String()
        msg.data = f'hello {self.count}'
        self.pub.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = Talker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
```

### بنانا اور چلانا

```bash
# From ~/ros_ws
colcon build --packages-select ros_tutorials
source install/setup.bash

ros2 run ros_tutorials talker
```

:::note تجویز کردہ
سینسر ڈیٹا، ادراک کا آؤٹ پٹ، یا سسٹم اسٹیٹ کو پھیلانے کے لیے پبلشرز استعمال کریں۔
یہ پیٹرن آپ کا **کور کمیونیکیشن چینل** AI → روبوٹ میسج کے لیے ہے۔
:::

### ایک Python سبسکرائبر نوڈ تخلیق کرنا

**فائل: `listener.py`**

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Listener(Node):

    def __init__(self):
        super().__init__('listener')
        self.sub = self.create_subscription(
            String,
            'chatter',
            self.cb,
            10
        )

    def cb(self, msg):
        self.get_logger().info(f'I heard: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = Listener()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
```

### متوازی طور پر چلائیں

```bash
# ٹرمنل 1
ros2 run ros_tutorials talker

# ٹرمنل 2
ros2 run ros_tutorials listener
```

:::note تجویز کردہ
سبسکرائبرز ہلکے اور غیر مسدود ہونے چاہئیں۔
کبھی بھی سبسکرائبر کال بیکس میں بھاری ML انفرس نہ چلائیں۔
:::

## 2.2 — سروس سرور اور کلائنٹ (AddTwoInts)

### سرور: `add_two_ints_server.py`

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class AddTwoIntsServer(Node):

    def __init__(self):
        super().__init__('add_two_ints_server')

        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.callback
        )

    def callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(
            f'Received: {request.a} + {request.b} = {response.sum}'
        )
        return response


def main():
    rclpy.init()
    node = AddTwoIntsServer()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
```

### کلائنٹ: `add_two_ints_client.py`

```python
import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class Client(Node):

    def __init__(self):
        super().__init__('add_two_ints_client')

        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting...')

        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b

        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)

        return future.result()


def main(args=None):
    rclpy.init(args=args)

    client = Client()
    result = client.send_request(int(sys.argv[1]), int(sys.argv[2]))

    print('Result:', result.sum)

    client.destroy_node()
    rclpy.shutdown()
```

### چلائیں

```bash
# ٹرمنل 1
ros2 run ros_tutorials add_two_ints_server

# ٹرمنل 2
ros2 run ros_tutorials add_two_ints_client 2 3
```

:::note تجویز کردہ
سروسز **فوری درخواست/جواب کمانڈز** کے لیے استعمال ہوتی ہیں، ڈیٹا اسٹریمنگ کے لیے نہیں۔
:::

### ایکشن سرور اور کلائنٹ (کانسیپٹ پیٹرن)

ایکشن فائل تخلیق کریں:

```
action_interfaces/action/Count.action
```

```text
# گوئل
int32 order
---
# ریزلٹ
int32[] sequence
---
# فیڈ بیک
int32[] partial_sequence
```

### ایکشن سرور – پیٹرن

- گوئل قبول کریں
- فیڈ بیک پبلش کریں
- حتمی نتیجہ لوٹائیں
- منسوخی کی حمایت کریں

### ایکشن کلائنٹ – پیٹرن

- گوئل بھیجیں
- فیڈ بیک وصول کریں
- نتیجہ کا انتظار کریں

:::note تجویز کردہ
**طویل چلنے والے روبوٹکس کاموں** کے لیے ایکشنز کا استعمال کرنا چاہیے جیسے:
- بازوؤں کو حرکت دینا
- چلنا
- نیویگیشن
- گریسنگ
:::

# 1.5 — AI ایجنٹس کو ROS 2 کنٹرولرز سے جوڑنا

یہ **کور دماغ-سے-جسم برج** ہے۔

### معماری (تجویز کردہ)

```
[LLM / AI ایجنٹ] → [ایجنٹ انٹرفیس نوڈ (rclpy)] → [ROS کنٹرولرز / ایکشنز]
      Python               ROS میسجس اور ایکشنز           موٹرز / سیم
```

ایجنٹ ذمہ داریاں:

- ادراک کی تشریح
- منصوبہ بندی
- سٹرکچرڈ گوئلز کا استعمال کرتے ہوئے کمانڈ کرنا

:::note تجویز کردہ
**کبھی بھی خام LLM ٹیکسٹ کو براہ راست موٹرز کنٹرول کرنے کی اجازت نہ دیں۔**
ہمیشہ پارس → ولیڈیٹ → کنورٹ → ایگزیکیوٹ کریں۔
:::

## 2.3 — ایجنٹ-سے-ROS تفصیلی فلو

AI آؤٹ پٹ کی مثال:

```json
[
  {"action": "move_base", "params": {"pose": [1.2, 0.5, 1.57]}},
  {"action": "manipulator.move_to", "params": {"joint_positions":[0.1,0.5,0.3]}},
  {"action": "gripper.close", "params": {}}
]
```

پائپ لائن:

1. ان پٹ: تقریر / ٹیکسٹ / کیمرہ فیڈ
2. کوگنیٹو ماڈیول: LLM / پلینر
3. والیڈیٹر: حفاظتی قواعد چیک کرتا ہے
4. ایگزیکیوٹر → ROS ایکشن کلائنٹ

:::note تجویز کردہ
ہمیشہ **ہارڈویئر سے پہلے** سیمولیشن میں ٹیسٹ کریں۔
:::

## 2.4 — کمانڈ میپنگ (YAML)

`action_map.yaml` تخلیق کریں

```yaml
move_base:
  type: action
  action_name: /nav2_actions/navigate_to_pose
  message_type: nav2_msgs/ActionNavigateToPose

manipulator.move_to:
  type: action
  action_name: /arm_controller/follow_joint_trajectory
  message_type: control_msgs/FollowJointTrajectory

gripper.close:
  type: service
  service_name: /gripper_controller/close
```

:::note تجویز کردہ
کنفیگریشن کو لچک اور حفاظت کے لیے کوڈ کے باہر رہنا چاہیے۔
:::

## 2.5 — ریل ٹائم کارکردگی کے قواعد

- کنٹرولرز: **100–1000 Hz**
- ایجنٹ نوڈز: **ایگزیکیوٹر کو بلاک نہیں کرنا چاہیے**
- استعمال کریں:
  - MultiThreadedExecutor
  - Async ایکشن کالز
  - ML کے لیے ورکر پروسیسز

## مثالی فلو (مکمل تصور)

**صارف:**
"میز پر لال کپ اٹھاؤ۔"

**ایجنٹ منصوبہ:**
```
[
  "navigate_to(table)",
  "detect(red cup)",
  "approach",
  "grasp"
]
```

سسٹم انجام دیتا ہے:

1. ہر ایک کو ROS گوئل میں تبدیل کریں
2. باؤنڈز + حفاظت کی تصدیق کریں
3. ایکشن بھیجیں
4. فیڈ بیک حاصل کریں
5. ناکام ہونے پر دوبارہ منصوبہ بندی کریں

:::note تجویز کردہ
حقیقی دنیا کی کارروائی سے پہلے **سیمولیشن-ان-دی-لوپ** کا استعمال کریں!
:::
</div>
