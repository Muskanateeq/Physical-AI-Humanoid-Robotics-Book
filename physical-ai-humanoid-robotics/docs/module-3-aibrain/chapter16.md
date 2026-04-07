--- 
id: nav2-and-mini-project
title: 'Chapter 16: The Bipedal Navigator: Nav2 Adaptation & Mini-Project'
sidebar_position: 7
---

import Admonition from '@theme/Admonition';
import LanguageToggle from '@site/src/components/LanguageToggle/LanguageToggle';

<LanguageToggle />


<div className="english-content">

# Chapter 16: The Bipedal Navigator: Nav2 Adaptation & Mini-Project

Your robot can see and map its world. But can it move with purpose? This chapter challenges you to adapt the industry-standard navigation stack, **Nav2**, for a bipedal humanoid. You will learn why algorithms designed for wheeled robots fail for legged systems and integrate high-level planning with a full end-to-end mini-project.

### Key Learning Objectives

*   Nav2: path planning & obstacle avoidance integration with perception.
*   Mini-project: train a perception model in Isaac Sim, export to TensorRT, run inference via Isaac ROS, map a room, and run autonomous navigation with Nav2.

## 16.1 — Navigation & Path Planning (Nav2)
### Path planning concepts (in short)

*   **Global planner**: finds a path in the global map (e.g., A*, navfn).
*   **Local planner/controller**: follows the path while reacting to dynamic obstacles (e.g., DWB, TEB).
*   **Costmap**: grid that marks static/dynamic obstacles and inflation layers for safety margin.

Nav2 (Navigation2) is the standard ROS 2 navigation stack and integrates with simulated robots for path planning and obstacle avoidance. Use Nav2 for humanoid high-level navigation (note: for full bipedal gait you may need specialized footstep planners).

### Obstacle avoidance & goal setting

*   Use perception outputs (object detector / segmentation) to augment costmaps (project detections into costmap) for semantic-aware navigation (e.g., ignore glass, avoid fragile objects).
*   For humanoids, set robot footprint and inflation radius conservatively because balance and stumble risk are higher than for wheeled robots.

**Practical**: Bridge the perception node to Nav2 by:

*   Running a detector node that publishes `geometry_msgs/PoseStamped` of obstacles or `costmap_2d` updates.
*   Use a custom costmap layer plugin to consume detections and mark regions as occupied.

--- 

## 16.2 — Mini Project (Module 3): Train navigation/perception model, map a room, autonomous movement
### Objective (end-to-end)

*   Use Isaac Sim to generate a synthetic dataset for a “red cup” object and room scenes.
*   Train a detector (YOLOv8 / Detectron2) on the synthetic dataset and validate on held-out simulated scenes.
*   Export the model to TensorRT (or Triton) and deploy it with Isaac ROS DNN inference node.
*   Run Nav2 in simulation with perception augmenting the costmap (semantic cost layer) and produce autonomous navigation to a table, detect the cup, and send manipulation action (trajectory goal) to the arm controller.
*   Evaluate success rate & produce logs.

### Project repo layout (recommended)
```bash
module3-aiscibrain/
├─ sim/                           # Isaac Sim scenes & replicator scripts
│  ├─ scenes/
│  └─ replicator_scripts/
├─ data/
│  ├─ synthetic/
│  │  ├─ images/
│  │  └─ annotations/
│  └─ real_validation/            # optional: collect small real images
├─ models/
│  ├─ detector/
│  │  ├─ best_model.pt
│  │  └─ model.onnx
│  └─ trt_engine/
├─ ros_ws/
│  ├─ src/
│  │  ├─ perception_nodes/        # Isaac ROS inference node wrappers
│  │  ├─ nav2_config/
│  │  └─ manipulation_bridge/
├─ experiments/
│  └─ run_2025-12-04/
└─ README.md
```

### Step-by-step reproducible lab (copy/paste)
#### A. Generate dataset in Isaac Sim via Replicator
```bash
# run a Python replicator script
python sim/replicator_scripts/generate_red_cup_dataset.py --out_dir data/synthetic --num_images 5000 --seed 42
```

Script responsibilities: open scene, randomize cup pose, randomize lights and textures, render RGB+depth+instance masks, save COCO annotations. (Use Replicator APIs; docs show examples.)

#### B. Train detector
```bash
# Example (YOLOv8 or Detectron2)
python train_detector.py --data data/synthetic --output models/detector --batch 16 --epochs 50
```

Monitor validation AP, use early stopping, log to WandB/MLflow.

#### C. Export to ONNX → TensorRT
```bash
python export_to_onnx.py --weights models/detector/best.pt --out models/detector/model.onnx
trtexec --onnx=models/detector/model.onnx --saveEngine=models/detector/model.trt --fp16
# Or deploy via Triton for multi-model serving
```

#### D. Deploy via Isaac ROS

Place `model.trt` in a known folder and configure Isaac ROS inference node parameters (engine path, input/output names, pre/post processors).

Launch Isaac ROS pipeline:

```bash
ros2 launch isaac_ros_dnn inference_launch.py model:=/path/to/model.trt use_sim_time:=True
```

Confirm detection topics (`/detected_objects`) publish BoundingBox messages.

#### E. Nav2 integration

Use a custom costmap layer that subscribes to `/detected_objects` and marks cells as occupied where an object is detected (project detection into 2D using camera intrinsics and estimated depth).

Launch Nav2 with `nav2_bringup` using your nav2 params tuned for humanoid footprint. Send a `NavigateToPose` goal near the table and verify the robot path replans around dynamic obstacles or the detected cup.

#### F. Autonomous pick pipeline (optional)

After arriving at table region, trigger a local manipulate routine:

*   Run a precise head/pan-tilt controller to center cup in camera FOV.
*   Use depth + pose estimation to compute pick pose.
*   Send trajectory to arm `FollowJointTrajectory` action server for grasp.

### Evaluation metrics (concrete)

*   **Detection AP@0.5**: target ≥ 0.7 on held-out synthetic/validation scenes.
*   **Navigation success rate**: target ≥ 0.8 (robot reaches table & stops within tolerance).
*   **Perception latency**: end-to-end detection time ≤ 50 ms (on RTX 40xx / Orin NX target) — depends on hardware. Use Isaac ROS NITROS nodes to meet low latency.

### Practical notes, pitfalls & best practices

*   **Hardware planning**: Isaac Sim + dataset rendering is heavy; plan VRAM and disk. Use headless rendering (no GUI) for large dataset generation.
*   **Version pinning**: Isaac Sim / Omniverse versions have strict driver/CUDA requirements — pin and document the exact release you used.
*   **Use Triton for scale**: for experiments that require many models or remote GPU serving, Triton simplifies model management and performant multi-model serving. Isaac ROS integrates with Triton.
*   **Sim2Real caution**: always validate perception models with a small set of real images (even 100 images) to estimate the sim-to-real gap. Use domain randomization (lighting, textures, noise) and post-processing (color jitter, blurring) to shrink the gap.

## Appendix — Useful commands & pointers

*   Isaac Sim requirements & compatibility checks.
*   Isaac ROS accelerated package docs and sample NITROS pipelines.
*   SDG / Replicator docs (dataset generation code examples).
*   Nav2 getting started & configuration guides.
*   ORB-SLAM3 ROS 2 community ports and VSLAM literature survey.

</div>

<div className="urdu-content">


# باب 16: بائی پیڈل نیویگیٹر: Nav2 ایڈاپٹیشن اور منی-پروجیکٹ

آپ کا روبوٹ اپنی دنیا کو دیکھ سکتا ہے اور نقشہ بنا سکتا ہے۔ لیکن کیا یہ مقصد کے ساتھ حرکت کر سکتا ہے؟ یہ باب آپ کو صنعتی معیار کے نیویگیشن اسٹیک، **Nav2**، کو بائی پیڈل ہیومنوائڈ کے لیے ایڈاپٹ کرنے کا چیلنج دیتا ہے۔ آپ سیکھیں گے کہ چکروں والے روبوٹس کے لیے ڈیزائن کردہ الگورتھم لیگڈ سسٹم کے لیے کیوں ناکام ہوتے ہیں اور اعلیٰ درجے کی منصوبہ بندی کو ایک مکمل اینڈ-ٹو-اینڈ منی پروجیکٹ کے ساتھ انضمام دیں گے۔

### کلیدی سیکھنے کے اہداف

*   Nav2: راستہ منصوبہ بندی اور رکاوٹ سے بچاؤ کو ادراک کے ساتھ انضمام دینا۔
*   منی-پروجیکٹ: Isaac Sim میں ادراک ماڈل تربیت دیں، TensorRT میں ایکسپورٹ کریں، Isaac ROS کے ذریعے انفرس چلائیں، کمرہ کا نقشہ بنائیں، اور Nav2 کے ساتھ خود مختار نیویگیشن چلائیں۔

## 16.1 — نیویگیشن اور راستہ منصوبہ بندی (Nav2)

### راستہ منصوبہ بندی کے تصورات (مختصر میں)

*   **گلوبل پلانر**: گلوبل نقشہ میں راستہ تلاش کرتا ہے (مثلاً، A*، navfn)۔
*   **مقامی پلانر/کنٹرولر**: راستہ کو فالو کرتا ہے جبکہ متحرک رکاوٹوں کا جواب دیتا ہے (مثلاً، DWB، TEB)۔
*   **کوسٹ میپ**: گرڈ جو سٹیٹک/متحرک رکاوٹوں اور محفوظ حد کے لیے اضافی لیئرز کو نشان زد کرتا ہے۔

Nav2 (نیویگیشن2) معیاری ROS 2 نیویگیشن اسٹیک ہے اور راستہ منصوبہ بندی اور رکاوٹ سے بچاؤ کے لیے سیمولیٹڈ روبوٹس کے ساتھ انضمام دیتا ہے۔ ہیومنوائڈ اعلیٰ درجے کی نیویگیشن کے لیے Nav2 استعمال کریں (نوٹ: مکمل بائی پیڈل گیٹ کے لیے آپ کو مخصوص فوٹ سٹیپ پلانرز کی ضرورت ہو سکتی ہے)۔

### رکاوٹ سے بچاؤ اور گوئل سیٹنگ

*   ادراک کے آؤٹ پٹ (اوبجیکٹ ڈیٹیکٹر / سیگمینٹیشن) کو کوسٹ میپس کو بڑھانے کے لیے استعمال کریں (کوسٹ میپ میں ڈیٹیکشنز پروجیکٹ کریں) تاکہ سیمینٹک-ویئر نیویگیشن کے لیے (مثلاً، گلاس کو نظر انداز کریں، نازک اشیاء سے بچیں)۔
*   ہیومنوائڈز کے لیے، روبوٹ کا فُٹ پرنٹ اور اضافی رداس کو محتاط طور پر سیٹ کریں کیونکہ توازن اور گرنے کا خطرہ چکروں والے روبوٹس کے مقابلے میں زیادہ ہے۔

**عملی**: ادراک نوڈ کو Nav2 سے جوڑنے کے لیے:

*   ایک ڈیٹیکٹر نوڈ چلائیں جو `geometry_msgs/PoseStamped` رکاوٹوں یا `costmap_2d` اپ ڈیٹس کو پبلش کرتا ہے۔
*   ڈیٹیکشنز کو استعمال کرنے اور علاقوں کو م occupied کے طور پر نشان زد کرنے کے لیے ایک حسب ضرورت کوسٹ میپ لیئر پلگ ان استعمال کریں۔

---

## 16.2 — منی پروجیکٹ (ماڈیول 3): نیویگیشن/ادراک ماڈل تربیت دیں، کمرہ کا نقشہ بنائیں، خود مختار حرکت

### ہدف (اینڈ-ٹو-اینڈ)

*   "لال کپ" اوبجیکٹ اور کمرہ کے مناظر کے لیے مصنوعی ڈیٹا سیٹ تیار کرنے کے لیے Isaac Sim استعمال کریں۔
*   مصنوعی ڈیٹا سیٹ پر ایک ڈیٹیکٹر (YOLOv8 / Detectron2) تربیت دیں اور جمع کردہ سیمولیٹڈ مناظر پر جائزہ لیں۔
*   ماڈل کو TensorRT (یا Triton) میں ایکسپورٹ کریں اور Isaac ROS DNN انفرس نوڈ کے ساتھ اسے ڈیپلائے کریں۔
*   ادراک کے ساتھ کوسٹ میپ کو بڑھاتے ہوئے (سیمینٹک کوسٹ لیئر) سیمولیشن میں Nav2 چلائیں اور ایک ٹیبل تک خود مختار نیویگیشن پیدا کریں، کپ کو ڈیٹیکٹ کریں، اور مینیپولیشن ایکشن (ٹریجیکٹری گوئل) کو آرم کنٹرولر پر بھیجیں۔
*   کامیابی کی شرح اور لاگز تیار کریں۔

### پروجیکٹ ریپو لے آؤٹ (تجویز کردہ)

```bash
module3-aiscibrain/
├─ sim/                           # Isaac Sim مناظر اور ریپلیکیٹر اسکرپٹس
│  ├─ scenes/
│  └─ replicator_scripts/
├─ data/
│  ├─ synthetic/
│  │  ├─ images/
│  │  └─ annotations/
│  └─ real_validation/            # اختیاری: چھوٹی اصل امیجز جمع کریں
├─ models/
│  ├─ detector/
│  │  ├─ best_model.pt
│  │  └─ model.onnx
│  └─ trt_engine/
├─ ros_ws/
│  ├─ src/
│  │  ├─ perception_nodes/        # Isaac ROS انفرس نوڈ وریپرز
│  │  ├─ nav2_config/
│  │  └─ manipulation_bridge/
├─ experiments/
│  └─ run_2025-12-04/
└─ README.md
```

### اسٹیپ-بائی-اسٹیپ دہرائے جانے والے آزمائش (کاپی/پیسٹ)

#### A. Isaac Sim میں Replicator کے ذریعے ڈیٹا سیٹ جنریٹ کریں

```bash
# ایک Python ریپلیکیٹر اسکرپٹ چلائیں
python sim/replicator_scripts/generate_red_cup_dataset.py --out_dir data/synthetic --num_images 5000 --seed 42
```

اسکرپٹ کے ذمے: منظر کھولنا، کپ کی پوز کو رینڈمائز کرنا، لائٹس اور ٹیکسچرز کو رینڈمائز کرنا، RGB+ڈیپتھ+مثال کے ماسکس رینڈر کرنا، COCO اینوٹیشنز محفوظ کرنا۔ (Replicator APIs استعمال کریں؛ ڈاکس مثالیں دکھاتے ہیں۔)

#### B. ڈیٹیکٹر تربیت دیں

```bash
# مثال (YOLOv8 یا Detectron2)
python train_detector.py --data data/synthetic --output models/detector --batch 16 --epochs 50
```

جائزہ والی AP کو مانیٹر کریں، ابتدائی روکاؤ استعمال کریں، WandB/MLflow پر لاگ ان کریں۔

#### C. ONNX → TensorRT میں ایکسپورٹ کریں

```bash
python export_to_onnx.py --weights models/detector/best.pt --out models/detector/model.onnx
trtexec --onnx=models/detector/model.onnx --saveEngine=models/detector/model.trt --fp16
# یا کثیر ماڈل سرور کے لیے Triton کے ذریعے ڈیپلائے کریں
```

#### D. Isaac ROS کے ذریعے ڈیپلائے کریں

`model.trt` کو ایک جانے والے فولڈر میں رکھیں اور Isaac ROS انفرس نوڈ پیرامیٹرز کنفیگر کریں (انجن کا راستہ، ان پٹ/آؤٹ پٹ کے نام، پری/پوسٹ پروسیسرز)۔

Isaac ROS پائپ لائن لانچ کریں:

```bash
ros2 launch isaac_ros_dnn inference_launch.py model:=/path/to/model.trt use_sim_time:=True
```

یقینی بنائیں کہ ڈیٹیکشن ٹاپکس (`/detected_objects`) BoundingBox میسجس پبلش کرتے ہیں۔

#### E. Nav2 انضمام

ایک حسب ضرورت کوسٹ میپ لیئر استعمال کریں جو `/detected_objects` کو سبسکرائب کرتا ہے اور جہاں کہیں اوبجیکٹ ڈیٹیکٹ ہوتا ہے وہاں سیلز کو occuppied کے طور پر نشان زد کرتا ہے (کیمرہ انٹرنسکس اور تخمینہ شدہ ڈیپتھ کا استعمال کرتے ہوئے 2D میں ڈیٹیکشن پروجیکٹ کریں)۔

اپنے humanoid footprint کے لیے ٹیون کردہ nav2 پیرامیٹرز کے ساتھ `nav2_bringup` استعمال کرتے ہوئے Nav2 لانچ کریں۔ ٹیبل کے قریب ایک `NavigateToPose` گوئل بھیجیں اور یقینی بنائیں کہ روبوٹ کا راستہ متحرک رکاوٹوں یا ڈیٹیکٹ کردہ کپ کے گرد دوبارہ منصوبہ بند کیا جاتا ہے۔

#### F. خود مختار پک پائپ لائن (اختیاری)

ٹیبل کے علاقے میں پہنچنے کے بعد، ایک مقامی مینیپولیٹ روتین ٹرگر کریں:

*   کیمرہ FOV میں کپ کو مرکز میں لانے کے لیے ایک درست ہیڈ/پین-ٹائلٹ کنٹرولر چلائیں۔
*   ڈیپتھ + پوز ایسٹیمیشن کا استعمال کر کے پک پوز کا حساب لگائیں۔
*   گریسپ کے لیے `FollowJointTrajectory` ایکشن سرور پر ٹریجیکٹری بھیجیں۔

### جائزہ میٹرکس (محل)

*   **ڈیٹیکشن AP@0.5**: جمع کردہ مصنوعی/جائزہ والے مناظر پر ہدف ≥ 0.7۔
*   **نیویگیشن کامیابی کی شرح**: ہدف ≥ 0.8 (روبوٹ ٹیبل تک پہنچتا ہے اور حد کے اندر رک جاتا ہے)۔
*   **ادراک کی تاخیر**: اینڈ-ٹو-اینڈ ڈیٹیکشن کا وقت ≤ 50 ms (RTX 40xx / Orin NX ہدف پر) — ہارڈویئر پر منحصر ہے۔ کم تاخیر کو پورا کرنے کے لیے Isaac ROS NITROS نوڈز استعمال کریں۔

### عملی نوٹس، الجھاؤ اور بہترین مشقیں

*   **ہارڈویئر منصوبہ بندی**: Isaac Sim + ڈیٹا سیٹ رینڈرنگ بھاری ہے؛ VRAM اور ڈسک کی منصوبہ بندی کریں۔ بڑے ڈیٹا سیٹ کی تخلیق کے لیے ہیڈ لیس رینڈرنگ (بغیر GUI) استعمال کریں۔
*   **ورژن پننگ**: Isaac Sim / Omniverse ورژن کے سخت ڈرائیور/CUDA تقاضے ہیں — وہ جو ریلیز آپ نے استعمال کیا وہ درج کریں اور دستاویز کریں۔
*   **سکیل کے لیے Triton استعمال کریں**: وہ تجربات جن کو کثیر ماڈلز یا دور دراز کے GPU سرور کی ضرورت ہو، Triton ماڈل مینجمنٹ اور کارآمد کثیر ماڈل سرور کو آسان بناتا ہے۔ Isaac ROS Triton کے ساتھ انضمام دیتا ہے۔
*   **Sim2Real احتیاط**: ہمیشہ ادراک ماڈلز کو اصل امیجز کے چھوٹے سیٹ (یہاں تک کہ 100 امیجز) کے ساتھ جائزہ لیں تاکہ sim-to-real گیپ کا تخمینہ لگایا جا سکے۔ ڈومین رینڈمائزیشن (لائٹنگ، ٹیکسچرز، نوائز) اور پوسٹ-پروسیسنگ (کلر جیٹر، بلر) استعمال کر کے گیپ کو کم کریں۔

## ایپنڈکس — مفید کمانڈز اور اشارے

*   Isaac Sim تقاضے اور مطابقت چیکس۔
*   Isaac ROS تیز کردہ پیکج ڈاکس اور نمونہ NITROS پائپ لائنز۔
*   SDG / Replicator ڈاکس (ڈیٹا سیٹ تخلیق کوڈ کی مثالیں)۔
*   Nav2 شروع کریں اور کنفیگریشن گائیڈز۔
*   ORB-SLAM3 ROS 2 کمیونٹی پورٹس اور VSLAM ادب کا جائزہ۔

</div>
