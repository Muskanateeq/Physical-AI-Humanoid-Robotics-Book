---
id: unity-rendering
title: 'Chapter 9: Photorealism and Interaction in Unity'
---

import LanguageToggle from '@site/src/components/LanguageToggle/LanguageToggle';

<LanguageToggle />
import Admonition from '@theme/Admonition';


<div className="english-content">

# Chapter 9: Photorealism and Interaction in Unity

While Gazebo is a powerful tool for physics simulation, for tasks requiring photorealism—such as generating synthetic training data or conducting human-robot interaction (HRI) studies—we turn to game engines. **Unity**, with its **High Definition Render Pipeline (HDRP)**, is a premier choice for creating visually stunning digital twins.

<Admonition type="info" title="Alternative: NVIDIA Omniverse">
  Another powerful, industry-standard platform is **NVIDIA Omniverse**, particularly with **Isaac Sim**. It offers exceptional photorealism, tight integration with ROS and AI tools, and is built for creating large-scale, physically accurate simulations. The principles discussed here for Unity often have direct parallels in Omniverse.
</Admonition>

## 9.1 Setting up Unity with HDRP

1.  **Install Unity Hub & Editor:** Download and install a recent version of Unity (e.g., 2022.3 LTS) with Linux support.
2.  **Create an HDRP Project:** When creating a new project, select the "High Definition RP" template.
3.  **Import Unity Robotics Hub:** From the `Window -> Package Manager` menu, add the official `com.unity.robotics.ros-tcp-connector` and other robotics packages. These will allow you to import a URDF and communicate with ROS 2.

### Physically-Based Rendering (PBR)

HDRP uses a PBR workflow to create realistic materials. Instead of just a color texture, you define a material's physical properties:
*   **Albedo:** The base color of the material.
*   **Metallic:** How much the material behaves like a metal.
*   **Smoothness:** How smooth or rough the surface is, affecting how light reflects.
*   **Normal Map:** A texture that simulates fine surface detail without adding more polygons.

By combining these, you can create materials that look like brushed aluminum, scuffed plastic, or polished chrome, making your robot look incredibly lif-like.

## 9.2 The Right Physics: ArticulationBody vs. Rigidbody

Unity's standard `Rigidbody` component is great for simple objects, but it's not ideal for a complex, high-degree-of-freedom (DoF) humanoid robot. The component uses an iterative solver that can become unstable with long chains of joints.

For this, Unity provides the **`ArticulationBody`** component.

**Why `ArticulationBody` is Essential for Robotics:**

*   **Reduced-Coordinate Representation:** It treats the entire robot as a single articulated system rather than a collection of independent rigid bodies with constraints. This is far more stable.
*   **Direct Drive Control:** It provides `ArticulationDrive`s to directly control joint positions or velocities, which is exactly how real robot joints are controlled.
*   **Featherstone's Algorithm:** It uses a more efficient and stable algorithm designed for articulated bodies, preventing joints from "stretching" or "exploding."

**ASCII Diagram: Conceptual Difference**

```
Rigidbody Approach:
[Link1] <-(Joint Constraint)-> [Link2] <-(Joint Constraint)-> [Link3]
(Each link solved independently, constraints can stretch)

ArticulationBody Approach:
[Link1 --- Link2 --- Link3]
(Entire chain solved as one system, no stretching)
```

When you import your robot's URDF using the Unity Robotics Hub, you should ensure it's configured to use `ArticulationBody` components for all moving joints.

### Human-Robot Interaction: The Gaze Tracking Scenario

A key advantage of a high-fidelity digital twin is the ability to study how humans might interact with the robot. A simple but effective HRI scenario is "gaze tracking," where the robot's head tracks a human avatar in the scene.

**C# Logic for an `ArticulationBody` Controller (Conceptual)**

This script would be attached to your robot's root, and it would receive joint targets from ROS 2 and apply them to the appropriate `ArticulationBody` drives.

```csharp title="ArticulationController.cs"
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
// ... other necessary imports

public class ArticulationController : MonoBehaviour
{
    // Dictionary to map joint names to their articulation bodies
    private Dictionary<string, ArticulationBody> jointMap;
    
    void Start()
    {
        // Populate the jointMap by searching through the robot's hierarchy
        
        // Subscribe to the /joint_command topic from ROS
        ROSConnection.GetOrCreateInstance().Subscribe<JointStateMsg>("/joint_command", OnJointCommand);
    }

    void OnJointCommand(JointStateMsg msg)
    {
        for (int i = 0; i < msg.name.Length; i++)
        {
            string jointName = msg.name[i];
            float targetPosition = (float)msg.position[i];

            if (jointMap.ContainsKey(jointName))
            {
                ArticulationBody joint = jointMap[jointName];
                var drive = joint.xDrive; // Assuming single-axis joint
                drive.target = targetPosition * Mathf.Rad2Deg; // Convert radians to degrees
                joint.xDrive = drive;
            }
        }
    }
}
```

<Admonition type="info" icon="💡" title="Hardware Optimization for RTX 4070 Ti">
  To achieve high framerates in a complex Unity HDRP scene with a photorealistic humanoid on an **NVIDIA RTX 4070 Ti**, follow these settings:
  *   **Resolution:** 2560x1440 is the sweet spot.
  *   **DLSS (Deep Learning Super Sampling):** Enable this under `HDRP Global Settings -> Upscaling`. Set it to **"Quality" or "Balanced"** mode for a significant performance boost with minimal visual impact.
  *   **Ray Tracing:** Use this sparingly for key effects. Enable ray-traced reflections but consider using screen-space global illumination (SSGI) instead of full ray-traced GI for better performance.
  *   **Shadows:** Use high-resolution shadow maps, but enable "Contact Shadows" to add fine-grained detail without a huge performance hit.
</Admonition>

</div>

<div className="urdu-content">

# باب 9: یونٹی میں فوٹو ریل سٹم اور تعامل

جبکہ Gazebo فزکس سیمولیشن کے لیے ایک طاقتور اوزار ہے، اس قسم کے کاموں کے لیے جن میں فوٹو ریل سٹم کی ضرورت ہوتی ہے—جیسے مصنوعی تربیتی ڈیٹا تیار کرنا یا انسان-روبوٹ تعامل (HRI) کے مطالعات کرنا—ہم گیم انجنوں کی طرف رجوع کرتے ہیں۔ **یونٹی**، اس کے **ہائی ڈیفینیشن رینڈر پائپ لائن (HDRP)** کے ساتھ، دلکش ڈیجیٹل ٹوئنز تیار کرنے کا ایک اعلیٰ انتخاب ہے۔

<Admonition type="info" title="متبادل: NVIDIA Omniverse">
  ایک اور طاقتور، صنعتی معیار کا پلیٹ فارم **NVIDIA Omniverse** ہے، خاص طور پر **Isaac Sim** کے ساتھ۔ یہ استثنائی فوٹو ریل سٹم، ROS اور AI ٹولز کے ساتھ گہرے انضمام، اور بڑے پیمانے پر، جسمانی طور پر درست سیمولیشن تیار کرنے کے لیے تیار کیا گیا ہے۔ یہاں یونٹی کے بارے میں بحث کردہ اصولوں کے Omniverse میں براہ راست موازنے ہوتے ہیں۔
</Admonition>

## 9.1 HDRP کے ساتھ یونٹی کی ترتیب

1.  **یونٹی ہب اور ایڈیٹر انسٹال کریں:** حالیہ ورژن کا یونٹی (مثلاً 2022.3 LTS) لینکس سپورٹ کے ساتھ ڈاؤن لوڈ اور انسٹال کریں۔
2.  **ایک HDRP پروجیکٹ تخلیق کریں:** نیا پروجیکٹ تخلیق کرتے وقت، "ہائی ڈیفینیشن RP" ٹیمپلیٹ منتخب کریں۔
3.  **یونٹی روبوٹکس ہب درآمد کریں:** `Window -> Package Manager` مینو سے، سرکاری `com.unity.robotics.ros-tcp-connector` اور دیگر روبوٹکس پیکجز شامل کریں۔ یہ آپ کو URDF درآمد کرنے اور ROS 2 کے ساتھ رابطہ کرنے کی اجازت دے گا۔

### جسمانی طور پر مبنی رینڈرنگ (PBR)

HDRP PBR ورک فلو کا استعمال حقیقی میٹریلز تخلیق کرنے کے لیے کرتا ہے۔ محض ایک رنگ کے ٹیکسچر کے بجائے، آپ ایک میٹریل کی جسمانی خصوصیات کی وضاحت کرتے ہیں:

*   **البیڈو:** میٹریل کا بنیادی رنگ۔
*   **میٹلک:** یہ کہ میٹریل کتنا میٹل کی طرح طرز عمل اختیار کرتا ہے۔
*   **چکناہٹ:** سطح کتنا چکنا یا کھرچ ہے، جو لائٹ کے عکاس کو متاثر کرتا ہے۔
*   **نارمل میپ:** ایک ٹیکسچر جو زیادہ پولی گونز شامل کیے بغیر فائن سطح کی تفصیل کی نقل کرتا ہے۔

ان کو ملا کر، آپ ایسے میٹریلز تخلیق کر سکتے ہیں جو برشڈ الیومنیم، سکفڈ پلاسٹک، یا پولشڈ کروم کی طرح نظر آئیں، جس سے آپ کا روبوٹ انتہائی زندہ نظر آئے۔

## 9.2 درست فزکس: ArticulationBody بمقابلہ Rigidbody

یونٹی کا معیاری `Rigidbody` کمپوننٹ سادہ اشیاء کے لیے بہت اچھا ہے، لیکن یہ ایک پیچیدہ، زیادہ ڈیگریز آف فریڈم (DoF) والے ہیومنوائڈ روبوٹ کے لیے مثالی نہیں ہے۔ یہ کمپوننٹ ایک اٹریٹو سالور کا استعمال کرتا ہے جو جوائنٹس کی لمبی زنجیروں کے ساتھ غیر مستحکم ہو سکتا ہے۔

اس کے لیے، یونٹی **`ArticulationBody`** کمپوننٹ فراہم کرتا ہے۔

**`ArticulationBody` کیوں روبوٹکس کے لیے ضروری ہے:**

*   **کم کوآرڈینیٹ نمائندگی:** یہ پورے روبوٹ کو ایک مربوط سسٹم کے طور پر لیتا ہے بجائے محدود جسموں کے مجموعہ کے جن کے ساتھ محدودیتیں ہوں۔ یہ کہیں زیادہ مستحکم ہے۔
*   **براہ راست ڈرائیو کنٹرول:** یہ `ArticulationDrive` فراہم کرتا ہے جو جوائنٹ پوزیشنز یا ویلوسٹیز کو براہ راست کنٹرول کرنے کے لیے ہوتے ہیں، جو بالکل ویسے ہی ہے جیسے حقیقی روبوٹ جوائنٹس کنٹرول کیے جاتے ہیں۔
*   **فیتھرسٹون الگورتھم:** یہ ایک زیادہ کارآمد اور مستحکم الگورتھم کا استعمال کرتا ہے جو مربوط جسموں کے لیے ڈیزائن کیا گیا ہے، جو جوائنٹس کو "پھیلنے" یا "پھٹنے" سے روکتا ہے۔

**ASCII ڈائیگرام: تصوراتی فرق**

```
Rigidbody کا انداز:
[Link1] <-(Joint Constraint)-> [Link2] <-(Joint Constraint)-> [Link3]
(ہر لنک کو آزادانہ حل کیا جاتا ہے، قیدیں پھیل سکتی ہیں)

ArticulationBody کا انداز:
[Link1 --- Link2 --- Link3]
(پوری زنجیر کو ایک سسٹم کے طور پر حل کیا جاتا ہے، کوئی پھیلاؤ نہیں)
```

جب آپ یونٹی روبوٹکس ہب کا استعمال کرتے ہوئے اپنے روبوٹ کا URDF درآمد کریں، تو آپ کو یقینی بنانا چاہیے کہ تمام حرکت پذیر جوائنٹس کے لیے `ArticulationBody` کمپوننٹس استعمال کرنے کے لیے اسے ترتیب دیا گیا ہو۔

### انسان-روبوٹ تعامل: گیز ٹریکنگ کا منظر

ڈیجیٹل ٹوئن کا ایک کلیدی فائدہ یہ ہے کہ اس کے مطالعہ کے ذریعے یہ سمجھا جا سکتا ہے کہ انسان روبوٹ کے ساتھ کیسے تعامل کر سکتے ہیں۔ ایک سادہ لیکن مؤثر HRI منظر "گیز ٹریکنگ" ہے، جہاں روبوٹ کا سر منظر میں ایک انسانی اوتار کو ٹریک کرتا ہے۔

**C# منطق برائے `ArticulationBody` کنٹرولر (تصوراتی)**

یہ سکرپٹ آپ کے روبوٹ کی جڑ سے منسلک ہوگا، اور یہ ROS 2 سے جوائنٹ ٹارگٹس وصول کرے گا اور انہیں مناسب `ArticulationBody` ڈرائیو پر لاگو کرے گا۔

```csharp title="ArticulationController.cs"
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
// ... دیگر ضروری درآمدات

public class ArticulationController : MonoBehaviour
{
    // جوائنٹ کے ناموں کو ان کے articulation bodies کے ساتھ میپ کرنے کے لیے ڈکشنری
    private Dictionary<string, ArticulationBody> jointMap;

    void Start()
    {
        // روبوٹ کی سلسلہ واری میں تلاش کر کے jointMap کو بھریں

        // ROS سے /joint_command ٹاپک کو سبسکرائب کریں
        ROSConnection.GetOrCreateInstance().Subscribe<JointStateMsg>("/joint_command", OnJointCommand);
    }

    void OnJointCommand(JointStateMsg msg)
    {
        for (int i = 0; i < msg.name.Length; i++)
        {
            string jointName = msg.name[i];
            float targetPosition = (float)msg.position[i];

            if (jointMap.ContainsKey(jointName))
            {
                ArticulationBody joint = jointMap[jointName];
                var drive = joint.xDrive; // ایک مانیں کہ سنگل-ایکسیس جوائنٹ
                drive.target = targetPosition * Mathf.Rad2Deg; // ریڈین کو ڈگری میں تبدیل کریں
                joint.xDrive = drive;
            }
        }
    }
}
```

<Admonition type="info" icon="💡" title="RTX 4070 Ti کے لیے ہارڈویئر کی بہترین کارکردگی">
  ایک پھوٹو ریل سٹک ہیومنوائڈ والے ایک پیچیدہ یونٹی HDRP منظر میں **NVIDIA RTX 4070 Ti** پر زیادہ فریم ریٹ حاصل کرنے کے لیے، ان سیٹنگز پر عمل کریں:
  *   **ریزولوشن:** 2560x1440 میٹھا نقطہ ہے۔
  *   **DLSS (ڈیپ لرننگ سوپر سیمپلینگ):** `HDRP گلوبل سیٹنگز -> اپ سکیلنگ` کے تحت اسے فعال کریں۔ **"کوالٹی" یا "بالنسڈ"** موڈ پر سیٹ کریں تاکہ کم بصری اثر کے ساتھ نمایاں کارکردگی کا فائدہ حاصل ہو۔
  *   **رے ٹریسنگ:** اہم اثرات کے لیے اس کا محتاط استعمال کریں۔ رے ٹریسڈ ریفلیکشنز فعال کریں لیکن بہتر کارکردگی کے لیے مکمل رے ٹریسڈ GI کے بجائے اسکرین اسپیس گلوبل ایلیومنیشن (SSGI) استعمال کرنا غور کریں۔
  *   **سایہ:** زیادہ ریزولوشن والے سایہ میپ استعمال کریں، لیکن چھوٹی تفصیل شامل کرنے کے لیے "کانٹیکٹ سایہ" فعال کریں بغیر کارکردگی پر زیادہ اثر ڈالے۔
</Admonition>

</div>
