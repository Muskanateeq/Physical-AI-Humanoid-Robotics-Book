---
id: unity-rendering
title: 'Chapter 5: Photorealism and Interaction in Unity'
---

import Admonition from '@theme/Admonition';

## Beyond Physics: High-Fidelity Rendering

While Gazebo is a powerful tool for physics simulation, for tasks requiring photorealism—such as generating synthetic training data or conducting human-robot interaction (HRI) studies—we turn to game engines. **Unity**, with its **High Definition Render Pipeline (HDRP)**, is a premier choice for creating visually stunning digital twins.

<Admonition type="info" title="Alternative: NVIDIA Omniverse">
  Another powerful, industry-standard platform is **NVIDIA Omniverse**, particularly with **Isaac Sim**. It offers exceptional photorealism, tight integration with ROS and AI tools, and is built for creating large-scale, physically accurate simulations. The principles discussed here for Unity often have direct parallels in Omniverse.
</Admonition>

### Setting up Unity with HDRP

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

---

## The Right Physics: ArticulationBody vs. Rigidbody

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

---

## Human-Robot Interaction: The Gaze Tracking Scenario

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
