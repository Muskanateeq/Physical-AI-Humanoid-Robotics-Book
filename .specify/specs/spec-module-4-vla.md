# SPECIFICATION: Module 4 - Vision-Language-Action (VLA)

**Version:** 1.0.0
**Date:** 2025-11-30

## 1. Introduction

This document provides the detailed technical specifications for **Module 4: Vision-Language-Action (VLA)**. The VLA module is designed to bridge the gap between high-level human language instructions and low-level robotic actions. Students will learn to build and integrate systems that can see the world, understand voice commands, and plan and execute complex tasks in a physical or simulated environment.

This specification is grounded in the principles outlined in the **Constitution (Version 1.0.1)** and serves as the foundational document for all subsequent planning, development, and content creation for this module.

## 2. Module Overview & Core Focus

As mandated by **Constitution Section 1.2 (Course Modules Breakdown)**, this module will provide in-depth, hands-on experience in the following core areas:

-   **LLMs and Robotics Convergence:** Exploring the synergy between Large Language Models and robotic systems to create more intuitive and powerful human-robot interactions.
-   **Voice-to-Action:** Implementing a complete pipeline from spoken command to robotic execution.
-   **Cognitive Planning:** Using LLMs to perform reasoning and task decomposition for autonomous operations.
-   **Capstone Project:** Applying all learned concepts to build a multi-faceted autonomous humanoid robot system.

---

## 3. Core Technical Requirements

This section details the hardware, software, and methodologies required for the successful implementation of Module 4.

### 3.1. Hardware Setup & Optimization

In accordance with **Constitution Section VIII (Course Hardware Requirements)**, this module requires specific hardware for edge AI computation and audio input.

<Admonition type="info" title="Mandatory Hardware">

-   **Edge AI Compute:** NVIDIA Jetson Orin Nano (Developer Kit)
-   **Audio Input:** ReSpeaker 4-Mic Array (or compatible USB Mic Array)

</Admonition>

#### 3.1.1. Jetson Orin Nano Setup

-   **OS:** The Jetson Orin Nano must be flashed with the latest NVIDIA JetPack SDK.
-   **Dependencies:** Essential libraries such as `pip`, `python3-dev`, `libffi-dev`, and `portaudio19-dev` must be installed.
-   **ROS 2:** ROS 2 Humble Hawksbill (or the version specified by the course curriculum) must be installed and configured.

#### 3.1.2. ReSpeaker USB Mic Array Setup

-   **Connection:** The microphone array is to be connected via USB to the Jetson Orin Nano.
-   **Audio Configuration:** The device must be configured as the default ALSA input device to ensure consistent audio capture. Students will need to identify the device's card and device number using `arecord -l`.

<Admonition type="tip" title="Optimization Tip">

For optimal performance on the Jetson Orin Nano, it is recommended to run the system in MAX power mode (`sudo jetson_clocks`). Furthermore, a swapfile should be created to prevent memory-related issues when running large models.

</Admonition>

### 3.2. Voice-to-Action Implementation

This section defines the requirements for converting spoken language into text for processing by the LLM.

#### 3.2.1. OpenAI Whisper for Transcription

The primary tool for Speech-to-Text (STT) will be **OpenAI Whisper**.

-   **Installation:** Students must install the Whisper library and its dependency, `ffmpeg`.
    ```bash
    pip install openai-whisper
    sudo apt-get install ffmpeg
    ```
-   **Model Selection:** The specification must include guidance on model selection. For the Jetson Orin Nano, the `base.en` or `small.en` models are recommended for a balance of performance and accuracy. The larger models may be used if performance allows.
-   **Python Implementation:** The system must include a Python script (as a ROS 2 node) that can:
    1.  Listen for audio from the ReSpeaker Mic Array using a library like `sounddevice` or `PyAudio`.
    2.  Trigger transcription when speech is detected.
    3.  Use the Whisper library to transcribe the audio.
    4.  Publish the resulting text to a ROS 2 topic (e.g., `/voice_command`).

**Example Python Code Snippet:**

```python
import whisper
import sounddevice as sd
import numpy as np
import rclpy
from std_msgs.msg import String

# Assume rclpy and a ROS 2 node are initialized elsewhere

# Load the Whisper model
model = whisper.load_model("base.en")

def audio_callback(indata, frames, time, status):
    """This is called (from a separate thread) for each audio block."""
    # Basic energy-based VAD (Voice Activity Detection) can be implemented here
    # For simplicity, we assume a fixed recording duration
    
    # Transcribe the audio
    audio_data = indata.flatten().astype(np.float32)
    result = model.transcribe(audio_data)
    
    # Publish the transcribed text
    command_publisher = node.create_publisher(String, '/voice_command', 10)
    msg = String()
    msg.data = result['text']
    command_publisher.publish(msg)
    rclpy.logging.get_logger('voice_node').info(f"Published command: {msg.data}")

# --- In the main execution block ---
# fs = 16000 # Sample rate
# with sd.InputStream(callback=audio_callback, channels=1, samplerate=fs):
#     print("Listening for commands...")
#     # Keep the script running
```

#### 3.2.2. Security Mandates: API Key & Data Privacy

As mandated by **Constitution Section III (Security)**, all interactions with external APIs must be handled securely.

<Admonition type="danger" title="CRITICAL: API Key Security">

API keys for services like OpenAI **MUST NOT** be hardcoded into source files. They must be loaded from environment variables or a secure configuration manager.

**Implementation Requirement:**

All Python scripts using the OpenAI API must retrieve the key as follows:

```python
import os
from openai import OpenAI

# Load the API key from an environment variable
api_key = os.getenv("OPENAI_API_KEY")
if not api_key:
    raise ValueError("OPENAI_API_KEY environment variable not set.")

client = OpenAI(api_key=api_key)
```
The `.bashrc` file on the Jetson should be updated to include `export OPENAI_API_KEY='your-key-here'`.

</Admonition>

**Data Privacy:**
The specification must include a warning regarding data privacy. When using cloud-based STT or LLM services, voice data is sent to third-party servers. For sensitive applications, local models (like a locally run Whisper) are strongly preferred.

### 3.3. Cognitive Planning with LLMs

The core of this module is using an LLM to translate a high-level command into a sequence of executable ROS 2 actions.

#### 3.3.1. Methodology: LLM as a Task Planner

1.  **Action Primitives:** A set of well-defined, safe, and parameterized **ROS 2 actions** must be created. Examples: `Navigate(target: str)`, `PickUp(object_id: str)`, `Place(location: str)`.
2.  **Prompt Engineering:** A ROS 2 node (the "Planner Node") will subscribe to the `/voice_command` topic. Upon receiving a command, it will construct a detailed prompt for the LLM. This prompt must include:
    -   The user's command (e.g., "Please tidy up the table").
    -   The current state of the world (if available from a perception system).
    -   A "manifest" of available ROS 2 actions, their descriptions, and their parameters.
3.  **LLM Interaction:** The Planner Node will send the prompt to an LLM (e.g., GPT-4) via its API.
4.  **Plan Parsing:** The LLM is expected to return a structured response, such as a JSON array of action calls. The Planner Node must parse this response.

**Example Prompt Structure:**

```text
You are an AI assistant controlling a humanoid robot. Your task is to convert a user's request into a sequence of simple, executable actions. You must only use the functions provided below.

Available Functions:
1. Navigate(target: str) - Moves the robot to a predefined location (e.g., 'kitchen', 'table', 'trash_bin').
2. PickUp(object_id: str) - Picks up a specific object.
3. Place(location: str) - Places the currently held object at a location.
4. ScanForObjects(object_type: str) - Looks for objects of a certain type (e.g., 'cup', 'bottle').

User Request: "Can you please find the empty bottle and throw it away?"

Based on this, generate a JSON list of functions to execute in sequence.
```

#### 3.3.2. Safety and Execution
-   **Plan Validation:** Before execution, the Planner Node **MUST** validate the plan received from the LLM. Validation includes:
    -   Ensuring every action in the plan exists in the action manifest.
    -   Verifying that the parameters for each action are of the correct type and within a safe range.
-   **Sequential Execution:** The Planner Node will execute the validated actions one by one, waiting for each to complete successfully before starting the next.
-   **Error Handling:** If any action fails, the execution sequence must be halted, and the system should report the failure back to the user.

### 3.4. Capstone Project: Autonomous Humanoid

The capstone project will integrate all subsystems to create an autonomous humanoid robot capable of performing helpful tasks in a simulated or real-world environment.

**Sub-system Requirements:**

1.  **Voice Sub-system:**
    -   **Requirement:** Must capture and transcribe real-time voice commands using the Whisper pipeline.
    -   **Interface:** Publishes `std_msgs/String` messages to the `/voice_command` topic.

2.  **Cognitive Planning Sub-system:**
    -   **Requirement:** Must subscribe to `/voice_command`, generate an action plan using an LLM, validate it, and execute it.
    -   **Interface:** Calls the necessary ROS 2 action servers for navigation, perception, and manipulation.

3.  **Navigation Sub-system:**
    -   **Requirement:** Must be able to autonomously navigate to predefined locations within the environment using the ROS 2 Navigation Stack (Nav2).
    -   **Interface:** Provides a `NavigateToPose` action server that the Planning Sub-system can call.

4.  **Perception Sub-system:**
    -   **Requirement:** Must be able to detect, classify, and locate objects in the environment. This may use models like YOLO or DETR running on the Jetson.
    -   **Interface:** Provides a ROS 2 service or action that returns a list of detected objects and their poses (e.g., `(object_id, x, y, z)`).

5.  **Manipulation Sub-system:**
    -   **Requirement:** Must be able to perform basic pick-and-place operations using a robotic arm, controlled via MoveIt2 or a custom IK solver.
    -   **Interface:** Provides `PickUp` and `Place` action servers that accept object IDs or poses as goals.

---
**End of Specification**
---
