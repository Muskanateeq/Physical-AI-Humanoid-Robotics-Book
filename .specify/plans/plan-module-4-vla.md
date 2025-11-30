# DEVELOPMENT PLAN: Module 4 - Vision-Language-Action (VLA)

**Version:** 1.0.0
**Date:** 2025-11-30
**References:**
*   Specification: `.specify/specs/spec-module-4-vla.md`
*   Constitution: `.specify/memory/constitution.md`

## 1. Goal & Scope

### 1.1. Goal
The primary goal of this plan is to outline the development strategy for creating the Docusaurus course content for **Module 4: Vision-Language-Action (VLA)**. This plan details *how* to implement the requirements laid out in the specification, ensuring the content is practical, educational, and adheres to all project constraints.

### 1.2. Scope
The scope covers the creation of educational material that will guide a student through:
1.  **Hardware Setup:** Configuring the Jetson Orin Nano and ReSpeaker Mic Array.
2.  **Voice-to-Action Pipeline:** Building a ROS 2 system that uses OpenAI Whisper to transcribe voice commands.
3.  **Cognitive Planning:** Using LLMs to translate natural language into validated, safe, and executable ROS 2 action sequences.
4.  **Capstone Project Integration:** Providing a clear guide on how to integrate these VLA components with existing navigation, perception, and manipulation systems to complete the final project.

The content will be in-depth, featuring complete code examples and detailed architectural explanations.

---

## 2. Docusaurus Frontend Plan

### 2.1. Directory & File Structure
All content for this module will be located within the `physical-ai-humanoid-robotics/docs/module-4-vla/` directory. The planned file structure is as follows:

```
docs/
└── module-4-vla/
    ├── _category_.json
    ├── 1-introduction-and-setup.md
    ├── 2-voice-to-action-pipeline.md
    ├── 3-cognitive-planning-with-llms.md
    └── 4-capstone-project-integration.md
```

*   `_category_.json`: Defines the sidebar label as "Module 4: Vision-Language-Action" and its position.
*   `1-introduction-and-setup.md`: Covers the module's goals and provides step-by-step instructions for setting up the Jetson Orin and ReSpeaker Mic, as per the specification.
*   `2-voice-to-action-pipeline.md`: Details the creation of the Speech-to-Text ROS 2 node using Whisper.
*   `3-cognitive-planning-with-llms.md`: Explains how to build the planner node that interacts with an LLM to generate and validate action plans.
*   `4-capstone-project-integration.md`: Provides a guide to combine the VLA system with other robot subsystems (perception, navigation, manipulation) for the final project.

### 2.2. Use of Visual Aids & Admonitions
The content will be enriched using Docusaurus features to improve clarity and student learning:

*   **Admonitions:** Will be used extensively as defined in the specification.
    *   `<Admonition type="danger" title="CRITICAL: API Key Security">`: For all security-related warnings, especially API key handling.
    *   `<Admonition type="info" title="Mandatory Hardware">`: To clearly state hardware requirements from the Constitution.
    *   `<Admonition type="tip" title="Optimization Tip">`: For performance tips on the Jetson Orin Nano.
*   **Code Blocks:** All code will be presented in formatted blocks with language identifiers (e.g., ````python`, ````bash`).
*   **Diagrams:** Complex architectural flows will be visualized using text/ASCII diagrams as mandated below.

---

## 3. Technical Strategy

### 3.1. LLM Integration Flow Explanation
The core of the module, the cognitive planning flow, will be explained methodically. The plan is to present it as a continuous data journey:

1.  **Input:** Starts with the user's voice.
2.  **Capture:** The ReSpeaker mic captures the audio waveform.
3.  **Transcription:** A ROS 2 node on the Jetson Orin uses the Whisper model to convert the audio into a text string.
4.  **Publication:** This text is published on a ROS 2 topic (e.g., `/voice_command`).
5.  **Planning:** A dedicated "Planner Node" subscribes to this topic. It constructs a prompt containing the command and a manifest of available robot actions.
6.  **Reasoning:** The prompt is sent to an LLM, which returns a structured JSON plan.
7.  **Validation & Execution:** The Planner Node validates the plan for safety and then executes the actions sequentially by calling the appropriate ROS 2 action servers.

This entire flow will be illustrated with the diagram described below.

### 3.2. Diagram Mandate: The VLA Architecture Flowchart
To explain the complex VLA architecture, a detailed ASCII diagram will be included in `3-cognitive-planning-with-llms.md`. This diagram will provide a clear, high-level overview of the entire system, from voice input to robot action.

**Planned ASCII Diagram:**
```text
+-----------------------+      +--------------------------+
|   User Voice Command  |      |   (Humanoid Robot Env)   |
+-----------------------+      +--------------------------+
           |                                  ^
           v                                  | (6. Action Execution)
+-----------------------+      +--------------------------+
| ReSpeaker Mic Array   |      | ROS 2 Action Servers     |
| (Captures Audio)      |      | (Navigate, Pick, Place)  |
+-----------------------+      +--------------------------+
           |                                  ^
(1. Audio Stream)                             |
           v                                  |
+-------------------------------------------------------------+
| NVIDIA Jetson Orin Nano (ROS 2 Humble)                      |
|                                                             |
|  +-----------------------+      +-----------------------+   |
|  | Whisper STT Node      |----->| Planner Node          |---'
|  | (Transcribes Audio)   |      | (Cognitive Engine)    | (5. Calls Actions)
|  +-----------------------+      +-----------------------+   |
|           | (2. Text Command)            | (4. Validates Plan) |
|           |                              |                     |
|           '------------------------------' (3. Sends Prompt)  |
|                                            |                   |
|                                            v                   |
+-------------------------------------------------------------+
                                     +-----------------------+
                                     |  LLM API (e.g., OpenAI) |
                                     |  (Returns JSON Plan)    |
                                     +-----------------------+
```

### 3.3. Code Snippet Strategy
Code is critical for this module. The plan is to:
-   Provide **complete, functional ROS 2 nodes** as Python code snippets.
-   Ensure all code rigorously follows the security mandate from the constitution (i.e., loading API keys from environment variables, never hardcoding).
-   Include inline comments that explain *why* certain things are done, particularly in the prompt engineering and plan validation sections.
-   Structure the code to be easily copy-pasted and run by students on their target hardware.

---

## 4. Task Breakdown Mandate

This development plan mandates that the subsequent **Tasks** phase must break down the implementation of this module into highly granular, actionable steps. The generated task list must be detailed enough for an AI agent or developer to execute them one by one with minimal ambiguity.

**Examples of required task granularity:**

-   Task: Create the file `docs/module-4-vla/1-introduction-and-setup.md` with introductory content.
-   Task: Add a section to `1-introduction-and-setup.md` detailing the Jetson Orin Nano setup steps.
-   Task: Write the Python code for the Whisper STT ROS 2 node.
-   Task: Implement the `audio_callback` function within the STT node.
-   Task: Write the Python code for the Planner ROS 2 node.
-   Task: Implement the prompt engineering logic that combines the user command with the action manifest.
-   Task: Implement the safety validation function that checks the LLM's returned plan against the manifest.
-   Task: Create the ASCII diagram for the VLA architecture.
-   Task: Insert the ASCII diagram into `3-cognitive-planning-with-llms.md`.
