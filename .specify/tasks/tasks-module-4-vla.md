# DEVELOPMENT TASKS: Module 4 - Vision-Language-Action (VLA)

**Version:** 1.0.0
**Date:** 2025-11-30
**References:**
*   Plan: `.specify/plans/plan-module-4-vla.md`
*   Specification: `.specify/specs/spec-module-4-vla.md`
*   Constitution: `.specify/memory/constitution.md`

---

## 1. Content Scaffolding & Initial Setup

| Task ID  | Description                                                                                             | Component | Effort |
| :------- | :------------------------------------------------------------------------------------------------------ | :-------- | :----- |
| M4.T1.1  | Create the directory `physical-ai-humanoid-robotics/docs/module-4-vla/`.                                  | Content   | Small  |
| M4.T1.2  | Create the file `docs/module-4-vla/_category_.json` and define the sidebar label and position.            | Content   | Small  |
| M4.T1.3  | Create the placeholder file `docs/module-4-vla/1-introduction-and-setup.md`.                                | Content   | Small  |
| M4.T1.4  | Create the placeholder file `docs/module-4-vla/2-voice-to-action-pipeline.md`.                              | Content   | Small  |
| M4.T1.5  | Create the placeholder file `docs/module-4-vla/3-cognitive-planning-with-llms.md`.                          | Content   | Small  |
| M4.T1.6  | Create the placeholder file `docs/module-4-vla/4-capstone-project-integration.md`.                          | Content   | Small  |

---

## 2. Hardware & Environment Setup Documentation

| Task ID  | Description                                                                                                                                 | Component | Effort |
| :------- | :------------------------------------------------------------------------------------------------------------------------------------------ | :-------- | :----- |
| M4.T2.1  | In `1-introduction-and-setup.md`, write the section detailing the hardware requirements (Jetson Orin, ReSpeaker Mic) using an `Admonition`.  | Content   | Small  |
| M4.T2.2  | In `1-introduction-and-setup.md`, write the step-by-step guide for flashing and setting up the NVIDIA Jetson Orin Nano with ROS 2 Humble.      | Content   | Medium |
| M4.T2.3  | In `1-introduction-and-setup.md`, write the section on how to connect and configure the ReSpeaker USB Mic Array as the default ALSA device. | Content   | Medium |
| M4.T2.4  | In `1-introduction-and-setup.md`, add an `Admonition` tip for Jetson power mode optimization (`jetson_clocks`) and creating a swapfile.      | Content   | Small  |

---

## 3. Voice-to-Action Pipeline (Whisper)

| Task ID  | Description                                                                                                                                              | Component | Effort |
| :------- | :------------------------------------------------------------------------------------------------------------------------------------------------------- | :-------- | :----- |
| M4.T3.1  | In `2-voice-to-action-pipeline.md`, write the introductory content explaining the VLA pipeline and the role of Speech-to-Text.                            | Content   | Small  |
| M4.T3.2  | Create the Python script for the Whisper STT ROS 2 node (`voice_transcriber_node.py`).                                                                    | Code      | Small  |
| M4.T3.3  | Implement the audio capture logic in `voice_transcriber_node.py` using `sounddevice` to listen to the microphone.                                          | Code      | Medium |
| M4.T3.4  | Implement the function in `voice_transcriber_node.py` to load the OpenAI Whisper model (e.g., `base.en`).                                                  | Code      | Small  |
| M4.T3.5  | Implement the transcription logic in the audio callback to process the audio data with the Whisper model.                                                  | Code      | Medium |
| M4.T3.6  | Implement the ROS 2 publisher in `voice_transcriber_node.py` to publish the transcribed text to the `/voice_command` topic.                                | Code      | Small  |
| M4.T3.7  | In `2-voice-to-action-pipeline.md`, embed and explain the complete `voice_transcriber_node.py` code.                                                         | Content   | Medium |
| M4.T3.8  | In `2-voice-to-action-pipeline.md`, add a `danger` Admonition explaining the critical importance of not hardcoding API keys, showing the `os.getenv` method. | Content   | Small  |

---

## 4. Cognitive Planning Pipeline (LLM)

| Task ID  | Description                                                                                                                                                             | Component | Effort |
| :------- | :---------------------------------------------------------------------------------------------------------------------------------------------------------------------- | :-------- | :----- |
| M4.T4.1  | In `3-cognitive-planning-with-llms.md`, write the introduction to LLMs as task planners.                                                                                   | Content   | Small  |
| M4.T4.2  | Create the detailed ASCII diagram of the full VLA architecture as specified in the plan.                                                                                    | Diagram   | Medium |
| M4.T4.3  | Insert and explain the VLA architecture diagram in `3-cognitive-planning-with-llms.md`.                                                                                     | Content   | Small  |
| M4.T4.4  | Create the Python script for the LLM Planner ROS 2 node (`llm_planner_node.py`).                                                                                            | Code      | Small  |
| M4.T4.5  | Implement the ROS 2 subscriber in `llm_planner_node.py` to listen for commands on the `/voice_command` topic.                                                               | Code      | Small  |
| M4.T4.6  | Implement the prompt engineering function in `llm_planner_node.py` that formats the user command and the robot's action manifest into a single prompt for the LLM.          | Code      | Medium |
| M4.T4.7  | Implement the function to call the LLM API (e.g., OpenAI API) and retrieve the JSON-formatted action plan. Ensure API keys are loaded securely.                             | Code      | Medium |
| M4.T4.8  | Implement the safety validation layer in `llm_planner_node.py` to parse and check the LLM's response, ensuring all actions and parameters are valid and safe.                | Code      | Large  |
| M4.T4.9  | Implement the action execution loop in `llm_planner_node.py` that iterates through the validated plan and calls the corresponding ROS 2 action servers.                    | Code      | Large  |
| M4.T4.10 | In `3-cognitive-planning-with-llms.md`, add and explain the full `llm_planner_node.py` code, breaking it down into sections (subscribing, planning, validation, execution). | Content   | Large  |

---

## 5. Capstone Project Integration

| Task ID  | Description                                                                                                                                               | Component | Effort |
| :------- | :-------------------------------------------------------------------------------------------------------------------------------------------------------- | :-------- | :----- |
| M4.T5.1  | In `4-capstone-project-integration.md`, write the overview of the capstone project, defining the goal of integrating all subsystems.                          | Content   | Medium |
| M4.T5.2  | **[Navigation]** Write the section explaining how the `llm_planner_node` calls the Nav2 `NavigateToPose` action server, with a sample action call.           | Content   | Medium |
| M4.T5.3  | **[Perception]** Write the section explaining how a perception service can be used to provide the LLM with real-world context (e.g., object locations).        | Content   | Medium |
| M4.T5.4  | **[Manipulation]** Write the section explaining how the `llm_planner_node` calls the `PickUp` and `Place` action servers, likely from a MoveIt2 interface. | Content   | Medium |
| M4.T5.5  | Provide a final launch file example in `4-capstone-project-integration.md` that brings up all the necessary nodes for the full VLA capstone project.         | Content   | Large  |
