---
id: introduction-and-setup
title: 'Chapter 11: Introduction and Hardware Setup'
sidebar_position: 1
---

import Admonition from '@theme/Admonition';

# 1. Introduction and Hardware Setup

Welcome to Module 4: Vision-Language-Action (VLA). This is where we bridge the gap between human intent and robotic action. By the end of this module, you will have built a system that allows a humanoid robot to understand your voice commands, think about how to execute them, and perform the tasks in the physical world.

We will build a complete pipeline:
1.  **Voice to Text:** Capturing your voice and transcribing it into a command.
2.  **Text to Plan:** Using a Large Language Model (LLM) to "understand" the command and create a logical sequence of actions.
3.  **Plan to Action:** Executing the plan using ROS 2 action servers that control the robot.

Let's begin by setting up the powerful hardware that makes this possible.

## Mandatory Hardware

As outlined in the course constitution, this module requires specific, powerful hardware for edge AI computation and high-quality audio input.

<Admonition type="info" title="Mandatory Hardware">

-   **Edge AI Compute:** **NVIDIA Jetson Orin Nano (8GB Developer Kit)**. This device will run our ROS 2 nodes, AI models for transcription, and the cognitive planning logic.
-   **Audio Input:** **ReSpeaker 4-Mic Array**. This microphone provides excellent voice capture capabilities, which is crucial for accurate transcription.

</Admonition>

## Jetson Orin Nano Setup

Your Jetson Orin Nano is the brain of our operation. Follow these steps carefully to prepare it.

### Step 1: Flash the Operating System
Ensure your Jetson is flashed with the latest **NVIDIA JetPack SDK**. This provides the Linux operating system, CUDA drivers, and all necessary libraries for AI development.
-   Follow the official NVIDIA guide to flash your microSD card and boot the device for the first time.

### Step 2: Install ROS 2 Humble
Our robot's nervous system is ROS 2. We will use the **Humble Hawksbill** distribution.
```bash
# Follow the official ROS 2 installation guide for Ubuntu
# Ensure you install the 'ros-humble-desktop' version and source the setup script
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 3: Install System Dependencies
We need a few more system libraries for our Python code to work correctly.
```bash
sudo apt-get update
sudo apt-get install python3-pip python3-dev libffi-dev portaudio19-dev
```

<Admonition type="tip" title="Optimization: Max Performance & Swap">

For optimal performance when running AI models, run your Jetson in MAX power mode and create a swapfile to prevent out-of-memory errors.

1.  **Enable Max Power Mode:**
    ```bash
    sudo jetson_clocks
    ```

2.  **Create a Swapfile (e.g., 8GB):**
    ```bash
    sudo fallocate -l 8G /var/swapfile
    sudo chmod 600 /var/swapfile
    sudo mkswap /var/swapfile
    sudo swapon /var/swapfile
    sudo cp /etc/fstab /etc/fstab.bak
    echo '/var/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
    ```
</Admonition>

## ReSpeaker USB Mic Array Setup

For the robot to hear us clearly, we need to configure the ReSpeaker microphone.

### Step 1: Connect the Microphone
Plug the ReSpeaker Mic Array into one of the Jetson's USB ports.

### Step 2: Identify the Microphone
We need to find out how the system sees our microphone. Use the `arecord` command to list all recording devices.
```bash
arecord -l
```
Look for your ReSpeaker device in the output. It will look something like this:
`card 1: ArrayUAC10 [ReSpeaker 4 Mic Array (UAC1.0)], device 0: USB Audio [USB Audio]`
In this example, the card number is `1` and the device number is `0`.

### Step 3: Set as Default Device
Create a system-wide ALSA configuration file to make the ReSpeaker the default microphone.
```bash
sudo nano /etc/asound.conf
```
Paste the following content into the file. **Remember to replace `card 1` and `device 0` with the numbers you found in the previous step.**
```text
pcm.!default {
  type asym
  capture.pcm "mic"
  playback.pcm "speaker"
}

pcm.mic {
  type plug
  slave {
    pcm "hw:1,0"
  }
}

pcm.speaker {
  type plug
  slave {
    pcm "hw:1,0"
  }
}
```
Save the file (`Ctrl+X`, then `Y`, then `Enter`). Reboot your Jetson for the changes to take effect.

### Exercise: Test Your Microphone
Record a short audio clip to verify everything is working.
```bash
# Record a 5-second WAV file
arecord -d 5 -f S16_LE -r 16000 -c 1 test.wav

# Play it back
aplay test.wav
```
If you can hear yourself, your microphone is set up correctly! You are now ready to build the Voice-to-Action pipeline.
