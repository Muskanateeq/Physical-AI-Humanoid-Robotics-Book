---
id: voice-to-action-whisper
title: 'Chapter 12: Voice-to-Action with Whisper'
sidebar_position: 2
---

import Admonition from '@theme/Admonition';

# 2. Voice-to-Action Pipeline with Whisper

Now that our hardware is configured, it's time to build the first part of our VLA pipeline: the robot's hearing. In this section, we will create a ROS 2 node that:
1.  Listens for audio from the ReSpeaker microphone.
2.  Uses the powerful **OpenAI Whisper** model to transcribe speech into text in real-time.
3.  Publishes the transcribed text onto a ROS 2 topic for other parts of the robot's brain to use.

## Step 1: Install Whisper
First, we need to install the `openai-whisper` library and its required dependency `ffmpeg`.

```bash
# Install the python package
pip install openai-whisper

# Install the system dependency
sudo apt-get update && sudo apt-get install -y ffmpeg
```

## Step 2: Create the ROS 2 Node (`voice_transcriber_node.py`)

Create a new Python file in your ROS 2 package's source directory. This node will contain all the logic for our voice transcription service.

<Admonition type="danger" title="CRITICAL: API Key Security">

If you were using the Whisper API directly (instead of a local model), you would need an API key. As mandated by the course Constitution, keys **MUST NEVER** be hardcoded. They should always be loaded from environment variables. For a local model like this, no key is needed, but the principle is critical for the next section.

Example of safe key handling:
```python
import os
# This is how you would safely load a key if needed
# api_key = os.getenv("OPENAI_API_KEY")
# if not api_key:
#     raise ValueError("OPENAI_API_KEY environment variable not set.")
```
</Admonition>

Here is the complete code for the node. We will break down how it works below.

```python
# voice_transcriber_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np
import sounddevice as sd
import whisper

class VoiceTranscriberNode(Node):
    def __init__(self):
        super().__init__('voice_transcriber_node')
        self.get_logger().info('Voice Transcriber Node started.')

        # ROS 2 Publisher for the transcribed command
        self.publisher_ = self.create_publisher(String, '/voice_command', 10)

        # --- Whisper Model Configuration ---
        self.model_name = "base.en"  # "base.en" is a good balance for the Jetson Orin
        self.get_logger().info(f"Loading Whisper model: {self.model_name}...")
        self.model = whisper.load_model(self.model_name)
        self.get_logger().info("Whisper model loaded successfully.")

        # --- Audio Stream Configuration ---
        self.sample_rate = 16000  # 16kHz
        self.channels = 1
        self.block_size = 4096 # Buffer size for each audio chunk
        
        # Start listening
        self.stream = sd.InputStream(
            callback=self.audio_callback,
            samplerate=self.sample_rate,
            channels=self.channels,
            blocksize=self.block_size,
            dtype='int16' # Use int16, then convert to float32 for Whisper
        )
        self.stream.start()
        self.get_logger().info("Listening for voice commands...")

    def audio_callback(self, indata, frames, time, status):
        """
        This function is called by the sounddevice stream for each new audio chunk.
        """
        if status:
            self.get_logger().warn(f'Audio callback status: {status}')

        # --- Simple Voice Activity Detection (VAD) ---
        # We check the energy of the audio chunk to see if it's loud enough to be speech.
        volume_norm = np.linalg.norm(indata) * 10
        if volume_norm > 100: # This threshold may need tuning
            self.get_logger().info(f'Speech detected! (Volume: {volume_norm:.2f}) Transcribing...')

            # Convert audio data from int16 to float32 as required by Whisper
            audio_float32 = indata.flatten().astype(np.float32) / 32768.0
            
            # Transcribe the audio chunk
            result = self.model.transcribe(audio_float32)
            transcribed_text = result['text'].strip()

            if transcribed_text:
                self.get_logger().info(f"Whisper transcribed: '{transcribed_text}'")
                
                # Publish the transcribed text to the ROS 2 topic
                msg = String()
                msg.data = transcribed_text
                self.publisher_.publish(msg)
        else:
            # This is useful for debugging your microphone and VAD threshold
            # self.get_logger().info(f'Silence detected. (Volume: {volume_norm:.2f})')
            pass


def main(args=None):
    rclpy.init(args=args)
    voice_transcriber_node = VoiceTranscriberNode()
    try:
        rclpy.spin(voice_transcriber_node)
    except KeyboardInterrupt:
        pass
    finally:
        voice_transcriber_node.stream.stop()
        voice_transcriber_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

```

### How It Works
1.  **Initialization**: The `VoiceTranscriberNode` class is initialized. It creates a ROS 2 publisher for the `/voice_command` topic and loads the specified Whisper model (`base.en`) into memory.
2.  **Audio Stream**: It opens a `sounddevice.InputStream`. This is a live feed from your default microphone (which we set to the ReSpeaker array). For every chunk of audio it records, it calls the `audio_callback` function.
3.  **Voice Activity Detection (VAD)**: Inside `audio_callback`, we calculate the "energy" or volume of the audio chunk. If it's above a certain threshold, we assume someone is speaking and proceed to transcribe. This prevents the model from wasting resources trying to transcribe silence.
4.  **Transcription**: The audio data is converted to the format Whisper expects (a float32 NumPy array) and passed to `model.transcribe()`.
5.  **Publishing**: The resulting text is cleaned up and published as a `std_msgs/String` on the `/voice_command` topic.

### Exercise: Test the Transcription Node
1.  Save the code above as `voice_transcriber_node.py` inside your ROS 2 package.
2.  Build your package (`colcon build`).
3.  Source your workspace (`source install/setup.bash`).
4.  In one terminal, run your new node:
    ```bash
    ros2 run your_package_name voice_transcriber_node
    ```
5.  In a second terminal, listen to the `/voice_command` topic:
    ```bash
    ros2 topic echo /voice_command
    ```
6.  Speak a command into the ReSpeaker microphone, such as "Hello robot, what time is it?". You should see the node terminal log "Speech detected!" and then see the transcribed text appear in the `ros2 topic echo` terminal.

If you see your words appear, congratulations! Your robot can now hear. In the next section, we will teach it how to *understand*.
