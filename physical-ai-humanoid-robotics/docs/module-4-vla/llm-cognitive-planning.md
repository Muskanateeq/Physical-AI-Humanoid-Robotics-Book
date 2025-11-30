---
id: llm-cognitive-planning
title: 'Chapter 13: LLM Cognitive Planning'
sidebar_position: 3
---

import Admonition from '@theme/Admonition';

# 3. LLM Cognitive Planning

Your robot can now hear commands. But how does it *understand* "clean the room" and know what actions to take? This is where Cognitive Planning using a Large Language Model (LLM) comes in.

In this section, we will build the brain of our operation: a ROS 2 node that:
1.  Subscribes to the `/voice_command` topic.
2.  Constructs a carefully engineered prompt for an LLM (like OpenAI's GPT series).
3.  Sends the command to the LLM and asks it to return a structured, step-by-step plan using only actions the robot knows how to do.
4.  Validates the plan to ensure it's safe to execute.

## The VLA Architecture

Before we code, it's crucial to understand the entire Vision-Language-Action (VLA) data flow. The following diagram illustrates how information moves through our system, from your voice to the robot's physical actions.

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

## Step 1: Install OpenAI Library
We'll use OpenAI's API to access their powerful language models.

```bash
pip install openai
```

## Step 2: Create the Planner Node (`llm_planner_node.py`)

This node is the "Cognitive Engine" from our diagram. It's responsible for thinking, planning, and validating.

<Admonition type="danger" title="CRITICAL: API Key Security">

This time, an API key is **REQUIRED**. Before running this node, you MUST set your OpenAI API key as an environment variable. **DO NOT a hardcode the key in the script.**

```bash
# In your terminal, or added to your ~/.bashrc file
export OPENAI_API_KEY='sk-...'
```
</Admonition>

Here is the complete code for the planner node.

```python
# llm_planner_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from openai import OpenAI
import os
import json

class LLMPlannerNode(Node):
    def __init__(self):
        super().__init__('llm_planner_node')
        self.get_logger().info('LLM Planner Node started.')

        # --- OpenAI Client Initialization ---
        try:
            self.client = OpenAI(api_key=os.environ['OPENAI_API_KEY'])
        except KeyError:
            self.get_logger().fatal("OPENAI_API_KEY environment variable not set! Shutting down.")
            raise Exception("OPENAI_API_KEY not set")

        # --- Robot's Known Actions (The "Action Manifest") ---
        # This is a critical part of the prompt. It tells the LLM what the robot can do.
        self.action_manifest = """
        [
            {"name": "navigate_to", "description": "Moves the robot to a specific, predefined location.", "parameters": [{"name": "location", "type": "string", "enum": ["kitchen", "living_room", "charging_dock"]}]},
            {"name": "find_object", "description": "Looks for an object of a certain type and returns its ID if found.", "parameters": [{"name": "object_type", "type": "string", "enum": ["bottle", "cup", "book"]}]},
            {"name": "pickup_object", "description": "Picks up an object using its ID.", "parameters": [{"name": "object_id", "type": "string"}]},
            {"name": "place_object", "description": "Places the held object at a location.", "parameters": [{"name": "location", "type": "string", "enum": ["table", "trash_bin"]}]}
        ]
        """
        self.known_actions = {action['name'] for action in json.loads(self.action_manifest)}

        # --- ROS 2 Subscriber ---
        self.subscription = self.create_subscription(
            String,
            '/voice_command',
            self.command_callback,
            10)
        self.get_logger().info("Subscribed to /voice_command. Awaiting commands.")

    def command_callback(self, msg):
        command = msg.data
        self.get_logger().info(f"Received command: '{command}'")
        self.get_logger().info("Requesting a plan from LLM...")
        
        # Construct the prompt for the LLM
        system_prompt = f"""
You are a helpful robot assistant. Your task is to break down a user's command into a series of robotic actions.
You can ONLY use the functions available in the action manifest provided.
Respond with a valid JSON array of action objects. Each object should have a 'function' name and a 'parameters' object.
Do not add any explanations or conversational text in your response. Only the JSON array.
If the command cannot be fulfilled with the available actions, respond with an empty JSON array [].

Action Manifest:
{self.action_manifest}
"""

        try:
            # --- Send request to LLM ---
            response = self.client.chat.completions.create(
                model="gpt-4-turbo",
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": command}
                ],
                response_format={"type": "json_object"}
            )
            
            plan_json_str = response.choices[0].message.content
            self.get_logger().info(f"LLM proposed plan (raw): {plan_json_str}")
            
            # --- Parse and Validate the Plan ---
            plan = json.loads(plan_json_str).get('plan', []) # Expecting {"plan": [...]}
            if self.is_plan_valid(plan):
                self.get_logger().info(f"Plan is valid! Executing plan...")
                self.execute_plan(plan)
            else:
                self.get_logger().error("LLM returned an invalid or unsafe plan. Aborting.")

        except Exception as e:
            self.get_logger().error(f"An error occurred while communicating with OpenAI or processing the plan: {e}")

    def is_plan_valid(self, plan):
        """
        Safety First! This function validates the plan from the LLM.
        """
        if not isinstance(plan, list):
            self.get_logger().error("Validation failed: Plan is not a list.")
            return False
        
        for action in plan:
            if 'function' not in action or 'parameters' not in action:
                self.get_logger().error(f"Validation failed: Action missing 'function' or 'parameters': {action}")
                return False
            if action['function'] not in self.known_actions:
                self.get_logger().error(f"Validation failed: Unknown function '{action['function']}'")
                return False
        
        self.get_logger().info("Plan validation successful.")
        return True

    def execute_plan(self, plan):
        """
        This is a placeholder for where you would call ROS 2 action servers.
        In the final capstone, this will trigger real robot actions.
        """
        self.get_logger().info("--- STARTING PLAN EXECUTION ---")
        for i, action in enumerate(plan):
            self.get_logger().info(f"Step {i+1}: Executing {action['function']} with params {action['parameters']}")
            # In a real system, you would do something like:
            # future = self.action_client.send_goal_async(action_goal)
            # rclpy.spin_until_future_complete(self, future)
            # self.get_logger().info("Step completed.")
        self.get_logger().info("--- PLAN EXECUTION FINISHED ---")


def main(args=None):
    rclpy.init(args=args)
    try:
        llm_planner_node = LLMPlannerNode()
        rclpy.spin(llm_planner_node)
    except Exception as e:
        print(f"Node failed to initialize or run: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### How It Works
1.  **Action Manifest**: This is the most important part of the prompt. It's a JSON string that explicitly tells the LLM what functions the robot can perform and what parameters they accept. This constrains the LLM, preventing it from hallucinating impossible actions.
2.  **System Prompt**: We give the LLM a persona ("You are a helpful robot assistant") and clear instructions: use only the functions provided and respond *only* with a JSON array. This is called prompt engineering.
3.  **LLM Call**: When a command is received, the node sends the system prompt and the user command to the OpenAI API, requesting a JSON response.
4.  **Validation**: This is a critical safety step. The `is_plan_valid` function checks the LLM's response to ensure it's a list and that every action in the list is one we defined in our manifest. **Never trust an LLM's output without validation.**
5.  **Execution (Placeholder)**: The `execute_plan` function currently just prints the steps. In the final capstone project, this is where you will make calls to your ROS 2 action clients (for navigation, manipulation, etc.) to make the robot move.

### Exercise: Test the Planner Node
For this exercise, we will use the two terminals from the previous step.
1.  Save the code above as `llm_planner_node.py` in your package.
2.  Make sure your `OPENAI_API_KEY` is exported.
3.  Build and source your workspace again.
4.  In terminal 1, run the voice transcriber node:
    ```bash
    ros2 run your_package_name voice_transcriber_node
    ```
5.  In terminal 2, run the new planner node:
    ```bash
    ros2 run your_package_name llm_planner_node
    ```
6.  Now, speak a command like: **"Find the bottle and then go to the kitchen."**
    -   Watch the first terminal. You should see it transcribe your text.
    -   Watch the second terminal. It should log that it received the command, requested a plan, and then print the step-by-step plan it received from the LLM.

You have now built a system that can listen and think. The final step is to connect this thinking engine to the robot's body.
