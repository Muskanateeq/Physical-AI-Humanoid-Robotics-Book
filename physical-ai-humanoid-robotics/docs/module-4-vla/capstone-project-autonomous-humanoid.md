---
id: capstone-project-autonomous-humanoid
title: 'Final Chapter: Capstone Project - The Autonomous Humanoid'
sidebar_position: 4
---

import Admonition from '@theme/Admonition';

<Admonition type="info" title="The Grand Challenge">

Welcome to your final project. You have taught your robot to listen and to think. Now, you will empower it to **act with purpose**. This capstone is not just about connecting nodes; it's about orchestrating a symphony of subsystems to perform complex, real-world tasks autonomously.

</Admonition>

## The Scenario: "The Proactive Office Assistant"

Your goal is to program the humanoid robot to act as a truly intelligent office assistant. The mission is an evolution of the last one, with more emphasis on intelligence and robustness:

> **"A user gives a natural language command. The robot must understand the intent, form a plan, and execute it. If it fails, it must attempt to recover. If it lacks information, it must seek it."**

This is the pinnacle of the course, testing your ability to integrate navigation, perception, and manipulation under the guidance of a powerful cognitive engine.

---

## System Architecture & Interfaces

The foundation of a robust system is its architecture. The interfaces between our VLA system and the robot's subsystems must be clear and well-defined.

| Subsystem    | Interface Name     | Type   | Definition (`.srv` or `.action`)                                                                |
| :----------- | :----------------- | :----- | :---------------------------------------------------------------------------------------------- |
| **Navigation** | `/navigate_to_pose`  | Action | `nav2_msgs/action/NavigateToPose` (Standard with Nav2)                                          |
| **Perception** | `/find_objects`    | Service | `string object_type --- YourObject[] found_objects` (where `YourObject` has an `id` and `pose`) |
| **Manipulation**| `/pickup_object`   | Action | `string object_id --- bool success`                                                             |
| **Manipulation**| `/place_at_pose`   | Action | `geometry_msgs/PoseStamped target_pose --- bool success`                                        |

---

## Phase 1: The World Model - The Robot's Memory

A truly intelligent agent needs a memory, or a "world model." This isn't a complex database; for our purpose, it's a simple Python dictionary within our planner node that stores the state of the world as the robot perceives it.

```text
+---------------------+      +----------------------+      +--------------------+
| Perception System   |----->|     World Model      |<-----|   LLM Planner      |
| (e.g., YOLO node)   |      | (A Python Dictionary)|      | (Reads state before|
|                     |      |                      |      |   planning)        |
| "I see bottle_01 at |      | state = {            |      |                    |
|  (x,y,z)"           |      |  'bottle_01': {      |      | "Where did I last |
|                     |      |    'pose': ...       |      |  see a bottle?"    |
+---------------------+      |  }                    |      +--------------------+
                             | }                      |
                             +----------------------+
```

Our planner will use this world model to make more informed decisions. For example, before planning to pick up an object, it can check its memory to see if it knows where that object is.

---

## Phase 2: The Professional Planner Node

We will now write the final version of our planner node. This version includes a world model, proper ROS 2 action clients, and a detailed execution loop, complete with an illustrative diagram.

### Visualizing the Execution Logic

The `execute_plan` function is the heart of the node. Here is the logic we will implement:

```text
(inside execute_plan)
      +-----------------+
      |  For each step  |
      |   in LLM plan   |
      +-----------------+
             |
             v
+----------------------------+
|  Route to specific handler |
|  (e.g., handle_navigate)   |
+----------------------------+
             |
             v
+----------------------------+
|  Send goal to ROS 2 Action |
|    Server (e.g., Nav2)     |
+----------------------------+
             |
             v
+----------------------------+
|  Wait for action to finish |
+----------------------------+
             |
             v
+----------------------------+
|   Was it successful?       |
+-------------+--------------+
              |
    +---------+----------+
    | (Yes)              | (No)
    v                    v
+----------------+   +-------------------+
| Continue to    |   | Log Error & Abort |
| next step      |   |       Plan        |
+----------------+   +-------------------+
```

### The Code: `llm_planner_node_pro_final.py`

```python
# llm_planner_node_pro_final.py
# ... (imports for rclpy, action clients, services, etc.)

class LLMPlannerNodeProFinal(Node):
    def __init__(self):
        super().__init__('llm_planner_node_pro_final')
        self.get_logger().info('THE ULTIMATE LLM PLANNER NODE IS ALIVE.')
        
        # Phase 1: The World Model
        self.world_state = {
            'last_known_objects': [] 
        }

        # ... (OpenAI Client and Action Manifest are the same)

        # --- Action & Service Clients ---
        self.get_logger().info("Connecting to robot's subsystems...")
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        # self.pickup_client = ActionClient(self, PickupObject, '/pickup_object')
        # self.place_client = ActionClient(self, PlaceAtPose, '/place_at_pose')
        # self.find_objects_client = self.create_client(FindObjects, '/find_objects')
        
        # A list of clients to check for availability
        self.action_clients = [self.nav_client, self.pickup_client, self.place_client]
        for client in self.action_clients:
            client.wait_for_server()
        self.get_logger().info("SUCCESS: All action servers are online.")

        # --- Subscriber ---
        self.subscription = self.create_subscription(String, '/voice_command', self.command_callback, 10)
        self.get_logger().info("Ready for your command.")

    def command_callback(self, msg):
        # This function gets the plan from the LLM, as before.
        # Let's assume it gets a valid plan called 'plan'.
        # ...
        self.execute_plan(plan)

    def execute_plan(self, plan):
        """
        Orchestrates the execution of the validated plan from the LLM.
        This function follows the logic from our diagram.
        """
        self.get_logger().info("--- Orchestrating Plan Execution ---")
        for i, action in enumerate(plan):
            self.get_logger().info(f"STEP {i+1}/{len(plan)}: {action['function']}({action['parameters']})")
            
            # Route the action to its handler
            success = self.execute_action(action)
            
            # Critical: Check for failure and abort
            if not success:
                self.get_logger().error(f"FATAL: Action failed at step {i+1}. Aborting mission.")
                # Future Enhancement: Trigger a recovery protocol here.
                return
        self.get_logger().info("--- MISSION COMPLETE: Plan executed successfully. ---")

    def execute_action(self, action):
        """
        Routes a single action from the plan to the corresponding
        ROS 2 client and waits for its completion.
        """
        function_name = action['function']
        params = action['parameters']
        
        # This is a simple router. In a larger system, this could be more dynamic.
        if function_name == 'navigate_to':
            return self.handle_navigate(params)
        elif function_name == 'find_object':
            return self.handle_find_object(params)
        elif function_name == 'pickup_object':
            return self.handle_pickup(params)
        else:
            self.get_logger().error(f"Unknown function '{function_name}' in execution router.")
            return False

    def handle_navigate(self, params):
        # 1. Get coordinates for the named location
        # goal_pose = self.get_pose_for_location(params['location'])
        # 2. Create the goal message
        # nav_goal = NavigateToPose.Goal(pose=goal_pose)
        # 3. Send the goal and wait
        # future = self.nav_client.send_goal_async(nav_goal)
        # rclpy.spin_until_future_complete(self, future)
        # 4. Return True or False based on the result status
        self.get_logger().info(f"Executing Navigation to {params['location']}...")
        return True # Placeholder for a successful action

    # ... (Implement handle_find_object, handle_pickup, etc. similarly)
    
# ... (main function)
```

---

## Phase 3: Launch & Final Challenge

Your final `ros2 launch` file brings this complex system to life. Using `IncludeLaunchDescription` is key to keeping your project organized as it grows.

### Final Capstone Challenge: Advanced Scenarios

A passing project completes the basic "fetch" task. A truly advanced project demonstrates robustness. Choose one of these challenges to elevate your project.

<Admonition type="success" icon="🚀" title="Challenge: Intelligent Error Recovery">

**Goal:** Don't just abort on failure. Recover.

1.  Modify `execute_plan` so that if an action returns `False`, it doesn't just abort.
2.  It should call a **new function** `request_recovery_plan(failed_action, reason)`.
3.  This function will prompt the LLM again, saying something like: *"I was executing a plan, but the action `navigate_to(kitchen)` failed because the path was blocked. Please provide a new plan to achieve the original goal."*
4.  The robot then attempts this new recovery plan.

</Admonition>

<Admonition type="success" icon="🤖" title="Challenge: Proactive Clarification">

**Goal:** Don't act on ambiguous commands. Ask for help.

1.  Create a "confidence score" for the LLM's plan. If the user says "get me the thing" and the LLM's plan involves an object with a generic name, consider the confidence low.
2.  If confidence is low, don't execute the plan. Instead, use a Text-to-Speech (TTS) engine (like `pyttsx3`) to make the robot ask a question.
3.  "I see multiple objects on the table. Could you please be more specific?"
4.  The robot then uses your Whisper node to listen for the answer and re-plans with the new, more specific information.

</Admonition>

You have reached the end of the formal instruction. The path forward is now yours to forge. Build, test, fail, and learn. Good luck.
