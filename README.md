Voice-Robot: LLM-Driven Multimodal Robotic Interaction System
📌 Project Overview
Voice-Robot is an integrated robotic control system that allows users to operate the SO-101 Robotic Arm using natural language (including dialects). By combining real-time speech recognition, LLM-based intent reasoning, and the ROS 2 communication framework, this project bridges the gap between high-level human commands and low-level hardware execution.

Key Highlight: This system is designed for robustness in real-world environments, featuring a custom-built VAD (Voice Activity Detection) mechanism and a fallback logic that ensures stability even when the network or LLM fails.

🏗️ System Architecture
(Note: Replace this with your uploaded image in the /docs folder)

The system is divided into three primary layers:

Cognition Layer: Handles multi-dialect speech-to-text (STT) via iFlytek and processes complex intentions using an LLM Reasoner.

Control Layer (ROS 2): Manages the TF Tree, inverse kinematics (IK), and state monitoring through specialized ROS 2 nodes.

Hardware Layer: Interfaces with the SO-101 Arm (LeRobot framework) via serial communication with overload protection and smooth trajectory playback.

🚀 Core Features
Intelligent Reasoning: Uses LLM (GPT/Llama) to parse non-standard commands (e.g., "I'm finished with this, take it away" → PLACE command).

Robust ROS 2 Integration: Real-time TF broadcasting and marker visualization in RVIZ.

Hardware Safety: Implements overload recovery and smooth start-up sequences to prevent servo damage.

Dialect Support: Specifically optimized for Mandarin and Southwestern dialects (Chongqing/Sichuan).

🛠️ Installation & Setup
Prerequisites
Ubuntu 22.04 + ROS 2 Humble (Recommended)

Python 3.10+

SO-101 Robotic Arm (or simulated environment)

Environment Configuration
Clone the repository and configure your API keys:

Bash
git clone https://github.com/your-username/Voice-Robot.git
cd Voice-Robot
cp config/env.example .env
# Edit .env with your iFlytek and OpenAI credentials
📂 File Structure
voice_robot.py: Main entry point for the voice interaction loop.

tf_node.py: ROS 2 node for kinematics and TF broadcasting.

llm_reasoner.py: Intent extraction and JSON-based command normalization.

lerobot_hardware.py: Low-level servo control and data collection.

commands.yaml: Extensible command mapping and TTS feedback definitions.
