# Voice-Robot: An Embodied AI Interaction System with LLM & ROS 2

[![ROS 2](https://img.shields.io/badge/ROS2-Humble/Foxy-blue)](https://docs.ros.org/)
[![Python](https://img.shields.io/badge/Python-3.10+-yellow)](https://www.python.org/)
[![Framework](https://img.shields.io/badge/Framework-LeRobot-green)](https://github.com/huggingface/lerobot)

## 📖 Project Overview
**Voice-Robot** is an end-to-end **Embodied AI control pipeline** for the SO-101 robotic arm. It bridges the gap between high-level human natural language (including Mandarin and Southwestern dialects) and low-level hardware execution. By integrating **LLM reasoning** with the **ROS 2** communication framework, the system can parse ambiguous instructions into precise robotic trajectories.

> **Engineering Highlight:** Features a robust multi-threaded architecture with optimized VAD (Voice Activity Detection), automated hardware overload protection, and data collection interfaces for training future Vision-Language-Action (VLA) models.

---

## 🏗️ System Architecture
The following diagram represents the integrated pipeline, derived directly from the project's codebase (`voice_robot.py`, `tf_node.py`, `llm_reasoner.py`, and `lerobot_hardware.py`).

```mermaid
%%{init: {'theme': 'base', 'themeVariables': { 'primaryColor': '#E1F5FE', 'edgeLabelBackground':'#ffffff', 'tertiaryColor': '#fff'}}}%%
graph TD
    %% Define Styles
    classDef Cognition fill:#E1F5FE,stroke:#01579B,stroke-width:2px,rx:5,ry:5;
    classDef Control fill:#E8F5E9,stroke:#1B5E20,stroke-width:2px,rx:5,ry:5;
    classDef Execution fill:#FFFDE7,stroke:#FBC02D,stroke-width:2px,rx:5,ry:5;
    classDef ROS2 fill:#E3F2FD,stroke:#0D47A1,stroke-width:1px,stroke-dasharray: 5 5;
    classDef Fix fill:#FFEBEE,stroke:#B71C1C,stroke-width:1px,rx:10,ry:10;

    %% 1. Cognition Layer
    subgraph Cognition[<b>Layer 1: COGNITION (AI & Perception)</b><br/>voice_robot.py, llm_reasoner.py]
        direction TB
        Mic[Microphone Input] --> VAD[<b>Local VAD & Calibration</b><br/>audio_callback(), calibrate_mic()]
        VAD -- Valid Audio --> STT{STT Engine}
        STT -- Online --> iFly[iFlytek WS API]
        STT -- Local Fallback --> Whisper[WhisperEngine<br/>(openai-whisper)]
        
        iFly & Whisper -- Spoken Text --> Reasoner[<b>LLM Reasoner</b><br/>llm_reasoner.py]
        Yaml[commands.yaml<br/>(Configs/Keywords)] -. Fuzzy Match .-> Reasoner
    end

    %% 2. Control Layer
    subgraph Control[<b>Layer 2: CONTROL (ROS 2 & Dispatch)</b><br/>voice_robot.py, tf_node.py]
        direction TB
        Queue[<b>Mission Queue</b><br/>ROBOT_QUEUE]
        Dispatcher[<b>Mission Dispatcher</b><br/>robot_worker()]
        ROS2Node[<b>ROS 2 TF Node</b><br/>tf_node.py]
        
        Reasoner -- JSON Intent --> Queue
        Queue --> Dispatcher
        Dispatcher -- Arm Cmd --> ROS2Node
    end

    %% 3. Execution Layer
    subgraph Execution[<b>Layer 3: EXECUTION (Hardware & Data)</b><br/>lerobot_hardware.py]
        direction TB
        HWInterface[<b>Hardware Interface</b><br/>SO101LeaderArm]
        Serial[Serial Comm<br/>(SCServo SDK)]
        Arm[SO-101 Leader Arm]
        
        Recorder[<b>Gesture Recorder</b><br/>GestureRecorder]
        
        subgraph Fixes[v2.1 Stability Fixes]
            Overload[Overload Protection<br/>clear_overload_error()]
            Smooth[Smooth Startup<br/>approach_steps]
        end
        
        HWInterface --> Serial
        Serial <--> Arm
        
        Arm -. Proprioception .-> Recorder
    end

    %% Cross-layer Connections
    Dispatcher -- Manual Ctrl --> HWInterface
    Arm -. Feedback .-> ROS2Node
    
    %% VLA Data Pipeline
    Recorder -. Export .-> VLA[<b>VLA Dataset Ready</b><br/>(JSON/CSV for Training)]

    %% Styling Application
    class Cognition,Mic,VAD,STT,iFly,Whisper,Reasoner,Yaml Cognition;
    class Control,Queue,Dispatcher Control;
    class ROS2Node ROS2;
    class Execution,HWInterface,Serial,Arm,Recorder,Fixes,VLA Execution;
    class Overload,Smooth Fix;

```

---

## 🚀 Core Features

* **LLM-Based Intent Reasoning:** Uses LLM (GPT/Qwen) to parse ambiguous, non-standard natural language into structured JSON robot commands, ensuring robust intent extraction.
* **ROS 2 & Kinematics Monitoring:** Dedicated `tf_node.py` broadcasts the **TF Tree** and calculates real-time Forward/Inverse Kinematics based on the URDF model, viewable in RVIZ.
* **Hardware Overload Protection:** Implements critical engineering safety logic in `lerobot_hardware.py` to reset servo error states (`clear_overload_error`) and prevent damage.
* **Embodied Data Capture:** Gesture Recording module is ready to generate VLA (Vision-Language-Action) datasets for future policy model training.
* **Multi-Dialect STT:** Specifically optimized for Mandarin and Southwestern dialects (Chongqing/Sichuan) through dialect-specific API config and fuzzy keyword matching.

---

## 🛠️ Installation & Setup

### Prerequisites

* Ubuntu 22.04 + ROS 2 Humble (Recommended) or Foxy
* Python 3.10+
* SO-101 Robotic Arm (or simulated environment)

### Environment Configuration

1. **Clone the repository:**
```bash
git clone [https://github.com/elilin349-eli/Robot-Interaction.git](https://github.com/elilin349-eli/Robot-Interaction.git)
cd Robot-Interaction

```


2. **Install Dependencies:**
```bash
pip install -r requirements.txt 
# Includes: rclpy, numpy, sounddevice, websocket-client, scservo_sdk, python-dotenv

```


3. **Setup Keys:**
Update your iFlytek and LLM API credentials in `.env` (refer to `config/env.example`).

---

## 📂 File Structure

* **`voice_robot.py`**: The central mission control. Manages the high-level voice interaction loop, VAD, threading, and safe system shutdown.
* **`tf_node.py`**: A specialized **ROS 2 Node** that handles kinematics and broadcasts the **TF Tree** for visualization.
* **`llm_reasoner.py`**: The "brain" of the robot, leveraging LLMs to parse natural language into structured JSON commands.
* **`lerobot_hardware.py`**: Low-level hardware abstraction layer with servo communication, **v2.1 overload protection**, and VLA data collection interfaces.
* **`commands.yaml`**: Configuration file for mapping multi-dialect keywords to robot actions and customized TTS feedback.

```

```
