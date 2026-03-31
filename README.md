# Dialect-Driven Embodied AI Control System 🤖🗣️

## 📖 Project Overview
This project implements a high-performance **Voice-to-Command** pipeline designed for robotic interaction. It features robust support for **Chinese dialects** (specifically Shanghainese and Chongqing) and is optimized for real-world noisy environments through custom VAD (Voice Activity Detection) calibration.

## ✨ Key Technical Highlights
- **Hardware-Aware Audio Processing:** Handles 2-channel input with automatic mono conversion and 1.5x digital gain for low-sensitivity microphones.
- **Dynamic VAD Calibration:** Real-time RMS-based ambient noise estimation to set adaptive start/end thresholds.
- **Dialect Logic Engine:** Comprehensive keyword mapping for Shanghainese (e.g., "伐要动", "拿那") and Chongqing dialects.
- **Industrial Pipeline:** Multi-threaded architecture with a dedicated `ROBOT_QUEUE` for non-blocking command execution.

## 🛠️ Installation & Setup
1. **Clone the repository:**
   `git clone https://github.com/elilin349-eli/Robot-Interaction.git`
2. **Install dependencies:**
   `pip install sounddevice numpy websocket-client requests python-dotenv`
3. **Configure Environment:**
   Create a `.env` file with your iFlytek credentials (refer to `.env.example`).

## 🚀 Future Roadmap
- [x] Multi-dialect ASR Integration
- [ ] **Next:** YOLOv8-based Vision Module for spatial object localization
- [ ] **Next:** ROS2 Integration for Robotic Arm Kinematics
