# Adaptive Formation Control For Collaborative Swarm Robots 🤖🛰️
<p align="center">
  <img src="https://github.com/user-attachments/assets/b18a6806-85b2-4496-8701-9b37eef12816" alt="Swarm Robots" width="800">
</p>
This repository contains the software architecture, control algorithms, and hardware integration codes for the **Adaptive Formation Control of Swarm Robots** project. The system utilizes centralized vision processing (AprilTags) combined with decentralized swarm behaviors powered by Fuzzy Logic and ESP-NOW communication.

> **📖 Academic Citation (IEEE)**
> This project has been published and presented at the 8th International Congress on Human-Computer Interaction, Optimization and Robotic Applications (ICHORA 2026). 
> *Emir Bekar et al., "Adaptive Formation Control for Collaborative Swarm Robots," IEEE.*
> [Read the full paper on IEEE Xplore](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=11537161&isnumber=11536968)

## 🛠️ System Architecture

The architecture is divided into two main layers: **High-Level Control (Python)** and **Low-Level Execution (ESP32/C++)**.

### 1. High-Level Python Controller (`/Python`)
The central brain (`leader_nav.py`) runs on a PC and processes real-time camera feeds. 
* **Perception:** Uses `pupil_apriltags` to track robot positions and headings in real-time.
* **Tracking:** Implements a Constant Velocity **Kalman Filter** to predict robot movements during occlusions and filter camera noise.
* **Navigation:** Employs **Adaptive Pure Pursuit** with cross-track and curvature lookahead adjustments.
* **Swarm Intelligence:** Uses **Fuzzy Logic (skfuzzy)** to calculate Adaptive Cruise Control (ACC) speeds and maintain formation cohesion without hard braking.
  
* **Obstacle Avoidance:** Implements Artificial Potential Fields (APF) to repel robots from walls and each other dynamically.

### 2. ESP32 Gateway & Swarm Nodes (`/Arduino`)
* **Gateway (`Typce_C_Gateway_RL.ino`):** Acts as a high-speed serial bridge between the Python controller and the swarm network. Optimized to 80MHz to prevent thermal throttling while managing continuous serial-to-ESP-NOW conversion.
* **Swarm Robots (`Robot_ESP32_RL_x.ino`):** Each robot runs a localized execution loop. They receive target PWM commands via **ESP-NOW broadcast (zero-latency)**, map the commands to L298N motor drivers, and include a 500ms dead-man's switch failsafe.

## 🚀 Quick Start

### Dependencies
Ensure you have Python 3.8+ installed. Install the required packages:
```bash
pip install -r requirements.txt
