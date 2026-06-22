<div align="center">
  
# Adaptive Formation Control For Collaborative Swarm Robots 🤖🛰️


*Developed by: Emir Bekar*

<br>

<p align="center">
  <!-- GÖREV 1: BURAYA SLAYT 3'TEKİ 4'LÜ ROBOT FOTOĞRAFINI (Leader, R1, R2, R3 YAZAN) EKLE -->
  <img src="https://github.com/user-attachments/assets/d77515b5-d672-4fab-b070-261ce372201c" alt="Swarm Robots" width="250">
</p>

</div>

## 🎯 Objective
The primary goal of this project is to design an adaptive formation control system so a swarm of simple, decentralized mobile robots can move together collectively, navigate dynamic environments, and adapt their formation geometry based on spatial constraints.

---

## ⚙️ Hardware Components
Each swarm agent is designed from scratch, utilizing a custom 3D-printed chassis and integrated electronics for real-time responsiveness.
* **Microcontroller:** ESP32 (Optimized to 80MHz for thermal efficiency)
* **Motor Driver:** TB6612FNG Dual Motor Driver
* **Actuators:** N20 Micro Gear Motors & Caster Wheel
* **Power:** 2x 18650 Li-ion Batteries with LM2596 Voltage Regulator
* **Localization:** Top-mounted AprilTag (tag36h11) for absolute positioning

---

## 🧠 System Architecture & Control Flow

The architecture consists of a centralized perception layer and a decentralized execution layer.

1. **Perception & Vision:** An overhead camera (Samsung S24 streaming at 720p/30fps) captures the arena. `pupil_apriltags` extracts the Pose (x, y, θ) of each robot.
2. **Main Controller (Python):** Processes the vision data, filters noise using a Constant Velocity **Kalman Filter**, and runs the navigation algorithms.
3. **Wireless Communication:** A dedicated Type-C Gateway ESP32 broadcasts PWM commands to the swarm via **ESP-NOW** protocol for zero-latency execution.

<p align="center">
  <!-- GÖREV 2: BURAYA SLAYT 13'TEKİ "FUZZY CONTROL ARCHITECTURE" AKIŞ ŞEMASINI EKLE -->
  <img src="BURAYA_AKIS_SEMASI_GELECEK" alt="Control Architecture" width="700">
</p>

### 🚙 Steering Branch (Pure-Pursuit)
Robots maintain their trajectories using an Adaptive Pure-Pursuit controller. The lookahead distance dynamically adjusts based on the robot's current speed and cross-track error to prevent corner-cutting and oscillatory behavior.

### 🚀 Speed Branch (Fuzzy Logic ACC & Cohesion)
* **Line Formation (ACC):** Uses a Fuzzy Logic Adaptive Cruise Control. Memberships (Danger, Caution, Safe) calculate a scaling factor (Stop, Slow, Fast) based on the distance to the robot ahead, maintaining a specific slot gap (0.55 * Spacing).
* **Triangle Formation (Cohesion):** Focuses on group's Center of Mass (COM). A centroid-offset fuzzy logic prevents the apex robot from rushing ahead, ensuring lateral alignment.

---

## 📊 Results and Data Analysis

The system was rigorously tested in various obstacle courses (S-curves, narrow corridors, and parking slots).

<p align="center">
  <!-- GÖREV 3: BURAYA SLAYT 33'TEKİ "STATE DISTRIBUTION PER ROBOT" (PIE CHARTS) GRAFİĞİNİ EKLE -->
  <img src="BURAYA_PASTA_GRAFIKLERI_GELECEK" alt="State Distribution" width="800">
</p>

**Key Telemetry Highlights:**
* **Path Efficiency:** Achieved up to 0.96 net/gross path efficiency for the leader.
* **Collision Avoidance:** Zero physical collisions during the full-course run. The Artificial Potential Field (APF) effectively repelled robots from walls and each other.
* **Tag Reliability:** Maintained a 100% AprilTag detection rate throughout the runs, supported by predictive occlusion handling.

---

## 🎮 Custom Parkour Editor (Sim-to-Reality)
The Python backend includes a built-in, Minecraft-style grid editor to design custom physical courses. 

<p align="center">
  <!-- GÖREV 4: BURAYA SLAYT 40'TAKİ "PARKOUR DESIGN" (KIRMIZI DUVAR ÇİZİM) EKRAN GÖRÜNTÜSÜNÜ EKLE -->
  <img src="BURAYA_DUVAR_CIZIM_FOTOSU_GELECEK" alt="Parkour Design" width="400">
</p>

* **[Left Click]:** Paint a 40x40px obstacle block.
* **[Right Click]:** Erase block.
* Obstacles are instantly merged into larger bounding boxes for optimized visibility graph routing (Dijkstra) and wall-repulsion algorithms.

---

## 🚀 Quick Start & Installation

### Requirements
Ensure you have Python 3.8+ installed. Install the dependencies:
```bash
pip install -r requirements.txt
