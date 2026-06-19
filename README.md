# Adaptive Formation Control For Collaborative Swarm Robots

This project was developed to establish adaptive formation control for collaborative swarm robots. The system consists of 4 robots, a gateway managing the communication traffic, and a main control software that coordinates the entire swarm autonomously or manually.

## 📂 Project Components and File Structure

*   **Robot Codes (Arduino):** Arduino codes providing motor, sensor, and low-level movement controls for the 4 individual robots in the swarm.
*   **Gateway Code (Arduino):** The communication bridge (gateway) code that manages the data flow and distributes commands between the main controller and the 4 robots.
*   **`leader_nav` (Main Controller):** The brain of the system. This is the core navigation file used to plan, control, and execute the movements and adaptive formation arrangements of all robots.
