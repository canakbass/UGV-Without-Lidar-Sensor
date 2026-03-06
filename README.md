# TEKNOFEST 2026 UGV - Autonomous System (V1 Simulation)

This repository contains the full software stack for the TEKNOFEST 2026 Unmanned Ground Vehicle competition.
The V1 release focuses on **Navigation Logic**, **Vision Processing**, and **Mock Physics Simulation**.

## 🚀 Features (V1)
*   **Modular ROS2 Architecture**: Divided into Control, Vision, Decision, and Bridge packages.
*   **Smart Navigation**:
    *   **Ultrasonic PID**: Lane centering using side distance difference.
    *   **Dynamic Offset**: Automatically hugs the right wall (-30cm) during "Side Slope" and "Sliding Obstacle" stages.
*   **Vision System**:
    *   **Stage Detection**: Recognizes traffic signs to switch autonomous states.
    *   **Optimized Processing**: Only activates Cone Detection algorithms when in the specific stage.
*   **Mock Simulation**:
    *   Simulates inertia, differential drive kinematics, and sensor noise.
    *   Outputs synthetic Odometry and Ultrasonic data for logic verification.
*   **Web Interface**:
    *   React + Vite frontend for real-time telemetry and manual/auto control.

## 🛠️ Installation (WSL / Ubuntu 24.04)
1.  **Clone the Repository**:
    ```bash
    git clone https://github.com/Start-Up-Tech/2026-UGV.git
    cd 2026-UGV
    ```

2.  **Setup Environment**:
    ```bash
    bash setup_wsl.sh
    ```

3.  **Launch Simulation**:
    ```bash
    source ~/ika_ws/install/setup.bash && ros2 launch ika_bringup system.launch.py
    ```

4.  **Start Web UI**:
    ```bash
    cd ika_web
    npm install
    npm run dev
    ```

## 🧠 Architecture
*   **Decisions**: `ika_decision/state_machine_node`
*   **Eyes**: `ika_vision/detector_node`
*   **Legs**: `ika_control/navigation_node` & `acceleration_node`
*   **Nerves**: `ika_bridge/serial_bridge_node`

## 📅 Roadmap
- [x] **V1**: Mock Simulation & Logic Verification
- [ ] **V2**: Unity Digital Twin Integration
- [ ] **V3**: Real Hardware Deployment (Jetson Nano + ESP32)
