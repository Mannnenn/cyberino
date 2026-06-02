# Cyberino - Wheeled Inverted Pendulum Project

## 📹 Demo - 実機動作の様子



https://github.com/user-attachments/assets/4efd1a39-2690-4c71-840e-aec63ef2f1b4



---

**Cyberino** is a complete robotics project combining embedded C++ firmware and Python simulation/control research for a two-wheeled inverted pendulum robot with advanced motor control, state estimation, and reinforcement learning capabilities.

## 📚 Project Overview

This repository contains two integrated projects:

### 1. **Embedded Controller** (`inverted_pendulum/`)
Real-time firmware for ESP32-based wheeled inverted pendulum robot.

- **Language**: C++ / Arduino
- **Motor Control**: CAN-based CyberGear motor protocol
- **Sensors**: IMU (ICM42688) with Madgwick angle fusion
- **Communication**: Wi-Fi UDP telemetry streaming
- **Control**: State-feedback torque control with PID utilities

### 2. **Simulation & Control Research** (`inverted_pendulum_sim/`)
Python-based modeling, simulation, and learning framework.

- **Language**: Python 3.13
- **Dynamics**: SymPy symbolic modeling + numerical simulation
- **Control**: LQR, MPC (CasADi), and PPO reinforcement learning
- **Assets**: URDF/MJCF robot model with STL meshes
- **Hardware Bridge**: Live telemetry receiver for real-time data visualization

---

## 🗂️ Repository Structure

```
cyberino/
├── inverted_pendulum/                  # Embedded firmware
│   ├── inverted_pendulum.ino           # Main entry point & control loop
│   ├── RobotState.{h,cpp}              # State estimation & storage
│   ├── StateFeedback.{h,cpp}           # Linear state-feedback controller
│   ├── CyberGear.{h,cpp}               # CAN motor control protocol
│   ├── PID.{h,cpp}                     # PID/PI/PD controller utilities
│   └── madwick_1axis.{h,cpp}           # Madgwick IMU fusion
│
└── inverted_pendulum_sim/              # Simulation & research
    ├── pyproject.toml                  # Python dependencies
    ├── .python-version                 # Python 3.13
    ├── README.md                       # Detailed simulation docs
    │
    ├── Model & Dynamics:
    │   ├── calc_inverted_pendulum.py   # SymPy model derivation
    │   ├── non_linear_EoM.py           # Nonlinear dynamics wrapper
    │   ├── linear_EoM.py               # Linear state-space wrapper
    │   ├── generated_eq*.py            # Auto-generated dynamics functions
    │   └── matrices.npz                # Saved A/B/K matrices
    │
    ├── Control & Simulation:
    │   ├── mpc_sim_casadi.py           # MPC controller simulation
    │   ├── ppo_sim.py                  # RL environment & PPO training
    │   └── ppo_test.py                 # Trained model testing & visualization
    │
    ├── Hardware Integration:
    │   ├── get_telemetry.py            # UDP telemetry receiver/plotter
    │   ├── state_feedbacl.py           # State feedback implementation
    │   └── sensor_data.csv             # Recorded sensor logs
    │
    ├── Robot Model Assets:
    │   └── InvertPendulam/
    │       ├── urdf/InvertPendulam.urdf
    │       ├── mjcf/InvertPendulam.xml
    │       └── meshes/                 # STL mesh files for visualization
    │
    └── Artifacts:
        ├── ppo_wheeled_pendulum.zip    # Trained PPO model
        ├── ppo_wheeled_pendulum_training_plot.png
        ├── ppo_wheeled_pendulum_test_plot.png
        └── ppo_wheeled_pendulum_metrics.json

```

---

## 🚀 Quick Start

### **Option 1: Run Simulation & RL Training**

#### Prerequisites
```bash
python --version  # Should be 3.13+
```

#### Installation
```bash
cd inverted_pendulum_sim
pip install -e .  # or: uv sync (if using uv)
```

#### Model Derivation (Optional)
```bash
python calc_inverted_pendulum.py
# Generates: generated_eq1_func.py, generated_eq2_func.py, matrices.npz
```

#### MPC Simulation
```bash
python mpc_sim_casadi.py
```

#### PPO Training
```bash
python ppo_sim.py --help
# Typical: python ppo_sim.py --train --seed 42
```

#### PPO Testing & Visualization
```bash
python ppo_test.py --help
# Typical: python ppo_test.py --load ppo_wheeled_pendulum.zip --plot
```

#### Live Hardware Telemetry (if robot is running)
```bash
python get_telemetry.py
# Listens on UDP port 3333 for robot telemetry
```

---

### **Option 2: Flash Firmware to Robot**

#### Requirements
- ESP32 microcontroller (e.g., ESP32-S3)
- CAN transceiver (SN65HVD230 or equivalent)
- CyberGear motor + driver
- ICM42688 IMU
- Arduino IDE or PlatformIO

#### Steps
1. Install Arduino IDE + ESP32 board support
2. Install required libraries (CAN driver, IMU driver, etc.)
3. Update `inverted_pendulum.ino`:
   - Configure Wi-Fi SSID/password (lines 14-17)
   - Set telemetry endpoint IP/port
   - Adjust control gains as needed (line 105)
4. Flash via Arduino IDE or PlatformIO

---

## 📊 System Architecture

### **Control Flow**

```
┌─────────────────────┐
│   IMU Sensors       │
│ (ICM42688 @ 1kHz)   │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│ Madgwick Filtering  │
│ (angle estimation)  │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│ State Feedback      │ ◄─────── Gains from control tuning
│ Controller          │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│ CAN Motor Commands  │
│ (CyberGear protocol)│
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│  Wheel Motors       │
│ (left + right)      │
└─────────────────────┘
```

### **State Vector**

```
x = [θ, ω, φ_L, ω_L, φ_R, ω_R]
    pendulum angle/rate, left wheel angle/rate, right wheel angle/rate
```

### **Control Signal**

```
τ = K·x = [k_θ, k_ω, ...] · x
    Linear state feedback applied to wheel torques
```

---

## 🔧 Key Components

### **Embedded Firmware** (`inverted_pendulum/`)

| File | Purpose |
|------|---------|
| `inverted_pendulum.ino` | Main control loop, sensor polling, Wi-Fi/UDP, telemetry |
| `RobotState.h/.cpp` | Wheel encoder integration, angle conversion |
| `StateFeedback.h/.cpp` | Linear state-feedback torque law |
| `CyberGear.h/.cpp` | CAN protocol for motor command/feedback |
| `PID.h/.cpp` | PID/PI/PD utilities with anti-windup |
| `madwick_1axis.h/.cpp` | Madgwick-style IMU fusion for tilt estimate |

### **Simulation & Research** (`inverted_pendulum_sim/`)

| Module | Purpose |
|--------|---------|
| `calc_inverted_pendulum.py` | Symbolic model derivation, linearization, LQR computation |
| `non_linear_EoM.py` | Nonlinear ODE solver wrapper |
| `linear_EoM.py` | Linear discrete-time state-space simulator |
| `mpc_sim_casadi.py` | Model Predictive Control using CasADi |
| `ppo_sim.py` | Gym environment & PPO training loop |
| `ppo_test.py` | Evaluation and real-time visualization of trained policy |
| `get_telemetry.py` | UDP listener for hardware telemetry & plotting |

---

## 📡 Hardware Telemetry Format

The firmware sends 6-field UDP packets (port 3333) at regular intervals:

```
[timestamp, θ, ω, φ_L, ω_R, status]
```

The simulation's `get_telemetry.py` module listens for these packets and plots live data for hardware debugging.

---

## 🧠 Control & Learning Approaches

### **1. Linear Feedback Control**
- Derives linear model around equilibrium
- Computes LQR gains (`calc_inverted_pendulum.py:279–293`)
- Implemented in firmware as state-feedback law

### **2. Model Predictive Control (MPC)**
- Uses linearized discrete model
- Solves finite-horizon constrained optimization in real-time (CasADi)
- Useful for reference trajectory tracking

### **3. Reinforcement Learning (PPO)**
- Proximal Policy Optimization with continuous action space
- Trained in simulated Gym environment
- Policy can be evaluated in simulation or transferred to hardware

---

## 📝 Configuration & Tuning

### **Firmware**
- **Network** (lines 14–17): Update Wi-Fi SSID, telemetry endpoint
- **Timing** (lines 73–79): Loop rates, sensor update frequencies
- **Control Gains** (line 105): State-feedback gains K

### **Simulation**
- **Model parameters**: Defined in `calc_inverted_pendulum.py` (mass, inertia, motor limits)
- **LQR weights**: Tuned in `calc_inverted_pendulum.py` for desired performance
- **PPO hyperparameters**: Configured in `ppo_sim.py` (learning rate, batch size, entropy coeff, etc.)

---

## 🛠️ Dependencies

### **Embedded**
- Arduino core for ESP32
- CAN library (e.g., `mcp_can`)
- IMU driver (ICM42688)
- Optional: Wi-Fi & UDP libraries

### **Simulation**
```toml
# From pyproject.toml
sympy
numpy
scipy
matplotlib
casadi
gym
stable-baselines3
mujoco
```

Install via:
```bash
pip install -e inverted_pendulum_sim
# or with uv:
uv sync
```

---

## 📈 Example Workflows

### **Scenario 1: Tune control gains in simulation, then deploy to hardware**
1. Run `calc_inverted_pendulum.py` to derive the model
2. Adjust LQR weights in `calc_inverted_pendulum.py`
3. Test with `ppo_test.py` or `mpc_sim_casadi.py` in simulation
4. Export gains and update firmware with new K values
5. Flash and test on real robot

### **Scenario 2: Train RL policy in simulation, analyze performance**
1. Run `ppo_sim.py --train` to train a policy
2. Plot training curves: `ppo_sim.py --plot`
3. Evaluate on test scenarios: `ppo_test.py --load ppo_wheeled_pendulum.zip`
4. (Optional) Deploy trained model to hardware for real-world testing

### **Scenario 3: Collect real hardware data and compare to simulation**
1. Start robot with current firmware
2. Run `get_telemetry.py` to capture live telemetry
3. Compare recorded CSV against simulated trajectories
4. Refine model parameters if needed

---

## 🔗 Project Links & References

- **SymPy Mechanics**: [sympy.org](https://www.sympy.org/)
- **CasADi** (MPC): [casadi.org](https://casadi.org/)
- **Stable Baselines 3** (RL): [stable-baselines3.readthedocs.io](https://stable-baselines3.readthedocs.io/)
- **MuJoCo** (physics): [mujoco.org](https://mujoco.org/)

---

## 📧 Notes

- **Hardware Compatibility**: Designed for ESP32 with CyberGear motors and ICM42688 IMU. Adapt sensor/motor drivers for other platforms.
- **Model Accuracy**: Simulation assumes ideal motors and perfect state information. Real hardware includes friction, delays, and sensor noise.
- **Control Stability**: Linearized models are valid near equilibrium. For large disturbances, consider nonlinear control or RL-based adaptive policies.

---

## 📄 License

[Add license information if applicable]

---

**Last Updated**: 2026-06-02  
**Language Composition**: Python 60.8% • C++ 39.2%
