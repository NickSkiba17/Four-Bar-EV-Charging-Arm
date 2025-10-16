# Four-Bar-EV-Charging-Arm
Designed, simulated, fabricated, and programmed an automated robotic charging arm for electric vehicles using a four-bar linkage mechanism. The system extends from within a compact frame to reach an EV charging port when triggered by an infrared sensor, demonstrating the engineering design cycle from mathematical optimization to physical prototype.

## 📋 Table of Contents
- [Overview](#overview)
- [Features](#features)
- [Technical Specifications](#technical-specifications)
- [Project Structure](#project-structure)
- [Getting Started](#getting-started)
- [Phase 1: MATLAB Optimization](#phase-1-matlab-optimization)
- [Phase 2: CAD Design & Simulation](#phase-2-cad-design--simulation)
- [Phase 3: Fabrication & Programming](#phase-3-fabrication--programming)
- [Results](#results)
- [Future Improvements](#future-improvements)
- [Acknowledgments](#acknowledgments)
- [License](#license)

---

## 🎯 Overview

This project demonstrates the complete engineering design process for an automated robotic charging arm using a **four-bar linkage mechanism**. The system autonomously extends from a compact frame to connect with an electric vehicle charging port when triggered by an infrared sensor.

### Project Objectives
1. Design a kinematically optimized four-bar linkage using MATLAB
2. Model and simulate the mechanism in 3D CAD (Autodesk Inventor)
3. Fabricate precision parts using laser cutting
4. Program intelligent control system with Arduino
5. Validate performance through physical testing

---

## ✨ Features

- **🔧 Optimized Kinematics**: MATLAB fmincon optimization satisfies 20 inequality and 2 equality constraints
- **📐 Parametric CAD Design**: Fully constrained 3D models with engineering drawings
- **⚡ Dynamic Simulation**: Validated motion and torque requirements before fabrication
- **🤖 Intelligent Control**: Arduino-based IR sensor triggering with state machine logic
- **🎯 Precision Targeting**: Reaches charging port (152.4mm away, 240mm height) with <10mm error
- **♻️ Repeatable Operation**: Autonomous extend-charge-retract cycles with 3-second holds
- **🛡️ Collision Avoidance**: Smooth servo control prevents EV deflection

