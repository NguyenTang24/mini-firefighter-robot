# 🔥 Semi-Autonomous Firefighter Robot

> A mini firefighter robot that autonomously detects and suppresses fires using an ESP32, IR flame sensor, ultrasonic obstacle avoidance, and a water valve system.

**Built by:** Gia Tang · Carlos Garcia · Gianfranco  
**Course:** Intro to Robotics — University of Central Florida  

---

## Overview

The Mini Firefighter Robot is a semi-autonomous ground robot designed to detect fires in an enclosed environment and extinguish them using a water-based suppression system. The robot operates in two modes:

- **Wandering Mode** — The robot patrols its environment, using an ultrasonic sensor and IR obstacle module to navigate around objects while scanning for flame signatures via an IR flame sensor.
- **Fire Suppression Mode** — Once a flame is detected, the robot navigates toward it, aims a servo-controlled nozzle, and opens a relay-controlled water valve. It continuously monitors the flame sensor and closes the valve the moment the fire is confirmed out.

---

## Hardware

| Component | Description |
|---|---|
| ESP32 (ACEBOTT) | Main microcontroller — WiFi + Bluetooth |
| 4× TT DC Gear Motors | Mecanum wheel drive |
| HC-SR04 | Ultrasonic obstacle detection |
| IR Obstacle Module | Short-range obstacle avoidance (LAFVIN) |
| IR Flame Sensor | Digital fire detection (DO pin) |
| SG90 Servo | Nozzle aiming (0–180°) |
| 2-Channel Relay | Water valve and pump control |
| 0.96" OLED (SSD1306) | Status display |
| Buzzer | Audio alerts |

---

## Software

Written in **Arduino C++** for the ESP32 using the Arduino IDE.

**Required libraries:**
- `ESP32Servo` by Kevin Harrington
- `Adafruit SSD1306`
- `Adafruit GFX Library`

**State Machine:**
```
WANDERING → FIRE_FOUND → SUPPRESSING → DONE → WANDERING
```

---

## Repository Structure

```
mini-firefighter-robot/
├── README.md
├── firefighter_robot.ino   # Main Arduino sketch
└── firefighter_robot.py    # Original MicroPython version
```

---

## Team

| Name | Role |
|---|---|
| Gia Tang | Team member |
| Carlos Garcia | Team member |
| Gianfranco | Team member|

---

*UCF — Intro to Robotics · Spring 2026*
