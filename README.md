# Mini Firefighter Robot

UCF Intro to Robotics, Spring 2026 - Gia Tang, Carlos Garcia, Gianfranco

Semi-autonomous robot that follows a set of colored landmark cards around a course, finds a fire, and puts it out with a water pump.

## How it works

An ESP32-CAM mounted on the robot scans for colored cards (orange, pink, yellow, purple) and sends the bearing to the main controller over UART. The ESP32-S3 follows each card in order and turns left 90 degrees at each one. Once it reaches the center it activates the pump to spray water at the flame.

There is also a web page at 192.168.4.1 (connect to the FireFighter WiFi network) that lets you start, stop, and reset the robot without touching it.

## Hardware

- ESP32-S3 WROOM-1 - main controller
- AI-Thinker ESP32-CAM - color landmark detection
- L298N - motor driver
- TF-Luna LiDAR - obstacle avoidance
- 5 channel IR flame sensor array
- SG90 servo - LiDAR pan
- Water pump + relay

## Repo

- `firefighter_esp32s3/` - main robot firmware (ESP-IDF)
- `esp32cam_landmark/` - color detection firmware for the ESP32-CAM
- `esp32cam_calibrate/` - tool we used to calibrate the color thresholds
