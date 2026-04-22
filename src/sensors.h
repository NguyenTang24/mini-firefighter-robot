#pragma once
// =============================================================
// sensors.h — TF-Luna LiDAR (UART) + IR flame sensor (GPIO)
//             + servo sweep for 3-zone scanning
// =============================================================

#include "config.h"
#include <ESP32Servo.h>

// Call once from setup() — configures UART2, flame GPIO, and servo
void Sensors_Init();

// Parse one complete TF-Luna frame from the UART buffer.
// Returns true and populates *out if a valid frame was decoded.
// Returns false if no complete frame is available yet.
// Non-blocking; safe to call in a loop.
bool Sensors_ReadLidar(LidarReading_t *out);

// Sweep the TF-Luna servo to LEFT → CENTER → RIGHT, taking one
// distance reading at each position.  Blocking (~270 ms total).
// Mirrors Lab 4 obstacle_avoidance.py's front / left / right zones,
// but physically moves the sensor instead of slicing a 360° scan.
void Sensors_SweepLidar(LidarScan3Zone_t *out);

// Returns true when the flame sensor's D0 output is LOW (flame present).
// The sensor module drives D0 LOW when its IR threshold is exceeded.
bool Sensors_FlameDetected();
