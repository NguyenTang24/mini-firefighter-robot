#pragma once
// =============================================================
// camera_vision.h — ESP32-CAM landmark UART interface
//
// The ESP32-CAM runs esp32cam_landmark (ESP-IDF) and prints one
// line per frame to its UART0 GPIO1 at 115200 baud:
//   "LM:N bearing:±X.XX px:NNN"
//   "LM:N REACHED px:NNN"
//   "NO_LM"
//
// CameraVision_Init() starts UART1 on the S3 side.
// CameraVision_Read() is non-blocking; returns true when a
// complete line has been parsed into *out.
// =============================================================

#include "config.h"

void CameraVision_Init();
bool CameraVision_Read(LandmarkDetection_t *out);   // non-blocking
