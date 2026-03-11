// =============================================================
// camera_vision.cpp — UART line parser for esp32cam_landmark
//
// Input lines from ESP32-CAM GPIO1 at 115200 baud:
//   "LM:0 bearing:-0.12 px:380"
//   "LM:1 REACHED px:1240"
//   "NO_LM"
//
// LANDMARK_SEQUENCE and LM_TURN_LEFT are defined here so they
// live in one easy-to-edit place.  Edit LANDMARK_SEQUENCE to
// match the physical order of colored cubes in your room.
//   LM_RED=0 (turn right), LM_BLUE=1 (turn left),
//   LM_GREEN=2, LM_YELLOW=3, LM_ORANGE=4
// =============================================================

#include "camera_vision.h"
#include <Arduino.h>
#include <string.h>
#include <stdio.h>

// ── Landmark path — edit to match your room layout ────────────
// LM_TURN_LEFT[i] = true → turn left at waypoint i, false → turn right
const uint8_t LANDMARK_SEQUENCE[LANDMARK_SEQ_LEN] = {
    LM_RED, LM_BLUE, LM_RED, LM_BLUE, LM_RED
};
const bool LM_TURN_LEFT[LANDMARK_SEQ_LEN] = {
    false,  // LM_RED   → turn right
    true,   // LM_BLUE  → turn left
    false,  // LM_RED   → turn right
    true,   // LM_BLUE  → turn left
    false   // LM_RED   → turn right (final — path complete)
};

// ── Init ──────────────────────────────────────────────────────
void CameraVision_Init() {
    CAM_UART_PORT.begin(CAM_UART_BAUD, SERIAL_8N1,
                        PIN_CAM_UART_RX, PIN_CAM_UART_TX);
    Serial.println("[CAM_VIS] UART1 ready — GPIO9 @ 115200");
}

// ── Non-blocking line reader ──────────────────────────────────
// Accumulates characters until '\n', then parses and fills *out.
// Returns true only when a full line was received.
bool CameraVision_Read(LandmarkDetection_t *out) {
    static char buf[64];
    static int  pos = 0;

    while (CAM_UART_PORT.available()) {
        char c = (char)CAM_UART_PORT.read();
        if (c == '\r') continue;

        if (c == '\n') {
            buf[pos] = '\0';
            pos = 0;
            memset(out, 0, sizeof(*out));

            if (strcmp(buf, "NO_LM") == 0) {
                out->found = false;
                return true;
            }

            int id;
            if (sscanf(buf, "LM:%d", &id) == 1) {
                out->found = true;
                out->id    = (uint8_t)id;
                if (strstr(buf, "REACHED")) {
                    out->reached = true;
                    sscanf(buf, "LM:%*d REACHED px:%d", &out->px);
                } else {
                    sscanf(buf, "LM:%*d bearing:%f px:%d",
                           &out->bearing, &out->px);
                }
                return true;
            }
            // Unrecognised line — discard silently
        } else if (pos < (int)sizeof(buf) - 1) {
            buf[pos++] = c;
        }
    }
    return false;
}
