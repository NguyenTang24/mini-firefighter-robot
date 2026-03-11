// =============================================================
// sensors.cpp — TF-Luna UART frame parser + servo sweep +
//               IR flame sensor reader
//
// TF-Luna default UART frame (9 bytes, 115200 baud, 100 Hz):
//
//   Byte  Field       Notes
//   ────  ──────────  ─────────────────────────────────────────
//   [0]   0x59        Header byte 1
//   [1]   0x59        Header byte 2
//   [2]   DIST_L      Distance low  byte (cm, little-endian)
//   [3]   DIST_H      Distance high byte
//   [4]   STR_L       Signal strength low  byte (arbitrary units)
//   [5]   STR_H       Signal strength high byte
//   [6]   TEMP_L      Chip temperature low  byte (raw, not °C)
//   [7]   TEMP_H      Chip temperature high byte
//   [8]   CKSUM       Checksum = (sum of bytes 0–7) & 0xFF
//
// Sweep concept adapted from Lab 4 obstacle_avoidance.py:
//   Lab 4 sliced a full 360° LaserScan into front/left/right zones.
//   Here, a servo physically aims the single-point TF-Luna at each
//   of those three directions and takes a reading at each stop.
// =============================================================

#include "sensors.h"
#include <Arduino.h>

#define TFLUNA_FRAME_LEN  9
#define TFLUNA_HEADER     0x59

static Servo s_lidarServo;

// =============================================================
// Sensors_Init
// =============================================================
void Sensors_Init() {
    // UART2 — RX pin 16, TX pin 17
    LIDAR_UART_PORT.begin(LIDAR_BAUD_RATE, SERIAL_8N1,
                          PIN_LIDAR_RX, PIN_LIDAR_TX);

    // Flame sensor: active-LOW; GPIO34 is input-only (no pull resistor
    // needed — the sensor module has its own pull-up on D0).
    pinMode(PIN_FLAME_SENSOR, INPUT);

    // Servo: attach and sweep to center so the sensor starts pointing ahead
    s_lidarServo.attach(PIN_LIDAR_SERVO);
    s_lidarServo.write(SERVO_CENTER_ANGLE);
    vTaskDelay(pdMS_TO_TICKS(300));   // Let servo reach center before first scan

    Serial.println("[SENSORS] UART2, flame GPIO, and sweep servo initialised");
}

// =============================================================
// Sensors_ReadLidar  (internal helper — used by SweepLidar)
//
// Parses one complete TF-Luna frame from the UART ring buffer.
// Returns true when a valid frame is decoded; false if more bytes
// are still needed.  Persistent state (buf, idx) survives calls.
// =============================================================
bool Sensors_ReadLidar(LidarReading_t *out) {
    static uint8_t buf[TFLUNA_FRAME_LEN];
    static uint8_t idx = 0;

    while (LIDAR_UART_PORT.available()) {
        uint8_t b = (uint8_t)LIDAR_UART_PORT.read();

        // ── Frame sync ─────────────────────────────────────────
        if (idx == 0) {
            if (b != TFLUNA_HEADER) continue;
        } else if (idx == 1) {
            if (b != TFLUNA_HEADER) { idx = 0; continue; }
        }

        buf[idx++] = b;

        if (idx == TFLUNA_FRAME_LEN) {
            idx = 0;

            uint8_t cksum = 0;
            for (int i = 0; i < 8; i++) cksum += buf[i];

            if (cksum != buf[8]) {
                Serial.printf("[LIDAR] Checksum error (0x%02X != 0x%02X) — frame dropped\n",
                              buf[8], cksum);
                return false;
            }

            uint16_t dist     = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);
            out->distance_cm  = dist;
            out->timestamp_ms = millis();
            out->valid        = (dist > 0 && dist < LIDAR_VALID_MAX_CM);
            return true;
        }
    }
    return false;
}

// =============================================================
// Sensors_SweepLidar
//
// Adapted from Lab 4 obstacle_avoidance.py — that script divided
// a 360° LIDAR scan into three zones:
//   front = ranges[0:60]  + ranges[300:]
//   left  = ranges[60:180]
//   right = ranges[180:300]
// and reacted to the minimum distance in each.
//
// We achieve the same three zones by physically aiming the
// single-point TF-Luna with a servo.  The logic in TaskNavigation
// that consumes this struct mirrors the Python if/elif chain exactly.
// =============================================================
void Sensors_SweepLidar(LidarScan3Zone_t *out) {
    LidarReading_t reading;

    // ── Left zone ──────────────────────────────────────────────
    s_lidarServo.write(SERVO_LEFT_ANGLE);
    vTaskDelay(pdMS_TO_TICKS(SERVO_SETTLE_MS));
    // Flush stale bytes that arrived while the servo was moving
    while (LIDAR_UART_PORT.available()) LIDAR_UART_PORT.read();
    // Wait for a fresh frame (up to 20 ms = two 100 Hz frames)
    uint32_t t = millis();
    bool gotLeft = false;
    while (millis() - t < 20) {
        if (Sensors_ReadLidar(&reading)) { gotLeft = true; break; }
    }
    out->left_cm   = gotLeft ? reading.distance_cm : 0;
    out->valid_left = gotLeft && reading.valid;

    // ── Center zone ────────────────────────────────────────────
    s_lidarServo.write(SERVO_CENTER_ANGLE);
    vTaskDelay(pdMS_TO_TICKS(SERVO_SETTLE_MS));
    while (LIDAR_UART_PORT.available()) LIDAR_UART_PORT.read();
    t = millis();
    bool gotCenter = false;
    while (millis() - t < 20) {
        if (Sensors_ReadLidar(&reading)) { gotCenter = true; break; }
    }
    out->center_cm    = gotCenter ? reading.distance_cm : 0;
    out->valid_center = gotCenter && reading.valid;

    // ── Right zone ─────────────────────────────────────────────
    s_lidarServo.write(SERVO_RIGHT_ANGLE);
    vTaskDelay(pdMS_TO_TICKS(SERVO_SETTLE_MS));
    while (LIDAR_UART_PORT.available()) LIDAR_UART_PORT.read();
    t = millis();
    bool gotRight = false;
    while (millis() - t < 20) {
        if (Sensors_ReadLidar(&reading)) { gotRight = true; break; }
    }
    out->right_cm   = gotRight ? reading.distance_cm : 0;
    out->valid_right = gotRight && reading.valid;

    out->timestamp_ms = millis();

    // Return servo to center so it's ready for the next navigation decision
    s_lidarServo.write(SERVO_CENTER_ANGLE);
}

// =============================================================
// Sensors_FlameDetected
// =============================================================
bool Sensors_FlameDetected() {
    // Active-LOW: D0 pulled LOW by the sensor when flame IR is detected
    return (digitalRead(PIN_FLAME_SENSOR) == LOW);
}
