// =============================================================
// test/test_sensors.cpp — sensor subsystem isolation test
//
// Flash:  pio run -e test_sensors -t upload && pio device monitor
//
// What to test:
//   FLAME  — wave a lighter or candle in front of the sensor;
//            Serial should switch from "none" to "*** DETECTED ***"
//
//   LIDAR  — point the TF-Luna at a wall or object; distance_cm
//            should match a tape measure within a few cm
//
//   SWEEP  — every SWEEP_INTERVAL_MS a full left/center/right
//            servo sweep is triggered; all three distances print
//            on one line so you can compare zones side-by-side
// =============================================================

#include <Arduino.h>
#include "sensors.h"

// How often to trigger a full 3-zone sweep (milliseconds)
#define SWEEP_INTERVAL_MS  6000

static uint32_t s_lastSweep = 0;

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n[TEST] ── Sensors isolation test ────────────");
    Serial.println("[TEST] Flame: active-LOW on GPIO34");
    Serial.println("[TEST] LiDAR: UART2 RX=16 TX=17 115200 baud");
    Serial.println("[TEST] Servo: GPIO33  left=60° center=90° right=120°");
    Serial.println("[TEST] Full sweep every 6 s\n");

    Sensors_Init();
    s_lastSweep = millis();
}

void loop() {
    // ── Flame sensor (50 Hz poll) ─────────────────────────────
    bool flame = Sensors_FlameDetected();
    Serial.printf("[FLAME] %s\n",
                  flame ? "*** DETECTED ***" : "none");

    // ── Single-point LiDAR (raw UART frame reader) ────────────
    // Prints only when a complete valid frame has been parsed.
    // If nothing prints, check UART wiring or baud rate.
    LidarReading_t r;
    if (Sensors_ReadLidar(&r)) {
        Serial.printf("[LIDAR] %s  dist=%4u cm  @ %lu ms\n",
                      r.valid ? "OK " : "OOB",   // OOB = out of 0–600 cm range
                      r.distance_cm,
                      r.timestamp_ms);
    }

    // ── Full 3-zone servo sweep (blocking ~270 ms) ────────────
    // Triggered periodically; servo moves L→C→R, one reading each.
    if (millis() - s_lastSweep >= SWEEP_INTERVAL_MS) {
        s_lastSweep = millis();
        Serial.println("[TEST] ── 3-zone sweep ──────────────────────");

        LidarScan3Zone_t scan;
        Sensors_SweepLidar(&scan);

        Serial.printf("[SWEEP] L:%4u cm (%s)   C:%4u cm (%s)   R:%4u cm (%s)\n",
                      scan.left_cm,   scan.valid_left   ? "ok" : "--",
                      scan.center_cm, scan.valid_center ? "ok" : "--",
                      scan.right_cm,  scan.valid_right  ? "ok" : "--");
        Serial.println();
    }

    delay(200);   // ~5 Hz polling rate
}
