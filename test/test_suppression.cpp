// =============================================================
// test/test_suppression.cpp — pump subsystem isolation test
//
// Flash:  pio run -e test_suppression -t upload && pio device monitor
//
// Pattern: pump ON for 2 s → OFF for 3 s → repeat indefinitely.
//
// What to verify:
//   1. You hear/feel the pump engage within ~50 ms of "[PUMP] ON"
//   2. It runs for exactly 2 s then cuts off
//   3. IsActive() returns true while on, false while off
//
// WARNING: if the pump runs dry (no water supply attached)
//          limit test to a few cycles to avoid motor wear.
// =============================================================

#include <Arduino.h>
#include "suppression.h"

#define ON_DURATION_MS   2000
#define OFF_DURATION_MS  3000

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n[TEST] ── Suppression isolation test ────────");
    Serial.printf("[TEST] Pattern: ON %d ms → OFF %d ms → repeat\n",
                  ON_DURATION_MS, OFF_DURATION_MS);
    Serial.println("[TEST] Pump MOSFET gate: GPIO32  (HIGH = pump ON)");
    Suppression_Init();
}

void loop() {
    // ── Pump ON phase ─────────────────────────────────────────
    Suppression_Activate();
    Serial.printf("[PUMP] ON   IsActive=%s\n",
                  Suppression_IsActive() ? "true" : "false");
    delay(ON_DURATION_MS);

    // ── Pump OFF phase ────────────────────────────────────────
    Suppression_Deactivate();
    Serial.printf("[PUMP] OFF  IsActive=%s\n",
                  Suppression_IsActive() ? "true" : "false");
    delay(OFF_DURATION_MS);
}
