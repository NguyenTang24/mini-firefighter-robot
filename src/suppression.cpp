// =============================================================
// suppression.cpp — Pump MOSFET control
// =============================================================

#include "suppression.h"
#include <Arduino.h>

static bool s_active = false;

void Suppression_Init() {
    pinMode(PIN_PUMP, OUTPUT);
    digitalWrite(PIN_PUMP, LOW);    // Pump OFF on boot — critical safety default
    s_active = false;
    Serial.println("[PUMP] Initialised (OFF)");
}

void Suppression_Activate() {
    if (!s_active) {
        digitalWrite(PIN_PUMP, HIGH);
        s_active = true;
        Serial.println("[PUMP] ON");
    }
}

void Suppression_Deactivate() {
    if (s_active) {
        digitalWrite(PIN_PUMP, LOW);
        s_active = false;
        Serial.println("[PUMP] OFF");
    }
}

bool Suppression_IsActive() {
    return s_active;
}
