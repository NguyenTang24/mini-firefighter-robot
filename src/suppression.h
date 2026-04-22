#pragma once
// =============================================================
// suppression.h — DC pump control via logic-level MOSFET
//
// The MOSFET gate is driven directly from a GPIO pin.
// HIGH → MOSFET conducts → pump runs.
// LOW  → MOSFET off     → pump off.
// =============================================================

#include "config.h"

// Initialise pump GPIO; ensure pump is off at boot
void Suppression_Init();

// Turn the pump on (idempotent — safe to call when already on)
void Suppression_Activate();

// Turn the pump off (idempotent — safe to call when already off)
void Suppression_Deactivate();

// Returns true while the pump is running
bool Suppression_IsActive();
