// =============================================================
// test/test_motors.cpp — motor subsystem isolation test
//
// Flash:  pio run -e test_motors -t upload && pio device monitor
//
// Cycles every motion primitive and the Lab 6 Motors_SetVelocity
// API.  Watch the wheels physically and read Serial to confirm
// each direction and speed.
//
// Between steps the robot stops for 400 ms so you can see the
// transition clearly.  A 3-second pause at the end of each full
// cycle gives you time to reposition the robot if needed.
// =============================================================

#include <Arduino.h>
#include "motors.h"

// Print label, run motors for ms, then coast-stop + short gap
static void runStep(const char *label, uint32_t ms) {
    Serial.printf("[TEST] %-35s (%lu ms)\n", label, ms);
    delay(ms);
    Motors_Stop();
    delay(400);
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n[TEST] ── Motors isolation test ──────────────");
    Serial.println("[TEST] Watch wheels and Serial output.");
    Motors_Init();
}

void loop() {
    Serial.println("\n[TEST] ── NEW CYCLE ─────────────────────────");

    // ── Basic motion primitives ──────────────────────────────
    Motors_Forward(MOTOR_SPEED_DEFAULT);
    runStep("Forward  speed=180", 2000);

    Motors_Backward(MOTOR_SPEED_DEFAULT);
    runStep("Backward speed=180", 2000);

    Motors_TurnLeft(MOTOR_SPEED_TURN);
    runStep("TurnLeft  speed=140", 1000);

    Motors_TurnRight(MOTOR_SPEED_TURN);
    runStep("TurnRight speed=140", 1000);

    // ── PWM duty-cycle scaling ───────────────────────────────
    Motors_Forward(80);
    runStep("Forward  speed=80  (slow ~44%)", 2000);

    // ── Lab 6: normalized velocity API ───────────────────────
    // vx=+1.0 → full forward; vx=-1.0 → full reverse
    // wz=+1.0 → turn left;    wz=-1.0 → turn right
    Motors_SetVelocity( 1.0f,  0.0f);
    runStep("SetVelocity vx=+1.0 wz= 0.0  (full forward)", 1500);

    Motors_SetVelocity(-1.0f,  0.0f);
    runStep("SetVelocity vx=-1.0 wz= 0.0  (full reverse)", 1500);

    Motors_SetVelocity( 0.0f,  1.0f);
    runStep("SetVelocity vx= 0.0 wz=+1.0  (point left)", 1000);

    Motors_SetVelocity( 0.0f, -1.0f);
    runStep("SetVelocity vx= 0.0 wz=-1.0  (point right)", 1000);

    Motors_SetVelocity( 0.5f,  0.3f);
    runStep("SetVelocity vx=+0.5 wz=+0.3  (curve left)", 1500);

    Motors_SetVelocity( 0.5f, -0.3f);
    runStep("SetVelocity vx=+0.5 wz=-0.3  (curve right)", 1500);

    Serial.println("[TEST] Cycle complete — pausing 3 s");
    delay(3000);
}
