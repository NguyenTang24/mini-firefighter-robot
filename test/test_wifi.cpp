// =============================================================
// test/test_wifi.cpp — WiFi HTTP + UDP command interface test
//
// Flash:  pio run -e test_wifi -t upload && pio device monitor
//
// After flashing, watch Serial for the assigned IP address.
// Then from a laptop on the same network:
//
//   HTTP — test every endpoint with curl:
//     curl http://<IP>/status          → JSON state snapshot
//     curl http://<IP>/start           → FSM → SEARCH
//     curl http://<IP>/abort           → FSM → ABORT (latches)
//     curl http://<IP>/reset           → FSM → IDLE, clears latch
//     curl http://<IP>/nonexistent     → 404 with endpoint list
//
//   UDP — send a UdpCmd_t packet from Python:
//     python3 -c "
//       import socket, struct
//       s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
//       # mode=2 (start), vx=0, wz=0
//       s.sendto(struct.pack('bbb', 2, 0, 0), ('<IP>', 8082))
//     "
//     Expected Serial: [UDP] mode=2 → START
//
//   Teleop (manual drive in IDLE state):
//     s.sendto(struct.pack('bbb', 0, 80, 0), ('<IP>', 8082))
//     → Motors_SetVelocity(80/127, 0) ≈ 63% forward
//     (No motors in this test because motors are NOT initialised.
//      To test teleop with real motor movement, also call Motors_Init()
//      in setup() and add motors.cpp to the test_wifi src_filter.)
//
// NOTE: this test only exercises the WiFi subsystem.  No LiDAR,
//       sensors, or autonomous motion runs.  The FSM advances
//       through states but TaskNavigation is absent so wheels stay still.
// =============================================================

#include <Arduino.h>
#include "wifi_commander.h"
#include "state_machine.h"

// ── RTOS handle definitions ───────────────────────────────────
// In the production build these live in main.cpp.
// Here we own them so the linker is satisfied.
EventGroupHandle_t g_eventGroup = nullptr;
QueueHandle_t      g_lidarQueue  = nullptr;
SemaphoreHandle_t  g_stateMutex  = nullptr;

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n[TEST] ── WiFi / HTTP / UDP isolation test ──");
    Serial.println("[TEST] Connecting to WiFi — watch for IP address...");

    // Create the RTOS primitives that wifi_commander and state_machine need
    g_eventGroup  = xEventGroupCreate();
    g_lidarQueue  = xQueueCreate(5, sizeof(LidarScan3Zone_t));
    g_stateMutex  = xSemaphoreCreateMutex();

    configASSERT(g_eventGroup != nullptr);
    configASSERT(g_lidarQueue  != nullptr);
    configASSERT(g_stateMutex  != nullptr);

    StateMachine_Init();    // FSM starts in IDLE
    WiFiCommander_Init();   // connects WiFi, starts HTTP on :80, UDP on :8082

    Serial.println("[TEST] Ready — curl http://<IP>/status to verify");
}

void loop() {
    WiFiCommander_Handle();   // polls HTTP clients and UDP socket
    delay(10);
}
