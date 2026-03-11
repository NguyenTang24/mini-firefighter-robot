// =============================================================
// wifi_commander.cpp — Wi-Fi station + HTTP command interface
//
// Uses the built-in WebServer (ESP32 Arduino core); no extra
// libraries required.
//
// All route handlers only set/clear event group bits or call
// StateMachine_Reset().  They never write hardware directly —
// that responsibility belongs to TaskNavigation and
// state_machine.cpp respectively.
// =============================================================

#include "wifi_commander.h"
#include "state_machine.h"
#include "suppression.h"
#include "motors.h"         // For Motors_SetVelocity (UDP manual drive)
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiUDP.h>        // Lab 6: UDP socket — mirrors sdk.UDP in walk.py
#include <Arduino.h>

static WebServer s_server(HTTP_PORT);
static WiFiUDP   s_udp;     // UDP command receiver (Lab 6 analog)

// =============================================================
// Route handlers
// =============================================================

static void onStart() {
    xEventGroupSetBits(g_eventGroup, EVT_CMD_START);
    Serial.println("[WIFI] /start");
    s_server.send(200, "text/plain", "OK: START command queued\n");
}

static void onAbort() {
    xEventGroupSetBits(g_eventGroup, EVT_CMD_ABORT);
    Serial.println("[WIFI] /abort");
    s_server.send(200, "text/plain", "OK: ABORT latched\n");
}

static void onReset() {
    // StateMachine_Reset clears the ABORT latch and returns FSM to IDLE
    StateMachine_Reset();
    Serial.println("[WIFI] /reset");
    s_server.send(200, "text/plain", "OK: System reset to IDLE\n");
}

static void onStatus() {
    RobotState_t  state = StateMachine_GetState();
    EventBits_t   bits  = xEventGroupGetBits(g_eventGroup);
    FlameGoal_t   goal  = StateMachine_GetFlameGoal();

    // Peek at the latest 3-zone scan without consuming it from the queue
    LidarScan3Zone_t scan = {0, 0, 0, false, false, false, 0};
    xQueuePeek(g_lidarQueue, &scan, 0);

    // Build JSON with snprintf to avoid String heap fragmentation
    char buf[384];
    snprintf(buf, sizeof(buf),
        "{"
          "\"state\":\"%s\","
          "\"flame\":%s,"
          "\"abort_latch\":%s,"
          "\"pump_active\":%s,"
          "\"lidar_left_cm\":%u,"
          "\"lidar_center_cm\":%u,"
          "\"lidar_right_cm\":%u,"
          "\"lidar_age_ms\":%lu,"
          "\"flame_goal_active\":%s,"
          "\"flame_goal_dist_cm\":%u"
        "}",
        StateMachine_StateName(state),
        (bits & EVT_FLAME_DETECTED) ? "true" : "false",
        (bits & EVT_CMD_ABORT)      ? "true" : "false",
        Suppression_IsActive()      ? "true" : "false",
        scan.left_cm,
        scan.center_cm,
        scan.right_cm,
        scan.timestamp_ms > 0 ? (millis() - scan.timestamp_ms) : 0UL,
        goal.active             ? "true" : "false",
        goal.last_distance_cm
    );

    s_server.send(200, "application/json", buf);
}

static void onNotFound() {
    s_server.send(404, "text/plain",
        "404 Not Found\n"
        "Endpoints:\n"
        "  GET /start   — begin autonomous fire search\n"
        "  GET /abort   — emergency stop (latches)\n"
        "  GET /reset   — clear abort latch, return to IDLE\n"
        "  GET /status  — JSON status snapshot\n"
    );
}

// =============================================================
// WiFiCommander_Init
// =============================================================
void WiFiCommander_Init() {
    Serial.printf("[WIFI] Connecting to \"%s\" ...", WIFI_SSID);

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    // Wait up to 10 seconds; use vTaskDelay so the RTOS keeps scheduling
    uint8_t attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        vTaskDelay(pdMS_TO_TICKS(500));
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\n[WIFI] Connected  IP: %s\n",
                      WiFi.localIP().toString().c_str());
    } else {
        Serial.println("\n[WIFI] Connection failed — HTTP server unavailable");
        // Continue without Wi-Fi; robot can still operate autonomously
        return;
    }

    s_server.on("/start",  HTTP_GET, onStart);
    s_server.on("/abort",  HTTP_GET, onAbort);
    s_server.on("/reset",  HTTP_GET, onReset);
    s_server.on("/status", HTTP_GET, onStatus);
    s_server.onNotFound(onNotFound);

    s_server.begin();
    Serial.printf("[WIFI] HTTP server listening on port %d\n", HTTP_PORT);

    // Lab 6: open UDP socket — mirrors how walk.py opened sdk.UDP(HIGHLEVEL, 8080, IP, 8082)
    // The ESP32 listens on UDP_PORT (8082) for UdpCmd_t packets.
    s_udp.begin(UDP_PORT);
    Serial.printf("[WIFI] UDP command socket open on port %d\n", UDP_PORT);
}

// =============================================================
// WiFiCommander_Handle
// =============================================================
void WiFiCommander_Handle() {
    if (WiFi.status() != WL_CONNECTED) return;

    // ── Lab 6: UDP command poll ───────────────────────────────
    // Mirrors the recv loop in walk.py:
    //   udp.Recv(); udp.GetRecv(state);  …  udp.SetSend(cmd); udp.Send();
    //
    // Packet format (UdpCmd_t, 3 bytes):
    //   mode=0 → idle/stop
    //   mode=2 → START (like walk.py mode=2 "walk continuously")
    //   mode=5 → ABORT (like walk.py mode=5 "lay down" / emergency stop)
    //   vx, wz → manual velocity when in IDLE state (teleoperation)
    int pktSize = s_udp.parsePacket();
    if (pktSize >= (int)sizeof(UdpCmd_t)) {
        UdpCmd_t cmd;
        s_udp.read((uint8_t *)&cmd, sizeof(cmd));

        if (cmd.mode == 5) {
            xEventGroupSetBits(g_eventGroup, EVT_CMD_ABORT);
            Serial.println("[UDP] mode=5 → ABORT");
        } else if (cmd.mode == 2) {
            xEventGroupSetBits(g_eventGroup, EVT_CMD_START);
            Serial.println("[UDP] mode=2 → START");
        }

        // Manual velocity override (teleoperation) — intentional exception to
        // the "only TaskNavigation writes motors" rule in main.cpp.
        // Safe because: (a) restricted to STATE_IDLE only, (b) TaskNavigation
        // also calls Motors_Stop() in its IDLE case, so the last write wins
        // without conflict.  In all active autonomous states, this branch
        // never executes, preserving the single-writer guarantee.
        // Same safety principle as Lab 6: mode=0 returns the Go1 to idle/stand
        // before any velocity commands are applied.
        if (StateMachine_GetState() == STATE_IDLE && (cmd.vx != 0 || cmd.wz != 0)) {
            float vx = cmd.vx / 127.0f;    // scale -127..+127 → -1.0..+1.0
            float wz = cmd.wz / 127.0f;    // same as cmd.yawSpeed in Lab 6
            Motors_SetVelocity(vx, wz);
            Serial.printf("[UDP] teleop vx=%.2f wz=%.2f\n", vx, wz);
        }
    }

    // ── HTTP client polling ───────────────────────────────────
    s_server.handleClient();
}
