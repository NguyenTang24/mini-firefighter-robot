#pragma once
// =============================================================
// wifi_commander.h — HTTP command interface
//
// Endpoints:
//   GET /start   → sets EVT_CMD_START
//   GET /abort   → sets EVT_CMD_ABORT  (latches until /reset)
//   GET /reset   → calls StateMachine_Reset(), clears ABORT latch
//   GET /status  → returns JSON snapshot of robot state
// =============================================================

#include "config.h"

// Connects to Wi-Fi and starts the HTTP server.
// Call once from TaskWiFiCommand before entering the handle loop.
void WiFiCommander_Init();

// Process pending HTTP requests.  Call as fast as practical
// (every 10 ms from TaskWiFiCommand is sufficient).
void WiFiCommander_Handle();
