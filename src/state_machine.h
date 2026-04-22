#pragma once
// =============================================================
// state_machine.h — FSM enum, transition interface, and accessors
//
// Only TaskStateMachine (in main.cpp) should call StateMachine_Update()
// and StateMachine_SetState().  All other modules use
// StateMachine_GetState() as read-only observers.
// =============================================================

#include "config.h"

// -------------------------------------------------------------
// Robot FSM states
// -------------------------------------------------------------
typedef enum : uint8_t {
    STATE_IDLE     = 0, // Waiting for /start command
    STATE_SEARCH,       // Exploring; LiDAR obstacle avoidance active
    STATE_APPROACH,     // Flame found; closing to standoff distance
    STATE_SUPPRESS,     // Pump running for PUMP_SPRAY_DURATION_MS
    STATE_VERIFY,       // Checking whether flame is extinguished
    STATE_ABORT,        // Emergency stop — latched until /reset
} RobotState_t;

// -------------------------------------------------------------
// Flame Goal  (Lab 5 concept — inspired by go_to_goal.py)
//
// Lab 5 published a (x, y) goal pose to /move_base_simple/goal
// and the nav stack drove toward it.  Here we store the last
// known flame distance as a "goal" so the APPROACH state can
// hold course if the sensor flickers briefly, rather than
// immediately abandoning and returning to SEARCH.
// -------------------------------------------------------------
typedef struct {
    uint16_t last_distance_cm;  // LiDAR center reading when flame first acquired
    uint32_t lost_at_ms;        // millis() when EVT_FLAME_DETECTED was last cleared
    bool     active;            // true once a flame has been acquired this run
} FlameGoal_t;

// -------------------------------------------------------------
// API
// -------------------------------------------------------------

// Call once from setup() before tasks start
void StateMachine_Init();

// Called by TaskStateMachine every cycle.
// bits = snapshot of g_eventGroup (ABORT already handled by caller)
void StateMachine_Update(EventBits_t bits);

// Thread-safe state getter (uses g_stateMutex)
RobotState_t StateMachine_GetState();

// Thread-safe state setter; logs every transition to Serial
void StateMachine_SetState(RobotState_t newState);

// Clear the ABORT latch and return to IDLE.
// Called by the /reset HTTP endpoint.
void StateMachine_Reset();

// Human-readable state name for Serial/JSON output
const char *StateMachine_StateName(RobotState_t s);

// Flame goal accessors (called internally and by wifi_commander for /status)
void       StateMachine_SetFlameGoal(uint16_t dist_cm);
void       StateMachine_ClearFlameGoal();
FlameGoal_t StateMachine_GetFlameGoal();
