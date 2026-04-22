// =============================================================
// state_machine.cpp — FSM transition logic
//
// State ownership: only TaskStateMachine calls Update/SetState.
// Mutex protects s_state so other tasks can safely call GetState().
// =============================================================

#include "state_machine.h"
#include "suppression.h"
#include <Arduino.h>

// ── Module-private state ──────────────────────────────────────
static RobotState_t s_state         = STATE_IDLE;
static uint8_t      s_verifyRetry   = 0;
static uint32_t     s_suppressStart = 0;

// Lab 5 flame goal — tracks the last known flame position so the
// APPROACH state can hold course if the sensor flickers (FLAME_GOAL_TIMEOUT_MS)
// instead of immediately abandoning and returning to SEARCH.
static FlameGoal_t  s_flameGoal     = {0, 0, false};

// =============================================================
// StateMachine_Init
// =============================================================
void StateMachine_Init() {
    s_state         = STATE_IDLE;
    s_verifyRetry   = 0;
    s_suppressStart = 0;
    s_flameGoal     = {0, 0, false};
    Serial.println("[FSM] Initialised → IDLE");
}

// =============================================================
// StateMachine_GetState  (thread-safe)
// =============================================================
RobotState_t StateMachine_GetState() {
    RobotState_t st;
    xSemaphoreTake(g_stateMutex, portMAX_DELAY);
    st = s_state;
    xSemaphoreGive(g_stateMutex);
    return st;
}

// =============================================================
// StateMachine_SetState  (thread-safe, logs every transition)
// =============================================================
void StateMachine_SetState(RobotState_t newState) {
    xSemaphoreTake(g_stateMutex, portMAX_DELAY);
    RobotState_t prev = s_state;
    s_state = newState;
    xSemaphoreGive(g_stateMutex);

    if (prev != newState) {
        Serial.printf("[FSM] %s → %s\n",
                      StateMachine_StateName(prev),
                      StateMachine_StateName(newState));
    }
}

// =============================================================
// StateMachine_Reset
// =============================================================
void StateMachine_Reset() {
    Suppression_Deactivate();
    xEventGroupClearBits(g_eventGroup, EVT_CMD_ABORT | EVT_CMD_START);
    s_verifyRetry = 0;
    s_flameGoal   = {0, 0, false};
    StateMachine_SetState(STATE_IDLE);
    Serial.println("[FSM] Reset — ABORT latch cleared, back to IDLE");
}

// =============================================================
// Flame goal accessors  (Lab 5 concept)
// =============================================================
void StateMachine_SetFlameGoal(uint16_t dist_cm) {
    s_flameGoal.last_distance_cm = dist_cm;
    s_flameGoal.lost_at_ms       = 0;
    s_flameGoal.active           = true;
}

void StateMachine_ClearFlameGoal() {
    s_flameGoal = {0, 0, false};
}

FlameGoal_t StateMachine_GetFlameGoal() {
    return s_flameGoal;
}

// =============================================================
// StateMachine_Update
//
// Called every 50 ms by TaskStateMachine AFTER it has already
// handled the ABORT bit, so we can assume ABORT is clear here.
// =============================================================
void StateMachine_Update(EventBits_t bits) {
    switch (StateMachine_GetState()) {

        // ── IDLE ────────────────────────────────────────────────
        // Wait for a /start HTTP command.
        case STATE_IDLE:
            if (bits & EVT_CMD_START) {
                xEventGroupClearBits(g_eventGroup, EVT_CMD_START);
                s_verifyRetry = 0;
                StateMachine_ClearFlameGoal();
                StateMachine_SetState(STATE_SEARCH);
                Serial.println("[FSM] /start received — beginning SEARCH");
            }
            break;

        // ── SEARCH ─────────────────────────────────────────────
        // Roam using LiDAR obstacle avoidance (TaskNavigation drives).
        // When flame detected, record it as a goal (Lab 5) then approach.
        case STATE_SEARCH:
            if (bits & EVT_FLAME_DETECTED) {
                // Peek the latest LiDAR center distance as initial goal range
                LidarScan3Zone_t scan;
                uint16_t goalDist = 0;
                if (xQueuePeek(g_lidarQueue, &scan, 0) == pdTRUE && scan.valid_center) {
                    goalDist = scan.center_cm;
                }
                StateMachine_SetFlameGoal(goalDist);
                StateMachine_SetState(STATE_APPROACH);
            }
            break;

        // ── APPROACH ───────────────────────────────────────────
        // Drive toward the flame.  TaskNavigation executes proportional
        // speed control; we watch for arrival at standoff distance.
        //
        // Lab 5 goal-hold logic:
        //   go_to_goal.py published a fixed waypoint and kept driving
        //   toward it even if sensor feedback was temporarily absent.
        //   Here, if EVT_FLAME_DETECTED clears (sensor flicker), we
        //   hold course for FLAME_GOAL_TIMEOUT_MS before giving up —
        //   the robot keeps approaching the last known flame position.
        case STATE_APPROACH: {
            if (bits & EVT_FLAME_DETECTED) {
                // Flame visible — reset goal timeout and check standoff
                s_flameGoal.lost_at_ms = 0;

                LidarScan3Zone_t scan;
                if (xQueuePeek(g_lidarQueue, &scan, 0) == pdTRUE) {
                    if (scan.valid_center && scan.center_cm <= STANDOFF_DISTANCE_CM) {
                        s_suppressStart = millis();
                        Suppression_Activate();
                        StateMachine_SetState(STATE_SUPPRESS);
                    }
                }
            } else {
                // Flame signal lost — apply goal-hold timeout
                if (!s_flameGoal.active) {
                    // Never had a goal; abandon immediately
                    StateMachine_SetState(STATE_SEARCH);
                } else {
                    if (s_flameGoal.lost_at_ms == 0) {
                        s_flameGoal.lost_at_ms = millis();  // Start the timeout
                        Serial.println("[FSM] Flame lost — holding course toward goal");
                    }
                    if (millis() - s_flameGoal.lost_at_ms > FLAME_GOAL_TIMEOUT_MS) {
                        Serial.println("[FSM] Goal timeout — returning to SEARCH");
                        StateMachine_ClearFlameGoal();
                        StateMachine_SetState(STATE_SEARCH);
                    }
                    // else: continue APPROACH — TaskNavigation keeps moving forward
                }
            }
            break;
        }

        // ── SUPPRESS ───────────────────────────────────────────
        case STATE_SUPPRESS:
            if (millis() - s_suppressStart >= PUMP_SPRAY_DURATION_MS) {
                Suppression_Deactivate();
                StateMachine_SetState(STATE_VERIFY);
            }
            break;

        // ── VERIFY ─────────────────────────────────────────────
        case STATE_VERIFY:
            if (!(bits & EVT_FLAME_DETECTED)) {
                Serial.println("[FSM] *** Flame extinguished — mission complete ***");
                s_verifyRetry = 0;
                StateMachine_ClearFlameGoal();
                StateMachine_SetState(STATE_IDLE);

            } else if (s_verifyRetry < VERIFY_RETRY_MAX) {
                s_verifyRetry++;
                Serial.printf("[FSM] Flame persists — retry %u / %u\n",
                              s_verifyRetry, VERIFY_RETRY_MAX);
                s_suppressStart = millis();
                Suppression_Activate();
                StateMachine_SetState(STATE_SUPPRESS);

            } else {
                Serial.println("[FSM] Retries exhausted — entering ABORT");
                Suppression_Deactivate();
                StateMachine_SetState(STATE_ABORT);
            }
            break;

        // ── ABORT ──────────────────────────────────────────────
        case STATE_ABORT:
            break;

        default:
            break;
    }
}

// =============================================================
// StateMachine_StateName
// =============================================================
const char *StateMachine_StateName(RobotState_t s) {
    switch (s) {
        case STATE_IDLE:     return "IDLE";
        case STATE_SEARCH:   return "SEARCH";
        case STATE_APPROACH: return "APPROACH";
        case STATE_SUPPRESS: return "SUPPRESS";
        case STATE_VERIFY:   return "VERIFY";
        case STATE_ABORT:    return "ABORT";
        default:             return "UNKNOWN";
    }
}
