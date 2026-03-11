// =============================================================
// main.cpp — FireFighter Robot entry point
//
// Responsibilities:
//   1. Initialise all hardware subsystems
//   2. Create every FreeRTOS primitive (event group, queue, mutex)
//   3. Spawn all tasks at the correct priorities and core assignments
//   4. Define the five task functions
//
// ── Design rationale ─────────────────────────────────────────
//
// Event groups vs queues
//   Event bits (START, ABORT, FLAME_DETECTED) are used for signals
//   that are:
//     • Level-sensitive (multiple tasks need to read the same state)
//     • Rare (not a stream of data)
//     • Sticky until explicitly cleared
//   The LiDAR produces a continuous stream of 3-zone scan results
//   that must be consumed in order — a queue is the right primitive.
//
// Why only TaskNavigation writes motor outputs
//   If multiple tasks called motor functions concurrently they could
//   issue conflicting direction/speed commands in the same PWM period,
//   producing unpredictable hardware behaviour.  Centralising all
//   motor writes in one task eliminates this hazard without needing
//   a motor mutex, and makes the full motion repertoire auditable in
//   one place.  Other tasks express intent through state or event bits.
//
// Why ABORT overrides all states
//   ABORT is a safety command that must halt the robot regardless of
//   what it is doing.  Both TaskNavigation and TaskStateMachine check
//   the ABORT bit before processing their normal logic.  The bit
//   latches (is never cleared automatically) so a momentary /abort
//   HTTP request is not lost between task wake-ups.  Only an explicit
//   /reset HTTP call clears the latch and returns the FSM to IDLE.
// =============================================================

#include <Arduino.h>
#include "config.h"
#include "state_machine.h"
#include "sensors.h"
#include "motors.h"
#include "suppression.h"
#include "wifi_commander.h"
#include "camera_vision.h"

// ── RTOS handle definitions (extern'd in config.h) ────────────
EventGroupHandle_t g_eventGroup = nullptr;
QueueHandle_t      g_lidarQueue  = nullptr;
SemaphoreHandle_t  g_stateMutex  = nullptr;
QueueHandle_t      g_camQueue    = nullptr;

// ── Forward declarations ──────────────────────────────────────
void TaskLidarScan    (void *pvParams);
void TaskFlameDetect  (void *pvParams);
void TaskNavigation   (void *pvParams);
void TaskStateMachine (void *pvParams);
void TaskWiFiCommand  (void *pvParams);
void TaskCameraVision (void *pvParams);

// =============================================================
// setup()
// =============================================================
void setup() {
    Serial.begin(115200);
    vTaskDelay(pdMS_TO_TICKS(500));
    Serial.println("\n[BOOT] ── FireFighter Robot ──────────────────");

    // ── Hardware init ─────────────────────────────────────────
    // Sensors first: ESP32Servo attaches inside Sensors_Init() and
    // auto-allocates LEDC channels starting from 0.  Motors_Init()
    // then claims channels 4 & 5 (PWM_CHANNEL_LEFT/RIGHT in config.h),
    // so there is no overlap.
    Sensors_Init();
    Motors_Init();
    Suppression_Init();
    CameraVision_Init();

    // ── Create RTOS primitives ────────────────────────────────
    g_eventGroup = xEventGroupCreate();
    g_lidarQueue  = xQueueCreate(LIDAR_QUEUE_SIZE, sizeof(LidarScan3Zone_t));
    g_stateMutex  = xSemaphoreCreateMutex();
    g_camQueue    = xQueueCreate(3, sizeof(LandmarkDetection_t));

    configASSERT(g_eventGroup != nullptr);
    configASSERT(g_lidarQueue  != nullptr);
    configASSERT(g_stateMutex  != nullptr);
    configASSERT(g_camQueue    != nullptr);

    Serial.println("[BOOT] RTOS primitives created");
    StateMachine_Init();

    // ── Task creation ─────────────────────────────────────────
    // Priority 3 — flame detection is the most time-critical task
    xTaskCreatePinnedToCore(TaskFlameDetect,  "FlameDetect",  1024, nullptr, 3, nullptr, 1);

    // Priority 2 — LiDAR, navigation, FSM, camera vision (round-robin)
    xTaskCreatePinnedToCore(TaskLidarScan,    "LidarScan",    2048, nullptr, 2, nullptr, 1);
    xTaskCreatePinnedToCore(TaskNavigation,   "Navigation",   3072, nullptr, 2, nullptr, 1);
    xTaskCreatePinnedToCore(TaskStateMachine, "StateMachine", 2048, nullptr, 2, nullptr, 1);
    xTaskCreatePinnedToCore(TaskCameraVision, "CamVis",       2048, nullptr, 2, nullptr, 1);

    // Priority 1 — WiFi on core 0 where the TCP/IP stack runs
    xTaskCreatePinnedToCore(TaskWiFiCommand,  "WiFiCommand",  8192, nullptr, 1, nullptr, 0);

    Serial.println("[BOOT] All tasks created — entering scheduler");
}

void loop() {
    vTaskDelete(nullptr);   // Free the Arduino loop task stack
}

// =============================================================
// TaskLidarScan  — Priority 2, Core 1
//
// Drives the TF-Luna servo sweep (left → center → right) via
// Sensors_SweepLidar(), which takes ~270 ms per cycle.  Posts
// the resulting LidarScan3Zone_t to g_lidarQueue so TaskNavigation
// and TaskStateMachine can react to all three zones.
//
// This replaces the old single-point read loop — the 3-zone struct
// carries exactly the same information that Lab 4's obstacle_avoidance.py
// computed from a 360° LaserScan (front, left, right minimums).
// =============================================================
void TaskLidarScan(void *pvParams) {
    LidarScan3Zone_t scan;

    for (;;) {
        Sensors_SweepLidar(&scan);

        // Evict oldest entry if queue is full, then post latest scan
        if (xQueueSend(g_lidarQueue, &scan, 0) != pdTRUE) {
            LidarScan3Zone_t discard;
            xQueueReceive(g_lidarQueue, &discard, 0);
            xQueueSend(g_lidarQueue, &scan, 0);
        }

        if (scan.valid_center) {
            xEventGroupSetBits(g_eventGroup, EVT_LIDAR_VALID);
        }

        Serial.printf("[LIDAR] L:%3ucm  C:%3ucm  R:%3ucm  @ %lu ms\n",
                      scan.left_cm, scan.center_cm, scan.right_cm,
                      scan.timestamp_ms);

        // Sweep takes ~270 ms; small extra delay before next cycle
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// =============================================================
// TaskFlameDetect  — Priority 3 (highest), Core 1
//
// Polls the flame sensor at 50 Hz and updates EVT_FLAME_DETECTED.
// Highest priority ensures detection latency is always bounded.
// =============================================================
void TaskFlameDetect(void *pvParams) {
    bool prevFlame = false;

    for (;;) {
        bool flameNow = Sensors_FlameDetected();

        if (flameNow && !prevFlame) {
            xEventGroupSetBits(g_eventGroup, EVT_FLAME_DETECTED);
            Serial.println("[FLAME] *** Flame DETECTED ***");
        } else if (!flameNow && prevFlame) {
            xEventGroupClearBits(g_eventGroup, EVT_FLAME_DETECTED);
            Serial.println("[FLAME] Flame cleared");
        }

        prevFlame = flameNow;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// =============================================================
// TaskNavigation  — Priority 2, Core 1
//
// THE ONLY TASK THAT WRITES MOTOR OUTPUTS.
//
// Three lab-derived algorithms are applied depending on FSM state:
//
//   SEARCH  — Lab 4 obstacle_avoidance.py
//     Direct structural port of the front/left/right if-elif chain.
//       "if min(front) > 0.3  → forward"
//       "elif min(right) > 0.3 → turn right"
//       "elif min(left) > 0.3  → turn left"
//       "else                  → reverse"
//     Threshold: 0.3 m → OBSTACLE_DISTANCE_CM (30 cm).
//
//   APPROACH — Lab 4 go_to_blue.py
//     Proportional speed control: speed = Kp × (distance − standoff).
//     As the robot closes on the flame, speed decreases smoothly —
//     the same proportional error term used for the blue-box follower.
//     (Kp = KP_APPROACH, analogous to go_to_blue's "Kp = -0.001")
//
//   REVERSE fallback — Lab 2 moveturtlefwd.py / code3
//     Motions are expressed as (speed, duration_ms) timed by millis(),
//     matching the change_time / rospy.Duration pattern from the
//     turtle movement exercises.
// =============================================================
void TaskNavigation(void *pvParams) {
    LidarScan3Zone_t scan = {0, 0, 0, false, false, false, 0};

    // Lab 2: timed reverse state — mirrors the change_time boolean
    // flag from moveturtlefwd.py
    static bool     s_reversing    = false;
    static uint32_t s_reverseEndMs = 0;

    for (;;) {
        // ── ABORT check (always first) ─────────────────────────
        EventBits_t bits = xEventGroupGetBits(g_eventGroup);
        if (bits & EVT_CMD_ABORT) {
            Motors_Stop();
            s_reversing = false;
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // ── Refresh scan cache ────────────────────────────────
        {
            LidarScan3Zone_t tmp;
            while (xQueueReceive(g_lidarQueue, &tmp, 0) == pdTRUE) {
                scan = tmp;
            }
        }

        switch (StateMachine_GetState()) {

            case STATE_IDLE:
                Motors_Stop();
                s_reversing = false;
                break;

            // ── SEARCH — Camera landmark waypoint navigation ──────
            // Sequence-filtered: robot only steers toward the NEXT
            // expected landmark color (LANDMARK_SEQUENCE[s_seqIdx]).
            // Wrong colors are silently ignored.  Falls back to LiDAR
            // obstacle avoidance when no expected landmark is visible,
            // and starts a slow reacquire spin after REACQUIRE_DELAY_MS.
            // Lab 2 millis() timed-turn used for waypoint turns.
            // Lab 4 go_to_blue bearing steering used when visible.
            case STATE_SEARCH: {
                static bool     s_turning    = false;
                static uint32_t s_turnEndMs  = 0;
                static bool     s_turnLeft   = false;
                static uint32_t s_noLmSince  = 0;
                static int      s_seqIdx     = 0;

                LandmarkDetection_t lm;
                bool camOk = (xQueuePeek(g_camQueue, &lm, 0) == pdTRUE);

                bool pathDone   = (s_seqIdx >= LANDMARK_SEQ_LEN);
                bool lmExpected = camOk && lm.found && !pathDone
                                  && (lm.id == LANDMARK_SEQUENCE[s_seqIdx]);

                if (s_turning) {
                    // Lab 2 millis() timed-turn (mirrors s_reversing pattern)
                    if (millis() < s_turnEndMs) {
                        s_turnLeft ? Motors_TurnLeft(MOTOR_SPEED_TURN)
                                   : Motors_TurnRight(MOTOR_SPEED_TURN);
                    } else {
                        s_turning   = false;
                        s_reversing = false;
                        Motors_Stop();
                    }

                } else if (lmExpected && lm.reached) {
                    s_noLmSince = 0;
                    // Correct landmark reached — timed turn, advance sequence
                    s_turning   = true;
                    s_turnLeft  = LM_TURN_LEFT[s_seqIdx];
                    s_turnEndMs = millis() + LANDMARK_TURN_MS;
                    s_seqIdx++;
                    Serial.printf("[NAV] LM:%d REACHED — %s, seq→%d\n",
                                  lm.id, s_turnLeft ? "left" : "right", s_seqIdx);

                } else if (lmExpected && !lm.reached) {
                    s_noLmSince = 0;
                    // Steer toward landmark (Lab 4 go_to_blue bearing concept)
                    if (lm.bearing < -CAM_BEARING_THRESH) {
                        Motors_TurnLeft(MOTOR_SPEED_TURN);
                    } else if (lm.bearing > CAM_BEARING_THRESH) {
                        Motors_TurnRight(MOTOR_SPEED_TURN);
                    } else {
                        Motors_Forward(MOTOR_SPEED_SEARCH);
                    }
                    s_reversing = false;

                } else {
                    // Wrong color or NO_LM — drive forward + LiDAR avoidance
                    if (s_noLmSince == 0) s_noLmSince = millis();

                    if (millis() - s_noLmSince >= REACQUIRE_DELAY_MS) {
                        // Lost too long — slow spin to sweep camera FOV
                        Motors_TurnRight(80);
                    } else {
                        // Short window — normal Lab 4 3-zone obstacle avoidance
                        uint16_t front = scan.center_cm;
                        uint16_t left  = scan.left_cm;
                        uint16_t right = scan.right_cm;

                        if (!scan.valid_center || front > OBSTACLE_DISTANCE_CM) {
                            Motors_Forward(MOTOR_SPEED_SEARCH);
                            s_reversing = false;
                        } else if (scan.valid_right && right > OBSTACLE_DISTANCE_CM) {
                            Motors_TurnRight(MOTOR_SPEED_TURN);
                            s_reversing = false;
                        } else if (scan.valid_left && left > OBSTACLE_DISTANCE_CM) {
                            Motors_TurnLeft(MOTOR_SPEED_TURN);
                            s_reversing = false;
                        } else {
                            if (!s_reversing) {
                                s_reversing    = true;
                                s_reverseEndMs = millis() + REVERSE_DURATION_MS;
                                Serial.println("[NAV] All zones blocked — reversing");
                            }
                            Motors_Backward(MOTOR_SPEED_DEFAULT);
                            if (millis() >= s_reverseEndMs) {
                                s_reversing = false;
                            }
                        }
                    }
                }
                break;
            }

            // ── APPROACH ───────────────────────────────────────
            // Proportional speed adapted from Lab 4 go_to_blue.py:
            //   Kp = -0.001
            //   move_cmd.angular.z = Kp * error_x  (turn toward object)
            //   if w < 500: move_cmd.linear.x = 0.2
            //
            // Our version: error = center_cm - standoff_cm
            //              speed = KP_APPROACH * error
            // Negative error (too close) is caught by the stop condition.
            case STATE_APPROACH: {
                if (scan.valid_center && scan.center_cm <= STANDOFF_DISTANCE_CM) {
                    Motors_Stop();   // FSM will notice and advance to SUPPRESS
                } else {
                    int error = (int)scan.center_cm - STANDOFF_DISTANCE_CM;
                    int speed = KP_APPROACH * error;
                    if (speed < MIN_APPROACH_SPEED) speed = MIN_APPROACH_SPEED;
                    if (speed > MAX_APPROACH_SPEED) speed = MAX_APPROACH_SPEED;
                    Motors_Forward((uint8_t)speed);
                }
                break;
            }

            case STATE_SUPPRESS:
            case STATE_VERIFY:
                Motors_Stop();
                break;

            case STATE_ABORT:
                Motors_Stop();
                break;

            default:
                Motors_Stop();
                break;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// =============================================================
// TaskStateMachine  — Priority 2, Core 1
//
// Owns all FSM state transitions.  Only this task calls
// StateMachine_SetState(), so transitions are serialised without
// needing an additional lock around transition logic itself.
// =============================================================
void TaskStateMachine(void *pvParams) {
    for (;;) {
        EventBits_t bits = xEventGroupGetBits(g_eventGroup);

        // ABORT: highest-priority check — latches until /reset
        if (bits & EVT_CMD_ABORT) {
            if (StateMachine_GetState() != STATE_ABORT) {
                Suppression_Deactivate();
                StateMachine_SetState(STATE_ABORT);
            }
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        StateMachine_Update(bits);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// =============================================================
// TaskWiFiCommand  — Priority 1, Core 0
// =============================================================
void TaskWiFiCommand(void *pvParams) {
    WiFiCommander_Init();
    for (;;) {
        WiFiCommander_Handle();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// =============================================================
// TaskCameraVision  — Priority 2, Core 1
//
// Reads landmark detection lines from the ESP32-CAM over UART1
// (CameraVision_Read is non-blocking) and posts the parsed
// LandmarkDetection_t to g_camQueue (depth 3).
// If the queue is full, evicts the oldest entry so TaskNavigation
// always sees the freshest detection.
// =============================================================
void TaskCameraVision(void *pvParams) {
    LandmarkDetection_t det;
    for (;;) {
        if (CameraVision_Read(&det)) {
            if (xQueueSend(g_camQueue, &det, 0) != pdTRUE) {
                LandmarkDetection_t old;
                xQueueReceive(g_camQueue, &old, 0);
                xQueueSend(g_camQueue, &det, 0);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
