#pragma once
// =============================================================
// config.h — Single source of truth for pin assignments,
//             tuning constants, and shared RTOS handles.
//
// Include this in every .cpp file.  Nothing else in the project
// defines pins or RTOS primitives.
// =============================================================

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// -------------------------------------------------------------
// PIN DEFINITIONS
// -------------------------------------------------------------

// TF-Luna LiDAR — connected to UART2
// ESP32 default UART2 pins: RX=16, TX=17
#define PIN_LIDAR_RX        16
#define PIN_LIDAR_TX        17
#define LIDAR_BAUD_RATE     115200
#define LIDAR_UART_PORT     Serial2     // HardwareSerial instance

// IR Flame Sensor — active-LOW digital output
// GPIO34 is input-only (no internal pull-up), which is fine here
#define PIN_FLAME_SENSOR    34

// L298N Motor Driver — Left motor (Motor A)
#define PIN_MOTOR_A_IN1     25
#define PIN_MOTOR_A_IN2     26
#define PIN_MOTOR_A_EN      27          // PWM speed (ENA)

// L298N Motor Driver — Right motor (Motor B)
#define PIN_MOTOR_B_IN3     14
#define PIN_MOTOR_B_IN4     12
#define PIN_MOTOR_B_EN      13          // PWM speed (ENB)

// Pump — logic-level MOSFET gate
// HIGH = pump ON, LOW = pump OFF
#define PIN_PUMP            32

// Servo — physically sweeps the TF-Luna to scan left/center/right
// (Lab 4: obstacle_avoidance.py divided the LIDAR into 3 zones;
//  this servo does the same thing for a single-point rangefinder)
#define PIN_LIDAR_SERVO     33

// -------------------------------------------------------------
// PWM / LEDC CONFIG  (ESP32 LEDC peripheral)
// -------------------------------------------------------------
// Channels 0–3 are reserved for ESP32Servo (servo library auto-allocates
// from 0).  Motor channels start at 4 to avoid conflict.
#define PWM_CHANNEL_LEFT    4
#define PWM_CHANNEL_RIGHT   5
#define PWM_FREQ_HZ         1000        // 1 kHz — audible but effective for DC motors
#define PWM_RESOLUTION_BITS 8           // 0–255 duty cycle range

// Default motion speeds (0–255)
#define MOTOR_SPEED_DEFAULT 180
#define MOTOR_SPEED_TURN    140

// -------------------------------------------------------------
// SERVO CONFIG  (TF-Luna sweep angles)
// -------------------------------------------------------------
#define SERVO_LEFT_ANGLE    60          // degrees — left zone
#define SERVO_CENTER_ANGLE  90          // degrees — straight ahead
#define SERVO_RIGHT_ANGLE   120         // degrees — right zone
#define SERVO_SETTLE_MS     80          // wait after move before reading (ms)

// -------------------------------------------------------------
// NAVIGATION THRESHOLDS
// -------------------------------------------------------------

// Obstacle avoidance threshold — intentionally matches Lab 4's 0.3 m value
// (obstacle_avoidance.py: "if min(front) > 0.3")
#define OBSTACLE_DISTANCE_CM    30

// Distance at which the robot stops and begins suppression
#define STANDOFF_DISTANCE_CM    25

// Proportional gain for APPROACH speed — mirrors Lab 4 go_to_blue.py "Kp"
// speed = KP_APPROACH * (distance_cm - STANDOFF_DISTANCE_CM)
#define KP_APPROACH             3
#define MIN_APPROACH_SPEED      55      // Minimum motor speed during approach
#define MAX_APPROACH_SPEED      160     // Maximum motor speed during approach

// Duration of timed reverse when all 3 zones are blocked (Lab 2 concept:
// motion expressed as speed + time, like turtle movement sequences)
#define REVERSE_DURATION_MS     700

// How long to hold course toward the last known flame position if the
// sensor flickers — inspired by Lab 5 goal navigation (go_to_goal.py)
#define FLAME_GOAL_TIMEOUT_MS   1500

// TF-Luna datasheet reliable range upper limit
#define LIDAR_VALID_MAX_CM      600

// LiDAR queue depth — holds up to N timestamped 3-zone scans
#define LIDAR_QUEUE_SIZE        5

// -------------------------------------------------------------
// SUPPRESSION CONFIG
// -------------------------------------------------------------
#define PUMP_SPRAY_DURATION_MS  3000    // How long the pump runs per attempt
#define VERIFY_RETRY_MAX        1       // How many times to retry before ABORT

// -------------------------------------------------------------
// FREERTOS EVENT GROUP BIT ASSIGNMENTS
//
// Why event groups for commands and flame?
//   Commands (START/ABORT) are edge-triggered, rare, and need to
//   be visible to multiple tasks simultaneously — a sticky event
//   bit is perfect.  Queues would require every reader to dequeue,
//   which is error-prone with multiple consumers.
//
//   Flame detection is also a shared, level-sensitive signal:
//   multiple tasks (FSM, Navigation) need to read it without
//   consuming it.  A bit in an event group is the right fit.
// -------------------------------------------------------------
#define EVT_CMD_START       (1 << 0)    // HTTP /start received
#define EVT_CMD_ABORT       (1 << 1)    // HTTP /abort received; latches until /reset
#define EVT_FLAME_DETECTED  (1 << 2)    // Flame sensor currently active
#define EVT_LIDAR_VALID     (1 << 3)    // LiDAR returned a valid reading (optional)

// -------------------------------------------------------------
// SHARED RTOS HANDLES
// Defined once in main.cpp; every other module uses extern.
// -------------------------------------------------------------
extern EventGroupHandle_t g_eventGroup;   // All event bits above
extern QueueHandle_t      g_lidarQueue;   // LidarScan3Zone_t stream (producer: TaskLidarScan)
extern SemaphoreHandle_t  g_stateMutex;   // Guards FSM state in state_machine.cpp

// -------------------------------------------------------------
// SHARED DATA STRUCTURES
// -------------------------------------------------------------

// Raw single-point reading — used internally by Sensors_ReadLidar()
typedef struct {
    uint16_t distance_cm;
    uint32_t timestamp_ms;
    bool     valid;
} LidarReading_t;

// 3-zone sweep result — posted to g_lidarQueue by TaskLidarScan.
// Mirrors Lab 4 obstacle_avoidance.py: front / left / right zones.
typedef struct {
    uint16_t left_cm;
    uint16_t center_cm;
    uint16_t right_cm;
    bool     valid_left;
    bool     valid_center;
    bool     valid_right;
    uint32_t timestamp_ms;
} LidarScan3Zone_t;

// -------------------------------------------------------------
// CAMERA VISION — ESP32-CAM landmark detector (UART slave)
// ESP32-CAM GPIO1 (printf/UART0 TX) → ESP32-S3 GPIO9 (UART1 RX)
// -------------------------------------------------------------
#define CAM_UART_PORT       Serial1
#define CAM_UART_BAUD       115200
#define PIN_CAM_UART_RX     9       // GPIO9 ← ESP32-CAM GPIO1 (printf)
#define PIN_CAM_UART_TX     10      // assigned but unused

// Landmark IDs — match LM:N in esp32cam_landmark printf output
#define LM_RED              0       // turn right at this waypoint
#define LM_BLUE             1       // turn left  at this waypoint
#define LM_GREEN            2
#define LM_YELLOW           3
#define LM_ORANGE           4

// Landmark path sequence — edit to match the physical room layout.
// Robot only reacts to LANDMARK_SEQUENCE[s_seqIdx]; all others ignored.
// LM_RED = turn right, LM_BLUE = turn left.
#define LANDMARK_SEQ_LEN    5
extern const uint8_t  LANDMARK_SEQUENCE[LANDMARK_SEQ_LEN];
extern const bool     LM_TURN_LEFT[LANDMARK_SEQ_LEN];

// Camera navigation tuning
#define CAM_BEARING_THRESH  0.10f   // ±0.10 rad dead-band before steering
#define MOTOR_SPEED_SEARCH  100     // slow forward while no landmark visible
#define LANDMARK_TURN_MS    650     // ms for 90° turn — tune for your robot
#define REACQUIRE_DELAY_MS  3000    // ms without landmark before reacquire spin

// Landmark detection result — parsed from one UART line
typedef struct {
    bool    found;
    bool    reached;    // pixel count ≥ REACHED_PIXELS on CAM side
    uint8_t id;         // 0=Red 1=Blue 2=Green 3=Yellow 4=Orange
    float   bearing;    // radians: negative=left, positive=right of center
    int     px;         // pixel count (larger = closer)
} LandmarkDetection_t;

extern QueueHandle_t g_camQueue;    // depth 3, LandmarkDetection_t

// -------------------------------------------------------------
// WI-FI CREDENTIALS — fill in before flashing
// -------------------------------------------------------------
#define WIFI_SSID       "YOUR_SSID"
#define WIFI_PASSWORD   "YOUR_PASSWORD"
#define HTTP_PORT       80

// UDP command port — matches the recv port used in Lab 6:
//   sdk.UDP(HIGHLEVEL, 8080, "192.168.123.161", 8082)
//   (8082 is the port the Go1 *received* commands on)
#define UDP_PORT        8082

// Lab 6 inspired — mirrors the HighCmd struct fields from walk.py:
//   cmd.mode         → mode  (0=idle/stop, 2=start/walk, 5=abort like lay-down)
//   cmd.velocity[0]  → vx    (forward speed, scaled -127..+127)
//   cmd.yawSpeed     → wz    (turn rate,     scaled -127..+127)
typedef struct __attribute__((packed)) {
    uint8_t mode;   // 0 = stop/idle,  2 = start (walk),  5 = abort (lay down)
    int8_t  vx;     // forward velocity  -127..+127  (like cmd.velocity[0])
    int8_t  wz;     // yaw/turn speed    -127..+127  (like cmd.yawSpeed)
} UdpCmd_t;
