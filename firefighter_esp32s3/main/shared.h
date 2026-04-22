// shared.h - globals and queues shared between all tasks

#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include <stdbool.h>

typedef enum {
    STATE_IDLE = 0,
    STATE_PATH_FOLLOW,
    STATE_OBSTACLE_AVOID,
    STATE_FIRE_APPROACH,
    STATE_EXTINGUISH,
    STATE_DONE,
} robot_state_t;

// message posted to q_camera by the camera uart task
typedef struct {
    int   lm_id;    // 1-4 for landmarks, -1 if nothing seen
    float bearing;  // radians, negative = left of center
    int   px_size;  // pixel count, larger = closer to camera
    bool  is_fire;  // true if this came from a FIRE line, not LM
} cam_msg_t;

extern QueueHandle_t      q_camera;
extern QueueHandle_t      q_motor;
extern EventGroupHandle_t eg_commands;

#define CMD_START_BIT   BIT0
#define CMD_STOP_BIT    BIT1
#define CMD_RESUME_BIT  BIT2
#define CMD_RESET_BIT   BIT3
#define CMD_CALIB_BIT   BIT4

extern volatile int           g_lidar_cm;
extern volatile bool          g_flame_detected;
extern volatile float         g_flame_bearing;
extern volatile int           g_flame_strength;  // max raw ADC of center-3 (proxy for distance)
extern volatile bool          g_flame_centered;  // true when L1 and R1 are balanced
extern volatile robot_state_t g_state;
extern volatile int           g_turn_ms;
extern volatile int           g_motor_r_trim;  // runtime right motor trim

void shared_init(void);
