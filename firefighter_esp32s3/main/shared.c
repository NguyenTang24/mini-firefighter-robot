// shared.c - initialize all shared state

#include "shared.h"
#include "config.h"

QueueHandle_t      q_camera    = NULL;
QueueHandle_t      q_motor     = NULL;
EventGroupHandle_t eg_commands = NULL;

volatile int           g_lidar_cm      = 999;
volatile bool          g_flame_detected = false;
volatile float         g_flame_bearing  = 0.0f;
volatile robot_state_t g_state          = STATE_IDLE;
volatile int           g_turn_ms        = TURN_90_MS;

void shared_init(void)
{
    q_camera    = xQueueCreate(8, sizeof(cam_msg_t));
    q_motor     = xQueueCreate(4, sizeof(int));
    eg_commands = xEventGroupCreate();
}
