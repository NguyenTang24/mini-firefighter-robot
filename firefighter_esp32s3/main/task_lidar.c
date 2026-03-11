// task_lidar.c - TF-Luna lidar driver + servo sweep
// TODO: to be implemented
// should write distance readings to g_lidar_cm in shared.h
// servo functions are called by task_navigation for obstacle avoidance

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "config.h"
#include "shared.h"

static const char *TAG = "LIDAR";

void lidar_servo_center(void) {}
void lidar_servo_left(void)   {}
void lidar_servo_right(void)  {}

// sweep servo and return which side is clearer
// returns -1 = left, +1 = right, 0 = both blocked
int lidar_scan_obstacle_side(void)
{
    return 0;
}

void task_lidar_start(void)
{
    ESP_LOGI(TAG, "lidar task not implemented yet");
}
