// task_lidar.c - TF-Luna lidar
// TODO: not implemented yet

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "config.h"
#include "shared.h"

static const char *TAG = "LIDAR";

void lidar_servo_center(void) {}
void lidar_servo_left(void)   {}
void lidar_servo_right(void)  {}

int lidar_scan_obstacle_side(void)
{
    return 0;
}

void task_lidar_start(void)
{
    ESP_LOGI(TAG, "lidar task not implemented yet");
}
