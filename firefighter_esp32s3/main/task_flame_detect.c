// task_flame_detect.c - 5 channel IR flame sensor array
// TODO: to be implemented
// should set g_flame_detected = true in shared.h when flame is confirmed

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "config.h"
#include "shared.h"

static const char *TAG = "FLAME";

void task_flame_detect_start(void)
{
    ESP_LOGI(TAG, "flame detect task not implemented yet");
}
