// main.c

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "config.h"
#include "shared.h"

static const char *TAG = "MAIN";

void task_motor_start(void);
void task_camera_uart_start(void);
void task_navigation_start(void);
void task_state_machine_start(void);
void task_wifi_start(void);

void app_main(void)
{
    ESP_LOGI(TAG, "FireFighter starting...");

    shared_init();
    task_motor_start();
    task_camera_uart_start();
    task_navigation_start();
    task_wifi_start();
    // task_state_machine_start();  // re-enable when fire suppression is ready

    ESP_LOGI(TAG, "all tasks started");
}
