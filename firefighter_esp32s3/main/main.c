// main.c

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "config.h"
#include "shared.h"

static const char *TAG = "MAIN";

void task_motor_start(void);
void task_camera_uart_start(void);
void task_navigation_start(void);
void task_state_machine_start(void);
void task_wifi_start(void);
void task_flame_detect_start(void);
void task_lidar_start(void);

void app_main(void)
{
    ESP_LOGI(TAG, "FireFighter starting...");

    // pre-load output register low, then enable output — no glitch on boot
    gpio_set_level(PUMP_GPIO, 0);
    gpio_set_direction(PUMP_GPIO, GPIO_MODE_OUTPUT);

    shared_init();
    task_motor_start();
    task_lidar_start();
    task_camera_uart_start();
    task_navigation_start();
    task_wifi_start();
    task_flame_detect_start();
    task_state_machine_start();

    ESP_LOGI(TAG, "all tasks started");
}
