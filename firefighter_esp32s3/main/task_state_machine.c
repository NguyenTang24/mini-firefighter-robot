// task_state_machine.c - drive toward fire and spray

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "config.h"
#include "shared.h"

static const char *TAG = "FSM";

extern void motor_drive(int left_pct, int right_pct);
extern void motor_stop(void);

static void pump_on(void)
{
    gpio_set_level(PUMP_GPIO, 1);
    ESP_LOGI(TAG, "pump on");
}

static void pump_off(void)
{
    gpio_set_level(PUMP_GPIO, 0);
    ESP_LOGI(TAG, "pump off");
}

static void do_extinguish(void)
{
    g_state = STATE_EXTINGUISH;
    motor_stop();
    vTaskDelay(pdMS_TO_TICKS(200));  // let motors fully stop before pump draws current

    for (int attempt = 0; attempt < 2; attempt++) {
        ESP_LOGI(TAG, "spraying (attempt %d)", attempt + 1);
        pump_on();
        vTaskDelay(pdMS_TO_TICKS(SPRAY_DURATION_MS));
        pump_off();
        vTaskDelay(pdMS_TO_TICKS(SPRAY_SETTLE_MS));

        if (!g_flame_detected) {
            ESP_LOGI(TAG, "flame out");
            break;
        }
        ESP_LOGW(TAG, "flame still there, retrying");
    }

    g_state = STATE_DONE;
    motor_stop();
    ESP_LOGI(TAG, "done - waiting for resume");
}

static void do_fire_approach(void)
{
    ESP_LOGI(TAG, "fire approach started");
    uint32_t start      = xTaskGetTickCount() * portTICK_PERIOD_MS;
    bool     seen_flame   = false;

    while (g_state == STATE_FIRE_APPROACH) {
        if ((xTaskGetTickCount() * portTICK_PERIOD_MS - start) > FIRE_APPROACH_TIMEOUT_MS) {
            ESP_LOGW(TAG, "approach timed out without reaching fire");
            motor_stop();
            g_state = STATE_DONE;
            return;
        }

        if (g_flame_centered) {
            ESP_LOGI(TAG, "close enough, spraying");
            motor_stop();
            do_extinguish();
            return;
        }

        if (g_flame_detected) {
            seen_flame = true;
            float b = g_flame_bearing;

            // drive forward, continuously steer toward flame
            if (b < -FSM_BEAR_TOL) {
                motor_drive(FIRE_APPROACH_SPEED - FSM_TURN_OFFSET,
                            FIRE_APPROACH_SPEED + FSM_TURN_OFFSET);
            } else if (b > FSM_BEAR_TOL) {
                motor_drive(FIRE_APPROACH_SPEED + FSM_TURN_OFFSET,
                            FIRE_APPROACH_SPEED - FSM_TURN_OFFSET);
            } else {
                motor_drive(FIRE_APPROACH_SPEED, FIRE_APPROACH_SPEED);
            }
        } else if (seen_flame) {
            ESP_LOGW(TAG, "flame lost mid-approach, stopping");
            motor_stop();
            g_state = STATE_DONE;
            return;
        } else {
            motor_drive(FIRE_APPROACH_SPEED, FIRE_APPROACH_SPEED);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void state_machine_task(void *arg)
{
    gpio_set_direction(PUMP_GPIO, GPIO_MODE_OUTPUT);
    pump_off();

    while (1) {
        switch (g_state) {
            case STATE_FIRE_APPROACH:
                do_fire_approach();
                break;

            case STATE_DONE:
                motor_stop();
                pump_off();
                xEventGroupWaitBits(eg_commands, CMD_RESUME_BIT | CMD_START_BIT,
                                    pdTRUE, pdFALSE, portMAX_DELAY);
                g_state = STATE_PATH_FOLLOW;
                ESP_LOGI(TAG, "resumed");
                break;

            default:
                vTaskDelay(pdMS_TO_TICKS(100));
                break;
        }
    }
}

void task_state_machine_start(void)
{
    xTaskCreate(state_machine_task, "fsm", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "state machine task created");
}
