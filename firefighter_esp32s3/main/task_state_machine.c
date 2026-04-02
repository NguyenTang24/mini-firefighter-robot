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

// steer toward the flame bearing
static void steer_toward_flame(void)
{
    float b     = g_flame_bearing;
    int   left  = FIRE_APPROACH_SPEED;
    int   right = FIRE_APPROACH_SPEED;

    if (b < -FSM_BEAR_TOL)
        right += (int)(b * -FSM_STEER_GAIN);
    else if (b > FSM_BEAR_TOL)
        left  -= (int)(b *  FSM_STEER_GAIN);

    if (left  < 0)   left  = 0;
    if (right < 0)   right = 0;
    if (left  > 100) left  = 100;
    if (right > 100) right = 100;

    motor_drive(left, right);
}

static void do_extinguish(void)
{
    g_state = STATE_EXTINGUISH;
    motor_stop();

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
    uint32_t start = xTaskGetTickCount() * portTICK_PERIOD_MS;

    while (g_state == STATE_FIRE_APPROACH) {
        if ((xTaskGetTickCount() * portTICK_PERIOD_MS - start) > FIRE_APPROACH_TIMEOUT_MS) {
            ESP_LOGW(TAG, "approach timed out");
            motor_stop();
            do_extinguish();
            return;
        }

        if (g_lidar_cm > 0 && g_lidar_cm <= STANDOFF_DIST_CM) {
            motor_stop();
            ESP_LOGI(TAG, "at standoff (%dcm)", g_lidar_cm);
            do_extinguish();
            return;
        }

        if (!g_flame_detected) {
            motor_stop();
            vTaskDelay(pdMS_TO_TICKS(500));
            if (!g_flame_detected) {
                ESP_LOGW(TAG, "flame lost, aborting approach");
                g_state = STATE_PATH_FOLLOW;
                return;
            }
        }

        steer_toward_flame();
        vTaskDelay(pdMS_TO_TICKS(50));
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
