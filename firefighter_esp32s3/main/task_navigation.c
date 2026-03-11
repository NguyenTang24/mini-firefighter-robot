// task_navigation.c - landmark following and obstacle avoidance
//
// Reads from q_camera. Commands motors via motor_drive/stop/turn.
// Path: orange -> pink -> yellow -> purple (center)
// Turns left 90 degrees at each landmark.

#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "config.h"
#include "shared.h"

static const char *TAG = "NAV";

extern void motor_drive(int left_pct, int right_pct);
extern void motor_stop(void);
extern void motor_turn_left_90(void);
extern void motor_turn_right_90(void);

static int seq_idx = 0;

static int expected_lm(void)
{
    if (seq_idx < LANDMARK_SEQUENCE_LEN)
        return LANDMARK_SEQUENCE[seq_idx];
    return LM_NONE;
}

// drive forward but steer to keep the landmark centered
static void drive_with_bearing(float bearing)
{
    int left  = FWD_SPEED_PCT;
    int right = FWD_SPEED_PCT;

    if (bearing < -NAV_BEAR_TOL) {
        right += (int)(bearing * -STEER_ADJUST_PCT);
        if (right > 100) right = 100;
    } else if (bearing > NAV_BEAR_TOL) {
        left -= (int)(bearing * STEER_ADJUST_PCT);
        if (left < 0) left = 0;
    }
    motor_drive(left, right);
}

// slowly rotate until we see the target landmark or time out
static float scan_for_landmark(int target_id)
{
    ESP_LOGI(TAG, "scanning for LM:%d", target_id);
    uint32_t start = xTaskGetTickCount() * portTICK_PERIOD_MS;
    cam_msg_t msg;

    for (;;) {
        if ((xTaskGetTickCount() * portTICK_PERIOD_MS - start) > SCAN_TIMEOUT_MS) {
            ESP_LOGW(TAG, "scan timed out");
            motor_stop();
            return 0.0f;
        }
        motor_drive(-SCAN_TURN_PCT, SCAN_TURN_PCT);
        if (xQueueReceive(q_camera, &msg, pdMS_TO_TICKS(SCAN_STEP_MS)) == pdTRUE) {
            if (!msg.is_fire && msg.lm_id == target_id) {
                motor_stop();
                ESP_LOGI(TAG, "reacquired LM:%d bearing:%.3f", target_id, msg.bearing);
                return msg.bearing;
            }
        }
    }
}

// obstacle avoidance requires the lidar task - re-enable when lidar is back
static void handle_obstacle(void)
{
    motor_stop();
    g_state = STATE_OBSTACLE_AVOID;
    ESP_LOGI(TAG, "obstacle at %dcm", g_lidar_cm);

    int side = 1; // default right when no lidar

    if (side <= 0) {
        motor_turn_right_90();
        motor_drive(FWD_SPEED_PCT, FWD_SPEED_PCT);
        vTaskDelay(pdMS_TO_TICKS(1500));
        motor_turn_left_90();
        motor_drive(FWD_SPEED_PCT, FWD_SPEED_PCT);
        vTaskDelay(pdMS_TO_TICKS(1500));
        motor_turn_left_90();
        motor_drive(FWD_SPEED_PCT, FWD_SPEED_PCT);
        vTaskDelay(pdMS_TO_TICKS(1500));
        motor_turn_right_90();
    } else {
        motor_turn_left_90();
        motor_drive(FWD_SPEED_PCT, FWD_SPEED_PCT);
        vTaskDelay(pdMS_TO_TICKS(1500));
        motor_turn_right_90();
        motor_drive(FWD_SPEED_PCT, FWD_SPEED_PCT);
        vTaskDelay(pdMS_TO_TICKS(1500));
        motor_turn_right_90();
        motor_drive(FWD_SPEED_PCT, FWD_SPEED_PCT);
        vTaskDelay(pdMS_TO_TICKS(1500));
        motor_turn_left_90();
    }

    motor_stop();

    // realign to landmark after going around obstacle
    float bear = scan_for_landmark(expected_lm());
    if (bear != 0.0f) {
        uint32_t align_start = xTaskGetTickCount() * portTICK_PERIOD_MS;
        cam_msg_t align_msg;
        while (fabsf(bear) > NAV_BEAR_TOL) {
            drive_with_bearing(bear);
            if (xQueueReceive(q_camera, &align_msg, pdMS_TO_TICKS(100)) == pdTRUE)
                if (!align_msg.is_fire && align_msg.lm_id == expected_lm())
                    bear = align_msg.bearing;
            if ((xTaskGetTickCount() * portTICK_PERIOD_MS - align_start) > NAV_ALIGN_TIMEOUT_MS) break;
        }
    }

    motor_stop();
    g_state = STATE_PATH_FOLLOW;
    ESP_LOGI(TAG, "avoidance done");
}

static void navigation_task(void *arg)
{
    cam_msg_t msg;
    float     last_bearing = 0.0f;
    int       lm_confirm   = 0;

    static const char *color_names[] = {
        "RED", "ORANGE", "PINK", "YELLOW", "PURPLE"
    };

    while(1) {
        // wait for start button from web UI
        xEventGroupWaitBits(eg_commands, CMD_START_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
        g_state      = STATE_PATH_FOLLOW;
        seq_idx      = 0;
        lm_confirm   = 0;
        last_bearing = 0.0f;
        ESP_LOGI(TAG, "navigation started");

        // don't look for landmarks right away, give the robot time to drive away
        uint32_t lm_enable_ms = xTaskGetTickCount() * portTICK_PERIOD_MS + LM_COOLDOWN_MS;

        bool running = true;
        while (running) {
            EventBits_t bits = xEventGroupGetBits(eg_commands);

            if (bits & CMD_RESET_BIT) {
                xEventGroupClearBits(eg_commands, CMD_RESET_BIT | CMD_STOP_BIT | CMD_RESUME_BIT);
                motor_stop();
                g_state = STATE_IDLE;
                ESP_LOGI(TAG, "reset");
                running = false;
                continue;
            }

            if (bits & CMD_STOP_BIT) {
                motor_stop();
                g_state = STATE_IDLE;
                xEventGroupClearBits(eg_commands, CMD_STOP_BIT);
                xEventGroupWaitBits(eg_commands, CMD_RESUME_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
                g_state = STATE_PATH_FOLLOW;
                ESP_LOGI(TAG, "resumed");
            }

            if (g_state == STATE_DONE) {
                motor_stop();
                vTaskDelay(pdMS_TO_TICKS(500));
                continue;
            }

            // obstacle check - disabled until lidar task is implemented
            /* if (g_lidar_cm > 0 && g_lidar_cm < OBSTACLE_DIST_CM &&
                g_state == STATE_PATH_FOLLOW) {
                handle_obstacle();
                continue;
            } */

            // keep driving toward last known bearing between camera frames
            drive_with_bearing(last_bearing);

            if (xQueueReceive(q_camera, &msg, pdMS_TO_TICKS(50)) != pdTRUE)
                continue;

            if (msg.is_fire) {
                g_flame_bearing = msg.bearing;
                continue;
            }

            if (msg.lm_id >= 0 && msg.lm_id <= 4) {
                ESP_LOGI(TAG, "saw %s (LM:%d), want %s (LM:%d)",
                         color_names[msg.lm_id], msg.lm_id,
                         color_names[expected_lm()], expected_lm());
            }

            bool lm_ready = xTaskGetTickCount() * portTICK_PERIOD_MS >= lm_enable_ms;

            if (lm_ready && msg.lm_id == expected_lm() && msg.px_size >= MIN_LM_PIXEL_SIZE) {
                lm_confirm++;
                last_bearing = msg.bearing;
                ESP_LOGI(TAG, "LM:%d px:%d confirm %d/%d",
                         msg.lm_id, msg.px_size, lm_confirm, LM_CONFIRM_FRAMES);

                if (lm_confirm >= LM_CONFIRM_FRAMES) {
                    lm_confirm = 0;
                    motor_stop();

                    if (msg.lm_id == LM_PURPLE) {
                        g_state = STATE_DONE;
                        ESP_LOGI(TAG, "reached center - done");
                        continue;
                    }

                    ESP_LOGI(TAG, "LM:%d confirmed, turning left", msg.lm_id);
                    motor_turn_left_90();
                    seq_idx++;
                    last_bearing  = 0.0f;
                    lm_enable_ms  = xTaskGetTickCount() * portTICK_PERIOD_MS + LM_COOLDOWN_MS;
                    vTaskDelay(pdMS_TO_TICKS(200));
                }

            } else if (msg.lm_id >= 0 && msg.lm_id != expected_lm()) {
                lm_confirm = 0;
            }

            // if nothing seen, slowly return bearing to straight
            if (msg.lm_id < 0)
                last_bearing *= 0.85f;
        }
    }
}

void task_navigation_start(void)
{
    xTaskCreate(navigation_task, "nav", 8192, NULL, 4, NULL);
    ESP_LOGI(TAG, "navigation task created");
}
