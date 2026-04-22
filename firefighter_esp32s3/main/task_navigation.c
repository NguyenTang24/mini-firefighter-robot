// task_navigation.c - follow landmarks and turn at each one

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
extern int  motor_calib_trim(int speed_pct, uint32_t duration_ms);
extern int  lidar_scan_obstacle_side(void);

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
    int corr  = (int)(fabsf(bearing) * STEER_ADJUST_PCT);

    if (bearing < -NAV_BEAR_TOL) {
        left  += corr;   // speed up outer (left) to turn left
        right -= corr;   // slow down inner (right)
    } else if (bearing > NAV_BEAR_TOL) {
        right += corr;   // speed up outer (right) to turn right
        left  -= corr;   // slow down inner (left)
    }

    if (left  > 100) left  = 100;
    if (left  <   0) left  =   0;
    if (right > 100) right = 100;
    if (right <   0) right =   0;

    if (left != right)
        ESP_LOGI("STEER", "bear:%.3f L:%d R:%d", bearing, left, right);
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

static void handle_obstacle(void)
{
    motor_stop();
    g_state = STATE_OBSTACLE_AVOID;
    ESP_LOGI(TAG, "obstacle at %dcm", g_lidar_cm);

    int side = lidar_scan_obstacle_side();

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
        "NONE", "ORANGE", "GREEN", "RED"
    };

    while(1) {
        // wait for start or calibrate command
        EventBits_t cmd = xEventGroupWaitBits(eg_commands,
            CMD_START_BIT | CMD_CALIB_BIT, pdTRUE, pdFALSE, portMAX_DELAY);

        if (cmd & CMD_CALIB_BIT) {
            ESP_LOGI(TAG, "calibration: driving straight %d%% for 3s", FWD_SPEED_PCT);
            motor_calib_trim(65, 5000);
            continue;
        }

        g_state      = STATE_PATH_FOLLOW;
        seq_idx      = 0;
        lm_confirm   = 0;
        last_bearing = 0.0f;
        ESP_LOGI(TAG, "navigation started");

#if FIRE_TEST_MODE
        ESP_LOGI(TAG, "FIRE TEST: skipping navigation, going straight to fire approach");
        g_state = STATE_FIRE_APPROACH;
        continue;
#endif

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
                EventBits_t wb = xEventGroupWaitBits(eg_commands,
                    CMD_RESUME_BIT | CMD_RESET_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
                if (wb & CMD_RESET_BIT) {
                    xEventGroupClearBits(eg_commands, CMD_RESET_BIT | CMD_STOP_BIT | CMD_RESUME_BIT);
                    motor_stop();
                    g_state = STATE_IDLE;
                    ESP_LOGI(TAG, "reset during stop");
                    running = false;
                    continue;
                }
                g_state = STATE_PATH_FOLLOW;
                ESP_LOGI(TAG, "resumed");
            }

            if (g_state == STATE_DONE) {
                motor_stop();
                vTaskDelay(pdMS_TO_TICKS(500));
                continue;
            }

            if (g_lidar_cm > 0 && g_lidar_cm < OBSTACLE_DIST_CM &&
                g_state == STATE_PATH_FOLLOW) {
                handle_obstacle();
                continue;
            }

            // keep driving toward last known bearing between camera frames
            drive_with_bearing(last_bearing);

            if (xQueueReceive(q_camera, &msg, pdMS_TO_TICKS(50)) != pdTRUE) {
                last_bearing *= 0.90f;
                continue;
            }

            if (msg.is_fire) {
                g_flame_bearing = msg.bearing;
                continue;
            }

            if (msg.lm_id >= 0 && msg.lm_id < (int)(sizeof(color_names)/sizeof(color_names[0]))) {
                ESP_LOGI(TAG, "saw %s (LM:%d) px:%d want %s (LM:%d)",
                         color_names[msg.lm_id], msg.lm_id, msg.px_size,
                         color_names[expected_lm()], expected_lm());
            }

            bool lm_ready = xTaskGetTickCount() * portTICK_PERIOD_MS >= lm_enable_ms;

            // update bearing only after cooldown so post-turn pre-bias isn't overwritten
            if (lm_ready && msg.lm_id == expected_lm())
                last_bearing = msg.bearing;

            // trigger turn once close enough
            if (lm_ready && msg.lm_id == expected_lm() && msg.px_size >= MIN_LM_PIXEL_SIZE
                && fabsf(msg.bearing) < TURN_BEAR_TOL) {
                lm_confirm++;
                ESP_LOGI(TAG, "LM:%d px:%d confirm %d/%d",
                         msg.lm_id, msg.px_size, lm_confirm, LM_CONFIRM_FRAMES);

                if (lm_confirm >= LM_CONFIRM_FRAMES) {
                    lm_confirm = 0;
                    motor_stop();

                    if (seq_idx == LANDMARK_SEQUENCE_LEN - 1) {  // last landmark triggers fire approach
                        g_state = STATE_FIRE_APPROACH;
                        ESP_LOGI(TAG, "reached center - looking for fire");
                        continue;
                    }

                    ESP_LOGI(TAG, "LM:%d confirmed, turning left", msg.lm_id);
                    motor_turn_left_90();
                    seq_idx++;
                    last_bearing  = 0.0f;
                    lm_enable_ms  = xTaskGetTickCount() * portTICK_PERIOD_MS + LM_COOLDOWN_MS;
                    // reverse to swing caster toward forward-aligned position
                    motor_drive(-(FWD_SPEED_PCT / 2), -(FWD_SPEED_PCT / 2));
                    vTaskDelay(pdMS_TO_TICKS(600));
                    motor_stop();
                    vTaskDelay(pdMS_TO_TICKS(100));
                    // left motor has higher stiction after a stop; run it stronger
                    // for 400ms to break free and counter post-turn leftward caster pull
                    motor_drive(POST_TURN_L_PCT, POST_TURN_R_PCT);
                    vTaskDelay(pdMS_TO_TICKS(POST_TURN_MS));
                    motor_stop();
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
    xTaskCreate(navigation_task, "nav", 10240, NULL, 4, NULL);
    ESP_LOGI(TAG, "navigation task created");
}
