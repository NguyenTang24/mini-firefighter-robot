// task_flame_detect.c - IR flame sensor array (5 channels)
//
// Reads FLAME_ADC_CH0..CH4 (GPIO 5-9, ADC1 CH4-CH8 on ESP32-S3).
// Uses the IDF 5.x oneshot ADC API (esp_adc/adc_oneshot.h).
//
// Writes to shared globals every poll cycle:
//   g_flame_detected  - true if any channel exceeds FLAME_THRESHOLD
//                       for at least FLAME_CONFIRM_MS consecutively
//   g_flame_bearing   - weighted-average bearing in radians
//                       (negative = left, positive = right, 0 = center)
//                       matches the convention used by task_navigation.c
//
// ── To activate this task, add to main.c ──────────────────────────────
//   Forward declaration (with the other externs at the top):
//     void task_flame_detect_start(void);
//
//   Call in app_main() after shared_init():
//     task_flame_detect_start();
// ─────────────────────────────────────────────────────────────────────

#include <string.h>
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "config.h"
#include "shared.h"

static const char *TAG = "FLAME";

// ── Poll interval ─────────────────────────────────────────────────────
// Run faster than FLAME_CONFIRM_MS so the confirmation timer is accurate.
// 50 ms gives ~6 samples inside the 300 ms confirm window.
#define POLL_MS  50

// ── Sensor count ──────────────────────────────────────────────────────
#define NUM_SENSORS  5

// ── GPIO → ADC1 channel mapping for ESP32-S3 ─────────────────────────
// On ESP32-S3, ADC1_CHANNEL_N maps to GPIO (N + 1):
//   GPIO 5 = ADC1_CHANNEL_4
//   GPIO 6 = ADC1_CHANNEL_5
//   GPIO 7 = ADC1_CHANNEL_6
//   GPIO 8 = ADC1_CHANNEL_7
//   GPIO 9 = ADC1_CHANNEL_8
//
// config.h defines FLAME_ADC_CH0..4 as the GPIO numbers (5-9),
// so we subtract 1 to get the ADC1 channel index.
static const adc_channel_t FLAME_CHANNELS[NUM_SENSORS] = {
    (adc_channel_t)(FLAME_ADC_CH0 - 1),   // CH0: GPIO5 → ADC1_CHANNEL_4
    (adc_channel_t)(FLAME_ADC_CH1 - 1),   // CH1: GPIO6 → ADC1_CHANNEL_5
    (adc_channel_t)(FLAME_ADC_CH2 - 1),   // CH2: GPIO7 → ADC1_CHANNEL_6 (center)
    (adc_channel_t)(FLAME_ADC_CH3 - 1),   // CH3: GPIO8 → ADC1_CHANNEL_7
    (adc_channel_t)(FLAME_ADC_CH4 - 1),   // CH4: GPIO9 → ADC1_CHANNEL_8
};

// ── Bearing weights for each sensor position ──────────────────────────
// Evenly spaced across ±0.52 rad (~±30°), matching the camera's FOV
// convention used by task_navigation.c and task_state_machine.c.
//   index 0 = leftmost  → -0.52 rad
//   index 2 = center    →  0.00 rad
//   index 4 = rightmost → +0.52 rad
static const float SENSOR_BEARINGS[NUM_SENSORS] = {
    -0.52f, -0.26f, 0.00f, 0.26f, 0.52f
};

// ── ADC handle (module-level so it stays alive for the task lifetime) ─
static adc_oneshot_unit_handle_t s_adc1_handle = NULL;

// ─────────────────────────────────────────────────────────────────────
//  adc_init() — configure the ADC1 unit and all five channels
// ─────────────────────────────────────────────────────────────────────
static esp_err_t adc_init(void)
{
    // Create the ADC1 oneshot unit
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id  = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    esp_err_t err = adc_oneshot_new_unit(&unit_cfg, &s_adc1_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "adc_oneshot_new_unit failed: %s", esp_err_to_name(err));
        return err;
    }

    // Configure each channel: 12-bit, 12 dB attenuation (0–3.6 V range)
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,   // 0–4095, matches FLAME_THRESHOLD=2000
    };
    for (int i = 0; i < NUM_SENSORS; i++) {
        err = adc_oneshot_config_channel(s_adc1_handle, FLAME_CHANNELS[i], &chan_cfg);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "channel %d config failed: %s", i, esp_err_to_name(err));
            return err;
        }
    }

    ESP_LOGI(TAG, "ADC1 ready — %d channels, threshold=%d", NUM_SENSORS, FLAME_THRESHOLD);
    return ESP_OK;
}

// ─────────────────────────────────────────────────────────────────────
//  compute_bearing() — weighted average of all active sensor positions
//
//  Each sensor that reads above FLAME_THRESHOLD contributes its
//  position bearing weighted by how far above threshold it is.
//  Returns 0.0 and sets *any_active = false if nothing is detected.
// ─────────────────────────────────────────────────────────────────────
static float compute_bearing(const int raw[NUM_SENSORS], bool *any_active)
{
    float weight_sum  = 0.0f;
    float bearing_sum = 0.0f;
    *any_active = false;

    for (int i = 0; i < NUM_SENSORS; i++) {
        if (raw[i] >= FLAME_THRESHOLD) {
            float w    = (float)(raw[i] - FLAME_THRESHOLD);  // stronger = more weight
            bearing_sum += SENSOR_BEARINGS[i] * w;
            weight_sum  += w;
            *any_active  = true;
        }
    }

    if (!(*any_active) || weight_sum == 0.0f)
        return 0.0f;

    return bearing_sum / weight_sum;
}

// ─────────────────────────────────────────────────────────────────────
//  flame_detect_task() — main polling loop
// ─────────────────────────────────────────────────────────────────────
static void flame_detect_task(void *arg)
{
    int      raw[NUM_SENSORS]  = {0};
    uint32_t first_detect_ms   = 0;   // timestamp of first positive reading in streak
    bool     streak            = false;

    while (1) {
        // Read all five sensors
        bool read_ok = true;
        for (int i = 0; i < NUM_SENSORS; i++) {
            esp_err_t err = adc_oneshot_read(s_adc1_handle, FLAME_CHANNELS[i], &raw[i]);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "ADC read ch%d failed: %s", i, esp_err_to_name(err));
                raw[i]   = 0;
                read_ok  = false;
            }
        }

        bool     any_active = false;
        float    bearing    = 0.0f;

        if (read_ok) {
            bearing = compute_bearing(raw, &any_active);
        }

        uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

        if (any_active) {
            if (!streak) {
                // start of a new detection streak
                streak          = true;
                first_detect_ms = now_ms;
                ESP_LOGD(TAG, "flame candidate — waiting for confirm (%dms)", FLAME_CONFIRM_MS);
            }

            // update bearing immediately so approach steering is responsive
            g_flame_bearing = bearing;

            uint32_t streak_ms = now_ms - first_detect_ms;
            if (streak_ms >= (uint32_t)FLAME_CONFIRM_MS) {
                // confirmed — signal the rest of the system
                if (!g_flame_detected) {
                    ESP_LOGI(TAG, "FLAME CONFIRMED bearing=%.3f rad  streak=%lums",
                             bearing, (unsigned long)streak_ms);
                }
                g_flame_detected = true;
            }

            ESP_LOGD(TAG, "raw: %4d %4d %4d %4d %4d  bearing=%+.3f  streak=%lums",
                     raw[0], raw[1], raw[2], raw[3], raw[4],
                     bearing, (unsigned long)(now_ms - first_detect_ms));

        } else {
            // no active sensor — reset streak and clear globals
            if (streak) {
                ESP_LOGI(TAG, "flame gone — clearing detection");
            }
            streak           = false;
            first_detect_ms  = 0;
            g_flame_detected = false;
            g_flame_bearing  = 0.0f;
        }

        vTaskDelay(pdMS_TO_TICKS(POLL_MS));
    }
}

// ─────────────────────────────────────────────────────────────────────
//  task_flame_detect_start() — init ADC and launch the task
// ─────────────────────────────────────────────────────────────────────
void task_flame_detect_start(void)
{
    if (adc_init() != ESP_OK) {
        ESP_LOGE(TAG, "ADC init failed — flame detection disabled");
        return;
    }

    // Priority 6 — one above navigation (4) so sensor data is always fresh,
    // but below the motor speed controller (6 in task_motor.c; match is fine
    // since they run on different cores or brief slices).
    xTaskCreate(flame_detect_task, "flame", 3072, NULL, 6, NULL);
    ESP_LOGI(TAG, "flame detect task started");
}