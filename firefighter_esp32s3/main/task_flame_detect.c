// task_flame_detect.c - IR flame sensors

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "config.h"
#include "shared.h"

static const char *TAG = "FLAME";

// all 5 sensors (L2, L1, C, R1, R2)
static const adc_channel_t flame_ch[5] = {
    FLAME_ADC_CH0, FLAME_ADC_CH1, FLAME_ADC_CH2, FLAME_ADC_CH3, FLAME_ADC_CH4
};
static const float flame_weights[5] = { -1.0f, -0.5f, 0.0f, 0.5f, 1.0f };

static adc_oneshot_unit_handle_t adc_handle = NULL;

static void flame_task(void *arg)
{
    uint32_t confirm_start = 0;
    bool     confirming    = false;
    uint32_t last_print    = 0;
    while (1) {
        int raw[5];
        int filt[5];

        for (int i = 0; i < 5; i++) {
            adc_oneshot_read(adc_handle, flame_ch[i], &raw[i]);
            filt[i] = (i == 3) ? ((raw[i] > 400) ? raw[i] - 400 : 0) : raw[i];
        }

        {
            uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if (now - last_print >= 200) {
                last_print = now;
                ESP_LOGI(TAG, "RAW  L2:%4d  L1:%4d  C:%4d  R1:%4d  R2:%4d  str:%4d  bal:%s",
                         filt[0], filt[1], filt[2], filt[3], filt[4], g_flame_strength,
                         g_flame_centered ? "OK" : "NO");
            }
        }

        // find dominant sensor and second-highest
        int max_val = filt[0], max_idx = 0;
        for (int i = 1; i < 5; i++) {
            if (filt[i] > max_val) { max_val = filt[i]; max_idx = i; }
        }
        int second_max = 0;
        for (int i = 0; i < 5; i++) {
            if (i != max_idx && filt[i] > second_max) second_max = filt[i];
        }

        g_flame_strength = max_val;
        g_flame_centered = (second_max >= FLAME_CLOSE_ADC);

        // require dominant to beat second by margin — accounts for L1 sensitivity bias
        float bearing;
        if (max_val - second_max >= FLAME_SENSOR_MARGIN)
            bearing = flame_weights[max_idx];
        else
            bearing = 0.0f;  // too close to call, treat as centered

        if (max_val > FLAME_THRESHOLD) {
            if (!confirming) {
                confirming    = true;
                confirm_start = xTaskGetTickCount() * portTICK_PERIOD_MS;
            }
            uint32_t elapsed = xTaskGetTickCount() * portTICK_PERIOD_MS - confirm_start;
            if (elapsed >= FLAME_CONFIRM_MS) {
                g_flame_detected = true;
                g_flame_bearing  = bearing;
                ESP_LOGI(TAG, "flame bearing:%.2f (dom=%d val=%d margin=%d)",
                         g_flame_bearing, max_idx, max_val, max_val - second_max);
            }
        } else {
            confirming       = false;
            g_flame_detected = false;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void task_flame_detect_start(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    adc_oneshot_new_unit(&unit_cfg, &adc_handle);

    adc_oneshot_chan_cfg_t ch_cfg = {
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    for (int i = 0; i < 5; i++)
        adc_oneshot_config_channel(adc_handle, flame_ch[i], &ch_cfg);

    xTaskCreate(flame_task, "flame", 4096, NULL, 4, NULL);
    ESP_LOGI(TAG, "flame detect started");
}
