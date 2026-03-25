/*
 * esp32cam_calibrate — Color threshold calibration tool
 * Board  : AI-Thinker ESP32-CAM
 * Target : ESP-IDF v5.x
 *
 * HOW TO USE:
 *   1. Flash this project to the ESP32-CAM
 *   2. Open serial monitor at 115200 baud
 *   3. Hold a color card in front of the camera
 *   4. Press 'r' in the monitor to record that reading
 *   5. Repeat for each color
 *   6. Copy the recorded values into cam_colors.h with ±4 margin
 *
 * Live output (continuous):
 *   CAL  avg: R=18 G=8 B=5   min: R=14 G=5 B=3   max: R=22 G=11 B=7
 *
 * When you press 'r':
 *   RECORDED #1  avg: R=18 G=8 B=5   min: R=14 G=5 B=3   max: R=22 G=11 B=7
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_camera.h"

static const char *TAG = "CAL";

/* AI-Thinker pin map — do NOT change */
static camera_config_t cam_cfg = {
    .pin_pwdn     = 32,  .pin_reset    = -1,
    .pin_xclk     =  0,  .pin_sccb_sda = 26,  .pin_sccb_scl = 27,
    .pin_d7 = 35, .pin_d6 = 34, .pin_d5 = 39, .pin_d4 = 36,
    .pin_d3 = 21, .pin_d2 = 19, .pin_d1 = 18, .pin_d0 =  5,
    .pin_vsync = 25, .pin_href = 23, .pin_pclk = 22,
    .xclk_freq_hz  = 20000000,
    .ledc_timer    = LEDC_TIMER_0,
    .ledc_channel  = LEDC_CHANNEL_0,
    .pixel_format  = PIXFORMAT_RGB565,
    .frame_size    = FRAMESIZE_QQVGA,   /* 160x120 */
    .jpeg_quality  = 12,
    .fb_count      = 1,
    .fb_location   = CAMERA_FB_IN_DRAM,
    .grab_mode     = CAMERA_GRAB_WHEN_EMPTY,
};

typedef struct { int avg, mn, mx; } Chan;
typedef struct { Chan r, g, b; } Sample;

static Sample last_sample;
static volatile bool do_record = false;
static int record_count = 0;

static Sample sample_region(const uint8_t *buf, int w,
                             int x0, int y0, int x1, int y1)
{
    long rs = 0, gs = 0, bs = 0;
    int  rn = 31, gn = 63, bn = 31;
    int  rx = 0,  gx = 0,  bx = 0;
    int  n  = 0;

    for (int y = y0; y < y1; y++) {
        for (int x = x0; x < x1; x++) {
            int idx = (y * w + x) * 2;
            uint16_t px = ((uint16_t)buf[idx] << 8) | buf[idx + 1];
            int R = (px >> 11) & 0x1F;
            int G = (px >>  5) & 0x3F;
            int B =  px        & 0x1F;
            rs += R; gs += G; bs += B;
            if (R < rn) { rn = R; } if (R > rx) { rx = R; }
            if (G < gn) { gn = G; } if (G > gx) { gx = G; }
            if (B < bn) { bn = B; } if (B > bx) { bx = B; }
            n++;
        }
    }

    Sample s;
    s.r.avg = rs / n;  s.r.mn = rn;  s.r.mx = rx;
    s.g.avg = gs / n;  s.g.mn = gn;  s.g.mx = gx;
    s.b.avg = bs / n;  s.b.mn = bn;  s.b.mx = bx;
    return s;
}

/* ── UART input task — watches for 'r' keypress ───────────────────────── */
static void input_task(void *arg)
{
    uart_config_t cfg = {
        .baud_rate  = 115200,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &cfg);

    uint8_t ch;
    while (1) {
        int len = uart_read_bytes(UART_NUM_0, &ch, 1, portMAX_DELAY);
        if (len > 0 && (ch == 'r' || ch == 'R')) {
            do_record = true;
        }
    }
}

/* ── Main calibration task ────────────────────────────────────────────── */
static void calibrate_task(void *arg)
{
    esp_err_t err = esp_camera_init(&cam_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed: 0x%x", err);
        vTaskDelete(NULL);
        return;
    }

    /* Match landmark firmware camera settings exactly */
    sensor_t *sensor = esp_camera_sensor_get();
    sensor->set_whitebal(sensor, 0);      /* AWB OFF — fixed white balance */
    sensor->set_awb_gain(sensor, 0);
    sensor->set_wb_mode(sensor, 4);       /* 4 = home (indoor) */
    sensor->set_saturation(sensor, 0);
    sensor->set_contrast(sensor, 0);
    sensor->set_brightness(sensor, 0);
    vTaskDelay(pdMS_TO_TICKS(300));

    printf("\n=== ESP32-CAM Calibration Tool ===\n");
    printf("AWB OFF — home WB — default saturation/contrast\n");
    printf("Hold color card ~16 inches from camera.\n");
    printf("Press 'r' to record the current reading.\n");
    printf("Format: avg R/G/B   min R/G/B   max R/G/B\n\n");

    while (1) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        int cx = fb->width  / 2;
        int cy = fb->height / 2;
        Sample s = sample_region(fb->buf, fb->width,
                                 cx - 20, cy - 20,
                                 cx + 20, cy + 20);
        esp_camera_fb_return(fb);

        last_sample = s;

        printf("CAL  avg: R=%2d G=%2d B=%2d   "
               "min: R=%2d G=%2d B=%2d   "
               "max: R=%2d G=%2d B=%2d\n",
               s.r.avg, s.g.avg, s.b.avg,
               s.r.mn,  s.g.mn,  s.b.mn,
               s.r.mx,  s.g.mx,  s.b.mx);

        if (do_record) {
            do_record = false;
            record_count++;
            printf("\n>>> RECORDED #%d  avg: R=%2d G=%2d B=%2d   "
                   "min: R=%2d G=%2d B=%2d   "
                   "max: R=%2d G=%2d B=%2d\n"
                   "    Suggested thresholds (avg ±4):\n"
                   "    R_MIN=%d R_MAX=%d\n"
                   "    G_MIN=%d G_MAX=%d\n"
                   "    B_MIN=%d B_MAX=%d\n\n",
                   record_count,
                   s.r.avg, s.g.avg, s.b.avg,
                   s.r.mn,  s.g.mn,  s.b.mn,
                   s.r.mx,  s.g.mx,  s.b.mx,
                   s.r.avg - 4, s.r.avg + 4,
                   s.g.avg - 4, s.g.avg + 4,
                   s.b.avg - 4, s.b.avg + 4);
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main(void)
{
    xTaskCreate(input_task,     "input", 2048, NULL, 6, NULL);
    xTaskCreatePinnedToCore(calibrate_task, "cal", 8192, NULL, 5, NULL, 1);
}
