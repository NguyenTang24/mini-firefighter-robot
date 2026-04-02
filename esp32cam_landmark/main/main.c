// esp32cam_landmark - detects colored cards and prints bearing over serial

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_camera.h"
#include "cam_colors.h"

static const char *TAG = "CAM";

#define FRAME_W           160
#define FRAME_H           120
#define FOV_RAD           1.047f    // ~60 degree horizontal FOV
#define MIN_BLOB_PIXELS    40
#define MAX_BLOB_PIXELS  15000
#define MIN_BOX_W          5
#define MIN_BOX_H          5
#define MAX_BOX_W        160
#define MAX_BOX_H        120
#define LOOP_MS           80
#define CONFIRM_FRAMES     3       // frames in a row before reporting a landmark
#define NUM_LANDMARKS      5       // 0=Red(off) 1=Orange 2=Pink 3=Yellow 4=Purple

// AI-Thinker ESP32-CAM pinout
static camera_config_t cam_cfg = {
    .pin_pwdn     = 32,
    .pin_reset    = -1,
    .pin_xclk     = 0,
    .pin_sccb_sda = 26,
    .pin_sccb_scl = 27,
    .pin_d7 = 35, .pin_d6 = 34, .pin_d5 = 39, .pin_d4 = 36,
    .pin_d3 = 21, .pin_d2 = 19, .pin_d1 = 18, .pin_d0 =  5,
    .pin_vsync = 25, .pin_href = 23, .pin_pclk = 22,

    .xclk_freq_hz  = 20000000,
    .ledc_timer    = LEDC_TIMER_0,
    .ledc_channel  = LEDC_CHANNEL_0,
    .pixel_format  = PIXFORMAT_RGB565,
    .frame_size    = FRAMESIZE_QQVGA,   // 160x120
    .jpeg_quality  = 12,
    .fb_count      = 1,
    .fb_location   = CAMERA_FB_IN_DRAM,
    .grab_mode     = CAMERA_GRAB_WHEN_EMPTY,
};

typedef struct {
    int   pixel_count;
    float bearing;
    bool  found;
} Blob;

// scan frame for pixels matching the color range
static Blob scan_blob(const uint8_t *buf, int w, int h,
                      int r_min, int r_max,
                      int g_min, int g_max,
                      int b_min, int b_max,
                      int dom_ch, int dom_margin)
{
    Blob result = { .found = false };
    long cx_sum = 0;
    int  count  = 0;
    int  x0 = w, y0 = h, x1 = 0, y1 = 0;

    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            int idx = (y * w + x) * 2;
            uint16_t px = ((uint16_t)buf[idx] << 8) | buf[idx + 1];

            int R = (px >> 11) & 0x1F;
            int G = (px >>  5) & 0x3F;
            int B =  px        & 0x1F;

            if (!(R >= r_min && R <= r_max &&
                  G >= g_min && G <= g_max &&
                  B >= b_min && B <= b_max))
                continue;

            // scale R and B to 6-bit for comparison with G
            int R6 = R * 2, B6 = B * 2;
            if (dom_ch == 1 && !(R6 >= G  + dom_margin && R6 >= B6 + dom_margin)) continue;
            if (dom_ch == 2 && !(G  >= R6 + dom_margin && G  >= B6 + dom_margin)) continue;
            if (dom_ch == 3 && !(B6 >= R6 + dom_margin && B6 >= G  + dom_margin)) continue;

            cx_sum += x;
            count++;
            if (x < x0) x0 = x;
            if (x > x1) x1 = x;
            if (y < y0) y0 = y;
            if (y > y1) y1 = y;
        }
    }

    if (count < MIN_BLOB_PIXELS || count > MAX_BLOB_PIXELS)
        return result;

    int box_w    = x1 - x0 + 1;
    int box_h    = y1 - y0 + 1;
    int box_area = box_w * box_h;

    // reject boxes that are too small or too big
    bool size_ok  = (box_w >= MIN_BOX_W && box_w <= MAX_BOX_W) &&
                    (box_h >= MIN_BOX_H && box_h <= MAX_BOX_H);

    // real cards are dense (~70%), scattered noise is not
    bool dense_ok = (count * 100 >= box_area * 50);

    if (!size_ok || !dense_ok)
        return result;

    float cx       = (float)cx_sum / count;
    result.bearing     = (cx - w / 2.0f) / (w / 2.0f) * (FOV_RAD / 2.0f);
    result.pixel_count = count;
    result.found       = true;
    return result;
}

static void camera_task(void *arg)
{
    static int consec_id  = -1;
    static int consec_cnt =  0;

    ESP_LOGI(TAG, "detection running");

    while (1) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            vTaskDelay(pdMS_TO_TICKS(LOOP_MS));
            continue;
        }

        Blob detections[NUM_LANDMARKS] = {0};

        // LM:0 Red - disabled
        detections[0].found = false;

        // LM:1 Orange - B <= 8 separates it from everything else
        detections[1] = scan_blob(fb->buf, fb->width, fb->height,
                                  ORN_R_MIN, ORN_R_MAX,
                                  ORN_G_MIN, ORN_G_MAX,
                                  ORN_B_MIN, ORN_B_MAX, 0, 0);

        // LM:2 Pink - R must dominate by at least 6
        detections[2] = scan_blob(fb->buf, fb->width, fb->height,
                                  PNK_R_MIN, PNK_R_MAX,
                                  PNK_G_MIN, PNK_G_MAX,
                                  PNK_B_MIN, PNK_B_MAX, 1, 6);

        // LM:3 Yellow
        detections[3] = scan_blob(fb->buf, fb->width, fb->height,
                                  YLW_R_MIN, YLW_R_MAX,
                                  YLW_G_MIN, YLW_G_MAX,
                                  YLW_B_MIN, YLW_B_MAX, 0, 0);

        // LM:4 Purple (center)
        detections[4] = scan_blob(fb->buf, fb->width, fb->height,
                                  PUR_R_MIN, PUR_R_MAX,
                                  PUR_G_MIN, PUR_G_MAX,
                                  PUR_B_MIN, PUR_B_MAX, 0, 0);

        esp_camera_fb_return(fb);

        // pick the largest blob
        int best_id = -1;
        int best_px =  0;
        for (int i = 1; i < NUM_LANDMARKS; i++) {
            if (detections[i].found && detections[i].pixel_count > best_px) {
                best_px = detections[i].pixel_count;
                best_id = i;
            }
        }

        // require CONFIRM_FRAMES consecutive matching frames before reporting
        if (best_id >= 0) {
            if (best_id == consec_id) {
                consec_cnt++;
            } else {
                consec_id  = best_id;
                consec_cnt = 1;
            }
        } else {
            consec_id  = -1;
            consec_cnt =  0;
        }

        if (consec_cnt >= CONFIRM_FRAMES) {
            printf("LM:%d bearing:%+.2f px:%d\n",
                   best_id,
                   detections[best_id].bearing,
                   detections[best_id].pixel_count);
        } else {
            printf("NO_LM\n");
        }

        vTaskDelay(pdMS_TO_TICKS(LOOP_MS));
    }
}

void app_main(void)
{
    esp_err_t err = esp_camera_init(&cam_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "camera init failed: 0x%x", err);
        return;
    }

    // AWB must be OFF - AWB on makes orange undetectable
    sensor_t *s = esp_camera_sensor_get();
    s->set_whitebal(s, 0);
    s->set_awb_gain(s, 0);
    s->set_wb_mode(s, 4);         // 4 = home/indoor preset
    s->set_saturation(s, 0);
    s->set_contrast(s, 0);
    s->set_brightness(s, 0);
    s->set_exposure_ctrl(s, 1);
    s->set_gain_ctrl(s, 1);

    vTaskDelay(pdMS_TO_TICKS(300));
    ESP_LOGI(TAG, "camera ready");

    xTaskCreatePinnedToCore(camera_task, "cam", 8192, NULL, 5, NULL, 1);
}
