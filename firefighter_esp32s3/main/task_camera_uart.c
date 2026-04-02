// task_camera_uart.c - reads LM: lines from ESP32-CAM over UART

#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "config.h"
#include "shared.h"

static const char *TAG = "CAM_UART";

#define RX_BUF_SIZE  512
#define LINE_BUF_LEN 128

static void parse_line(const char *line)
{
    cam_msg_t msg = { .lm_id = -1, .bearing = 0.0f, .is_fire = false };

    if (strncmp(line, "LM:", 3) == 0) {
        int id = 0;
        float bearing = 0.0f;
        int px = 0;
        if (sscanf(line, "LM:%d bearing:%f px:%d", &id, &bearing, &px) >= 2) {
            msg.lm_id   = id;
            msg.bearing = bearing;
            msg.px_size = px;
            xQueueSend(q_camera, &msg, 0);
        }
    }
    // FIRE lines disabled - only used in fire approach state
}

static void camera_uart_task(void *arg)
{
    uint8_t rx_buf[RX_BUF_SIZE];
    char    line[LINE_BUF_LEN];
    int     line_pos = 0;

    while (1) {
        int len = uart_read_bytes(CAM_UART_NUM, rx_buf, sizeof(rx_buf) - 1,
                                  pdMS_TO_TICKS(100));
        ESP_LOGD(TAG, "RX %d bytes", len);
        for (int i = 0; i < len; i++) {
            char c = (char)rx_buf[i];
            if (c == '\n' || c == '\r') {
                if (line_pos > 0) {
                    line[line_pos] = '\0';
                    ESP_LOGD(TAG, "LINE: %s", line);
                    parse_line(line);
                    line_pos = 0;
                }
            } else if (line_pos < LINE_BUF_LEN - 1) {
                line[line_pos++] = c;
            }
        }
    }
}

void task_camera_uart_start(void)
{
    const uart_config_t cfg = {
        .baud_rate  = CAM_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_driver_install(CAM_UART_NUM, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(CAM_UART_NUM, &cfg);
    uart_set_pin(CAM_UART_NUM, UART_PIN_NO_CHANGE, CAM_RX_GPIO,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    xTaskCreate(camera_uart_task, "cam_uart", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "camera UART ready on GPIO %d", CAM_RX_GPIO);
}
