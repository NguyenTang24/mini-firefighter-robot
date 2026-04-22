// task_lidar.c - TF-Luna lidar (UART1) + SG90 pan servo (LEDC)

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "config.h"
#include "shared.h"

static const char *TAG = "LIDAR";

#define FRAME_LEN          9
#define HEADER             0x59
#define SERVO_LEDC_TIMER   LEDC_TIMER_1
#define SERVO_LEDC_CHANNEL LEDC_CHANNEL_1
#define SERVO_LEDC_FREQ    50
#define SERVO_LEDC_RES     LEDC_TIMER_14_BIT   // 16384 ticks per 20ms period

static uint32_t us_to_duty(int us)
{
    return (uint32_t)((us * 16384UL) / 20000);
}

static void servo_set_us(int us)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, SERVO_LEDC_CHANNEL, us_to_duty(us));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, SERVO_LEDC_CHANNEL);
}

void lidar_servo_center(void) { servo_set_us(SERVO_CENTER_US); }
void lidar_servo_left(void)   { servo_set_us(SERVO_LEFT_US);   }
void lidar_servo_right(void)  { servo_set_us(SERVO_RIGHT_US);  }

// read one valid TF-Luna frame; returns distance in cm or -1 on timeout
static int read_distance_cm(int timeout_ms)
{
    uint8_t b;
    uint32_t deadline = xTaskGetTickCount() * portTICK_PERIOD_MS + timeout_ms;

    while (xTaskGetTickCount() * portTICK_PERIOD_MS < deadline) {
        if (uart_read_bytes(LIDAR_UART_NUM, &b, 1, pdMS_TO_TICKS(50)) < 1) continue;
        if (b != HEADER) continue;
        if (uart_read_bytes(LIDAR_UART_NUM, &b, 1, pdMS_TO_TICKS(20)) < 1) continue;
        if (b != HEADER) continue;

        uint8_t frame[FRAME_LEN];
        frame[0] = HEADER;
        frame[1] = HEADER;
        if (uart_read_bytes(LIDAR_UART_NUM, &frame[2], 7, pdMS_TO_TICKS(20)) < 7) continue;

        uint8_t sum = 0;
        for (int i = 0; i < 8; i++) sum += frame[i];
        if (sum != frame[8]) continue;

        int dist     = (frame[3] << 8) | frame[2];
        int strength = (frame[5] << 8) | frame[4];
        if (strength > 100 && dist < 1200) return dist;
    }
    return -1;
}

// pan left and right to find the clearer side
// returns +1 for right, -1 for left
int lidar_scan_obstacle_side(void)
{
    lidar_servo_left();
    vTaskDelay(pdMS_TO_TICKS(400));
    int left_cm = read_distance_cm(300);

    lidar_servo_right();
    vTaskDelay(pdMS_TO_TICKS(400));
    int right_cm = read_distance_cm(300);

    lidar_servo_center();
    vTaskDelay(pdMS_TO_TICKS(300));

    ESP_LOGI(TAG, "scan: left=%dcm right=%dcm", left_cm, right_cm);

    if (left_cm < 0 && right_cm < 0) return 1;
    if (left_cm  < 0) return 1;
    if (right_cm < 0) return -1;
    return (right_cm >= left_cm) ? 1 : -1;
}

static void lidar_task(void *arg)
{
    while (1) {
        int d = read_distance_cm(200);
        if (d > 0) g_lidar_cm = d;
    }
}

void task_lidar_start(void)
{
    uart_config_t uart_cfg = {
        .baud_rate = LIDAR_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(LIDAR_UART_NUM, &uart_cfg);
    uart_set_pin(LIDAR_UART_NUM, LIDAR_TX_GPIO, LIDAR_RX_GPIO,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(LIDAR_UART_NUM, 256, 0, 0, NULL, 0);

    ledc_timer_config_t timer_cfg = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = SERVO_LEDC_TIMER,
        .duty_resolution = SERVO_LEDC_RES,
        .freq_hz         = SERVO_LEDC_FREQ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer_cfg);

    ledc_channel_config_t ch_cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = SERVO_LEDC_CHANNEL,
        .timer_sel  = SERVO_LEDC_TIMER,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = SERVO_GPIO,
        .duty       = us_to_duty(SERVO_CENTER_US),
        .hpoint     = 0,
    };
    ledc_channel_config(&ch_cfg);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, SERVO_LEDC_CHANNEL);

    xTaskCreate(lidar_task, "lidar", 2048, NULL, 4, NULL);
    ESP_LOGI(TAG, "lidar task started");
}
