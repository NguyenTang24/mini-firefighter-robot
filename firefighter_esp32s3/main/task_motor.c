// task_motor.c - motor control

#include <math.h>
#include <stdatomic.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "config.h"
#include "shared.h"

static const char *TAG = "MOTOR";

// LEDC config for motor PWM
#define LEDC_TIMER      LEDC_TIMER_1
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_FREQ_HZ    5000
#define LEDC_RESOLUTION LEDC_TIMER_10_BIT   // 0-1023
#define LEDC_CH_L       LEDC_CHANNEL_1
#define LEDC_CH_R       LEDC_CHANNEL_2

#if USE_ENCODERS
static volatile int32_t enc_l_count = 0;
static volatile int32_t enc_r_count = 0;

static void IRAM_ATTR enc_l_isr(void *arg) { enc_l_count++; }
static void IRAM_ATTR enc_r_isr(void *arg) { enc_r_count++; }

// pulses needed for a 90 degree point turn
static int turn_90_pulses(void)
{
    float arc_mm  = (WHEELBASE_MM / 2.0f) * (M_PI / 2.0f);
    float circ_mm = M_PI * WHEEL_DIAMETER_MM;
    return (int)(arc_mm / circ_mm * ENCODER_PPR + 0.5f);
}

// speed controller - runs every SPEED_CTRL_MS, keeps both wheels at same speed
static volatile int s_req_l = 0;
static volatile int s_req_r = 0;
static volatile int s_act_l = 0;
static volatile int s_act_r = 0;

static void speed_ctrl_task(void *arg)
{
    int32_t  last_l      = 0, last_r = 0;
    bool     kicking     = false;
    uint32_t kick_end_ms = 0;
    int      prev_req    = 0;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(SPEED_CTRL_MS));

        int rl = s_req_l;
        int rr = s_req_r;
        uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

        // detect stopped -> moving straight and apply startup kick
        // skip kick for turns (rl != rr) to avoid overshoot
        if (prev_req == 0 && rl > 0 && rr > 0 && rl == rr) {
            kicking     = true;
            kick_end_ms = now_ms + KICK_MS;
            ESP_LOGI(TAG, "startup kick %dms @ %d%%", KICK_MS, KICK_PCT);
        }
        prev_req = (rl > 0 || rr > 0) ? 1 : 0;

        if (kicking) {
            if (now_ms < kick_end_ms) {
                uint32_t kick_duty = (uint32_t)(KICK_PCT * 1023 / 100);
                ledc_set_duty(LEDC_MODE, LEDC_CH_L, kick_duty);
                ledc_update_duty(LEDC_MODE, LEDC_CH_L);
                ledc_set_duty(LEDC_MODE, LEDC_CH_R, kick_duty);
                ledc_update_duty(LEDC_MODE, LEDC_CH_R);
                last_l = enc_l_count;
                last_r = enc_r_count;
                continue;
            }
            kicking = false;
        }

        // only equalize during straight forward driving
        // during turns navigation sets different speeds intentionally
        if (rl <= 0 || rr <= 0 || rl != rr) {
            last_l = enc_l_count;
            last_r = enc_r_count;
            continue;
        }

        int32_t cur_l   = enc_l_count;
        int32_t cur_r   = enc_r_count;
        int     delta_l = (int)(cur_l - last_l);
        int     delta_r = (int)(cur_r - last_r);
        last_l = cur_l;
        last_r = cur_r;

        int error = delta_l - delta_r; // positive = left faster
        if (error > -SPEED_CTRL_DEADBAND && error < SPEED_CTRL_DEADBAND)
            continue;

        int corr = error * SPEED_CTRL_KP;
        if (corr >  SPEED_CTRL_MAX_CORR) corr =  SPEED_CTRL_MAX_CORR;
        if (corr < -SPEED_CTRL_MAX_CORR) corr = -SPEED_CTRL_MAX_CORR;

        int al = (rl + MOTOR_L_TRIM) - corr;
        int ar = (rr + MOTOR_R_TRIM) + corr;
        if (al <   0) al =   0;
        if (ar <   0) ar =   0;
        if (al > 100) al = 100;
        if (ar > 100) ar = 100;

        s_act_l = al;
        s_act_r = ar;

        ledc_set_duty(LEDC_MODE, LEDC_CH_L, (uint32_t)(al * 1023 / 100));
        ledc_update_duty(LEDC_MODE, LEDC_CH_L);
        ledc_set_duty(LEDC_MODE, LEDC_CH_R, (uint32_t)(ar * 1023 / 100));
        ledc_update_duty(LEDC_MODE, LEDC_CH_R);

        ESP_LOGI(TAG, "dl=%d dr=%d err=%d corr=%d al=%d ar=%d",
                 delta_l, delta_r, error, corr, al, ar);
    }
}
#endif

static void set_pwm(ledc_channel_t ch, int pct)
{
    uint32_t duty = (uint32_t)(abs(pct) * 1023 / 100);
    ledc_set_duty(LEDC_MODE, ch, duty);
    ledc_update_duty(LEDC_MODE, ch);
}

// L298N: forward = (1,0), reverse = (0,1), brake = (0,0)
static void set_dir(int in_a, int in_b, bool forward)
{
    gpio_set_level(in_a, forward ? 1 : 0);
    gpio_set_level(in_b, forward ? 0 : 1);
}

static void set_brake(int in_a, int in_b)
{
    gpio_set_level(in_a, 0);
    gpio_set_level(in_b, 0);
}

void motor_drive(int left_pct, int right_pct)
{
    int l = left_pct  + MOTOR_L_TRIM;
    int r = right_pct + g_motor_r_trim;
    if (l >  100) l =  100;
    if (l < -100) l = -100;
    if (r >  100) r =  100;
    if (r < -100) r = -100;

    set_dir(MOTOR_L_IN1_GPIO, MOTOR_L_IN2_GPIO, l >= 0);
    set_dir(MOTOR_R_IN3_GPIO, MOTOR_R_IN4_GPIO, r >= 0);

#if USE_ENCODERS
    s_req_l = left_pct;
    s_req_r = right_pct;
    // let the speed controller handle PWM during straight driving
    if (left_pct <= 0 || right_pct <= 0 || left_pct != right_pct) {
        set_pwm(LEDC_CH_L, l);
        set_pwm(LEDC_CH_R, r);
    }
#else
    set_pwm(LEDC_CH_L, l);
    set_pwm(LEDC_CH_R, r);
#endif
}

void motor_stop(void)
{
#if USE_ENCODERS
    s_req_l = 0;
    s_req_r = 0;
#endif
    set_pwm(LEDC_CH_L, 0);
    set_pwm(LEDC_CH_R, 0);
    set_brake(MOTOR_L_IN1_GPIO, MOTOR_L_IN2_GPIO);
    set_brake(MOTOR_R_IN3_GPIO, MOTOR_R_IN4_GPIO);
}

void motor_turn_left_90(void)
{
#if USE_ENCODERS
    int      target   = turn_90_pulses();
    uint32_t deadline = xTaskGetTickCount() * portTICK_PERIOD_MS + 3000;
    enc_l_count = 0;
    enc_r_count = 0;
    motor_drive(-TURN_SPEED_PCT, TURN_SPEED_PCT);
    while (enc_l_count < target || enc_r_count < target) {
        if (xEventGroupGetBits(eg_commands) & CMD_RESET_BIT) break;
        if (xTaskGetTickCount() * portTICK_PERIOD_MS >= deadline) {
            ESP_LOGW(TAG, "turn left timeout (l=%d r=%d)", enc_l_count, enc_r_count);
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    motor_stop();
    ESP_LOGI(TAG, "turn left done (%d pulses)", enc_l_count);
#else
    motor_drive(-TURN_SPEED_PCT, TURN_SPEED_PCT);
    vTaskDelay(pdMS_TO_TICKS(g_turn_ms));
    motor_stop();
    ESP_LOGI(TAG, "turn left done (%dms)", g_turn_ms);
#endif
}

void motor_turn_right_90(void)
{
#if USE_ENCODERS
    int      target   = turn_90_pulses();
    uint32_t deadline = xTaskGetTickCount() * portTICK_PERIOD_MS + 3000;
    enc_l_count = 0;
    enc_r_count = 0;
    motor_drive(TURN_SPEED_PCT, -TURN_SPEED_PCT);
    while (enc_l_count < target || enc_r_count < target) {
        if (xEventGroupGetBits(eg_commands) & CMD_RESET_BIT) break;
        if (xTaskGetTickCount() * portTICK_PERIOD_MS >= deadline) {
            ESP_LOGW(TAG, "turn right timeout (l=%d r=%d)", enc_l_count, enc_r_count);
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    motor_stop();
    ESP_LOGI(TAG, "turn right done (%d pulses)", enc_l_count);
#else
    motor_drive(TURN_SPEED_PCT, -TURN_SPEED_PCT);
    vTaskDelay(pdMS_TO_TICKS(g_turn_ms));
    motor_stop();
    ESP_LOGI(TAG, "turn right done");
#endif
}

// Drive straight for duration_ms with no trim, compare encoder counts,
// return the MOTOR_R_TRIM value that would make both wheels match speed.
// Also applies the result immediately to g_motor_r_trim.
// Each call runs one trial. Press CALIBRATE 5 times (repositioning between each).
// Accumulates results and applies the averaged trim on the 5th press.
int motor_calib_trim(int speed_pct, uint32_t duration_ms)
{
#if USE_ENCODERS
#define CALIB_TRIALS 5
    static int   s_count    = 0;
    static float s_trim_sum = 0.0f;

    // full stop first so speed controller sees s_req=0 and won't override our PWM
    motor_stop();
    vTaskDelay(pdMS_TO_TICKS(150));

    int original_trim = g_motor_r_trim;
    g_motor_r_trim    = 0;
    enc_l_count       = 0;
    enc_r_count       = 0;

    ESP_LOGI("CALIB", "trial %d/%d: driving %d%% for %lums ...",
             s_count + 1, CALIB_TRIALS, speed_pct, (unsigned long)duration_ms);

    set_dir(MOTOR_L_IN1_GPIO, MOTOR_L_IN2_GPIO, true);
    set_dir(MOTOR_R_IN3_GPIO, MOTOR_R_IN4_GPIO, true);
    set_pwm(LEDC_CH_L, speed_pct);
    set_pwm(LEDC_CH_R, speed_pct);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    motor_stop();

    g_motor_r_trim = original_trim;  // restore between trials

    int32_t el = enc_l_count;
    int32_t er = enc_r_count;
    ESP_LOGI("CALIB", "  L=%d  R=%d  diff=%d", (int)el, (int)er, (int)(er - el));

    if (er < 5) {
        ESP_LOGW("CALIB", "  too few pulses, trial not counted - check encoders");
        return original_trim;
    }

    float trial_trim = (float)speed_pct * ((float)el / (float)er - 1.0f);
    s_trim_sum += trial_trim;
    s_count++;
    ESP_LOGI("CALIB", "  this=%.1f  running avg=%.1f  (%d/%d done)",
             trial_trim, s_trim_sum / s_count, s_count, CALIB_TRIALS);

    if (s_count < CALIB_TRIALS) {
        ESP_LOGI("CALIB", "  reposition robot, then press CALIBRATE for trial %d/%d",
                 s_count + 1, CALIB_TRIALS);
        return original_trim;
    }

    int suggested = (int)lroundf(s_trim_sum / s_count);
    suggested = suggested < -30 ? -30 : (suggested > 30 ? 30 : suggested);
    s_count    = 0;
    s_trim_sum = 0.0f;

    ESP_LOGI("CALIB", "=== RESULT (5/5): MOTOR_R_TRIM = %d  (copy to config.h) ===", suggested);
    g_motor_r_trim = suggested;
    return suggested;
#undef CALIB_TRIALS
#else
    ESP_LOGW("CALIB", "USE_ENCODERS=0, cannot calibrate");
    return MOTOR_R_TRIM;
#endif
}

float encoder_get_yaw(void)
{
#if USE_ENCODERS
    float diff   = (float)(enc_r_count - enc_l_count);
    float circ   = M_PI * WHEEL_DIAMETER_MM;
    float arc_mm = diff / ENCODER_PPR * circ;
    return arc_mm / (WHEELBASE_MM / 2.0f);
#else
    return 0.0f;
#endif
}

static void motor_hw_init(void)
{
    // direction GPIOs
    gpio_set_direction(MOTOR_L_IN1_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_L_IN2_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_R_IN3_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_R_IN4_GPIO, GPIO_MODE_OUTPUT);

    // PWM timer
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_MODE,
        .timer_num       = LEDC_TIMER,
        .duty_resolution = LEDC_RESOLUTION,
        .freq_hz         = LEDC_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer);

    // left and right PWM channels - same settings except gpio and channel number
    ledc_channel_config_t ch = {
        .speed_mode = LEDC_MODE,
        .timer_sel  = LEDC_TIMER,
        .duty       = 0,
        .hpoint     = 0,
    };
    ch.gpio_num = MOTOR_L_PWM_GPIO; ch.channel = LEDC_CH_L; ledc_channel_config(&ch);
    ch.gpio_num = MOTOR_R_PWM_GPIO; ch.channel = LEDC_CH_R; ledc_channel_config(&ch);

    motor_stop();
    ESP_LOGI(TAG, "motors ready");

#if USE_ENCODERS
    gpio_config_t enc_io = {
        .pin_bit_mask = (1ULL << ENCODER_L_A_GPIO) | (1ULL << ENCODER_R_A_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .intr_type    = GPIO_INTR_POSEDGE,
    };
    gpio_config(&enc_io);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(ENCODER_L_A_GPIO, enc_l_isr, NULL);
    gpio_isr_handler_add(ENCODER_R_A_GPIO, enc_r_isr, NULL);
    ESP_LOGI(TAG, "encoders ready, %d pulses per 90deg", turn_90_pulses());
#endif
}

void task_motor_start(void)
{
    motor_hw_init();
#if USE_ENCODERS
    xTaskCreate(speed_ctrl_task, "mctrl", 3072, NULL, 6, NULL);
#endif
}
