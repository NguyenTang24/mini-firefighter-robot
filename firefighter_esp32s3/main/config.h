// config.h - pin assignments and tuning

#pragma once

// Motor pins (L298N H-bridge)
// ENA/ENB = PWM speed, IN1/IN2 = left direction, IN3/IN4 = right direction
#define MOTOR_L_PWM_GPIO    16
#define MOTOR_L_IN1_GPIO    11
#define MOTOR_L_IN2_GPIO    12
#define MOTOR_R_PWM_GPIO    13
#define MOTOR_R_IN3_GPIO    14
#define MOTOR_R_IN4_GPIO    15

// Encoders - set USE_ENCODERS 0 if not installed
#define ENCODER_L_A_GPIO    20
#define ENCODER_R_A_GPIO    38
#define USE_ENCODERS         0    // 0 = timed turns, 1 = encoder turns
#define ENCODER_PPR         20    // slots on the IR encoder disk
#define WHEEL_DIAMETER_MM   67
#define WHEELBASE_MM        80    // tuned: physical 146mm but 88 gives accurate 90deg turn

// TF-Luna lidar (UART1)
#define LIDAR_UART_NUM      UART_NUM_1
#define LIDAR_TX_GPIO        1
#define LIDAR_RX_GPIO        2
#define LIDAR_BAUD        115200

// ESP32-CAM (UART2, receive only)
#define CAM_UART_NUM        UART_NUM_2
#define CAM_TX_GPIO          3
#define CAM_RX_GPIO          4
#define CAM_BAUD          115200

// IR flame sensor (5 ADC channels on ADC1)
#define FLAME_ADC_CH0        5    // leftmost
#define FLAME_ADC_CH1        6
#define FLAME_ADC_CH2        7    // center
#define FLAME_ADC_CH3        8
#define FLAME_ADC_CH4        9    // rightmost

// SG90 servo for lidar pan
#define SERVO_GPIO          18
#define SERVO_CENTER_US   1500
#define SERVO_LEFT_US     2000
#define SERVO_RIGHT_US    1000

// Water pump relay
#define PUMP_GPIO           19    // HIGH = pump on

// Motor speed and turn tuning
#define FWD_SPEED_PCT       80    // straight drive speed 0-100
#define KICK_PCT            75    // startup burst PWM to break stiction
#define KICK_MS             20    // short burst to break stiction
#define MOTOR_L_TRIM         4   // left motor trim to fix drift (-10 to +10)
#define MOTOR_R_TRIM         0   // right motor trim
#define TURN_SPEED_PCT      73    // turn speed
#define TURN_90_MS         480    // 90 degree turn time when USE_ENCODERS=0
#define AVOID_SPEED_PCT     50    // obstacle avoidance speed

// Encoder speed controller
#define SPEED_CTRL_MS       100   // how often the controller runs (ms)
#define SPEED_CTRL_KP         1.1   // proportional gain
#define SPEED_CTRL_MAX_CORR  15   // max PWM correction per cycle
#define SPEED_CTRL_DEADBAND   2   // ignore encoder errors smaller than this

// Obstacle avoidance
#define OBSTACLE_DIST_CM    30    // stop and avoid if closer than this

// Fire approach and spray
#define STANDOFF_DIST_CM    20    // stop to spray distance
#define SPRAY_DURATION_MS  5000    // pump on time in ms
#define FLAME_THRESHOLD     150   // ADC reading that means flame detected
#define FLAME_CONFIRM_MS    250   // flame must read high for this long
#define FLAME_CLOSE_ADC    3350   // stop and spray when second sensor exceeds this
#define FLAME_SENSOR_MARGIN 200   // dominant sensor must beat second by this to steer (compensates L1 bias)
#define FLAME_BIAS          400   // subtract from R1 reading to zero out its baseline offset

// Landmark detection
#define MIN_LM_PIXEL_SIZE  2500   // partial card still counts, bearing check prevents early turns
#define LM_CONFIRM_FRAMES     2   // consecutive frames needed to confirm landmark

// Navigation
#define LM_COOLDOWN_MS       300  // ignore landmarks this long after a turn
#define NAV_BEAR_TOL        0.10f // bearing error small enough to go straight
#define TURN_BEAR_TOL       0.35f // max bearing allowed when confirming a turn
#define STEER_ADJUST_PCT      80  // how hard to correct bearing
#define SCAN_STEP_MS         200  // step time when scanning for a lost landmark
#define SCAN_TURN_PCT         30  // turn speed during scan
#define SCAN_TIMEOUT_MS      8000 // give up scanning after this long
#define NAV_ALIGN_TIMEOUT_MS 3000 // max time to align heading after avoidance

// Fire approach state machine
#define FIRE_APPROACH_SPEED       65
#define FSM_BEAR_TOL            0.12f
#define FSM_ALIGN_SPEED           60  // in-place turn speed during initial alignment
#define FSM_TURN_OFFSET            8  // wheel offset correction while driving forward
#define FIRE_APPROACH_TIMEOUT_MS   5000
#define SPRAY_SETTLE_MS           500

// Post-turn stiction correction drive
#define POST_TURN_L_PCT   82   // left wheel power after turn (increase if still drifting left)
#define POST_TURN_R_PCT   79   // right wheel power after turn
#define POST_TURN_MS     410   // duration ms

// Flame sensor calibration: set 1 to print raw ADC values every 500ms
#define FLAME_CALIB_MODE 0

// Fire approach test: set 1 to skip navigation and go straight to flame sensor approach
#define FIRE_TEST_MODE   0

// Web UI turn time limits
#define TURN_MS_MIN   100
#define TURN_MS_MAX  5000

// Landmark IDs, match the LM:N values the camera sends
#define LM_ORANGE   1
#define LM_GREEN    2
#define LM_RED      3   // center room, triggers fire approach
#define LM_NONE    -1

// Path: marker -> green -> marker (last one triggers fire approach)
#define LANDMARK_SEQUENCE_LEN  3
static const int LANDMARK_SEQUENCE[LANDMARK_SEQUENCE_LEN] = {
    LM_ORANGE, LM_GREEN, LM_ORANGE
};

// WiFi AP
#define WIFI_SSID    "FireFighter"
#define WIFI_PASS    "robot1234"
#define WEB_PORT     80
