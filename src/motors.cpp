// =============================================================
// motors.cpp — L298N motor driver via ESP32 LEDC PWM
//
// L298N truth table per motor channel:
//   IN1  IN2  ENA(PWM)  Motor
//   ───  ───  ────────  ──────────────────────────
//   H    L    duty      Forward  (current ∝ duty)
//   L    H    duty      Reverse
//   L    L    any       Coast (free-wheel)
//   H    H    any       Brake  (we do not use this)
//
// Speed mapping: uint8_t 0–255 maps 1:1 to LEDC 8-bit duty cycle.
// At duty=0 the motor sees no average voltage (coast/stop).
// =============================================================

#include "motors.h"
#include <Arduino.h>

// ── Internal helpers ──────────────────────────────────────────

// Clamp an integer to [0, 255] without using <algorithm>
static inline uint8_t clamp255(int v) {
    if (v < 0)   return 0;
    if (v > 255) return 255;
    return (uint8_t)v;
}

static void setLeftWheel(int speed) {
    if (speed > 0) {
        digitalWrite(PIN_MOTOR_A_IN1, HIGH);
        digitalWrite(PIN_MOTOR_A_IN2, LOW);
    } else if (speed < 0) {
        digitalWrite(PIN_MOTOR_A_IN1, LOW);
        digitalWrite(PIN_MOTOR_A_IN2, HIGH);
    } else {
        // speed == 0: coast
        digitalWrite(PIN_MOTOR_A_IN1, LOW);
        digitalWrite(PIN_MOTOR_A_IN2, LOW);
    }
    ledcWrite(PWM_CHANNEL_LEFT, clamp255(speed < 0 ? -speed : speed));
}

static void setRightWheel(int speed) {
    if (speed > 0) {
        digitalWrite(PIN_MOTOR_B_IN3, HIGH);
        digitalWrite(PIN_MOTOR_B_IN4, LOW);
    } else if (speed < 0) {
        digitalWrite(PIN_MOTOR_B_IN3, LOW);
        digitalWrite(PIN_MOTOR_B_IN4, HIGH);
    } else {
        digitalWrite(PIN_MOTOR_B_IN3, LOW);
        digitalWrite(PIN_MOTOR_B_IN4, LOW);
    }
    ledcWrite(PWM_CHANNEL_RIGHT, clamp255(speed < 0 ? -speed : speed));
}

// =============================================================
// Motors_Init
// =============================================================
void Motors_Init() {
    // Direction pins
    pinMode(PIN_MOTOR_A_IN1, OUTPUT);
    pinMode(PIN_MOTOR_A_IN2, OUTPUT);
    pinMode(PIN_MOTOR_B_IN3, OUTPUT);
    pinMode(PIN_MOTOR_B_IN4, OUTPUT);

    // LEDC PWM channels for enable pins
    ledcSetup(PWM_CHANNEL_LEFT,  PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
    ledcSetup(PWM_CHANNEL_RIGHT, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
    ledcAttachPin(PIN_MOTOR_A_EN, PWM_CHANNEL_LEFT);
    ledcAttachPin(PIN_MOTOR_B_EN, PWM_CHANNEL_RIGHT);

    Motors_Stop();
    Serial.println("[MOTORS] Initialised");
}

// =============================================================
// Motion primitives
// =============================================================

void Motors_Forward(uint8_t speed) {
    setLeftWheel( (int)speed);
    setRightWheel((int)speed);
}

void Motors_Backward(uint8_t speed) {
    setLeftWheel( -(int)speed);
    setRightWheel(-(int)speed);
}

// Point turn: left wheel reverses, right wheel advances
void Motors_TurnLeft(uint8_t speed) {
    setLeftWheel( -(int)speed);
    setRightWheel((int)speed);
}

// Point turn: right wheel reverses, left wheel advances
void Motors_TurnRight(uint8_t speed) {
    setLeftWheel( (int)speed);
    setRightWheel(-(int)speed);
}

// Coast stop — sets all direction pins LOW and PWM duty to 0
void Motors_Stop() {
    digitalWrite(PIN_MOTOR_A_IN1, LOW);
    digitalWrite(PIN_MOTOR_A_IN2, LOW);
    digitalWrite(PIN_MOTOR_B_IN3, LOW);
    digitalWrite(PIN_MOTOR_B_IN4, LOW);
    ledcWrite(PWM_CHANNEL_LEFT,  0);
    ledcWrite(PWM_CHANNEL_RIGHT, 0);
}

// Differential drive with signed per-wheel speeds (-255..255)
void Motors_SetWheels(int leftSpeed, int rightSpeed) {
    setLeftWheel(leftSpeed);
    setRightWheel(rightSpeed);
}

// Lab 6: normalized velocity interface — adapted from Go1 walk.py
//   cmd.velocity = [vx, vy]  and  cmd.yawSpeed
//
// Differential drive kinematics (same as ROS diff_drive_controller):
//   left  = (vx - wz) × scale
//   right = (vx + wz) × scale
//
// Example: vx=1.0, wz=0  → both wheels forward at MOTOR_SPEED_DEFAULT
//          vx=0,   wz=1.0 → left backward, right forward (turn left)
void Motors_SetVelocity(float vx, float wz) {
    int left  = (int)((vx - wz) * MOTOR_SPEED_DEFAULT);
    int right = (int)((vx + wz) * MOTOR_SPEED_DEFAULT);
    Motors_SetWheels(left, right);   // SetWheels → setLeftWheel/setRightWheel clamp internally
}
