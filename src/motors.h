#pragma once
// =============================================================
// motors.h — L298N motor driver, LEDC PWM, and motion primitives
//
// !! ONLY TaskNavigation (main.cpp) should call these functions !!
//
// Multiple tasks issuing concurrent motor commands would produce
// non-deterministic hardware state.  See design notes in main.cpp.
// =============================================================

#include "config.h"

// Initialise LEDC PWM channels and set all motor pins as outputs.
// Call once from setup() before tasks start.
void Motors_Init();

// ── Motion primitives ─────────────────────────────────────────
// speed: 0–255  (maps directly to 8-bit LEDC duty cycle)

void Motors_Forward (uint8_t speed);
void Motors_Backward(uint8_t speed);

// Point turns (one wheel forward, one backward)
void Motors_TurnLeft (uint8_t speed);
void Motors_TurnRight(uint8_t speed);

// Coast stop — both IN pins LOW, PWM duty = 0
void Motors_Stop();

// Direct differential drive — useful for curved paths.
// speed: -128 to +127  (negative = reverse that wheel)
void Motors_SetWheels(int leftSpeed, int rightSpeed);

// Lab 6: normalized velocity command — mirrors Go1 walk.py fields:
//   cmd.velocity[0] (vx) + cmd.yawSpeed (wz)
//
//   vx:  forward speed  -1.0 = full reverse,  +1.0 = full forward
//   wz:  turn rate      -1.0 = turn right,     +1.0 = turn left
//
// Converts to differential drive wheel speeds:
//   left  = (vx - wz) × MOTOR_SPEED_DEFAULT
//   right = (vx + wz) × MOTOR_SPEED_DEFAULT
void Motors_SetVelocity(float vx, float wz);
