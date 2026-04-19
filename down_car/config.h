#pragma once
#include <Arduino.h>

// =========================
// Board / IO
// =========================
static const uint32_t KEY_PIN = PA1;

// I2C line follower
static const uint8_t LINE_FOLLOWER_I2C_ADDR = 0x78;
static const uint32_t LINE_I2C_SCL_PIN = PB6;
static const uint32_t LINE_I2C_SDA_PIN = PB7;
static const uint8_t LINE_REG_DATA = 1;

// =========================
// Motor / PWM
// =========================
static const uint8_t PWM_MIN = 50;         // minimum pwm when command != 0
static const uint8_t PWM_MAX = 255;
static const int8_t  CMD_MAX = 100;        // command magnitude limit (-100..100)

// Per-wheel default direction inversion (true=HIGH means forward for that motor)
static const bool MOTOR_DIR_DEFAULT[4] = {true, false, false, true};

// =========================
// Motion parameters
// =========================
static const uint8_t DRIVE_SPEED = 80;

static const uint16_t TURN_90_MS  = 1100;
static const uint16_t TURN_180_MS = 2333;

static const uint16_t PUSH_FORWARD_MS  = 1200;
static const uint16_t PUSH_BACKWARD_MS = 900;

// =========================
// Line following / PID
// =========================
static const uint8_t TRACKING_SPEED = 100;   // 0..100
static const uint8_t SEARCH_SPEED   = 80;    // 0..100 (lost line recovery)

static const int8_t MAX_TURN_CMD = 75;       // limit PID output (rot command magnitude)

// PID gains: start here and tune on real car
static const float KP = 10.0f;   // proportional gain
static const float KI = 0.0f;    // integral gain (often 0 or small)
static const float KD = 25.0f;   // derivative gain

// Integral anti-windup
static const float ITERM_LIMIT = 80.0f;

// Control loop timing safety
static const uint16_t PID_DT_MIN_MS = 5;
static const uint16_t PID_DT_MAX_MS = 50;

// Lost-line recovery
static const uint16_t LOST_LINE_STRAFE_MS = 80;

// Intersection detection
static const uint8_t INTERSECTION_MIN_HIT = 6;      // how many sensors see black(0)
static const uint16_t INTERSECTION_BRAKE_MS = 233;
static const uint16_t INTERSECTION_DEBOUNCE_MS = 800;

// Start / debounce
static const uint16_t POWER_ON_SETTLE_MS = 1000;
static const uint16_t START_BUTTON_DEBOUNCE_MS = 20;

// Sensor weights (8 sensors, left->right)
static const int8_t SENSOR_WEIGHTS[8] = {-7, -5, -3, -1, 1, 3, 5, 7};
