#include <Arduino.h>
#include "config.h"
#include "motion.h"

// Motor count / encoder count
static const uint8_t MOTOR_COUNT = 4;
static const uint8_t ENCODER_COUNT = 4;

// PWM + DIR mapping (confirmed)
static const uint32_t motorPwmPin[MOTOR_COUNT] = {PE9, PE11, PA0, PA2};
static const uint32_t motorDirPin[MOTOR_COUNT] = {PE2, PE3, PA4, PA5};

// Reserved encoder interfaces (not used yet)
static const uint32_t encoderPinA[ENCODER_COUNT] = {PE13, PE4, PA6, PA8};
static const uint32_t encoderPinB[ENCODER_COUNT] = {PE14, PE5, PA7, PA11};

static inline int8_t clampCmd(int16_t v) {
  if (v > CMD_MAX) return CMD_MAX;
  if (v < -CMD_MAX) return -CMD_MAX;
  return (int8_t)v;
}

void Motor_Init(void) {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    pinMode(motorDirPin[i], OUTPUT);
    pinMode(motorPwmPin[i], OUTPUT);
  }

  for (uint8_t i = 0; i < ENCODER_COUNT; ++i) {
    pinMode(encoderPinA[i], INPUT_PULLUP);
    pinMode(encoderPinB[i], INPUT_PULLUP);
  }

  Motors_Set(0, 0, 0, 0);
}

// Mecanum mixing.
void Velocity_Controller(uint16_t angleDeg, uint8_t velocity, int8_t rot, bool drift) {
  float angle = (float)angleDeg + 90.0f;
  float rad = angle * PI / 180.0f;

  float v = (float)velocity / sqrtf(2.0f);

  // When rotating, slow down a bit for stability
  float speedScale = (rot == 0) ? 1.0f : 0.5f;

  float vx = v * cosf(rad);
  float vy = v * sinf(rad);

  float base0 = (vy - vx) * speedScale;
  float base1 = (vy + vx) * speedScale;
  float base2 = (vy - vx) * speedScale;
  float base3 = (vy + vx) * speedScale;

  float r = (float)rot * speedScale;

  float m0, m1, m2, m3;
  if (drift) {
    m0 = base0;
    m1 = base1;
    m2 = base2 - r * 2.0f;
    m3 = base3 + r * 2.0f;
  } else {
    m0 = base0 + r;
    m1 = base1 - r;
    m2 = base2 - r;
    m3 = base3 + r;
  }

  Motors_Set(clampCmd(lroundf(m0)),
             clampCmd(lroundf(m1)),
             clampCmd(lroundf(m2)),
             clampCmd(lroundf(m3)));
}

void Motors_Set(int8_t m0, int8_t m1, int8_t m2, int8_t m3) {
  int8_t cmd[4] = {m0, m1, m2, m3};

  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    bool dir = MOTOR_DIR_DEFAULT[i];
    int16_t c = cmd[i];

    if (c < 0) {
      dir = !dir;
      c = -c;
    }

    uint8_t pwm = 0;
    if (c == 0) {
      pwm = 0;
    } else {
      pwm = (uint8_t)map(constrain(c, 0, CMD_MAX), 0, CMD_MAX, PWM_MIN, PWM_MAX);
    }

    digitalWrite(motorDirPin[i], dir);
    analogWrite(motorPwmPin[i], pwm);
  }
}
