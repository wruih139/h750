#include <Arduino.h>
#include "config.h"
#include "motion.h"

// Motor count / encoder count
static const uint8_t MOTOR_COUNT = 4;
static const uint8_t ENCODER_COUNT = 4;

// ==========================================
// 【物理布局优化版 - 彻底避坑】
// 0、1号电机（前轮）：左侧 5V 区 (D14, A15, D9, D10)
// 2号电机（左后）：使用纯净的 B0(PWM), B1(DIR)
// 3号电机（右后）：使用测试通过的 A2(PWM), A5(DIR)
// ==========================================
static const uint32_t motorPwmPin[MOTOR_COUNT] = {PD14, PA15, PB0, PA2}; 
static const uint32_t motorDirPin[MOTOR_COUNT] = {PD9,  PD10, PB1, PA5};

// 编码器引脚暂时放一边（保留定义，但不影响当前测试）
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

  // 测试阶段暂时屏蔽编码器引脚的初始化，确保不受干扰
  /*
  for (uint8_t i = 0; i < ENCODER_COUNT; ++i) {
    pinMode(encoderPinA[i], INPUT_PULLUP);
    pinMode(encoderPinB[i], INPUT_PULLUP);
  }
  */

  Motors_Set(0, 0, 0, 0);
}

// 麦克纳姆轮底盘运动学解算
void Velocity_Controller(uint16_t angleDeg, uint8_t velocity, int8_t rot, bool drift) {
  float angle = (float)angleDeg + 90.0f;
  float rad = angle * PI / 180.0f;

  float v = (float)velocity / sqrtf(2.0f);

  // 旋转时稍微减速以保持稳定
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

// 底层电机硬件驱动
void Motors_Set(int8_t m0, int8_t m1, int8_t m2, int8_t m3) {
  int8_t cmd[4] = {m0, m1, m2, m3};
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    bool dir = MOTOR_DIR_DEFAULT[i];
    int16_t c = cmd[i];

    if (c < 0) {
      dir = !dir;
      c = -c;
    }

    // 先输出方向
    digitalWrite(motorDirPin[i], dir);

    // =====================================
    // 【核心修复】绕开 analogWrite(0) 悬空 Bug
    // =====================================
    if (c == 0) {
      digitalWrite(motorPwmPin[i], LOW);  // 强制下拉电平，死死踩住刹车
    } else {
      uint8_t pwm = (uint8_t)map(constrain(c, 0, CMD_MAX), 0, CMD_MAX, PWM_MIN, PWM_MAX);
      analogWrite(motorPwmPin[i], pwm);   // 大于 0 时正常输出 PWM
    }
  }
}
