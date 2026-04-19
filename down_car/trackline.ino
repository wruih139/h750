#include <Wire.h>
#include <Arduino.h>
#include "config.h"
#include "motion.h"
#include "trackline.h"

static const uint8_t SENSOR_COUNT = 8;

static uint8_t sensorByte = 0xFF;
static uint8_t rec_data[SENSOR_COUNT]; // 0=black hit, 1=white

static int8_t lastLineError = 0;  // -7..+7
static bool lostLine = true;

// PID state
static float iTerm = 0.0f;
static float lastErr = 0.0f;
static uint32_t lastPidMs = 0;

// Intersection debounce
static uint32_t lastIntersectionMs = 0;

static bool WireWriteByte(uint8_t val) {
  Wire.beginTransmission(LINE_FOLLOWER_I2C_ADDR);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool WireReadDataByte(uint8_t reg, uint8_t &val) {
  if (!WireWriteByte(reg)) return false;

  uint8_t n = Wire.requestFrom(LINE_FOLLOWER_I2C_ADDR, (uint8_t)1);
  if (n != 1) return false;
  if (!Wire.available()) return false;

  val = Wire.read();
  return true;
}

static void SetSensorStateLost() {
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) rec_data[i] = 1;
  lostLine = true;
}

void Trackline_Init(void) {
  Wire.setSCL(LINE_I2C_SCL_PIN);
  Wire.setSDA(LINE_I2C_SDA_PIN);
  Wire.begin();

  lastPidMs = millis();
  lastIntersectionMs = 0;
  SetSensorStateLost();
}

void Sensor_Receive(void) {
  if (!WireReadDataByte(LINE_REG_DATA, sensorByte)) {
    SetSensorStateLost();
    return;
  }

  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    rec_data[i] = (sensorByte >> i) & 0x01;
  }
  lostLine = false;
}

static uint8_t GetLineHitCount() {
  uint8_t hit = 0;
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    if (rec_data[i] == 0) ++hit;
  }
  return hit;
}

static int8_t ComputeLineError() {
  int16_t weightedSum = 0;
  uint8_t hit = 0;

  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    if (rec_data[i] == 0) {
      weightedSum += SENSOR_WEIGHTS[i];
      ++hit;
    }
  }

  if (hit == 0) return lastLineError;
  return (int8_t)(weightedSum / (int16_t)hit);
}

static bool IsIntersectionDetected() {
  uint8_t hit = GetLineHitCount();
  bool edges = (rec_data[0] == 0 && rec_data[7] == 0);
  bool wideCenter = (rec_data[2] == 0 && rec_data[3] == 0 && rec_data[4] == 0 && rec_data[5] == 0);
  return edges && wideCenter && (hit >= INTERSECTION_MIN_HIT);
}

static void RecoverLine() {
  if (lastLineError < 0) {
    LeftMove(SEARCH_SPEED);
  } else if (lastLineError > 0) {
    RightMove(SEARCH_SPEED);
  } else {
    mqx_Forward(SEARCH_SPEED);
  }
  delay(LOST_LINE_STRAFE_MS);
  Stop();
}

static int8_t PID_Update(int8_t errNow) {
  uint32_t now = millis();
  uint32_t dtMs = now - lastPidMs;
  if (dtMs < PID_DT_MIN_MS) dtMs = PID_DT_MIN_MS;
  if (dtMs > PID_DT_MAX_MS) dtMs = PID_DT_MAX_MS;
  lastPidMs = now;

  float dt = (float)dtMs / 1000.0f;
  float e = (float)errNow;

  // integral
  iTerm += e * dt * KI;
  if (iTerm > ITERM_LIMIT) iTerm = ITERM_LIMIT;
  if (iTerm < -ITERM_LIMIT) iTerm = -ITERM_LIMIT;

  // derivative
  float d = (e - lastErr) / dt;
  lastErr = e;

  float out = KP * e + iTerm + KD * d;
  if (out > (float)MAX_TURN_CMD) out = (float)MAX_TURN_CMD;
  if (out < (float)-MAX_TURN_CMD) out = (float)-MAX_TURN_CMD;

  return (int8_t)lroundf(out);
}

void Tracking_Line_Task(void) {
  Sensor_Receive();

  uint8_t hit = GetLineHitCount();
  if (lostLine || hit == 0) {
    RecoverLine();
    return;
  }

  int8_t err = ComputeLineError();
  lastLineError = err;

  int8_t turnCmd = PID_Update(err);

  // keep old convention: rot uses negative of turn
  Velocity_Controller(0, TRACKING_SPEED, (int8_t)(-turnCmd), false);
}

static void BrakeAtIntersection() {
  delay(INTERSECTION_BRAKE_MS);
  Stop();
  lastIntersectionMs = millis();
}

void Forward(uint8_t n) {
  if (n == 0) {
    Stop();
    return;
  }

  uint8_t passed = 0;

  while (passed < n) {
    Tracking_Line_Task();

    if (IsIntersectionDetected()) {
      uint32_t now = millis();
      if (now - lastIntersectionMs > INTERSECTION_DEBOUNCE_MS) {
        BrakeAtIntersection();
        ++passed;
      }
    }
  }
}
