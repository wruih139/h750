#include <Arduino.h>
#include "config.h"
#include "motion.h"

static const uint16_t STRAFE_ANGLE_LEFT  = 88;
static const uint16_t STRAFE_ANGLE_RIGHT = 270;

void LeftMove(uint8_t v)  { Velocity_Controller(STRAFE_ANGLE_LEFT,  v, 0, false); }
void RightMove(uint8_t v) { Velocity_Controller(STRAFE_ANGLE_RIGHT, v, 0, false); }

void LeftTurn() {
  Velocity_Controller(0, 0, +100, false);
  delay(TURN_90_MS);
  Stop();
}

void RightTurn() {
  Velocity_Controller(0, 0, -100, false);
  delay(TURN_90_MS);
  Stop();
}

void BackTurn() {
  Velocity_Controller(0, 0, -100, false);
  delay(TURN_180_MS);
  Stop();
}

void mqx_Forward(uint8_t v)  { Velocity_Controller(0, v, 0, false); }
void mqx_Backward(uint8_t v) { Velocity_Controller(180, v, 0, false); }

void Stop() {
  Motors_Set(0, 0, 0, 0);
}

void PushBlocks() {
  mqx_Forward(DRIVE_SPEED);
  delay(PUSH_FORWARD_MS);
  mqx_Backward(DRIVE_SPEED);
  delay(PUSH_BACKWARD_MS);
  Stop();
}
