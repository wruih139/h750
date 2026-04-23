#pragma once
#include <Arduino.h>

void Motor_Init(void);
void Velocity_Controller(uint16_t angle, uint8_t velocity, int8_t rot, bool drift);
void Motors_Set(int8_t Motor_0, int8_t Motor_1, int8_t Motor_2, int8_t Motor_3);

void LeftMove(uint8_t v);
void RightMove(uint8_t v);
void LeftTurn();
void RightTurn();
void BackTurn();
void mqx_Forward(uint8_t v);
void mqx_Backward(uint8_t v);
void Stop();
void PushBlocks();
