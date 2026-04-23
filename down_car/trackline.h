#pragma once
#include <Arduino.h>

void Trackline_Init(void);
void Sensor_Receive(void);
void Tracking_Line_Task(void);
void Forward(uint16_t n);
