#pragma once

#include "regulator.h"
#include "pathCoordination.h"

extern struct {
	int8_t Current_flag;
	bool Finish;
	float Speed[2];
	float Distance[2];
}Transmission;

extern float Transmit1_float;
extern float Transmit2_float;

void MoveTo(int direction, float Speed);

void SetVoltage(uint8_t Engine, float Duty);

void ParseEncoderData(void);

