#pragma once

#include "regulator.h"
#include "pathCoordination.h"

typedef struct {
	int8_t Current_flag;
	float Current_Dist;
	float Current_Dir;
	bool Finish;
	float Speed;
	float Center[2];
	float Transmit_float[2];
}Transmit;

extern Transmit Transmission;

void SetVoltage(uint8_t Engine, float Duty);

void ParseEncoderData(void);

