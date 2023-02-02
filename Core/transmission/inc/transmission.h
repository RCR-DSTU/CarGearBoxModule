#pragma once

#include "regulator.h"
#include "pathCoordination.h"

typedef struct {
	float Current_Dist[2];
	uint8_t Current_Dir;
	char *Finish;
	float X_pos;
	float Y_pos;
	float Transmit_float[2];
}Transmit;

extern Transmit Transmission;

void SetVoltage(uint8_t Engine, float Duty);

void ParseEncoderData(void);

void MoveTo(uint8_t direction, float Speed);
