#pragma once

#include "main.h"

typedef struct {
	float P_k;
	float I_k;
	float D_k;
	float Current;
	float Target;
	float Error;
	float Sum_error;
	float Prev_error;
	char  PID_on;
	char  PID_finish;
	float Max_sum_error;
	float Error_end;
	float Output;
	float PID_output_end;
	float PID_error_end;
	float Min_output;
	float Max_output;
    void (*coordinator)(void);
    void  (*performer)(uint8_t engine, float output);
    float *Goal;
}PID;
PID Regulator[2];

struct {
	int8_t Current_flag;
	bool Finish;
	float Speed[2];
	float Distance[2];
}Transmission;

void PID_init(void);
void PID_calc(uint8_t Reg);
float goal;
void SetVoltage(uint8_t Engine, float Duty);
void ParseEncoderData(void);

