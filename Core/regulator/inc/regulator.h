#pragma once
#include "main.h"
#include "transmission.h"

extern int16_t EncoderData[2];

typedef struct {
	float P_k;
	float I_k;
	float D_k;
	float Current;
	float *Target;
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
	float Dist;
    void  (*performer)(uint8_t engine, float output);
}PID;

extern PID Regulator[3];

void ParseEncoderData(void);

void PID_init(void);

void PID_stop(void);

void PID_start(void);

void PID_calc(uint8_t Reg);
