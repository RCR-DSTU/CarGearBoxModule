#include "regulator.h"

int16_t EncoderData[2];
Transmit Transmission;

float Transmit1_float;
float Transmit2_float;
#define StepTransmission  0.00007
#define Freq		0.02

/*!
 * Get encoder data and clear registers
 */
void ParseEncoderData(void)
{
	EncoderData[0] = TIM3->CNT;
	EncoderData[1] = TIM4->CNT;

	float Dist[2];
	Dist[0] = (((float)(EncoderData[0])) * StepTransmission);
	Dist[1] = (((float)(EncoderData[1])) * StepTransmission);
	Regulator[0].Current = Dist[0] / Freq;
	Regulator[1].Current = Dist[1] / Freq;

	Regulator[0].Dist += (Dist[0] * 0.000635);
	Regulator[1].Dist += (Dist[1] * 0.000635);

	TIM3->CNT = 0;
	TIM4->CNT = 0;
}

/*!
 * Initialization PID parameters
 */
void PID_init(void)
{
	Regulator[0].P_k = 0.130;
	Regulator[0].I_k = 0.270;
	Regulator[0].D_k = 0.15;
	Regulator[1].P_k = 0.120;
	Regulator[1].I_k = 0.290;
	Regulator[1].D_k = 0.15;

	Regulator[0].Target = &Transmission.Transmit_float[0];
	Regulator[1].Target = &Transmission.Transmit_float[1];
	for(int i = 0; i < 2; i++)
	{
		Regulator[i].Error = 0.0;
		Regulator[i].Sum_error = 0.0;
		Regulator[i].Prev_error = 0.0;
		Regulator[i].Current = 0.0;
		Regulator[i].Output = 0.0;
		Regulator[i].Min_output = 0.01;
		Regulator[i].Max_output = 1.0;
		Regulator[i].Max_sum_error = 6.0;
		Regulator[i].performer = &SetVoltage;
	}
}

void Track_init(void)
{
	Regulator[2].P_k = 10.0;
	Regulator[2].I_k = 14.0;
	Regulator[2].D_k = 1.0;
	Regulator[2].PID_finish = true;
	Regulator[2].PID_error_end = 0.00008;

	Regulator[2].Error = 0.0;
	Regulator[2].Sum_error = 0.0;
	Regulator[2].Prev_error = 0.0;
	Regulator[2].Current = 0.0;
	Regulator[2].Output = 0.0;
	Regulator[2].Min_output = 2.5;
	Regulator[2].Max_output = 9.0;
	Regulator[2].Max_sum_error = 13.0;

	Transmission.X_pos = 0.0;
	Transmission.Y_pos = 0.0;
	Transmission.Finish = &Regulator[2].PID_finish;
	Transmission.Current_Dir = 0;

	Regulator[2].performer = &MoveTo;
}

void PID_stop(void)
{
	for(int i = 0; i <= 1; i++)
	{
		*Regulator[i].Target = 0.0;
		Regulator[i].PID_on = 0;
		Regulator[i].Error = 0.0;
		Regulator[i].Sum_error = 0.0;
		Regulator[i].performer(i, 0.0);
	}
}

void PID_start(void)
{
	Regulator[0].Dist = 0.0;
	Regulator[1].Dist = 0.0;
	Regulator[0].PID_on = 1;
	Regulator[1].PID_on = 1;
}

void Start_track(uint8_t direction, float Dist)
{
	Transmission.Current_Dir = direction;
	if(direction == 1 || direction == 4)
	{
		Transmission.Current_Dist[0] = Dist / 10000;
		Regulator[2].PID_output_end = Dist / 10000;
		Regulator[2].Target = &Transmission.Current_Dist[0];
	}
	if(direction == 2 || direction == 3)
	{
		Transmission.Current_Dist[1] = Dist / 10000;
		Regulator[2].PID_output_end = Dist / 10000;
		Regulator[2].Target = &Transmission.Current_Dist[1];
	}

	Regulator[2].PID_finish = 0;
	Regulator[2].Current = 0.0;
	Regulator[2].PID_on = 1;
}

void Stop_track(void)
{
	Regulator[2].Current = 0.0;
	*Regulator[2].Target = 0.0;
	Regulator[2].Sum_error = 0.0;
	Regulator[2].Prev_error = 0.0;
	Regulator[2].PID_on = 0;

	PID_stop();
}

void PID_calc(uint8_t Reg) {
    Regulator[Reg].Error = *Regulator[Reg].Target - Regulator[Reg].Current;
    Regulator[Reg].Sum_error += Regulator[Reg].Error;

    if (Regulator[Reg].Sum_error > Regulator[Reg].Max_sum_error)
    	Regulator[Reg].Sum_error = Regulator[Reg].Max_sum_error;
    if (Regulator[Reg].Sum_error < -Regulator[Reg].Max_sum_error)
    	Regulator[Reg].Sum_error = -Regulator[Reg].Max_sum_error;

    if (Regulator[Reg].PID_on)
    {
        Regulator[Reg].Output = ((float)(Regulator[Reg].P_k * Regulator[Reg].Error)+(Regulator[Reg].I_k * Regulator[Reg].Sum_error)+
                               (Regulator[Reg].D_k * (Regulator[Reg].Prev_error - Regulator[Reg].Error)));

        if (Regulator[Reg].Output > Regulator[Reg].Max_output)
        	Regulator[Reg].Output = Regulator[Reg].Max_output;
        else if (Regulator[Reg].Output < -Regulator[Reg].Max_output)
        	Regulator[Reg].Output = -Regulator[Reg].Max_output;

        if (Regulator[Reg].Output < Regulator[Reg].Min_output && Regulator[Reg].Output > -Regulator[Reg].Min_output)
        	Regulator[Reg].Output = 0;

        if ((Regulator[Reg].Current <= Regulator[Reg].PID_output_end) &&
            (Regulator[Reg].Current >= -Regulator[Reg].PID_output_end) &&
            (Regulator[Reg].Error <= Regulator[Reg].PID_error_end) && (Regulator[Reg].Error >= -Regulator[Reg].PID_error_end))
        	Regulator[Reg].PID_finish = 1;
        else
        	Regulator[Reg].PID_finish = 0;
        Regulator[Reg].Prev_error = Regulator[Reg].Error;
        if(Reg != 2)  Regulator[Reg].performer(Reg, Regulator[Reg].Output);
        else
        {
        	Regulator[Reg].performer(Transmission.Current_Dir, Regulator[Reg].Output);
        }
    }
    else
    {
    	Regulator[Reg].Output = 0;
    	Regulator[Reg].PID_finish = 0;
    }
}
