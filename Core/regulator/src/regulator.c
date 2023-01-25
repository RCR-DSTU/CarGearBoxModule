#include "regulator.h"

int16_t EncoderData[2];
float Dist[2];

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

	Regulator[0].Current = (((float)(EncoderData[0])) * StepTransmission) / Freq;
	Regulator[1].Current = (((float)(EncoderData[1])) * StepTransmission) / Freq;

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

	Regulator[0].Target = &Transmit1_float;
	Regulator[1].Target = &Transmit2_float;

	for(int i = 0; i <= 1; i++)
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

void PID_stop(void)
{
	for(int i = 0; i <= 1; i++)
	{
		*Regulator[i].Target = 0.0;
		Regulator[i].PID_on = 0;
		Regulator[i].Error = 0.0;
		Regulator[i].Sum_error = 0.0;
	}
}

void PID_start(void)
{
	Regulator[0].PID_on = 1;
	Regulator[1].PID_on = 1;
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
        Regulator[Reg].performer(Reg, Regulator[Reg].Output);
    }
    else
    {
    	Regulator[Reg].Output = 0;
    	Regulator[Reg].PID_finish = 0;
    }
}
