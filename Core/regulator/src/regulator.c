#include "regulator.h"

uint16_t EncoderData[2];
Path PathPlan;
/*!
 * Get encoder data and clear registers
 */
void ParseEncoderData(void)
{
	EncoderData[0] = TIM3->CNT;
	EncoderData[1] = TIM4->CNT;
	TIM3->CNT = 0;
	TIM4->CNT = 0;

	Calculate_position(&(*EncoderData));
}

void Calculate_position(uint16_t *Enc)
{

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

	for(int i = 0; i < 1; i++)
	{
		Regulator[i].Error = 0.0;
		Regulator[i].Sum_error = 0.0;
		Regulator[i].Prev_error = 0.0;
		Regulator[i].Current = 0.0;
		Regulator[i].Target = 0.0;
		Regulator[i].Output = 0.0;
		Regulator[i].Min_output = 0.01;
		Regulator[i].Max_output = 1.0;
		Regulator[i].coordinator = &ParseEncoderData;
		Regulator[i].performer = &SetVoltage;
	}
}

void PID_stop(void)
{
	for(int i = 0; i < 1; i++)
	{
		*Regulator[i].Goal = 0.0;
		Regulator[i].Target = 0.0;
		Regulator[i].PID_on = 0;
		Regulator[i].Error = 0.0;
		Regulator[i].Sum_error = 0.0;
	}
}

void PID_start(void)
{
	for(int i = 0; i < 1; i++)
	{
		Regulator[i].PID_on = 1;
	}
}

void PID_calc(uint8_t Reg) {
    Regulator[Reg].Target = *Regulator[Reg].Goal;
    (void)Regulator[Reg].coordinator();
    Regulator[Reg].Current = EncoderData[Reg];
    Regulator[Reg].Error = Regulator[Reg].Target - Regulator[Reg].Current;
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
    }
    else
    {
    	Regulator[Reg].Output = 0;
    	Regulator[Reg].PID_finish = 0;
    }
    Regulator[Reg].Prev_error = Regulator[Reg].Error;
    Regulator[Reg].performer(Reg, Regulator[Reg].Output);
}
