#include "transmission.h"

/*!
 * Set voltage on engines
 * @param Engine number of engine
 * @param Duty duty value of timer [0.0 ... 1.0]
 */

void SetVoltage(uint8_t Engine, float Duty)
{
	if(Duty > 1.0) Duty = 1.0;
	if(Duty < -1.0) Duty = -1.0;
		if(Engine == 0) {
			if(Duty >= 0.0) {
				  HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, GPIO_PIN_RESET);
				  TIM2->CCR1 = ((int32_t) (Duty * TIM2->ARR));
			} else {
				  HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, GPIO_PIN_SET);
				  TIM2->CCR1 = ((int32_t)(TIM2->ARR + (Duty * TIM2->ARR)));
			}
		}
		if(Engine == 1) {
			if(Duty >= 0.0) {
				  HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, GPIO_PIN_RESET);
				  TIM2->CCR2 = ((int32_t) (Duty * TIM2->ARR));
			} else {
				  HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, GPIO_PIN_SET);
				  TIM2->CCR2 = ((int32_t)(TIM2->ARR + (Duty * TIM2->ARR)));
			}
		}
}

/*!
 *
 * @param direction
 * @param Speed
 */
void MoveTo(int direction, float Speed)
{
    switch(direction)
    {
    case 0: Transmit1_float = 0.0; Transmit2_float = 0.0;
    break;

    // right
    case 1: Transmit1_float = Speed; Transmit2_float = Speed;
    break;

    // down
    case 2:  Transmit1_float = Speed; Transmit2_float = -Speed;
    break;

    // up
    case 3: Transmit1_float = -Speed; Transmit2_float = Speed;
    break;

    // left
    case 4: Transmit1_float = -Speed; Transmit2_float = -Speed;
    break;
    }
}
