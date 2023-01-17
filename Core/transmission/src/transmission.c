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
		if(Engine == 1) {
			if(Duty >= 0.0) {
				  HAL_GPIO_WritePin(DIR1_PIN_GPIO_Port, DIR1_PIN_Pin, GPIO_PIN_RESET);
				  TIM2->CCR1 = ((int32_t) (Duty * TIM2->ARR));
			} else {
				  HAL_GPIO_WritePin(DIR1_PIN_GPIO_Port, DIR1_PIN_Pin, GPIO_PIN_SET);
				  TIM2->CCR1 = ((int32_t)(TIM2->ARR + (Duty * TIM2->ARR)));
			}
		}
		if(Engine == 2) {
			if(Duty >= 0.0) {
				  HAL_GPIO_WritePin(DIR2_PIN_GPIO_Port, DIR2_PIN_Pin, GPIO_PIN_RESET);
				  TIM2->CCR2 = ((int32_t) (Duty * TIM2->ARR));
			} else {
				  HAL_GPIO_WritePin(DIR2_PIN_GPIO_Port, DIR2_PIN_Pin, GPIO_PIN_SET);
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
    case 0: Regulator[0].Target = 0.0; Regulator[1].Target = 0.0;
    break;

    // right
    case 1: Regulator[0].Target = Speed; Regulator[1].Target = Speed;
    break;

    // down
    case 2: Regulator[0].Target = Speed; Regulator[1].Target = -Speed;
    break;

    // up
    case 3: Regulator[0].Target = -Speed; Regulator[1].Target = Speed;
    break;

    // left
    case 4: Regulator[0].Target = -Speed; Regulator[1].Target = -Speed;
    break;
    }
}

/*!
 *
 * @param flag
 */
void CreateNewJob(uint8_t flag)
{




}

