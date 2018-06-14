#include "coil_driver.h"

/* Private const --------------------------------------------------------------*/

/* Extern variables------------------------------------------------------------*/

/* Private variables-----------------------------------------------------------*/

/* Private variables-----------------------------------------------------------*/

/* Private functions-----------------------------------------------------------*/
static void _set_channel_pwm(coil_driver_t *coil_driver, float pwm_duty_cycle, uint8_t channel);

/******************************************************
*****************PRIMARY FUNCTION**********************
******************************************************/
void coil_driver_init(coil_driver_t *coil_driver, TIM_TypeDef *TIMx, uint8_t channel_pos, uint8_t channel_neg)
{
	coil_driver->TIMx = TIMx;
	coil_driver->channel_pos = channel_pos;
	coil_driver->channel_neg = channel_neg;

	return;
}


void coil_driver_set_control_signal(coil_driver_t *coil_driver, float pwm_duty_cycle, coil_driver_direction_t direction)
{
	switch (direction)
	{
		case COIL_FORWARD:
			_set_channel_pwm(coil_driver, pwm_duty_cycle, coil_driver->channel_pos);
			_set_channel_pwm(coil_driver, 0, 							coil_driver->channel_neg);
			break;

		case COIL_BACKWARD:
			_set_channel_pwm(coil_driver, 0, 							coil_driver->channel_pos);
			_set_channel_pwm(coil_driver, pwm_duty_cycle, coil_driver->channel_neg);
			break;
	}

}



/******************************************************
***************AUXILIARY FUNCTION**********************
******************************************************/
static void _set_channel_pwm(coil_driver_t *coil_driver, float pwm_duty_cycle, uint8_t channel)
{
	switch (channel)
	{
		case 1:
			coil_driver->TIMx->CCR1 = (uint32_t)((float)coil_driver->TIMx->ARR * pwm_duty_cycle);
			break;

		case 2:
			coil_driver->TIMx->CCR2 = (uint32_t)((float)coil_driver->TIMx->ARR * pwm_duty_cycle);
			break;

		case 3:
			coil_driver->TIMx->CCR3 = (uint32_t)((float)coil_driver->TIMx->ARR * pwm_duty_cycle);
			break;

		case 4:
			coil_driver->TIMx->CCR4 = (uint32_t)((float)coil_driver->TIMx->ARR * pwm_duty_cycle);
			break;

		default:
			break;
	}
}


