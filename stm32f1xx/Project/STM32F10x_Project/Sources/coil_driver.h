/* Includes ------------------------------------------------------------------*/

#ifndef __COIL_DRIVER_H
#define __COIL_DRIVER_H

#include "stm32f10x.h"

typedef struct {
  TIM_TypeDef *TIMx;
  uint8_t channel_pos;
  uint8_t channel_neg;
} coil_driver_t;

typedef enum{
  COIL_BACKWARD  = 0,
  COIL_FORWARD   = !COIL_BACKWARD 
} coil_driver_direction_t; 
/* Private define ------------------------------------------------------------*/

/* Function prototypes--------------------------------------------------------*/
void coil_driver_init(coil_driver_t *coil_driver, TIM_TypeDef *TIMx, uint8_t channel_pos, uint8_t channel_neg);
void coil_driver_set_control_signal(coil_driver_t *coil_driver, float pwm_duty_cycle, coil_driver_direction_t direction);

/* Extern Function prototypes-------------------------------------------------*/

/* Extern for external use ---------------------------------------------------*/

/* Auxiliary Function---------------------------------------------------------*/


#endif

