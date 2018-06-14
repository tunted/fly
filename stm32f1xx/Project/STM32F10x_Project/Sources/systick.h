/* Includes ------------------------------------------------------------------*/
#ifndef __SYSTICK_H
#define __SYSTICK_H

#include "stm32f10x.h"
#include <stdbool.h>

/* Private define task controller-------------------------------------------*/
typedef struct{
  uint32_t cycle;
  uint32_t timer; 
  bool is_enabled;
} systick_task_controller_t;

/* Private define ------------------------------------------------------------*/

/* Function prototypes--------------------------------------------------------*/
bool systick_add_task_controller(systick_task_controller_t *task_controller, uint32_t task_cycle);
bool systick_check_task_enabled(systick_task_controller_t *task_controller);
void systick_handler(void);


/* Extern Function prototypes-------------------------------------------------*/

/* Extern for external use ---------------------------------------------------*/

/* Auxiliary Function---------------------------------------------------------*/
#endif
