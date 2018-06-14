#include "systick.h"

/* Private constant ---------------------------------------------------------*/
#define SYSTICK_MAX_TASK_CONTROLLER						10

/* Extern variables ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static systick_task_controller_t *tasks[SYSTICK_MAX_TASK_CONTROLLER];
static uint8_t task_number = 0;

/******************************************************
*****************PRIMARY FUNCTION**********************
******************************************************/
bool systick_add_task_controller(systick_task_controller_t *task_controller, uint32_t task_cycle)
{
	if (task_number >= SYSTICK_MAX_TASK_CONTROLLER){
		return false;
	}

	task_controller->cycle = task_cycle;
	task_controller->timer = task_controller->cycle;
	tasks[task_number] = task_controller;

	task_number++;
	return true;
}

bool systick_check_task_enabled(systick_task_controller_t *task_controller)
{
	if (true == task_controller->is_enabled){
		// This algorithm makes sure that  task is enable only just 1 time
		// It will be enabled again after timer reaches its cycle
		task_controller->is_enabled = false;

		return true;
	}

	return false;
}

void systick_handler(void)
{
	uint8_t i;
	for (i = 0; i < task_number; i++)
	{
		tasks[i]->timer--;	
	}

	for (i = 0; i < task_number; i++)
	{
		if (0 == tasks[i]->timer)
		{
			tasks[i]->is_enabled = true;

			tasks[i]->timer = tasks[i]->cycle;
		}
	}
}


/******************************************************
****************INTERRUPT HANDLER**********************
******************************************************/

