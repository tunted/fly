#include "pid_controller.h"

/* Private const --------------------------------------------------------------*/
const float PID_CONTROLLER_UPPER_LIMIT			=	0.95;
const float PID_CONTROLLER_LOWER_LIMIT			= -0.95;

/* Extern variables------------------------------------------------------------*/

/* Private variables-----------------------------------------------------------*/

/* Private variables-----------------------------------------------------------*/

/* Private functions-----------------------------------------------------------*/


/******************************************************
*******************************************************
*****************PRIMARY FUNCTION**********************
*******************************************************
******************************************************/

void pid_controller_init(pid_controller_t *pid_controller, float kp, float ki, float kd)
{
	pid_controller->kp = kp;
	pid_controller->ki = ki;
	pid_controller->kd = kd;
	pid_controller->p_part = 0;
	pid_controller->i_part = 0;
	pid_controller->d_part = 0;
	pid_controller->error = 0;
	pid_controller->error_ = 0;
	pid_controller->error__ = 0;
	pid_controller->ref = 0;
	pid_controller->in = 0;
	pid_controller->out = 0;
}

void pid_controller_routine(pid_controller_t *pid_controller, float ts, float input)
{	
	pid_controller->in = input; 
	pid_controller->error__ = pid_controller->error_;
	pid_controller->error_ = pid_controller->error;
	pid_controller->error = pid_controller->ref - pid_controller->in;
	
	/* Calculate control signal */
	pid_controller->p_part = pid_controller->kp * (pid_controller->error - pid_controller->error_);
	pid_controller->i_part = pid_controller->ki * ts * pid_controller->error;
	pid_controller->d_part = pid_controller->kd / ts * (pid_controller->error - 2*pid_controller->error_ + pid_controller->error__);
	pid_controller->out += pid_controller->p_part + pid_controller->i_part + pid_controller->d_part;
}

void pid_controller_set_ref(pid_controller_t *pid_controller, float ref)
{
	pid_controller->ref = ref;
}

float pid_controller_get_control_signal(pid_controller_t *pid_controller)
{
	float control_signal;

	if (PID_CONTROLLER_UPPER_LIMIT <= pid_controller->out)
	{
		control_signal = PID_CONTROLLER_UPPER_LIMIT;
	}

	if (PID_CONTROLLER_LOWER_LIMIT >= pid_controller->out)
	{
		control_signal = PID_CONTROLLER_LOWER_LIMIT;
	}

	control_signal = pid_controller->out;

	return control_signal;
}

/******************************************************
*******************************************************
***************AUXILIARY FUNCTION**********************
*******************************************************
******************************************************/


