/* Includes ------------------------------------------------------------------*/

#ifndef __PID_CONTROLLER_H
#define __PID_CONTROLLER_H

typedef struct {
  float ref;
  float in;
  float out;

  float kp; 
  float ki; 
  float kd;

  float p_part; 
  float i_part; 
  float d_part;

  float error;
  float error_;
  float error__;
} pid_controller_t;

/* Private define ------------------------------------------------------------*/

/* Function prototypes--------------------------------------------------------*/
void pid_controller_init(pid_controller_t *pid_controller, float kp, float ki, float kd);
void pid_controller_routine(pid_controller_t *pid_controller, float ts, float input);
void pid_controller_set_ref(pid_controller_t *pid_controller, float ref);
float pid_controller_get_control_signal(pid_controller_t *pid_controller);


/* Extern Function prototypes-------------------------------------------------*/

/* Extern for external use ---------------------------------------------------*/

/* Auxiliary Function---------------------------------------------------------*/


#endif

