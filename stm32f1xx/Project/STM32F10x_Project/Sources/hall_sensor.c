#include "hall_sensor.h"

/* Private const --------------------------------------------------------------*/

/* Extern variables------------------------------------------------------------*/

/* Private variables-----------------------------------------------------------*/

/* Private variables-----------------------------------------------------------*/

/* Private functions-----------------------------------------------------------*/


/******************************************************
*****************PRIMARY FUNCTION**********************
******************************************************/
void hall_sensor_init(hall_sensor_t *hall_sensor, float calibration_factor)
{
	hall_sensor->calibration_factor = calibration_factor;
}

void hall_sensor_routine(hall_sensor_t *hall_sensor, uint16_t raw_data, uint16_t routine_interval)
{
	hall_sensor->raw_data = raw_data;


	(void)(routine_interval);
}

void hall_sensor_get_data(hall_sensor_t *hall_sensor)
{
	hall_sensor->data = (float)hall_sensor->raw_data * hall_sensor->calibration_factor;
}



/******************************************************
***************AUXILIARY FUNCTION**********************
******************************************************/


