/* Includes ------------------------------------------------------------------*/

#ifndef __HALL_SENSOR_H
#define __HALL_SENSOR_H

#include "stm32f10x.h"

typedef struct {
  uint16_t raw_data;
  float calibration_factor;
  float data;
} hall_sensor_t;

/* Private define ------------------------------------------------------------*/

/* Function prototypes--------------------------------------------------------*/
void hall_sensor_init(hall_sensor_t *hall_sensor, float calibration_factor);
void hall_sensor_routine(hall_sensor_t *hall_sensor, uint16_t raw_data, uint16_t routine_interval);
void hall_sensor_get_data(hall_sensor_t *hall_sensor);


/* Extern Function prototypes-------------------------------------------------*/

/* Extern for external use ---------------------------------------------------*/

/* Auxiliary Function---------------------------------------------------------*/


#endif

