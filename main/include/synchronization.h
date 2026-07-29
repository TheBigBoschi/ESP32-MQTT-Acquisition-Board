
#ifndef SYNCHRONIZATION_H_
#define SYNCHRONIZATION_H_



//########################## Sensor Includes ##########################
#include <sps30.h>
#include <ltr390.h>
#include <sht40.h>
#include <bmp280.h>

#define I2C_FREQ_HZ                 100000
#define I2C_OPERATION_TIMEOUT       100
#define I2C_MUTEX_TIMEOUT           100

void sensors_value_print(sps30_task_param_t *sps30_task_param, ltr390_task_param_t *ltr390_task_param, sht40_task_param_t *sht40_task_param, bmp280_task_param_t *bmp280_task_param);

void read_sensors(sps30_task_param_t *sps30_task_param, ltr390_task_param_t *ltr390_task_param, sht40_task_param_t *sht40_task_param, bmp280_task_param_t *bmp280_task_param, EventBits_t *error_mask);



#endif // SYNCHRONIZATION_H_
  





