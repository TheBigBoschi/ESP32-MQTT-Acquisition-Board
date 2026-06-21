#ifndef LTR390_H_
#define LTR390_H_

#include <stdint.h>
#include <stdbool.h>
#include <driver/i2c_master.h>
#include <freertos/FreeRTOS.h>


typedef struct {
    i2c_master_bus_handle_t bus_handle;
    SemaphoreHandle_t *semaphore;
    EventGroupHandle_t *event_group;
    int sensor_id;    float ambient_light; //lux
    float uvi; //UV index
    uint32_t sensor_counts_als; //ambient light count
    uint32_t sensor_counts_uvs; //UV count
    int holdup_time_ms;
    int samples;
} ltr390_task_param_t;

/**
 * @brief FreeRTOS task to read ambient light and UV index from an LTR390 sensor.
 *
 * This task initializes the LTR390 sensor on the I2C bus, reads ambient light and UV
 * data, performs averaging, and stores the results in the provided `ltr390_task_param_t`
 * structure. It signals completion or error via a FreeRTOS event group.
 *
 * @param pvParameters A pointer to a `ltr390_task_param_t` structure containing I2C bus
 *                     handle, semaphore, hold-up time, number of samples, and output variables.
 */
void read_ltr390_task(void *pvParameters);

#endif // LTR390_H_