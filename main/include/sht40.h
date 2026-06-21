#ifndef SHT40_H_
#define SHT40_H_

#include <stdint.h>
#include <stdbool.h>
#include <driver/i2c_master.h>
#include <freertos/FreeRTOS.h>


typedef struct {
    i2c_master_bus_handle_t bus_handle;
    SemaphoreHandle_t *semaphore;
    EventGroupHandle_t *event_group;
    int sensor_id;    float temperature;
    float humidity;
} sht40_task_param_t;

/**
 * @brief FreeRTOS task to read temperature and humidity from an SHT40 sensor.
 *
 * This task initializes the SHT40 sensor on the I2C bus, reads temperature and humidity
 * data, performs CRC checks, and stores the results in the provided `sht40_task_param_t` structure.
 * It signals completion or error via a FreeRTOS event group.
 *
 * @param pvParameters A pointer to a `sht40_task_param_t` structure containing I2C bus handle,
 *                     semaphore, and output variables.
 */
void read_sht40_task(void *pvParameters);

#endif // SHT40_H_