#ifndef BMP280_H_
#define BMP280_H_

#include <stdint.h>
#include <stdbool.h>
#include <driver/i2c_master.h>
#include <freertos/FreeRTOS.h>
//#include "freertos/task.h"



typedef struct {
    i2c_master_bus_handle_t bus_handle;
    SemaphoreHandle_t *semaphore;
    EventGroupHandle_t *event_group;
    int sensor_id;    float avg_pressure;
    float avg_temperature;
    int samples;
} bmp280_task_param_t;

typedef struct {
    uint16_t    dig_T1;
    int16_t     dig_T2;
    int16_t     dig_T3;
    uint16_t    dig_P1;
    int16_t     dig_P2;
    int16_t     dig_P3;
    int16_t     dig_P4;
    int16_t     dig_P5;
    int16_t     dig_P6;
    int16_t     dig_P7;
    int16_t     dig_P8;
    int16_t     dig_P9;
} bmp280_NVS_t;

 typedef struct {
    bmp280_NVS_t bmp280_NVS;
    int32_t adc_P;
    int32_t adc_T;
    uint32_t compensated_P;
    int32_t compensated_T;
} bmp280_data_t;

/**
 * @brief Compensate raw BMP280 ADC readings and calculate temperature and pressure.
 *
 * Performs temperature and pressure compensation on raw ADC values from the BMP280 sensor
 * using factory calibration coefficients from the device's non-volatile storage (NVS).
 * The algorithm is based on the official Bosch BMP280 datasheet compensation formulas.
 *
 * Temperature compensation must be performed first, as the calculated t_fine value is
 * required for accurate pressure compensation.
 *
 * @param[in,out] bmp280_data Pointer to the BMP280 data structure containing:
 *                - bmp280_NVS: Calibration coefficients (dig_T1-T3, dig_P1-P9)
 *                - adc_T: Raw temperature ADC value (20-bit format)
 *                - adc_P: Raw pressure ADC value (20-bit format)
 *                Output fields filled:
 *                - compensated_T: Temperature in 0.01°C resolution (e.g., 5123 = 51.23°C)
 *                - compensated_P: Pressure in Pa, Q24.8 format (e.g., 24674867/256 = 963.862 hPa)
 *
 * @return ESP_OK if compensation succeeded.
 * @return ESP_ERR_INVALID_ARG if pressure calibration coefficient validation fails
 *         (division by zero protection; sensor likely not properly calibrated).
 *
 * @note The calibration coefficients (bmp280_data->bmp280_NVS) must be read from
 *       the sensor before calling this function (register 0x88-0xA1, 24 bytes).
 *
 * @note Raw ADC values must be in the correct 20-bit format:
 *       - adc_T = (MSB << 12) | (LSB << 4) | (XLSB >> 4)
 *       - adc_P = (MSB << 12) | (LSB << 4) | (XLSB >> 4)
 */
esp_err_t bmp280_compensate(bmp280_data_t *bmp280_data);

/**
 * @brief FreeRTOS task to read pressure and temperature from a BMP280 sensor.
 *
 * This task initializes the BMP280 sensor on the I2C bus, reads pressure and temperature
 * data, performs compensation and averaging, and stores the results in the provided
 * `bmp280_task_param_t` structure. It signals completion or error via a FreeRTOS event group.
 *
 * @param pvParameters A pointer to a `bmp280_task_param_t` structure containing I2C bus
 *                     handle, semaphore, number of samples, and output variables.
 */
void read_bmp280_task(void *pvParameters);

#endif // BMP280_H_