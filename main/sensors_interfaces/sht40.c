#include <stdint.h>
#include <sht40.h>
#include <synchronization.h>
#include <stdbool.h>
#include <driver/i2c_master.h>
#include <freertos/FreeRTOS.h>


/**
 * @brief Calculates the CRC for a 2-byte buffer using the Sensirion algorithm.
 *
 * @param buffer Array containing the 2 bytes to calculate CRC for.
 * @return uint8_t The calculated CRC.
 */
static uint8_t sht40_calculate_crc(uint8_t buffer[2])
{
    uint8_t crc = 0xFF;
    for(int i = 0; i < 2; i++) {
        crc ^= buffer[i];
        for(uint8_t bit = 8; bit > 0; --bit) {
            if(crc & 0x80)
                crc = (crc << 1) ^ 0x31u;
            else
                crc = (crc << 1);
        }
    }
    return crc;
}

/**
 * @brief FreeRTOS task to read temperature and humidity from an SHT40 sensor.
 *
 * This task initializes the SHT40 sensor on the I2C bus, reads temperature and humidity
 * data, performs CRC checks, and stores the results in the provided `sht40_task_param_t` structure.
 * It signals completion or error via an FreeRTOS event group.
 *
 * @param pvParameters A pointer to a `sht40_task_param_t` structure containing I2C bus handle, semaphore, and output variables.
 */
void read_sht40_task(void *pvParameters)
{
    sht40_task_param_t *params = (sht40_task_param_t*)pvParameters;
    i2c_master_dev_handle_t dev_handle;
    uint8_t cmd = 0xFD; //High res read (around 8ms)
    uint8_t outBuff[6];
    uint16_t rawTemp;
    uint16_t rawHum;
    esp_err_t ret;
    bool device_added = false;

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x44,
        .scl_speed_hz = 100000,
    };

    if(xSemaphoreTake(*params->semaphore,pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT)) != pdTRUE) goto cleanup;
    ret = i2c_master_bus_add_device(params->bus_handle, &dev_config, &dev_handle);   
    if(ret == ESP_OK) ret = i2c_master_transmit(dev_handle, &cmd, 1, I2C_OPERATION_TIMEOUT);
    xSemaphoreGive(*params->semaphore);
    if(ret != ESP_OK) goto cleanup;
    device_added = true;

    vTaskDelay(pdMS_TO_TICKS(20));
    
    if(xSemaphoreTake(*params->semaphore,pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT)) != pdTRUE) goto cleanup;
    ret = i2c_master_receive(dev_handle,outBuff,6,I2C_OPERATION_TIMEOUT);
    ret = i2c_master_bus_rm_device(dev_handle);
    xSemaphoreGive(*params->semaphore);
    if(ret != ESP_OK) goto cleanup;

    rawTemp = outBuff[0]<<8 | outBuff[1];
    rawHum = outBuff[3]<<8 | outBuff[4];
    params->temperature = -45+175*rawTemp/65535.0f;
    params->humidity = -6+125*rawHum/65535.0f;
    if(outBuff[2] != sht40_calculate_crc(&outBuff[0]) || outBuff[5] != sht40_calculate_crc(&outBuff[3]))
        xEventGroupSetBits(*params->event_group, 1<<(params->sensor_id*2+1));
    else 
        xEventGroupSetBits(*params->event_group, 1<<(params->sensor_id*2));
    
    vTaskDelete(NULL);

cleanup:
    if (device_added) {
        if (xSemaphoreTake(*params->semaphore, portMAX_DELAY) == pdTRUE) {
            i2c_master_bus_rm_device(dev_handle);
            xSemaphoreGive(*params->semaphore);
        }
    }
    xEventGroupSetBits(*params->event_group, 1<<(params->sensor_id*2+1));
    vTaskDelete(NULL);
}