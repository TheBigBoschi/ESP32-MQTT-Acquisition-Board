#include <bmp280.h>
#include "synchronization.h"
#include "esp_log.h"



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
esp_err_t bmp280_compensate(bmp280_data_t *bmp280_data)
{
    int32_t var1, var2, t_fine;

    var1 = ((((bmp280_data->adc_T>>3) - ((int32_t)bmp280_data->bmp280_NVS.dig_T1<<1))) * ((int32_t)bmp280_data->bmp280_NVS.dig_T2)) >> 11;
    var2 = (((((bmp280_data->adc_T>>4) - ((int32_t)bmp280_data->bmp280_NVS.dig_T1)) * ((bmp280_data->adc_T>>4) - ((int32_t)bmp280_data->bmp280_NVS.dig_T1)))>> 12) * ((int32_t)bmp280_data->bmp280_NVS.dig_T3)) >> 14;
    t_fine = var1 + var2;
    bmp280_data->compensated_T = (t_fine * 5 + 128) >> 8;

    int64_t var1P, var2P, p;

    var1P = ((int64_t)t_fine) - 128000;
    var2P = var1P * var1P * (int64_t)bmp280_data->bmp280_NVS.dig_P6;
    var2P = var2P + ((var1P*(int64_t)bmp280_data->bmp280_NVS.dig_P5)<<17);
    var2P = var2P + (((int64_t)bmp280_data->bmp280_NVS.dig_P4)<<35);
    var1P = ((var1P * var1P * (int64_t)bmp280_data->bmp280_NVS.dig_P3)>>8) + ((var1P * (int64_t)bmp280_data->bmp280_NVS.dig_P2)<<12);
    var1P = (((((int64_t)1)<<47)+var1P))*((int64_t)bmp280_data->bmp280_NVS.dig_P1)>>33;
    
    if (var1P == 0){
        bmp280_data->compensated_P = 0; // avoid exception caused by division by zero
        return ESP_ERR_INVALID_ARG;
    }
    else{
        p = 1048576-bmp280_data->adc_P;
        p = (((p<<31)-var2P)*3125)/var1P;
        var1P = (((int64_t)bmp280_data->bmp280_NVS.dig_P9) * (p>>13) * (p>>13)) >> 25;
        var2P = (((int64_t)bmp280_data->bmp280_NVS.dig_P8) * p) >> 19;
        bmp280_data->compensated_P = ((p + var1P + var2P) >> 8) + (((int64_t)bmp280_data->bmp280_NVS.dig_P7)<<4);
        return ESP_OK;
    }
}

void read_bmp280_task(void *pvParams)
{
    bmp280_task_param_t *params = (bmp280_task_param_t*) pvParams;
    i2c_master_dev_handle_t dev_handle;
    uint8_t cmd[2];
    uint8_t buff[24];
    bmp280_data_t bmp280_data;
    uint32_t compensated_P_ACCUMULATOR = 0;
    int32_t compensated_T_ACCUMULATOR = 0;
    bool device_added = false;
    esp_err_t ret;

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x77,
        .scl_speed_hz = 100000,
    };

    //Send a soft reset command to the device
    cmd[0] = 0xE0;  //reset register
    cmd[1] = 0xB6;  //reset cmd    

    if(xSemaphoreTake(*params->semaphore,pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT)) != pdTRUE) goto cleanup;
    ret = i2c_master_bus_add_device(params->bus_handle , &dev_config, &dev_handle);
    if(ret == ESP_OK) ret = i2c_master_transmit(dev_handle,cmd,2,I2C_OPERATION_TIMEOUT);
    xSemaphoreGive(*params->semaphore);
    device_added = true;
    if(ret != ESP_OK) goto cleanup;
        
    // Wait for the NVS data to be ready, then loads it in memory.
    vTaskDelay(pdMS_TO_TICKS(10));

    cmd[0] = 0xF3;

    do{
        vTaskDelay(pdMS_TO_TICKS(10));
        if(xSemaphoreTake(*params->semaphore,pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT)) != pdTRUE) goto cleanup;
        ret = i2c_master_transmit_receive(dev_handle,cmd,1,buff,1,I2C_OPERATION_TIMEOUT);
        xSemaphoreGive(*params->semaphore);
        if(ret != ESP_OK) goto cleanup;
    }
    while((buff[0] & 0b00000001) != 0);

    //Read NVS data
    cmd[0]=0x88;
    if(xSemaphoreTake(*params->semaphore,pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT)) != pdTRUE) goto cleanup;
    ret = i2c_master_transmit_receive(dev_handle,cmd,1,buff,24,I2C_OPERATION_TIMEOUT);

    //  Device setup and sampling start

    // Bit organization: MSB 7 6 5 4 3 2 1 0 LSB
    // T_stby = 1000ms = 0b101 (bits 5 to 7)
    // IIR = 16 = 0b111 (bits 2 to 4)
    // 3-wire SPI disabled = 0b0 (bit 0)
    cmd[0] = 0xF5;
    cmd[1] = 0b10111100;
    if(ret == ESP_OK) ret = i2c_master_transmit(dev_handle,&cmd[0],2,I2C_OPERATION_TIMEOUT);

    //osrs_t = 2 = 0b010 (bits 5 to 7)
    //osrs_p = 16 = 0b111 (bits 2 to 4)
    //Normal Mode (continuos measurement) = 0b11 (bits 0 to 1)
    
    cmd[0] = 0xF4;
    cmd[1] = 0b01011111;
    if(ret == ESP_OK) ret = i2c_master_transmit(dev_handle,&cmd[0],2,I2C_OPERATION_TIMEOUT);
    xSemaphoreGive(*params->semaphore);
    if(ret != ESP_OK) goto cleanup;

    bmp280_data.bmp280_NVS.dig_T1 = (uint16_t)((buff[1] << 8) | buff[0]);
    bmp280_data.bmp280_NVS.dig_T2 = (int16_t)((buff[3] << 8)  | buff[2]);
    bmp280_data.bmp280_NVS.dig_T3 = (int16_t)((buff[5] << 8)  | buff[4]);
    bmp280_data.bmp280_NVS.dig_P1 = (uint16_t)((buff[7] << 8) | buff[6]);
    bmp280_data.bmp280_NVS.dig_P2 = (int16_t)((buff[9] << 8)  | buff[8]);
    bmp280_data.bmp280_NVS.dig_P3 = (int16_t)((buff[11] << 8) | buff[10]);
    bmp280_data.bmp280_NVS.dig_P4 = (int16_t)((buff[13] << 8) | buff[12]);
    bmp280_data.bmp280_NVS.dig_P5 = (int16_t)((buff[15] << 8) | buff[14]);
    bmp280_data.bmp280_NVS.dig_P6 = (int16_t)((buff[17] << 8) | buff[16]);
    bmp280_data.bmp280_NVS.dig_P7 = (int16_t)((buff[19] << 8) | buff[18]);
    bmp280_data.bmp280_NVS.dig_P8 = (int16_t)((buff[21] << 8) | buff[20]);
    bmp280_data.bmp280_NVS.dig_P9 = (int16_t)((buff[23] << 8) | buff[22]);

    //check if the sensor is still completing the first reading
    vTaskDelay(pdMS_TO_TICKS(10));
    cmd[0] = 0xF3;
    do{
        if(xSemaphoreTake(*params->semaphore,pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT)) != pdTRUE) goto cleanup;
        ret = i2c_master_transmit_receive(dev_handle,cmd,1,buff,1,I2C_OPERATION_TIMEOUT);
        xSemaphoreGive(*params->semaphore);
        if(ret != ESP_OK) goto cleanup;

    }while((buff[0] & 0b00001000) != 0);

    for(int i=0; i < params->samples; i++){
        //Read data 
        cmd[0] = 0xF7;
        if(xSemaphoreTake(*params->semaphore,pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT)) != pdTRUE) goto cleanup;
        ret = i2c_master_transmit_receive(dev_handle,cmd,1,buff,6,I2C_OPERATION_TIMEOUT);
        xSemaphoreGive(*params->semaphore);
        if(ret != ESP_OK) goto cleanup;

        bmp280_data.adc_P = ((int32_t)buff[0] << 12) | ((int32_t)buff[1] << 4) | ((int32_t)buff[2] >> 4);
        bmp280_data.adc_T = ((int32_t)buff[3] << 12) | ((int32_t)buff[4] << 4) | ((int32_t)buff[5] >> 4);
        
        if(bmp280_compensate(&bmp280_data) != ESP_OK) goto cleanup;
        
        compensated_P_ACCUMULATOR += bmp280_data.compensated_P;
        compensated_T_ACCUMULATOR += bmp280_data.compensated_T;
 
        // Slightly more than what a reading takes (1000ms of stby between radings + 40ms for the actual reading + 10ms to spare)
        vTaskDelay(pdMS_TO_TICKS(1050));
    }

    params->avg_pressure = (float)compensated_P_ACCUMULATOR/params->samples;
    params->avg_temperature = (float)compensated_T_ACCUMULATOR/params->samples;

    cmd[0] = 0xF4;
    cmd[1] = 0b01011100; //Put the sensor in sleep (0.1 uA current consumption)

    if(xSemaphoreTake(*params->semaphore,pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT)) != pdTRUE) goto cleanup;
    i2c_master_transmit(dev_handle,&cmd[0],2,I2C_OPERATION_TIMEOUT);
    ret = i2c_master_bus_rm_device(dev_handle);
    xSemaphoreGive(*params->semaphore);
    
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