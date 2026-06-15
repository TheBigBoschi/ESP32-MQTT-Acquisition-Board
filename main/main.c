#include "wifiLogin.h"
#include "MQTT.h"
#include "sps30.h"
#include <ltr390uv.h>

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_wifi_manager.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include "esp_netif.h"
#include "esp_bus.h"

#include "esp_wifi.h"
#include "wifiFastConnect.h"

//########################## defines ##########################

#define FACTORY_RESET_PIN           GPIO_NUM_21
#define I2C_PORT                    I2C_NUM_0
#define SDA_GPIO                    GPIO_NUM_23
#define SCL_GPIO                    GPIO_NUM_22
#define I2C_FREQ_HZ                 100000
#define I2C_OPERATION_TIMEOUT       100
#define I2C_MUTEX_TIMEOUT           100

//All times expressed in ms
#define SPS30_WARMUP_TIME           10000
#define SPS30_HOLDUP_TIME           1000
#define SPS30_AVERAGING_SAMPLES     3
#define LTR390_HOLDUP_TIME          500
#define LTR390_AVERAGING_SAMPLES    3
#define BMP280_AVERAGING_SAMPLES    3

#define EVENTGROUP_WAIT_TIMEOUT     20000
//########################## global variables ##########################

static const char *TAG = "WiFi_Body";
EventGroupHandle_t xEventGroupHandle = NULL;
SemaphoreHandle_t i2c_semaphore;

//########################## structs definition ##########################

//Defines the events used in the event groups
enum EventsDefinition{
    event_SPS30_read_ok,
    event_SPS30_error,
    event_LTR390_read_ok,
    event_LTR390_error,
    event_BMP280_read_ok,
    event_BMP280_error,
    event_SHT40_read_ok,
    event_SHT40_error,
    event_sensor_read_ok,
    event_sensor_error
};

typedef struct {
    i2c_master_bus_handle_t bus_handle;
    SemaphoreHandle_t *semaphore;
    sps30_measurement_float_t sps30_measurement;
    int warmup_time_ms;
    int holdup_time_ms;
    int samples;
} sps30_task_param_t;

typedef struct {
    i2c_master_bus_handle_t bus_handle;
    SemaphoreHandle_t *semaphore;
    float ambient_light; //lux
    float uvi; //UV index
    uint32_t sensor_counts_als; //ambient light count
    uint32_t sensor_counts_uvs; //UV count
    int holdup_time_ms;
    int samples;
} ltr390_task_param_t;

typedef struct {
    i2c_master_bus_handle_t bus_handle;
    SemaphoreHandle_t *semaphore;
    float temperature;
    float humidity;
} sht40_task_param_t;

typedef struct {
    i2c_master_bus_handle_t bus_handle;
    SemaphoreHandle_t *semaphore;
    float avg_pressure;
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

//########################## functions definition ##########################

/**
 * @brief Initialize the pin defined for the reset button (FACTORY_RESET_PIN). Initializes as an input-pullup
 */
void reset_pin_init()
{
    gpio_config_t Factory_Reset_Pin = {
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pin_bit_mask = 1ULL << FACTORY_RESET_PIN
    };
    gpio_config(&Factory_Reset_Pin);
}

/**
 * @brief When called, checks if FACTORY_RESET_PIN is being held down
 *  and if so clear the configuration in the NVS and reset the board.
 */
void reset_pin_check()
{
     if (gpio_get_level(FACTORY_RESET_PIN) == 0) {
        ESP_LOGW(TAG, "Factory reset requested, depress button to execute.");
        
        while(gpio_get_level(FACTORY_RESET_PIN) == 0)
            vTaskDelay(pdMS_TO_TICKS(100));

        esp_wifi_restore();
        ESP_LOGW(TAG, "wifi restore executed");
        esp_bus_init();
        wifi_manager_init(NULL);
        ESP_LOGW(TAG, "esp manager init");
        wifi_manager_factory_reset();
        ESP_LOGW(TAG, "Factory reset executed");
        vTaskDelay(pdMS_TO_TICKS(3000));
        esp_restart();
    }
}

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

uint8_t sps30_calculate_crc(uint8_t buffer[2])
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

//Maybe use a queue?
/**
 * @brief Initialized and reads an SPS30. Puts the device in sleep and 
 * deinitialize it before returning. Intended to be called as a thread function.
 * 
 * @param 
 * @return
 */
void read_sps30_task(void *pvParameters) 
{
    sps30_task_param_t *params = (sps30_task_param_t*)pvParameters;
    i2c_master_dev_handle_t dev_handle;
    esp_err_t ret;
    bool device_added = false;
    char sps30_SN[32];
    
    int holdup_time_ms = params->holdup_time_ms;
    int samples = params->samples;
    int warmup_time_ms = params->warmup_time_ms;

    sps30_measurement_float_t sps30_temp_meas;
    sps30_measurement_float_t sps30_averaging_buff = {
        .MC1p0 = 0,
        .MC2p5 = 0,
        .MC4p0 = 0,
        .MC10p0 = 0,
        .NC0p5 = 0,
        .NC1p0 = 0,
        .NC2p5 = 0,
        .NC4p0 = 0,
        .NC10p0 = 0,
        .TypicalParticleSize = 0
    };

    if(xSemaphoreTake(*params->semaphore,pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT)) != pdTRUE) goto cleanup;
    ret = sps30_init(params->bus_handle, &dev_handle, I2C_FREQ_HZ);
    // NO SUCCESS CHECK! The device is in sleep, the probing will wake up the interface, but it will fail.
    i2c_master_probe(params->bus_handle,0x69,10);
    if(ret == ESP_OK) ret = sps30_wake_up(dev_handle);
    xSemaphoreGive(*params->semaphore);

    if(ret != ESP_OK) goto cleanup;
    device_added = true;
    
    vTaskDelay(pdMS_TO_TICKS(50));

    if(xSemaphoreTake(*params->semaphore,pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT)) != pdTRUE) goto cleanup;
    if(ret == ESP_OK) ret = sps30_read_serial_number(dev_handle,sps30_SN);
    if(ret == ESP_OK) ret = sps30_start_measurement_float(dev_handle );
    xSemaphoreGive(*params->semaphore);
    if(ret != ESP_OK) goto cleanup;
    vTaskDelay(pdMS_TO_TICKS(20 + warmup_time_ms)); //20ms minimium to execute the start readings command

    //Results sampling and averaging (with the right timing)
    for(int i = 0; i < samples; i++)
    {
    if(xSemaphoreTake(*params->semaphore,pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT)) != pdTRUE) goto cleanup;
        ret = sps30_read_measured_values_float(dev_handle, &sps30_temp_meas);
        xSemaphoreGive(*params->semaphore);
        if(ret != ESP_OK) goto cleanup;
    
        sps30_averaging_buff.MC1p0 += sps30_temp_meas.MC1p0;
        sps30_averaging_buff.MC2p5 += sps30_temp_meas.MC2p5;
        sps30_averaging_buff.MC4p0 += sps30_temp_meas.MC4p0;
        sps30_averaging_buff.MC10p0 += sps30_temp_meas.MC10p0;
        sps30_averaging_buff.NC0p5 += sps30_temp_meas.NC0p5;
        sps30_averaging_buff.NC1p0 += sps30_temp_meas.NC1p0;
        sps30_averaging_buff.NC2p5 += sps30_temp_meas.NC2p5;
        sps30_averaging_buff.NC4p0 += sps30_temp_meas.NC4p0;
        sps30_averaging_buff.NC10p0 += sps30_temp_meas.NC10p0;
        sps30_averaging_buff.TypicalParticleSize += sps30_temp_meas.TypicalParticleSize;
        
        //I dont need to wait the holdup time before repeating the cycle if it's the last reading. 
        if(i != samples-1)
        {
            vTaskDelay(pdMS_TO_TICKS(holdup_time_ms));
            //ESP_LOGI("SPS30","samples counter: %d + asddd",i);
            }
        }

    if(xSemaphoreTake(*params->semaphore,pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT)) != pdTRUE) goto cleanup;
    ret = sps30_stop_measurement(dev_handle);
    vTaskDelay(pdMS_TO_TICKS(20));
    if(ret == ESP_OK) ret = sps30_sleep(dev_handle);
    if(ret == ESP_OK) ret = sps30_deinit(dev_handle);

    xSemaphoreGive(*params->semaphore);
    if(ret != ESP_OK) goto cleanup;
    //ESP_LOGI("SPS30","SN: %s",sps30_SN);

    params->sps30_measurement.MC1p0 = sps30_averaging_buff.MC1p0/samples;
    params->sps30_measurement.MC2p5 = sps30_averaging_buff.MC2p5/samples;
    params->sps30_measurement.MC4p0 = sps30_averaging_buff.MC4p0/samples;
    params->sps30_measurement.MC10p0 = sps30_averaging_buff.MC10p0/samples;
    params->sps30_measurement.NC0p5 = sps30_averaging_buff.NC0p5/samples;
    params->sps30_measurement.NC1p0 = sps30_averaging_buff.NC1p0/samples;
    params->sps30_measurement.NC2p5 = sps30_averaging_buff.NC2p5/samples;
    params->sps30_measurement.NC4p0 = sps30_averaging_buff.NC4p0/samples;
    params->sps30_measurement.NC10p0 = sps30_averaging_buff.NC10p0/samples;
    params->sps30_measurement.TypicalParticleSize = sps30_averaging_buff.TypicalParticleSize/samples;                               
        
    //Signals that the data is ready, then kill the thread
    xEventGroupSetBits(xEventGroupHandle,1<<event_SPS30_read_ok);
    vTaskDelete(NULL);
    
cleanup:

    if (device_added == true) {
        if (xSemaphoreTake(*params->semaphore, portMAX_DELAY) == pdTRUE) {
            sps30_deinit(dev_handle);
            xSemaphoreGive(*params->semaphore);
        }
    }
    xEventGroupSetBits(xEventGroupHandle,1<<event_SPS30_error);
    vTaskDelete(NULL);
}

void read_ltr390_task(void *pvParameters)
{
    ltr390_task_param_t *params = (ltr390_task_param_t*)pvParameters;
    ltr390uv_handle_t dev_handle;
    esp_err_t ret;
    bool device_added = false;

    float ambient_light;
    float uvi;
    uint32_t sensor_counts_als; //Ambient light
    uint32_t sensor_counts_uvs; //UV
    
    float ambient_light_AVG_ACC = 0;
    float uvi_AVG_ACC = 0;
    uint32_t sensor_counts_als_AVG_ACC = 0; //Ambient light
    uint32_t sensor_counts_uvs_AVG_ACC = 0; //UV 

    ltr390uv_config_t dev_cfg = {
        .i2c_address               = I2C_LTR390UV_DEV_ADDR,     
        .i2c_clock_speed           = I2C_LTR390UV_DEV_CLK_SPD,  
        .window_factor             = 1,                         
        .als_sensor_resolution     = LTR390UV_SR_18BIT,         
        .als_measurement_rate      = LTR390UV_MR_100MS,         
        .als_measurement_gain      = LTR390UV_MG_X3,            
        .uvs_sensor_resolution     = LTR390UV_SR_18BIT,         
        .uvs_measurement_rate      = LTR390UV_MR_100MS,         
        .uvs_measurement_gain      = LTR390UV_MG_X3
    };

    ltr390uv_control_register_t c_reg;
    ltr390uv_interrupt_config_register_t ic_reg;
    ltr390uv_measure_register_t m_reg;
    ltr390uv_gain_register_t    g_reg;
    
    if(xSemaphoreTake(*params->semaphore,pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT)) != pdTRUE) goto cleanup;
    
    ret = ltr390uv_init(params->bus_handle, &dev_cfg, &dev_handle);
    
    if (dev_handle == NULL) {
        xSemaphoreGive(*params->semaphore);
        goto cleanup;
    }
    device_added = true;

    if(ret == ESP_OK) ret = ltr390uv_get_measure_register(dev_handle, &m_reg);
    if(ret == ESP_OK) ret = ltr390uv_get_gain_register(dev_handle, &g_reg);
    if(ret == ESP_OK) ret = ltr390uv_get_interrupt_config_register(dev_handle, &ic_reg);
    if(ret == ESP_OK) ret = ltr390uv_get_control_register(dev_handle, &c_reg);
    xSemaphoreGive(*params->semaphore);
    if(ret != ESP_OK) goto cleanup;

    // averaging loop entry point
    for (int i = 0; i < params->samples; i++) {

        if(xSemaphoreTake(*params->semaphore,pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT)) != pdTRUE) goto cleanup;
        
        ret = ltr390uv_get_ambient_light(dev_handle, &ambient_light);
        if(ret == ESP_OK) ret = ltr390uv_get_als(dev_handle, &sensor_counts_als);
        if(ret == ESP_OK) ret = ltr390uv_get_ultraviolet_index(dev_handle, &uvi);
        if(ret == ESP_OK) ret = ltr390uv_get_uvs(dev_handle, &sensor_counts_uvs);
        
        if(i == params->samples - 1)
        {
            if(ret == ESP_OK) ret = ltr390uv_delete(dev_handle);
        }
        xSemaphoreGive(*params->semaphore);
        if(ret != ESP_OK) goto cleanup;

        ambient_light_AVG_ACC += ambient_light;
        sensor_counts_als_AVG_ACC += sensor_counts_als;
        uvi_AVG_ACC += uvi;
        sensor_counts_uvs_AVG_ACC += sensor_counts_uvs;
        vTaskDelay(pdMS_TO_TICKS(params->holdup_time_ms));
    }

    params->ambient_light = ambient_light_AVG_ACC/params->samples;
    params->uvi = uvi_AVG_ACC/params->samples;
    params->sensor_counts_als = sensor_counts_als_AVG_ACC/params->samples;
    params->sensor_counts_uvs = sensor_counts_uvs_AVG_ACC/params->samples;

    xEventGroupSetBits(xEventGroupHandle,1<<event_LTR390_read_ok);
    vTaskDelete(NULL);

cleanup:
    if (device_added) {
        if (xSemaphoreTake(*params->semaphore, portMAX_DELAY) == pdTRUE) {
            ltr390uv_delete(dev_handle);
            xSemaphoreGive(*params->semaphore);
        }
    }
    xEventGroupSetBits(xEventGroupHandle,1<<event_LTR390_error);
    vTaskDelete(NULL);
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
    if(outBuff[2] != sps30_calculate_crc(&outBuff[0]) || outBuff[5] != sps30_calculate_crc(&outBuff[3]))
        xEventGroupSetBits(xEventGroupHandle, 1<<event_SHT40_error);
    else 
        xEventGroupSetBits(xEventGroupHandle, 1<<event_SHT40_read_ok);
    
    vTaskDelete(NULL);

cleanup:
    if (device_added) {
        if (xSemaphoreTake(*params->semaphore, portMAX_DELAY) == pdTRUE) {
            i2c_master_bus_rm_device(dev_handle);
            xSemaphoreGive(*params->semaphore);
        }
    }
    xEventGroupSetBits(xEventGroupHandle,1<<event_SHT40_error);
    vTaskDelete(NULL);
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
    
    xEventGroupSetBits(xEventGroupHandle,1<<event_BMP280_read_ok);
    vTaskDelete(NULL);    

cleanup:
    if (device_added) {
        if (xSemaphoreTake(*params->semaphore, portMAX_DELAY) == pdTRUE) {
            i2c_master_bus_rm_device(dev_handle);
            xSemaphoreGive(*params->semaphore);
        }
    }
    xEventGroupSetBits(xEventGroupHandle,1<<event_BMP280_error);
    vTaskDelete(NULL);
}

void read_sensors(sps30_task_param_t *sps30_task_param, ltr390_task_param_t *ltr390_task_param, sht40_task_param_t *sht40_task_param, bmp280_task_param_t *bmp280_task_param)
{
    i2c_master_bus_handle_t bus_handle;
    EventBits_t EventBits;

    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_PORT,
        .sda_io_num = SDA_GPIO,
        .scl_io_num = SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));
    ESP_LOGI(TAG, "I2C bus initialized on port %d", I2C_PORT);
    //sps30_task_param_t *sps30_task_param = malloc(sizeof(sps30_task_param_t));
    //ltr390_task_param_t *ltr390_task_param = malloc(sizeof(ltr390_task_param_t));
    
    // ##################### ADD I2C DEVICE PROBING AND LOGIC MANAGING MISSING DEVICES ######################

    //sps30_task_param->sps30_measurement <-- the data gets returned trough this
    sps30_task_param->bus_handle = bus_handle;
    sps30_task_param->semaphore = &i2c_semaphore;
    sps30_task_param->warmup_time_ms = SPS30_WARMUP_TIME;
    sps30_task_param->holdup_time_ms = SPS30_HOLDUP_TIME;
    sps30_task_param->samples = SPS30_AVERAGING_SAMPLES;
    //ltr390 parameters
    ltr390_task_param->bus_handle = bus_handle;
    ltr390_task_param->semaphore = &i2c_semaphore;
    ltr390_task_param->holdup_time_ms = LTR390_HOLDUP_TIME; //check the sensor integration time
    ltr390_task_param->samples = LTR390_AVERAGING_SAMPLES;
    //sht40 parameters
    sht40_task_param->bus_handle = bus_handle;
    sht40_task_param->semaphore = &i2c_semaphore;
    // bmp280 parameters
    bmp280_task_param->bus_handle = bus_handle;
    bmp280_task_param->semaphore = &i2c_semaphore;
    bmp280_task_param->samples = BMP280_AVERAGING_SAMPLES;

    ESP_LOGI(TAG, "Creating Tasks");
    xTaskCreate(read_sps30_task,"SPS30 reader task",4096,(void*)sps30_task_param,1,NULL);
    //xTaskCreate(read_ltr390_task,"LTR390 reader task",4096,(void*)ltr390_task_param,1,NULL);
    xTaskCreate(read_sht40_task,"SHT40 reader task",4096,(void*)sht40_task_param,1,NULL);
    xTaskCreate(read_bmp280_task,"BMP280 reader task",4096,(void*)bmp280_task_param,1,NULL);
    
    ESP_LOGI(TAG, "Waiting for bits group");
    
    EventBits = xEventGroupWaitBits(
        xEventGroupHandle,
        1<<event_SPS30_read_ok  |
        //1<<event_LTR390_read_ok |
        1<<event_SHT40_read_ok  |
        1<<event_BMP280_read_ok,
        pdFALSE,
        pdTRUE,
        pdMS_TO_TICKS(EVENTGROUP_WAIT_TIMEOUT)
    );

    ESP_LOGI(TAG, "Waiting for LR390");

    //Not ideal, but using the new and the old i2c driver togheter was not working, and I did not want to re-write this library too.
    //I wait for all the other i2c-using task to finish and then I read the LTR390, avoiding any mixup of i2c drivers.
    xTaskCreate(read_ltr390_task,"LTR390 reader task",4096,(void*)ltr390_task_param,1,NULL);
    
    EventBits = xEventGroupWaitBits(
        xEventGroupHandle,
        1<<event_LTR390_read_ok,
        pdFALSE,
        pdTRUE,
        pdMS_TO_TICKS(EVENTGROUP_WAIT_TIMEOUT)
    );
    
    static const struct {
    EventBits_t bit;
    const char  *msg;
    } event_table[] = {
        { 1 << event_SPS30_read_ok,  "SPS30 read OK"    },
        { 1 << event_SPS30_error,    "SPS30 error"      },
        { 1 << event_LTR390_read_ok, "LTR390 read OK"   },
        { 1 << event_LTR390_error,   "LTR390 error"     },
        { 1 << event_SHT40_read_ok,  "SHT40 read OK"    },
        { 1 << event_SHT40_error,    "SHT40 error"      },
        { 1 << event_BMP280_read_ok, "BMP280 read OK"   },
        { 1 << event_BMP280_error,   "BMP280 error"     },
    };

    for (size_t i = 0; i < sizeof(event_table) / sizeof(event_table[0]); i++) {
        if (EventBits & event_table[i].bit) 
            ESP_LOGI(TAG, "Event set: %s", event_table[i].msg);
    }
    
    ESP_LOGI(TAG, "All sensor task terminated");

    i2c_del_master_bus(bus_handle);
    xEventGroupClearBits(
        xEventGroupHandle,
        1 << event_SPS30_read_ok    |
        1 << event_SPS30_error      |
        1 << event_LTR390_read_ok   |
        1 << event_LTR390_error     |
        1 << event_SHT40_read_ok    | 
        1 << event_SHT40_error      | 
        1 << event_BMP280_read_ok   |
        1 << event_BMP280_error);
        
    xEventGroupSetBits(xEventGroupHandle,1<<event_sensor_read_ok);
    vTaskDelay(pdMS_TO_TICKS(10));
    
}

void sensors_value_print(sps30_task_param_t *sps30_task_param, ltr390_task_param_t *ltr390_task_param, sht40_task_param_t *sht40_task_param, bmp280_task_param_t *bmp280_task_param)
{
    ESP_LOGI("sensors_value_print","    SPS30:");
    ESP_LOGI("sensors_value_print","            PM10: %.2f μg/m3, PM2.5: %.2f μg/m3", sps30_task_param->sps30_measurement.MC10p0, sps30_task_param->sps30_measurement.MC2p5);
    ESP_LOGI("sensors_value_print","    LTR390:");
    ESP_LOGI("sensors_value_print","            AL: %.2f Lux, UVI: %d",ltr390_task_param->ambient_light, ltr390_task_param->uvi);
    ESP_LOGI("sensors_value_print","    SHT40:");
    ESP_LOGI("sensors_value_print","            Humidity: %.2f%, Temperature: %.2f °C", sht40_task_param->humidity, sht40_task_param->temperature);
    ESP_LOGI("sensors_value_print","    BMP280:");
    ESP_LOGI("sensors_value_print","            Pressure: %.2f Pa, Temp: %.2f °C",(float)bmp280_task_param->avg_pressure/256,(((float)bmp280_task_param->avg_temperature)/100));
}


void app_main(void)
{
    xEventGroupHandle = xEventGroupCreate();
    i2c_semaphore = xSemaphoreCreateMutex();    
    
    sps30_task_param_t sps30_meas;
    ltr390_task_param_t ltr390_meas;
    sht40_task_param_t sht40_meas;
    bmp280_task_param_t bmp280_meas;
    char pub_str[128];

    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    


    while(1)
    {
        read_sensors(&sps30_meas, &ltr390_meas, &sht40_meas, &bmp280_meas);
        sensors_value_print(&sps30_meas, &ltr390_meas, &sht40_meas, &bmp280_meas);
        //xEventGroupWaitBits(xEventGroupHandle,1<<event_sensor_read_ok,pdTRUE,pdTRUE,pdMS_TO_TICKS(EVENTGROUP_WAIT_TIMEOUT));
    }
    // Minimal network stack init (required by WiFi)
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Create default STA interface
    esp_netif_create_default_wifi_sta();

    // Init WiFi driver
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Required before calling esp_wifi_get_config()
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    // Read stored configuration
    wifi_config_t config;
    memset(&config, 0, sizeof(config));

    reset_pin_init();
    reset_pin_check();

    ESP_ERROR_CHECK(esp_wifi_get_config(WIFI_IF_STA, &config));

    if (strlen((char *)config.sta.ssid)) {
        ESP_LOGW(TAG, "Stored SSID: %s", (char *)config.sta.ssid);
        esp_wifi_deinit();
        wifi_Fast_Connect();
    } else {
        ESP_LOGW(TAG, "No WiFi credentials stored");
        esp_wifi_deinit();
        WiFi_Login();
    }

    

    wifi_ap_record_t ap_record;

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(5000));  // Main loop delay
        ESP_LOGI(TAG, "WiFi Status: %s", esp_wifi_sta_get_ap_info(&ap_record) == ESP_OK ? "Connected" : "Disconnected");
        if(esp_wifi_sta_get_ap_info(&ap_record) == ESP_OK)
        {
            ESP_LOGI(TAG, "Connected to %s - Signal: %d%% - Uptime:",ap_record.ssid, ap_record.rssi);
            vTaskDelay(pdMS_TO_TICKS(1000));
                break;
        }
    }

    int counter = 0;
    //MQTT publisher setup

    ESP_LOGI(TAG, "In the main loooop");

    //sps30_task_param_t sps30_meas;
    //ltr390_task_param_t ltr390_meas;
    //sht40_task_param_t sht40_meas;
    //bmp280_task_param_t bmp280_meas;
    //char pub_str[128];

    while(1)
    {
        read_sensors(&sps30_meas, &ltr390_meas, &sht40_meas, &bmp280_meas);
        xEventGroupWaitBits(xEventGroupHandle,1<<event_sensor_read_ok,pdTRUE,pdTRUE,pdMS_TO_TICKS(60000));

        snprintf(pub_str, sizeof(pub_str), "PM10: %.3f PM2.5: %.3f", 
                 sps30_meas.sps30_measurement.MC10p0, 
                 sps30_meas.sps30_measurement.MC2p5);

        //vTaskDelay(pdMS_TO_TICKS(5000));
        
        esp_wifi_sta_get_ap_info(&ap_record);
        ESP_LOGI(TAG, "WiFi connected, signal %d dBm. Looping through.", ap_record.rssi);
        MQTT_Config();
        
        char string[10];
        MQTT_Publish("/esp32",itoa(counter, string,10),1);
        MQTT_Publish("/esp32/var",pub_str,1);
        counter++;
        vTaskDelay(pdMS_TO_TICKS(1000));

        MQTT_Deinit();
    }
}
