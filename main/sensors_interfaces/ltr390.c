#include <ltr390.h>
#include <ltr390uv.h>
#include "syncronization.h"


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

    xEventGroupSetBits(*params->event_group, 1<<(params->sensor_id*2));
    vTaskDelete(NULL);

cleanup:
    if (device_added) {
        if (xSemaphoreTake(*params->semaphore, portMAX_DELAY) == pdTRUE) {
            ltr390uv_delete(dev_handle);
            xSemaphoreGive(*params->semaphore);
        }
    }
    xEventGroupSetBits(*params->event_group, 1<<(params->sensor_id*2+1));
    vTaskDelete(NULL);
}
