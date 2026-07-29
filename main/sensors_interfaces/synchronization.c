#include <sps30.h>
#include <sht40.h>
#include <ltr390.h>
#include <bmp280.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/i2c_master.h>
#include <driver/gpio.h>
#include <esp_log.h>

//########################## I2C Configuration ##########################
#define I2C_PORT                    I2C_NUM_0
#define SDA_GPIO                    GPIO_NUM_23
#define SCL_GPIO                    GPIO_NUM_22

//########################## Sensor Readings Configuration ##########################

//All times expressed in ms
#define SPS30_WARMUP_TIME           10000   //at least 20S in production
#define SPS30_HOLDUP_TIME           1000
#define SPS30_AVERAGING_SAMPLES     3
#define LTR390_HOLDUP_TIME          500
#define LTR390_AVERAGING_SAMPLES    3
#define BMP280_AVERAGING_SAMPLES    3

#define EVENTGROUP_WAIT_TIMEOUT_LONG    30000
#define EVENTGROUP_WAIT_TIMEOUT_SHORT   5000

//########################## EventGroup bits definition ##########################

//Defines the events and the IDs used in the event groups

enum SensorsID{
    ID_SPS30,
    ID_LTR390,
    ID_BMP280,
    ID_SHT40
};

enum EventsDefinition{
    event_SPS30_read_ok = ID_SPS30*2,
    event_SPS30_error = ID_SPS30*2 + 1,
    event_LTR390_read_ok = ID_LTR390*2,
    event_LTR390_error = ID_LTR390*2 + 1,
    event_BMP280_read_ok = ID_BMP280*2,
    event_BMP280_error = ID_BMP280*2 + 1,
    event_SHT40_read_ok = ID_SHT40*2,
    event_SHT40_error = ID_SHT40*2 + 1,
    event_sensor_read_ok,
    event_sensor_error
};

//########################## Functions ##########################


void sensors_value_print(sps30_task_param_t* sps30_task_param, ltr390_task_param_t* ltr390_task_param,sht40_task_param_t* sht40_task_param,bmp280_task_param_t* bmp280_task_param)
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

void read_sensors(sps30_task_param_t *sps30_task_param, ltr390_task_param_t *ltr390_task_param, sht40_task_param_t *sht40_task_param, bmp280_task_param_t *bmp280_task_param, EventBits_t* error_mask)
{
    i2c_master_bus_handle_t bus_handle;
    SemaphoreHandle_t i2c_semaphore = xSemaphoreCreateMutex();
    EventGroupHandle_t event_group = xEventGroupCreate();
    EventBits_t event_bits;

    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_PORT,
        .sda_io_num = SDA_GPIO,
        .scl_io_num = SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));
    ESP_LOGI("SENSOR SYNC", "I2C bus initialized on port %d", I2C_PORT);
    
    // ##################### ADD I2C DEVICE PROBING AND LOGIC MANAGING MISSING DEVICES ######################

    //sps30_task_param->sps30_measurement <-- the data gets returned trough this
    sps30_task_param->bus_handle = bus_handle;
    sps30_task_param->semaphore = &i2c_semaphore;
    sps30_task_param->warmup_time_ms = SPS30_WARMUP_TIME;
    sps30_task_param->holdup_time_ms = SPS30_HOLDUP_TIME;
    sps30_task_param->samples = SPS30_AVERAGING_SAMPLES;
    sps30_task_param->sensor_id = ID_SPS30;
    sps30_task_param->event_group = &event_group;
    
    //ltr390 parameters
    ltr390_task_param->bus_handle = bus_handle;
    ltr390_task_param->semaphore = &i2c_semaphore;
    ltr390_task_param->holdup_time_ms = LTR390_HOLDUP_TIME; //check the sensor integration time
    ltr390_task_param->samples = LTR390_AVERAGING_SAMPLES;
    ltr390_task_param->sensor_id = ID_LTR390;
    ltr390_task_param->event_group = &event_group;
    
    //sht40 parameters
    sht40_task_param->bus_handle = bus_handle;
    sht40_task_param->semaphore = &i2c_semaphore;
    sht40_task_param->sensor_id = ID_SHT40;
    sht40_task_param->event_group = &event_group;
    
    // bmp280 parameters
    bmp280_task_param->bus_handle = bus_handle;
    bmp280_task_param->semaphore = &i2c_semaphore;
    bmp280_task_param->samples = BMP280_AVERAGING_SAMPLES;
    bmp280_task_param->sensor_id = ID_BMP280;
    bmp280_task_param->event_group = &event_group;

    ESP_LOGI("SENSOR SYNC", "Creating Tasks");
    xTaskCreate(read_sps30_task,"SPS30 reader task",4096,(void*)sps30_task_param,3,NULL);
    xTaskCreate(read_sht40_task,"SHT40 reader task",4096,(void*)sht40_task_param,3,NULL);
    xTaskCreate(read_bmp280_task,"BMP280 reader task",4096,(void*)bmp280_task_param,3,NULL);
    
    ESP_LOGI("SENSOR SYNC", "Waiting for bits group");
    
    event_bits = xEventGroupWaitBits(
        event_group,
        1<<event_SHT40_read_ok |
        1<<event_BMP280_read_ok |
        1<<event_SPS30_read_ok,
        pdFALSE,
        pdTRUE,
        pdMS_TO_TICKS(EVENTGROUP_WAIT_TIMEOUT_LONG));
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGI("SENSOR SYNC", "Waiting for LR390");

    //Not ideal, but using the new and the old i2c driver togheter was not working, and I did not want to re-write this library too.
    //I wait for all the other i2c-using task to finish and then I read the LTR390, avoiding any mixup of i2c drivers.
    xTaskCreate(read_ltr390_task,"LTR390 reader task",4096,(void*)ltr390_task_param,1,NULL);
    
    event_bits = xEventGroupWaitBits(event_group, 1<<event_LTR390_read_ok,pdFALSE,pdTRUE,pdMS_TO_TICKS(EVENTGROUP_WAIT_TIMEOUT_SHORT));

    static const struct {
    const uint32_t mask;
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
        if (event_bits & event_table[i].mask) 
            ESP_LOGI("SENSOR SYNC", "Event set: %s", event_table[i].msg);
    }
    
    ESP_LOGI("SENSOR SYNC", "All sensor task terminated");

    esp_err_t ret = i2c_del_master_bus(bus_handle);
    ESP_LOGI("SENSOR SYNC", "I2C bus deinitiated. ret = 0X%X", ret);

    xEventGroupClearBits(
        event_group,
        1 << event_SPS30_read_ok    |
        1 << event_SPS30_error      |
        1 << event_LTR390_read_ok   |
        1 << event_LTR390_error     |
        1 << event_SHT40_read_ok    | 
        1 << event_SHT40_error      | 
        1 << event_BMP280_read_ok   |
        1 << event_BMP280_error
    );   

    sensors_value_print(sps30_task_param, ltr390_task_param, sht40_task_param, bmp280_task_param);

    vTaskDelay(pdMS_TO_TICKS(2000));
    *error_mask = event_bits;
}