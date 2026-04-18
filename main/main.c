/**
 * @file main.c
 * @brief ESP WiFi Manager - Basic Example
 *
 * This example demonstrates basic usage of the WiFi Manager component:
 * - Initialize with default networks
 * - Enable HTTP REST API for configuration
 * - Enable captive portal for initial setup
 * - Subscribe to WiFi events
 */

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

#define FACTORY_RESET_PIN GPIO_NUM_12
#define I2C_PORT        I2C_NUM_0
#define SDA_GPIO        GPIO_NUM_23
#define SCL_GPIO        GPIO_NUM_22
#define I2C_FREQ_HZ     100000

//########################## global variables ##########################

static const char *TAG = "WiFi_Body";
EventGroupHandle_t xEventGroupHandle = NULL;
SemaphoreHandle_t i2c_semaphore;

//########################## structs definition ##########################

//Defines the events used in the event groups
enum EventsDefinition{
    event_SPS30_read_ok,
    event_SPS30_error,
    event_SPS30_deinit,
    event_LTR390_read_ok,
    event_LTR390_error,
    event_AHT20_read_ok,
    event_AHT20__error,
    event_BMP280_read_ok,
    event_BMP280_error,
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
    float ambient_light; //lux
    float uvi; //UV index
    uint32_t sensor_counts_als; //ambient light count
    uint32_t sensor_counts_uvs; //UV count
    int holdup_time_ms;
    int samples;
} bmp280_task_param_t;
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
    i2c_master_dev_handle_t dev_handle_sps30;
    esp_err_t ret;
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

    //SPS30 initialization and setup
    while(true)
    {
        if(xSemaphoreTake(i2c_semaphore,pdMS_TO_TICKS(10000)) == pdTRUE)
        {
            ESP_LOGI("SPS30 reader task","lock acquired");
            break;
        }
        else
            ESP_LOGI("SPS30 reader task","lock not acquired");
    }

    ESP_LOGI(TAG, "SPS30 init");
    ret = sps30_init(params->bus_handle, &dev_handle_sps30, I2C_FREQ_HZ);

    if(ret == ESP_OK) ret = sps30_wake_up(dev_handle_sps30);
    if(ret == ESP_OK) ret = sps30_wake_up(dev_handle_sps30);
    vTaskDelay(pdMS_TO_TICKS(50));
    if(ret == ESP_OK) ret = sps30_start_measurement_float(dev_handle_sps30 );
    if(ret == ESP_OK) ret = sps30_read_serial_number(dev_handle_sps30,sps30_SN);
    
    if(ret != ESP_OK) xEventGroupSetBits(xEventGroupHandle,1<<event_SPS30_error);
    xSemaphoreGive(i2c_semaphore);
    ESP_LOGI("SPS30 reader task", "SPS30 initialized, serial number: %s", sps30_SN);

    vTaskDelay(pdMS_TO_TICKS(warmup_time_ms));

    //Results sampling and averaging (with the right timing)
    for(int i = 0; i < samples; i++)
    {
        xSemaphoreTake(i2c_semaphore,pdMS_TO_TICKS(10000));
        if(sps30_read_measured_values_float(dev_handle_sps30, &sps30_temp_meas) != ESP_OK)
            xEventGroupSetBits(xEventGroupHandle,1<<event_SPS30_error);
        xSemaphoreGive(i2c_semaphore);
    
        ESP_LOGI("SPS30 reader task", "SPS30 read values: PM10:%.3f PM2.5:%.3f", sps30_temp_meas.MC10p0, sps30_temp_meas.MC2p5);

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
            vTaskDelay(pdMS_TO_TICKS(holdup_time_ms));
    }

    xSemaphoreTake(i2c_semaphore,pdMS_TO_TICKS(10000));
    sps30_stop_measurement(dev_handle_sps30);
    //sps30_sleep(dev_handle_sps30);    //<-- To FIX! It's not working properly. (The sensor does not wake up)
    sps30_deinit(dev_handle_sps30);
    xSemaphoreGive(i2c_semaphore);


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

    //Signals that the data is ready, clean the cars then kill the thread
    ESP_LOGI("SPS30 reader task","min free stack: %d bytes", uxTaskGetStackHighWaterMark(NULL));
    xEventGroupSetBits(xEventGroupHandle,1<<event_SPS30_read_ok);
    ESP_LOGI("SPS30 reader task","Terminating task");
    vTaskDelete(NULL);
}

void read_ltr390_task(void *pvParameters)
{
    ltr390_task_param_t *params = (ltr390_task_param_t*)pvParameters;
    esp_err_t result;
    ltr390uv_handle_t dev_handle;

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
    
    if(xSemaphoreTake(params->semaphore,1000) == pdTRUE)
    {
        ltr390uv_init(params->bus_handle, &dev_cfg, &dev_handle);
        
        if (dev_handle == NULL) {
            ESP_LOGI("LTR390 reader task", "ltr390uv handle init failed");
            assert(dev_handle);
        }

        ltr390uv_get_measure_register(dev_handle, &m_reg);
        ltr390uv_get_gain_register(dev_handle, &g_reg);
        ltr390uv_get_interrupt_config_register(dev_handle, &ic_reg);
        ltr390uv_get_control_register(dev_handle, &c_reg);
        xSemaphoreGive(params->semaphore);
    }
    else xEventGroupSetBits(xEventGroupHandle,1<<event_LTR390_error);

    ESP_LOGI("LTR390 reader task", "Control Register (0x%02x): %s", c_reg.reg, uint8_to_binary(c_reg.reg));
    ESP_LOGI("LTR390 reader task", "Measure Register (0x%02x): %s", m_reg.reg, uint8_to_binary(m_reg.reg));
    ESP_LOGI("LTR390 reader task", "Gain Register    (0x%02x): %s", g_reg.reg, uint8_to_binary(g_reg.reg));
    ESP_LOGI("LTR390 reader task", "IRQ Cfg Register (0x%02x): %s", ic_reg.reg, uint8_to_binary(ic_reg.reg));
    
    // averaging loop entry point
    for (int i; i < params->samples; i++) {

        if(xSemaphoreTake(params->semaphore,1000) == pdTRUE)
        {
            result = ltr390uv_get_ambient_light(dev_handle, &ambient_light);
            if(result == ESP_OK) result = ltr390uv_get_als(dev_handle, &sensor_counts_als);
            if(result == ESP_OK) result = ltr390uv_get_ultraviolet_index(dev_handle, &uvi);
            if(result == ESP_OK) result = ltr390uv_get_uvs(dev_handle, &sensor_counts_uvs);
            else{
                ESP_LOGI("LTR390 reader task","ltr390uv device read failed (%s)", esp_err_to_name(result));
                xEventGroupSetBits(xEventGroupHandle,1<<event_LTR390_error);
            }
            xSemaphoreGive(params->semaphore);
        }
        else xEventGroupSetBits(xEventGroupHandle,1<<event_LTR390_error);

        ESP_LOGI("LTR390 reader task", "ambient light:     %.2f Lux, %lu counts", ambient_light,sensor_counts_als);
        ESP_LOGI("LTR390 reader task", "ultraviolet index: %f,%lu counts", uvi,sensor_counts_uvs);

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
    
    ESP_LOGI("LTR390 reader task","min free stack: %d bytes", uxTaskGetStackHighWaterMark(NULL));
    ltr390uv_delete(dev_handle);
    vTaskDelete(NULL);
}

void read_aht40_task(void *pvParameters)
{
    
}

void read_sensors(sps30_task_param_t *sps30_task_param, ltr390_task_param_t *ltr390_task_param, bmp280_task_param_t *bmp280_task_param)
{
    i2c_master_bus_handle_t bus_handle;
    //ltr390_measurement_float_t ltr390_measurement;

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
    
    //sps30_task_param->sps30_measurement <-- the data gets returned trough this
    sps30_task_param->bus_handle = bus_handle;
    sps30_task_param->semaphore = &i2c_semaphore;
    sps30_task_param->warmup_time_ms = 30000;
    sps30_task_param->holdup_time_ms = 3000;
    sps30_task_param->samples = 3;
    //ltr390 parameters
    ltr390_task_param->bus_handle = bus_handle;
    ltr390_task_param->semaphore = &i2c_semaphore;
    ltr390_task_param->holdup_time_ms = 500; //check the sensor integration time
    ltr390_task_param->samples = 3;

    ESP_LOGI(TAG, "Creating Tasks");
    //xTaskCreate(read_sps30_task,"SPS30 reader task",2304,(void*)sps30_task_param,1,NULL);
    //xTaskCreate(read_ltr390_task,"LTR390 reader task",4096,(void*)ltr390_task_param,1,NULL);
    xTaskCreate(read_bmp280_task,"BMP280 reader task",4096,(void*)bmp280_task_param,1,NULL);
    
    vTaskDelay(pdMS_TO_TICKS(100000));
    ESP_LOGI(TAG, "Waiting for bits group");
    
    xEventGroupWaitBits(xEventGroupHandle,1<<event_SPS30_read_ok,pdTRUE,pdTRUE,pdMS_TO_TICKS(60000));
    //check the error bits
    //To Do!
    ESP_LOGI(TAG, "Returining data");
    //once all the sensors have been read, return the values (by using a pointer to a linked var) 
    memcpy(sps30_meas, &sps30_task_param->sps30_measurement,sizeof(sps30_measurement_float_t));
    
    //Before "killing" the task notify that all the measurements have been done and free the i2c bus
    free(sps30_task_param);
    i2c_del_master_bus(bus_handle);

    xEventGroupSetBits(xEventGroupHandle,1<<event_sensor_read_ok);
}


void app_main(void)
{

    xEventGroupHandle = xEventGroupCreate();
    i2c_semaphore = xSemaphoreCreateMutex();    
    
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
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

    sps30_task_param_t sps30_meas;
    ltr390_task_param_t ltr390_meas;
    char pub_str[128];

    while(1)
    {
        read_sensors(&sps30_meas, &ltr390_meas);
        xEventGroupWaitBits(xEventGroupHandle,1<<event_sensor_read_ok,pdTRUE,pdTRUE,pdMS_TO_TICKS(60000));

        snprintf(pub_str, sizeof(pub_str), "PM10: %.3f PM2.5: %.3f", 
                 sps30_meas.MC10p0, 
                 sps30_meas.MC2p5);

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
