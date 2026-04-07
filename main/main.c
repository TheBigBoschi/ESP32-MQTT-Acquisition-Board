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

#define I2C_PORT        I2C_NUM_0
#define SDA_GPIO        GPIO_NUM_23
#define SCL_GPIO        GPIO_NUM_22
#define I2C_FREQ_HZ     100000

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

#define FACTORY_RESET_PIN GPIO_NUM_12
//########################## global variables ##########################

static const char *TAG = "WiFi_Body";
EventGroupHandle_t xEventGroupHandle = NULL;


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
    sps30_measurement_float_t sps30_measurement;
    int warmup_time_ms;
    int holdup_time_ms;
    int samples;
} sps30_task_param_t;

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
 * @brief Initialized and reads an SPS30. Puts the device in sleep and 
 * deinitialize it before returning. Intended to be called as a thread function.
 * 
 * @param 
 * @return
 */
void read_sps30(void *pvParameters)
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
    ESP_LOGI(TAG, "SPS30 init");
    ret = sps30_init(params->bus_handle, &dev_handle_sps30, I2C_FREQ_HZ);

    if(ret == ESP_OK) ret = xEventGroupSetBits(xEventGroupHandle,1<<event_SPS30_error);
    if(ret == ESP_OK) ret = sps30_wake_up(dev_handle_sps30);
    if(ret == ESP_OK) ret = sps30_start_measurement_float(dev_handle_sps30 );
    if(ret == ESP_OK) ret = (sps30_read_serial_number(dev_handle_sps30,sps30_SN);
    
    if(ret != ESP_OK) xEventGroupSetBits(xEventGroupHandle,1<<event_SPS30_error);

    ESP_LOGI(TAG, "SPS30 initialized, serial number: %s", sps30_SN);

    vTaskDelay(pdMS_TO_TICKS(warmup_time_ms));

    //Results sampling and averaging (with the right timing)
    for(int i = 0; i < samples; i++)
    {
        if(sps30_read_measured_values_float(dev_handle_sps30, &sps30_temp_meas) != ESP_OK)
            xEventGroupSetBits(xEventGroupHandle,1<<event_SPS30_error);

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

    sps30_stop_measurement(dev_handle_sps30);
    sps30_sleep(dev_handle_sps30);

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
    sps30_deinit(dev_handle_sps30);
    ESP_LOGI("SPS30 reader task","min free stack: %d bytes", uxTaskGetStackHighWaterMark(NULL));
    xEventGroupSetBits(xEventGroupHandle,1<<event_SPS30_read_ok);
    ESP_LOGI("SPS30 reader task","Terminating task");
    vTaskDelete(NULL);
}

void read_ltr390(void *pvParameters)
{
    //Placeholder
}

void read_bmp280(void *pvParameters)
{
    //Placeholder
}

void read_sensors(sps30_measurement_float_t *sps30_meas)
{
    //set up the return variables of the sensor reader functions
    i2c_master_bus_handle_t bus_handle;
    sps30_measurement_float_t sps30_measurement;
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
    sps30_task_param_t *sps30_task_param = malloc(sizeof(sps30_task_param_t));
    
    //sps30_task_param->sps30_measurement <-- the data gets returned trough this
    sps30_task_param->bus_handle = bus_handle;
    sps30_task_param->warmup_time_ms = 30000;
    sps30_task_param->holdup_time_ms = 3000;
    sps30_task_param->samples = 3;
     
    ESP_LOGI(TAG, "Entering SPS30 task");
    xTaskCreate(read_sps30,"SPS30 reader task",16384,(void*)sps30_task_param,1,NULL);
    //read_ltr390(bus_handle, ltr390_measurement, 1000, 1000, 3);
    vTaskDelay(pdMS_TO_TICKS(1000));
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

    sps30_measurement_float_t sps30_meas;
    char pub_str[128];

    while(1)
    {
        read_sensors(&sps30_meas);
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
