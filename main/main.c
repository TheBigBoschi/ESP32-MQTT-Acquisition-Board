#include "wifiLogin.h"
#include "MQTT.h"
#include "synchronization.h"
#include "static_data.h"
#include "serializer.h"

#include <stdio.h>
#include <string.h>
#include <esp_sleep.h>
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

#include "WiFi.h"

#include "time.h"
#include "esp_sntp.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

//########################## defines ##########################

#define FACTORY_RESET_PIN           GPIO_NUM_21

#define TRANSMISSION_PERIOD 120*60 //Time between transmission, defined in seconds
#define TELEMETRY_STR_SIZE 250
#define BATT_VOLTAGE_PIN ADC_CHANNEL_3

#define ADC_CHANNEL  ADC_CHANNEL_3   // GPIO39 = SENSOR_VN = ADC1 CH3
#define ADC_ATTEN    ADC_ATTEN_DB_11 // 150 mV ~ 3100 mV range


//########################## global variables ##########################

static const char *TAG = "WiFi_Body";
EventGroupHandle_t xEventGroupHandle = NULL;
SemaphoreHandle_t i2c_semaphore;
RTC_DATA_ATTR time_t boot_time;
RTC_DATA_ATTR time_t last_transmission_time;

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

void app_main(void)
{
    
    if(esp_rom_get_reset_reason(0) != RESET_REASON_CORE_DEEP_SLEEP)
    {
        //probable loss of power.

        //Connect to wifi
        time_t now;

        wifi_init_connection();
        wifi_set_time();
        time(&now);
        boot_time = now;    //Tracks when the system initially got online
        last_transmission_time = now;   //Wait for the buffer to fill before sending data

    }
    else
    {
        //Read sensors
        //manage readings
        //if transmitting == yes
        //  transmit
        //else
        //  sleep

        EventBits_t error_mask;
        time_t now;
        float batt_voltage;

        sps30_task_param_t sps30_meas;
        ltr390_task_param_t ltr390_meas;
        sht40_task_param_t sht40_meas;
        bmp280_task_param_t bmp280_meas;
        
        read_sensors(&sps30_meas, &ltr390_meas, &sht40_meas, &bmp280_meas, &error_mask);
        time(&now);
        store_data(&sps30_meas, &ltr390_meas, &sht40_meas, &bmp280_meas, error_mask,now);

        if(now > last_transmission_time + TRANSMISSION_PERIOD)
        {
            time_t time_now;
            time(time_now);

            // Invoke transmission manager to send the data. it takes care of everything internally
            //json_generate_telemetry(telemetry_str,TELEMETRY_STR_SIZE,time_now,);
            // CAll transmission manager directly
        }


    }

    //

    char pub_str[128];

    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
/*
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
        //xEventGroupWaitBits(xEventGroupHandle,1<<event_sensor_read_ok,pdTRUE,pdTRUE,pdMS_TO_TICKS(60000));

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
    */
            while(1)
    {

        
        
    }
}
