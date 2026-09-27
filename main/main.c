#include "MQTT.h"
#include "synchronization.h"
#include "static_data.h"
#include "serializer.h"
#include "transmission_manager.h"

#include <stdio.h>
#include <string.h>
#include <esp_sleep.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include "esp_netif.h"

#include "esp_sleep.h"

#include "esp_wifi.h"
#include "wifiFastConnect.h"

#include "WiFi.h"

#include "time.h"
#include "esp_sntp.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

//########################## defines ##########################

#define FACTORY_RESET_PIN   GPIO_NUM_36 // adapted to the first board revision

#define TRANSMISSION_PERIOD 120*60 //Time between transmission, defined in seconds
#define TELEMETRY_STR_SIZE  250

#define MQTT_SAMPLE_NUMBER  10   //Number of enviromental acquisitions to include in MQTT transmission

#define SLEEP_INTERVAL      20*60 //Sleep interval in seconds

//########################## global variables ##########################

static const char *TAG = "WiFi_Body";
EventGroupHandle_t xEventGroupHandle = NULL;
SemaphoreHandle_t i2c_semaphore;
RTC_DATA_ATTR time_t boot_time;
RTC_DATA_ATTR time_t last_transmission_time;
RTC_DATA_ATTR int payload_group;

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

        //Connect to wifi and set the time
        time_t now;

        wifi_init_connection();
        wifi_set_time();
        time(&now);
        boot_time = now;    //Tracks when the system initially got online
        last_transmission_time = now;   //Wait for the buffer to fill before sending data
        payload_group = 0;  
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
            // Invoke transmission manager to send the data. it takes care of everything internally
            //json_generate_telemetry(telemetry_str,TELEMETRY_STR_SIZE,time_now,);
            // CAll transmission manager directly
            
            time_t time_now;
            time(&time_now);    //Gather the current system time
            transmission_manager(MQTT_SAMPLE_NUMBER, boot_time, payload_group, time_now + TRANSMISSION_PERIOD);
            payload_group++;
        }

        //Go to sleep!
        //After waking up from deep sleep the main will be executed again 

        esp_sleep_enable_timer_wakeup(SLEEP_INTERVAL * 1000000);
        esp_deep_sleep_start();

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
    
}
