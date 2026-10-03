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
#define _3V_EN_PIN          GPIO_NUM_22  // 3.3V enable
#define LED_PIN             GPIO_NUM_0
//#define _5V_EN_PIN          GPIO_NUM_35  // Input only
#define _3V_LORA_EN_PIN     GPIO_NUM_27  // 5V enable
//#define _3V_SAMPLE_EN_PIN   GPIO_NUM_35  // 5V enable

#define TRANSMISSION_PERIOD 60 //Time between transmission, defined in seconds
#define TELEMETRY_STR_SIZE  250

#define MQTT_SAMPLE_NUMBER  10   //Number of enviromental acquisitions to include in MQTT transmission

#define SLEEP_INTERVAL      5 //Sleep interval in seconds

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
void pin_init()
{
    gpio_set_direction(FACTORY_RESET_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(_3V_EN_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(_3V_EN_PIN, 1); //enable 3.3V
    gpio_set_direction(_3V_LORA_EN_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(_3V_LORA_EN_PIN, 1); //enable 3.3V for LORA
    //gpio_set_direction(_5V_EN_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PIN, 1); //enable 5V
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

        for(int i = 0; i < 5; i++) {
            gpio_set_level(LED_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(300));
            gpio_set_level(LED_PIN, 0);
            vTaskDelay(pdMS_TO_TICKS(300));
        }
        esp_restart();
    }
}

    void wifi_deinit_connection(){
        esp_wifi_disconnect();
        esp_wifi_stop();
        esp_wifi_deinit();
    }


void app_main(void)
{
    pin_init();  // Initialize the factory reset pin
    reset_pin_check(); // Check if the factory reset pin is being held down

    gpio_set_level(LED_PIN, 1); //enable 3.3V
    vTaskDelay(pdMS_TO_TICKS(500)); // Small delay to ensure the LED state is updated
    gpio_set_level(LED_PIN, 0); //enable 3.3V

    if(esp_rom_get_reset_reason(0) != RESET_REASON_CORE_DEEP_SLEEP)
    {
        //probable loss of power.

        //Connect to wifi and set the time
        ESP_LOGW(TAG, "Probable loss of power detected, reinitializing memory and setting time.");
        time_t now;

        wifi_init_connection();
        wifi_set_time();
        time(&now);
        boot_time = now;    //Tracks when the system initially got online
        last_transmission_time = now;   //Wait for the buffer to fill before sending data
        payload_group = 0;
        wifi_deinit_connection(); 
    }
    else
    {
        ESP_LOGW(TAG, "Normal boot detected.");
    }

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
    
    gpio_set_level(_3V_EN_PIN, 0); //enable power to the pheriperals
    read_sensors(&sps30_meas, &ltr390_meas, &sht40_meas, &bmp280_meas, &error_mask);
    time(&now);
    store_data(&sps30_meas, &ltr390_meas, &sht40_meas, &bmp280_meas, error_mask,now);

    if(now >= last_transmission_time)
    {
        // Invoke transmission manager to send the data. it takes care of everything internally
        //json_generate_telemetry(telemetry_str,TELEMETRY_STR_SIZE,time_now,);
        // Call transmission manager directly
        
        time_t time_now;
        time(&time_now);    //Gather the current system time
        transmission_manager(MQTT_SAMPLE_NUMBER, boot_time, payload_group, time_now + TRANSMISSION_PERIOD);
        payload_group++;
    }

    //Go to sleep!
    //After waking up from deep sleep the main will be executed again 

    ESP_LOGI("SLEEP", "Entering deep sleep for %d seconds. Payload group: %d", SLEEP_INTERVAL, payload_group);
    payload_group++;

    esp_sleep_enable_timer_wakeup(SLEEP_INTERVAL * 1000000);
    esp_deep_sleep_start();



    char pub_str[128];

    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
}
