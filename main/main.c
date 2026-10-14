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

#define TRANSMISSION_PERIOD 120 //Time between transmission, defined in seconds
#define TELEMETRY_STR_SIZE  250

#define MQTT_SAMPLE_NUMBER  10  //Number of enviromental acquisitions to include in MQTT transmission
#define SAMPLING_INTERVAL   20  //Time between samples, defined in seconds

#define SLEEP_INTERVAL      5   //Sleep interval in seconds

//########################## global variables ##########################

static const char *TAG = "WiFi_Body";
EventGroupHandle_t xEventGroupHandle = NULL;
RTC_DATA_ATTR time_t boot_time;
RTC_DATA_ATTR time_t last_transmission_time;
RTC_DATA_ATTR int payload_group;
RTC_DATA_ATTR time_t last_sampling_time;

//########################## ToDO ##########################

// 1 - Manage the reset button during sleep
// 2 - Low power management (ie adaptive frequency scaling and DTIM thing)
// 3 - Manage what happens if the wifi is not available when needing to transmit data
// 4 - Waaay later, switch from RTC memory to flash (Maybe sooner than later, ideal to save loss of power/wifi logs)
// 5 - Something to be able to change device / MQTT settings after/during provisioning 
// 6 - OTA updates for firmware and configuration
// 7 - Logging of power loss events
// 8 - Handling of failed MQTT transmissions
// 9 - MQTT xEventGroup timeout event not working reliably, nor always able to move forward after ACK
// 10 - event_handler_task_terminated and event_handler_task_timeout not working as inteded.
//          Some kind of race condition between when the bit is set and the task kills itself
// 11 - 

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

    sps30_task_param_t sps30_meas;
    ltr390_task_param_t ltr390_meas;
    sht40_task_param_t sht40_meas;
    bmp280_task_param_t bmp280_meas;
    time(&now);

    if(now >= last_sampling_time + SAMPLING_INTERVAL - 1) //A bit of headroom to account for timing inaccuracies
    {
        ESP_LOGI(TAG, "Starting sensor sampling.");
        last_sampling_time = now;
        gpio_set_level(_3V_EN_PIN, 0); //enable power to the pheriperals
        vTaskDelay(pdMS_TO_TICKS(100)); // Small delay to ensure the peripherals are powered up before reading sensors
        read_sensors(&sps30_meas, &ltr390_meas, &sht40_meas, &bmp280_meas, &error_mask);
        store_data(&sps30_meas, &ltr390_meas, &sht40_meas, &bmp280_meas, error_mask,now);
        gpio_set_level(_3V_EN_PIN, 1); //disable power to the pheriperals
    }
    else
    {
        ESP_LOGI(TAG, "Not time for sensor sampling yet.");
    }

    if(now >= last_transmission_time + TRANSMISSION_PERIOD)
    {
        last_transmission_time = now;
        // Invoke transmission manager to send the data. it takes care of everything internally
        // Call transmission manager directly
        ESP_LOGI(TAG, "Starting data transmission.");
        time_t time_now;
        time(&time_now);    //Gather the current system time
        wifi_init_connection();
        wifi_set_time();
        transmission_manager(MQTT_SAMPLE_NUMBER, boot_time, payload_group, last_transmission_time + (time_t)TRANSMISSION_PERIOD);
        wifi_deinit_connection();
        payload_group++;
    }
    else
    {
        ESP_LOGI(TAG, "Not time for data transmission yet.");
    }

    //Go to sleep!
    //After waking up from deep sleep the main will be executed again 
    //Calculates how much time to sleep to wake up after SAMPLING_INTERVAL seconds after the last sampling
    time(&now);
    int32_t sleep_time = (last_sampling_time + SAMPLING_INTERVAL - now);

    if(sleep_time < 0)
        sleep_time = 1; //Ensure we don't pass a negative sleep time

    ESP_LOGI("SLEEP", "Entering deep sleep for %d seconds. Payload group: %d", sleep_time, payload_group);

    esp_sleep_enable_timer_wakeup(sleep_time * 1000000);
    esp_deep_sleep_start();
    
}
