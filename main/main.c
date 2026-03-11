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
static const char *TAG = "WiFi_Body";

/**
 * @brief Initialize the pin defined for the reset button. Initializes as an input-pullup
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

void app_main(void)
{

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

    if (gpio_get_level(FACTORY_RESET_PIN) == 0) {
        ESP_LOGW(TAG, "Factory reset requested, depress button to execute.");
        
        while(gpio_get_level(FACTORY_RESET_PIN) == 0)
            vTaskDelay(pdMS_TO_TICKS(100));

        esp_wifi_restore();
        ESP_LOGW(TAG, "wifi restore executed");
        esp_bus_init();
        wifi_manager_init(NULL);
        ESP_LOGW(TAG, "esp manager init");
        //WiFi_Factory_Reset();
        wifi_manager_factory_reset();
        //nvs_flash_erase();
        ESP_LOGW(TAG, "Factory reset executed");
        vTaskDelay(pdMS_TO_TICKS(3000));
        esp_restart();
    }

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

    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    sps30_measurement_float_t sps30_measurement;

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
    sps30_init(bus_handle, &dev_handle, I2C_FREQ_HZ);
    char sps30_SN[32];
    sps30_start_measurement_float(dev_handle);
    sps30_read_serial_number(dev_handle,sps30_SN);
    ESP_LOGI(TAG, "SPS30 initialized, serial number: %s", sps30_SN);


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

    char MC10[8];
    char MC2p5[8];
    char pub_str[64];

    while(1)
    {
        sps30_read_measured_values_float(dev_handle, &sps30_measurement);
        snprintf(pub_str, sizeof(pub_str), "PM10: %.3f PM2.5: %.3f", 
                 sps30_measurement.MC10p0, 
                 sps30_measurement.MC2p5);

        vTaskDelay(pdMS_TO_TICKS(5000));
        
        esp_wifi_sta_get_ap_info(&ap_record);
        ESP_LOGI(TAG, "WiFi connected, signal %d dBm. Looping through.", ap_record.rssi);
        MQTT_Config();
        
        char string[10];
        MQTT_Publish("/esp32",itoa(counter, string,10),1);
        MQTT_Publish("/esp32/var",pub_str,1);
        counter++;
        vTaskDelay(pdMS_TO_TICKS(500));

        MQTT_Deinit();
    }
}


