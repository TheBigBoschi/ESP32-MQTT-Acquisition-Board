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

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "wifiLogin.h"
#include "esp_wifi_manager.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "WiFi_Body";





void app_main(void)
{
    WiFi_Login();

    ESP_LOGI(TAG, "In the main loooop");
    wifi_status_t status;

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(5000));  // Main loop delay
        ESP_LOGI(TAG, "WiFi Status: %s", WiFi_Get_Status(&status) == ESP_OK ? "Connected" : "Disconnected");
        if(WiFi_Get_Status(&status) == ESP_OK)
        {
            ESP_LOGI(TAG, "Connected to %s - Signal: %d%% - Uptime: %lu ms",status.ssid, status.quality, (unsigned long)status.uptime_ms);
            if(status.uptime_ms > 1000)
                break;
        }
    }

    //MQTT publisher setup
    while(1)
    {
        vTaskDelay(pdMS_TO_TICKS(5000));
        WiFi_Get_Status(&status);
        ESP_LOGI(TAG, "WiFi connected, signal %d. Looping through.", status.quality);
    }
}