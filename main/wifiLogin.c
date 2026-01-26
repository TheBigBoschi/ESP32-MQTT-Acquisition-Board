#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_wifi_manager.h"
#include "esp_bus.h"

#define DEF_WIFI_NAME ""
#define DEF_WIFI_PASS ""


static const char *TAG = "WiFi_Login";

/**
 * @brief Callback for WiFi connected event
 */
static void on_wifi_connected(const char *event, const void *data, size_t len, void *ctx)
{
    const wifi_connected_t *info = (const wifi_connected_t *)data;
    ESP_LOGI(TAG, "WiFi connected to %s (RSSI: %d dBm, Channel: %d)",
             info->ssid, info->rssi, info->channel);
}

/**
 * @brief Callback for WiFi disconnected event
 */
static void on_wifi_disconnected(const char *event, const void *data, size_t len, void *ctx)
{
    const wifi_disconnected_t *info = (const wifi_disconnected_t *)data;
    ESP_LOGW(TAG, "WiFi disconnected from %s (reason: %d)", info->ssid, info->reason);
}

/**
 * @brief Callback for got IP event
 */
static void on_wifi_got_ip(const char *event, const void *data, size_t len, void *ctx)
{
    ESP_LOGI(TAG, "Got IP address");

    // Get full status
    wifi_status_t status;
    if (wifi_manager_get_status(&status) == ESP_OK) {
        ESP_LOGI(TAG, "IP: %s", status.ip);
        ESP_LOGI(TAG, "Gateway: %s", status.gateway);
        ESP_LOGI(TAG, "Netmask: %s", status.netmask);
        ESP_LOGI(TAG, "DNS: %s", status.dns);
        ESP_LOGI(TAG, "MAC: %s", status.mac);
        ESP_LOGI(TAG, "Signal quality: %d%%", status.quality);
    }
}

/**
 * @brief Callback for variable changed event
 */
static void on_var_changed(const char *event, const void *data, size_t len, void *ctx)
{
    const wifi_var_t *var = (const wifi_var_t *)data;
    ESP_LOGI(TAG, "Variable changed: %s = %s", var->key, var->value);
}

/**
 *  @brief Check if WiFi is connected
 */
bool WiFi_Connected()
{
    return wifi_manager_is_connected();
}

/**
 *  @brief Get current WiFi status
 */
esp_err_t WiFi_Get_Status(wifi_status_t* status)
{
    return wifi_manager_get_status(status);
}

/**
 *  @brief Initialize and connect to WiFi. If no known networks, start captive portal. 
 */
void WiFi_Login(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "Starting WiFi Manager example");

    // Initialize esp_bus first (required by wifi_manager)
    ret = esp_bus_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize esp_bus: %s", esp_err_to_name(ret));
        return;
    }

    // Subscribe to WiFi events (before wifi_manager_init to catch early events)
    esp_bus_sub(WIFI_EVT(WIFI_MGR_EVT_CONNECTED), on_wifi_connected, NULL);
    esp_bus_sub(WIFI_EVT(WIFI_MGR_EVT_DISCONNECTED), on_wifi_disconnected, NULL);
    esp_bus_sub(WIFI_EVT(WIFI_MGR_EVT_GOT_IP), on_wifi_got_ip, NULL);
    esp_bus_sub(WIFI_EVT(WIFI_MGR_EVT_VAR_CHANGED), on_var_changed, NULL);


    // Initialize WiFi Manager
    wifi_manager_config_t config = {

        // SoftAP configuration (for captive portal)
        .default_ap = {
            .ssid = "ESP_{id}",
            .password = "",           // Open network for easy setup
            .channel = 0,             // Auto channel selection
            .max_connections = 4,
            .ip = "192.168.4.1",
            .netmask = "255.255.255.0",
            .gateway = "192.168.4.1",
            .dhcp_start = "192.168.4.2",
            .dhcp_end = "192.168.4.20",
        },
        .enable_captive_portal = true,   // Start AP if no networks available
        .stop_ap_on_connect = true,      // Stop AP after successful connection
    };

    ret = wifi_manager_init(&config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize WiFi Manager: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "WiFi Manager initialized");
    ESP_LOGI(TAG, "HTTP API available at http://<device-ip>/api/wifi/");

    // Wait for connection with timeout
    ESP_LOGI(TAG, "Waiting for WiFi connection...");
    ret = wifi_manager_wait_connected(30000);  // 30 second timeout

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "WiFi connected successfully!");

    } else {
        ESP_LOGW(TAG, "WiFi connection timeout - captive portal should be active");
        ESP_LOGI(TAG, "Connect to AP '%s' and configure WiFi via http://192.168.4.1/api/wifi/",
                 config.default_ap.ssid);
    }
}
