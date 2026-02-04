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
#include "mqtt_client.h"
#include "esp_netif.h"



static const char *TAG = "WiFi_Body";

#define CONFIG_BROKER_URI "mqtt://esp32:test@192.168.1.20:1883"

static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "%s: 0x%x", message, error_code);
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        msg_id = esp_mqtt_client_publish(client, "/esp32", "data_3", 0, 1, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/esp32", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/esp32", 1);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_unsubscribe(client, "/esp32");
        ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        msg_id = esp_mqtt_client_publish(client, "/esp32", "data", 0, 0, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

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

    esp_mqtt_client_config_t MQTT_Config = {
        .broker.address.uri = CONFIG_BROKER_URI,
    };

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&MQTT_Config);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);

    //MQTT publisher setup
    while(1)
    {
        vTaskDelay(pdMS_TO_TICKS(5000));
        WiFi_Get_Status(&status);
        ESP_LOGI(TAG, "WiFi connected, signal %d. Looping through.", status.quality);
        
    }
}


