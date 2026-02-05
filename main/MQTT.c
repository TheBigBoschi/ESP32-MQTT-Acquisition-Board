
#include "wifiLogin.h"
#include "MQTT.h"

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

static const char *TAG = "MQTT Handler";

#define CONFIG_BROKER_URI "mqtt://192.168.1.20:1883"
#define CONFIG_BROKER_USERNAME "esp32"
#define CONFIG_BROKER_PASSWORD "test"

static esp_mqtt_client_handle_t globalClient = 0;


static void MQTT_Event_Handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    //esp_mqtt_client_handle_t client = event->client;
    //int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        /*msg_id = esp_mqtt_client_publish(client, "/esp32", "data_3", 0, 1, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        
        msg_id = esp_mqtt_client_subscribe(client, "/esp32", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_unsubscribe(client, "/esp32");
        ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id); */
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        //msg_id = esp_mqtt_client_publish(client, "/esp32", "data", 0, 0, 0);
        //ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
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

static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "%s: 0x%x", message, error_code);
    }
}


void MQTT_Config(){

    esp_mqtt_client_config_t MQTT_Config = {
        .broker.address.uri = CONFIG_BROKER_URI,
        .credentials.username = CONFIG_BROKER_USERNAME,
        .credentials.authentication.password = CONFIG_BROKER_PASSWORD,
    };

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());

    globalClient = esp_mqtt_client_init(&MQTT_Config);
    esp_mqtt_client_register_event(globalClient, ESP_EVENT_ANY_ID, MQTT_Event_Handler, NULL);
    esp_mqtt_client_start(globalClient);
};

int  MQTT_Deinit(){
    if(globalClient == 0){
        ESP_LOGI(TAG,"Error: Client not initialized");
        return -1;
    }
    else{
        esp_mqtt_client_destroy(globalClient);
        ESP_LOGI(TAG,"Client closed");
        globalClient = 0;
        return 0;
    }
};


int MQTT_Subscribe(char *topic, int qos){
    int msg_id = esp_mqtt_client_subscribe(globalClient, topic, qos);
    return msg_id;
};

void MQTT_Unsubscribe(char *topic){
    int msg_id = esp_mqtt_client_unsubscribe(globalClient, topic);
    ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
};

void MQTT_Publish(char *topic, char *payload,int qos){
    int msg_id = esp_mqtt_client_publish(globalClient, topic, payload, 0, qos, 0);
    ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
};
