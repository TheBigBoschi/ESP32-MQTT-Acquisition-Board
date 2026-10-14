#include <time.h>
#include <static_data.h>
#include <serializer.h>
#include <stdio.h>
#include <string.h>
#include <esp_log.h>
#include "wifi_provisioner.h"
#include "mqtt_client.h"
#include <esp_log.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <esp_mac.h>
#include "esp_wifi_types.h"
#include "esp_wifi.h"

#include "adc_battery_estimation.h"

#define CONFIG_BROKER_URI "mqtt://192.168.1.20:1883"
#define CONFIG_BROKER_USERNAME "esp32"
#define CONFIG_BROKER_PASSWORD "test"

#define RTC_DATA_NOMINAL_SAMPLES_COUNT 6    //How many data samples will be sent out during each transmission
#define RTC_DATA_RETRANSMISSION_SAMPLES_COUNT 24
#define MQTT_CONSUMER_OUT_STR_SIZE 1000
#define MQTT_CONSUMER_TIMEOUT 30000
#define MQTT_EVENT_GROUP_TIMEOUT 120000
#define MQTT_DATA_STR_SIZE 3000             //calculated for a max of 24 samples in one go, with some space to spare
#define MQTT_TELEMETRY_STR_SIZE 400         //realistically 120 characters whould be enough, but memory is cheap.

//Battery SOC estimation

#define TEST_ADC_UNIT (ADC_UNIT_1)
#define TEST_ADC_BITWIDTH (ADC_BITWIDTH_DEFAULT)
#define TEST_ADC_ATTEN (ADC_ATTEN_DB_12)
#define TEST_ADC_CHANNEL (ADC_CHANNEL_3)
#define TEST_RESISTOR_UPPER (100000)
#define TEST_RESISTOR_LOWER (100000)
#define TEST_ESTIMATION_TIME (50)

static const char *TAG = "transmission_manager";

enum event_group_flags{
    event_ack_received,
    event_data_retransmit_ok,
    event_data_missing,
    event_error,
    event_handler_task_timed_out,
    event_handler_task_terminated
};

typedef struct{
    QueueHandle_t queue;
    EventGroupHandle_t event_group_handle;
    esp_mqtt_client_handle_t mqtt_handle;
    uint8_t MAC[6];
} mqtt_cmd_handler_task_params_t;

typedef struct{
    char* str;
    uint16_t size;
} queue_element_t;

//Read battery SOC
//NB using the standard PCB you have to toggle 3.3V_E high to enable the voltage divider.
esp_err_t read_battery_SOC(int* SOC)
{
    adc_battery_estimation_t config = {
            .internal = {
                .adc_unit = TEST_ADC_UNIT,
                .adc_bitwidth = TEST_ADC_BITWIDTH,
                .adc_atten = TEST_ADC_ATTEN,
            },
            .adc_channel = TEST_ADC_CHANNEL,
            .lower_resistor = TEST_RESISTOR_LOWER,
            .upper_resistor = TEST_RESISTOR_UPPER,
        };

    adc_battery_estimation_handle_t adc_battery_estimation_handle = adc_battery_estimation_create(&config);

    esp_err_t ret = ESP_OK;
    float capacity = 0;
    ret = adc_battery_estimation_get_capacity(adc_battery_estimation_handle, &capacity);
    *SOC = (int)capacity;
    printf("Battery capacity: %d\n", *SOC);
    
    if (ret == ESP_OK) ret = adc_battery_estimation_destroy(adc_battery_estimation_handle);
    else
        *SOC = -1;

    return ret;
}

static void mqtt_cmd_consumer_task(void* pvParameters)
{
    char *data_str = malloc(sizeof(char)*MQTT_CONSUMER_OUT_STR_SIZE); //calculated for a max of 24 samples in one go, with some space to spare. Heap, no stack
    mqtt_cmd_handler_task_params_t* task_params = (mqtt_cmd_handler_task_params_t*) pvParameters;
    queue_element_t* queue_element = NULL;
    char topic[32];

    sprintf(topic,MACSTR "/DATA", MAC2STR(task_params->MAC));

    while(true)
    {
        if(xQueueReceive(task_params->queue, &queue_element, pdMS_TO_TICKS(MQTT_CONSUMER_TIMEOUT)) != pdTRUE)
        {
            xEventGroupSetBits(task_params->event_group_handle,1<<event_error | 1<<event_handler_task_terminated);
            queue_element = NULL;
            break;  //No element received, so timeout reached.
        }
        if(strstr(queue_element->str,"DATA_ACK") != NULL)
        {
            xEventGroupSetBits(task_params->event_group_handle,1<<event_ack_received | 1<<event_handler_task_terminated);
            free(queue_element->str);
            free(queue_element);
            queue_element = NULL;
            break;
        }
        else if(strstr(queue_element->str,"RETRANSMIT") != NULL)  //The string RETRANSMIT is present in the message. 
        {
            long int init_time, end_time;
            rtc_data_t *rtc_data = malloc(sizeof(rtc_data_t)*24);

            if(sscanf(queue_element->str,"RETRANSMIT %ld - %ld", &init_time, &end_time) == 2)
            {
                while(true)
                {
                    int ret = get_data_interval(init_time,end_time,rtc_data,24);
                    if(ret == 0)
                    {
                        //json_generate_data(data_str,3000,rtc_data,ret);
                        json_generate_string(data_str, MQTT_CONSUMER_OUT_STR_SIZE, "NO_DATA");
                        esp_mqtt_client_publish(task_params->mqtt_handle,topic,data_str,0,1,0);
                        break;
                    }
                    else if(ret <=24)
                    {
                        json_generate_data(data_str,MQTT_CONSUMER_OUT_STR_SIZE,rtc_data,ret);
                        esp_mqtt_client_publish(task_params->mqtt_handle,topic,data_str,0,1,0);
                        break;
                    }
                    else
                    {
                        do
                        {
                            json_generate_data(data_str,MQTT_CONSUMER_OUT_STR_SIZE,rtc_data,RTC_DATA_RETRANSMISSION_SAMPLES_COUNT);
                            esp_mqtt_client_publish(task_params->mqtt_handle,topic,data_str,0,1,0);
                            init_time = rtc_data[RTC_DATA_RETRANSMISSION_SAMPLES_COUNT].time+1;    
                        }while(rtc_data[RTC_DATA_RETRANSMISSION_SAMPLES_COUNT].time < end_time);
                    }
                }

                free(rtc_data);
                free(queue_element->str);
                free(queue_element);
                queue_element = NULL;   
                xEventGroupSetBits(task_params->event_group_handle,1<<event_ack_received | 1<<event_handler_task_terminated);
                goto consumer_task_cleanup;
            }
        }
    }
    //Timeout reached. Set an error flag and kill the task.
consumer_task_cleanup:

    free(data_str);
    if(queue_element != NULL)
    {
        free(queue_element->str);
        free(queue_element);
    }
    vTaskDelete(NULL);

}

static void cmd_topic_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    mqtt_cmd_handler_task_params_t *task_params = (mqtt_cmd_handler_task_params_t*) handler_args;
    QueueHandle_t mqtt_queue = task_params->queue;
    EventGroupHandle_t event_group_handle = task_params->event_group_handle;
    queue_element_t* queue_element;

    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    

    switch((esp_mqtt_event_id_t)event_id){
    
    //Handles the CMD stream. put the messages in a queue, that then feeds the handler task
    case MQTT_EVENT_DATA:
        ESP_LOGI("MQTT", "Received MQTT data.");
        //The data pointed by event is tied to the event loop.
        //As soon as the handler returns the data is not guaranteed to be valid, hence I need to copy it before passing to the handler task.
        // The handler task will need to deallocate it.
        char *str = malloc(event->data_len+1);

        if(str == NULL)
        {
            xEventGroupSetBits(event_group_handle,1<<event_error);
            ESP_LOGI("MQTT", "Failed to allocate memory for MQTT data string.");
            return;
        }
        
        memcpy(str,event->data,event->data_len);
        str[event->data_len] = '\0';
        queue_element = malloc(sizeof(queue_element_t));

        if(queue_element == NULL)
        {
            xEventGroupSetBits(event_group_handle,1<<event_error);
            ESP_LOGI("MQTT", "Failed to allocate memory for MQTT queue element.");
            return;
        }

        queue_element->size = event->data_len+1;
        queue_element->str = str;

        if(xQueueSend(mqtt_queue, &queue_element, pdMS_TO_TICKS(500)) != pdTRUE)
        {
            free(str);
            free(queue_element);
            ESP_LOGI("MQTT", "Failed to send MQTT queue element.");
            xEventGroupSetBits(event_group_handle,1<<event_error);
        }
        break;

    case MQTT_EVENT_CONNECTED:
        ESP_LOGI("MQTT", "MQTT_EVENT_CONNECTED");
        char cmd_topic[32];
        snprintf(cmd_topic,sizeof(cmd_topic),MACSTR "/CMD", MAC2STR(task_params->MAC));
        if((esp_mqtt_client_subscribe_single(event->client,cmd_topic,1) < 0))
        {
            xEventGroupSetBits(event_group_handle,1<<event_error);
            ESP_LOGI("MQTT", "Failed to subscribe to topic: %s", event->topic);
        }
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI("MQTT", "MQTT_EVENT_SUBSCRIBED");
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, topic=%s", event->topic);
        break;

    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI("MQTT", "MQTT_EVENT_PUBLISHED, topic=%s", event->topic);
        break;

    default:
        ESP_LOGI("MQTT", "MQTT_EVENT_DEFAULT");
        xEventGroupSetBits(event_group_handle,1<<event_error);
        break;
    }
}

//Initiates the mqtt client, open the streams to publish the data and the telemetry, create a consumer task to handle the CMD stream 
//while listening to the command channel
esp_err_t transmission_manager(uint8_t sample_number, time_t boot_time, int32_t payload_group, time_t next_trasmission)
{    
    char *data_str = malloc(sizeof(char)*MQTT_DATA_STR_SIZE); 
    char *telemetry_str = malloc(sizeof(char)*MQTT_TELEMETRY_STR_SIZE);
    rtc_data_t *rtc_data = malloc(sizeof(rtc_data_t)*RTC_DATA_NOMINAL_SAMPLES_COUNT);
    mqtt_cmd_handler_task_params_t* task_params = malloc(sizeof(mqtt_cmd_handler_task_params_t));
    EventGroupHandle_t event_group_handle = xEventGroupCreate();

    QueueHandle_t mqtt_queue = NULL;
    esp_mqtt_client_handle_t mqtt_handle = NULL;
    TaskHandle_t consumer_task = NULL;
    esp_err_t ret = ESP_OK;
    uint8_t MAC[6];
    char data_topic[32];
    char telemetry_topic[32];
    char cmd_topic[32];
    char MAC_str[32];

    if(data_str == NULL || telemetry_str == NULL || task_params == NULL || event_group_handle == NULL || rtc_data == NULL)
    {
        ret = ESP_ERR_NO_MEM;
        goto event_handler_cleanup;
    }

    if(esp_read_mac(MAC,ESP_MAC_WIFI_STA) != ESP_OK)
    {
        ret = ESP_ERR_INVALID_MAC;
        goto event_handler_cleanup;
    }
    //Generates the MQTT topics based on the device MAC address and append the channel (data and command)
    snprintf(data_topic, sizeof(data_topic),MACSTR "/DATA", MAC2STR(MAC));
    snprintf(telemetry_topic, sizeof(telemetry_topic),MACSTR "/TELEMETRY", MAC2STR(MAC));
    snprintf(cmd_topic,sizeof(cmd_topic),MACSTR "/CMD", MAC2STR(MAC));

    //MQTT client init

    esp_mqtt_client_config_t mqtt_config = {
        .broker.address.uri = CONFIG_BROKER_URI,
        .credentials.username = CONFIG_BROKER_USERNAME,
        .credentials.authentication.password = CONFIG_BROKER_PASSWORD,
    };

    if((mqtt_handle = esp_mqtt_client_init(&mqtt_config)) == NULL)
    {
        ret =  ESP_ERR_INVALID_RESPONSE;
        goto event_handler_cleanup;
    }
    
    //Task and support data var init

    mqtt_queue = xQueueCreate(5, sizeof(queue_element_t*));  //Overkill, i have space for 5 messages
    
    if(mqtt_queue == NULL)
    {
        ret =  ESP_ERR_INVALID_RESPONSE;
        goto event_handler_cleanup;
    }
    
    task_params->queue = mqtt_queue;
    task_params->event_group_handle = event_group_handle;
    task_params->mqtt_handle = mqtt_handle;
    memcpy(task_params->MAC,MAC,6);

    if(ret == ESP_OK) ret = esp_mqtt_client_register_event(mqtt_handle, MQTT_EVENT_DATA, cmd_topic_event_handler,(void*) task_params);
    if(ret == ESP_OK) ret = esp_mqtt_client_register_event(mqtt_handle, MQTT_EVENT_ERROR, cmd_topic_event_handler,(void*) task_params);
    if(ret == ESP_OK) ret = esp_mqtt_client_register_event(mqtt_handle, MQTT_EVENT_CONNECTED, cmd_topic_event_handler,(void*) task_params);
    if(ret == ESP_OK) ret = esp_mqtt_client_register_event(mqtt_handle, MQTT_EVENT_SUBSCRIBED, cmd_topic_event_handler,(void*) task_params);
    if(ret == ESP_OK) ret = esp_mqtt_client_register_event(mqtt_handle, MQTT_EVENT_PUBLISHED, cmd_topic_event_handler,(void*) task_params);
    if(ret == ESP_OK) xTaskCreate(mqtt_cmd_consumer_task,"MQTT cmd handler task",2048,(void*) task_params,2,&consumer_task);
    if(ret == ESP_OK) ret = esp_mqtt_client_start(mqtt_handle);

    
    if(ret != ESP_OK)
    {
        if(ret == ESP_ERR_NO_MEM)
            ESP_LOGI("MQTT", "Failed to subscribe to topic due to no memory.");
        else if(ret == ESP_ERR_INVALID_ARG)
            ESP_LOGI("MQTT", "Failed to subscribe to topic due to invalid arguments.");
        else
            ESP_LOGI("MQTT", "Failed to subscribe to topic due to unknown error.");
        
        goto event_handler_cleanup;
    }

    time_t time_now;    
    time(&time_now);
    sprintf(MAC_str,MACSTR, MAC2STR(MAC));
    get_data_sample_number(rtc_data,RTC_DATA_NOMINAL_SAMPLES_COUNT);

    int SOC;
    int rssi;

    if(ret == ESP_OK) ret = read_battery_SOC(&SOC);
    esp_wifi_sta_get_rssi(&rssi);

    if(ret != ESP_OK || json_generate_data(data_str,MQTT_DATA_STR_SIZE,rtc_data,RTC_DATA_NOMINAL_SAMPLES_COUNT) == 0)
    {
        ret = ESP_FAIL;
        ESP_LOGI("MQTT", "Failed to generate data JSON.");
    }

    if(ret != ESP_OK || json_generate_telemetry(telemetry_str, MQTT_TELEMETRY_STR_SIZE, time_now, MAC_str,SOC,rssi, boot_time,payload_group, next_trasmission) == 0)
    {
        ret = ESP_FAIL;
        ESP_LOGI("MQTT", "Failed to generate data JSON.");
    }

    ESP_LOGI("MQTT", "data_str: %s", data_str);
    ESP_LOGI("MQTT", "telemetry_str: %s", telemetry_str);

    if(ret != ESP_OK || esp_mqtt_client_publish(mqtt_handle,data_topic, data_str, 0, 1, 0) < 0)
        ESP_LOGI("MQTT", "Failed to publish data.");

    if(ret != ESP_OK || esp_mqtt_client_publish(mqtt_handle,telemetry_topic, telemetry_str, 0, 1, 0) < 0)
        ESP_LOGI("MQTT", "Failed to publish telemetry.");

    EventBits_t xEventGroup_ret = xEventGroupWaitBits(
        event_group_handle,
        1<<event_ack_received,
        pdFALSE,
        pdTRUE,
        pdMS_TO_TICKS(MQTT_EVENT_GROUP_TIMEOUT));

    //I get inside here only if the task has not killed itself, but it has timed out (else it returned ack).
    if(eTaskGetState(consumer_task) != eDeleted)
    {
        xEventGroupSetBits(event_group_handle, 1 << event_handler_task_timed_out);
        vTaskDelete(consumer_task);
        consumer_task = NULL;
    }

    static const struct {
    const uint32_t mask;
    const char  *msg;
    } event_table[] = {
        { 1 << event_ack_received,              "ACK received"                      },
        { 1 << event_data_retransmit_ok,        "data retransmit OK"                },
        { 1 << event_data_missing,              "data missing"                      },
        { 1 << event_error,                     "transmission/reception error"      },
        { 1 << event_handler_task_timed_out,    "MQTT handler task timed out"       },
        { 1 << event_handler_task_terminated,   "MQTT handler task terminated OK"   },
    };

    for (size_t i = 0; i < sizeof(event_table) / sizeof(event_table[0]); i++)
        if (xEventGroup_ret & event_table[i].mask) 
            ESP_LOGI("MQTT", "Event set: %s", event_table[i].msg);

    //MQTT resources clearing
    
    esp_mqtt_client_unregister_event(mqtt_handle, MQTT_EVENT_DATA, cmd_topic_event_handler);
    esp_mqtt_client_unregister_event(mqtt_handle, MQTT_EVENT_ERROR, cmd_topic_event_handler);
    esp_mqtt_client_unregister_event(mqtt_handle, MQTT_EVENT_CONNECTED, cmd_topic_event_handler);
    esp_mqtt_client_unregister_event(mqtt_handle, MQTT_EVENT_SUBSCRIBED, cmd_topic_event_handler);

    if(xEventGroup_ret & 1<<event_error)
        ret = ESP_FAIL;
    else
        ret = ESP_OK;
    
event_handler_cleanup:
    // Cleanup in REVERSE order of allocation
    if(mqtt_queue != NULL)
        vQueueDelete(mqtt_queue);
    if(event_group_handle != NULL)
        vEventGroupDelete(event_group_handle);
    if(mqtt_handle != NULL)
        esp_mqtt_client_destroy(mqtt_handle);
    
    //NB freeing a NULL pointer is safe in C
    free(task_params);
    free(data_str);
    free(telemetry_str);
    free(rtc_data);
    
    return ret;
}