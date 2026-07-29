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

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#define CONFIG_BROKER_URI "mqtt://192.168.1.20:1883"
#define CONFIG_BROKER_USERNAME "esp32"
#define CONFIG_BROKER_PASSWORD "test"

#define RTC_DATA_NOMINAL_SAMPLES_COUNT 6    //How many data samples will be sent out during each transmission
#define RTC_DATA_RETRANSMISSION_SAMPLES_COUNT 24
#define MQTT_CONSUMER_OUT_STR_SIZE 1000
#define MQTT_CONSUMER_TIMEOUT 60000
#define MQTT_EVENT_GROUP_TIMEOUT 120000
#define MQTT_DATA_STR_SIZE 3000             //calculated for a max of 24 samples in one go, with some space to spare
#define MQTT_TELEMETRY_STR_SIZE 250         //realistically 120 characters whould be enough, but memory is cheap.

#define ADC_CHANNEL  ADC_CHANNEL_3   // GPIO39 = SENSOR_VN = ADC1 CH3
#define ADC_ATTEN    ADC_ATTEN_DB_12 // 150 mV ~ 3100 mV range

static const char *TAG = "transmission_manager";

enum event_group_flags{
    event_ack_received,
    event_data_retransmit_ok,
    event_data_missing,
    event_error,
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

//Read battery voltage in mV
//NB using the standard PCB you have to toggle 3.3V_E high to enable the voltage divider.
esp_err_t read_battery_voltage(int* voltage)
{
    esp_err_t ret;
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    
    adc_oneshot_new_unit(&init_cfg, &adc1_handle);

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT, // highest available bitwidth
    };
    ret = adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL, &chan_cfg);

    adc_cali_handle_t cali_handle = NULL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    // Preferred scheme (ESP32-S2, ESP32-S3, ESP32-C3, etc.)
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id  = ADC_UNIT_1,
        .chan     = ADC_CHANNEL,
        .atten    = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if(ret != ESP_OK)
        return ret;
    if ((ret = adc_cali_create_scheme_curve_fitting(&cali_cfg, &cali_handle)) == ESP_OK) {
        calibrated = true;
        ESP_LOGI(TAG, "Calibration: Curve Fitting");
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    // Fallback scheme (ESP32, ESP32-S2)
    if (!calibrated) {
        adc_cali_line_fitting_config_t cali_cfg = {
            .unit_id  = ADC_UNIT_1,
            .atten    = ADC_ATTEN,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        if(ret != ESP_OK)
            return ret;
        if ((ret = adc_cali_create_scheme_line_fitting(&cali_cfg, &cali_handle)) == ESP_OK) {
            calibrated = true;
            ESP_LOGI(TAG, "Calibration: Line Fitting");
        }
    }
#endif


    //Averages the readings to improve accuracy
    int raw = 0;
    int accumulator = 0;
    for(int i = 0; i < 32; i++)
    {

        if(ret == ESP_OK) ret = adc_oneshot_read(adc1_handle, ADC_CHANNEL, &raw);
        accumulator += raw;
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    if (calibrated)
    {
        int voltage_mv = 0;
        if(ret == ESP_OK) ret = adc_cali_raw_to_voltage(cali_handle, accumulator, &voltage_mv);
        *voltage = (voltage_mv / 32)*2450/4096;
    }
    else
        *voltage = (accumulator / 32)*2450/4096;

    //--- 5. Teardown ---
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
    if (calibrated) {
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
        if(ret == ESP_OK) ret = adc_cali_delete_scheme_curve_fitting(cali_handle);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
        if(ret == ESP_OK) ret = adc_cali_delete_scheme_line_fitting(cali_handle);
#endif
    }
    return ret;
}

static void mqtt_cmd_consumer_task(void* pvParameters)
{
    char data_str[MQTT_CONSUMER_OUT_STR_SIZE]; //calculated for a max of 24 samples in one go, with some space to spare
    mqtt_cmd_handler_task_params_t* task_params = (mqtt_cmd_handler_task_params_t*) pvParameters;
    queue_element_t* queue_element = NULL;
    char topic[32];

    sprintf(topic,MACSTR "/DATA", MAC2STR(task_params->MAC));

    while(true)
    {
        if(xQueueReceive(task_params->queue, &queue_element, pdMS_TO_TICKS(MQTT_CONSUMER_TIMEOUT)) != pdTRUE)
        {
            break;  //No element received, so timeout reached.
        }
        if(strstr(queue_element->str,"DATA_ACK") != NULL)
        {
            xEventGroupSetBits(task_params->event_group_handle,1<<event_ack_received | 1<<event_handler_task_terminated);
            free(queue_element->str);
            free(queue_element);

            vTaskDelete(NULL);
        }
        else if(strstr(queue_element->str,"RETRANSMIT") != NULL)  //The string RETRANSMIT is present in the message. 
        {
            time_t init_time, end_time;
            rtc_data_t rtc_data[24];

            if(sscanf(queue_element->str,"RETRANSMIT %ld - %ld", &init_time, &end_time) == 2)
            {
                while(true)
                {
                    int ret = get_data_interval(init_time,end_time,rtc_data,24);
                    if(ret == 0)
                    {
                        //json_generate_data(data_str,3000,rtc_data,ret);
                        json_generate_string(data_str,"NO_DATA");
                        esp_mqtt_client_publish(task_params->mqtt_handle,topic,data_str,0,1,0);
                        break;
                    }
                    else if(ret <=24)
                    {
                        json_generate_data(data_str,3000,rtc_data,ret);
                        esp_mqtt_client_publish(task_params->mqtt_handle,topic,data_str,0,1,0);
                        break;
                    }
                    else
                    {
                        json_generate_data(data_str,3000,rtc_data,RTC_DATA_RETRANSMISSION_SAMPLES_COUNT);
                        esp_mqtt_client_publish(task_params->mqtt_handle,topic,data_str,0,1,0);
                        init_time = rtc_data[23].time+1;         
                    }
                }
                free(queue_element->str);
                free(queue_element);
                xEventGroupSetBits(task_params->event_group_handle,1<<event_ack_received | 1<<event_handler_task_terminated);
                vTaskDelete(NULL);
            }
        }
        // Free the queue element if not already processed
        if(queue_element != NULL)
        {
            free(queue_element->str);
            free(queue_element);
            queue_element = NULL;
        }
    }
    //Timeout reached. Set an error flag and kill the task.
    xEventGroupSetBits(task_params->event_group_handle,1<<event_error | 1<<event_handler_task_terminated);
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
        //The data pointed by event is tied to the event loop.
        //As soon as the handler returns the data is not guaranteed to be valid, hence I need to copy it before passing to the handler task.
        // The handler task will need to deallocate it.
        char *str = malloc(event->data_len+1);

        if(str == NULL)
        {
            xEventGroupSetBits(event_group_handle,1<<event_error);
            return;
        }
        
        memcpy(str,event->data,event->data_len);
        str[event->data_len] = '\0';
        queue_element = malloc(sizeof(queue_element_t));

        if(queue_element == NULL)
        {
            xEventGroupSetBits(event_group_handle,1<<event_error);
            return;
        }

        queue_element->size = event->data_len+1;
        queue_element->str = str;

        if(xQueueSend(mqtt_queue, &queue_element, pdMS_TO_TICKS(500)) != pdTRUE)
        {
            free(str);
            free(queue_element);
            xEventGroupSetBits(event_group_handle,1<<event_error);
        }
        break;
    default:
        xEventGroupSetBits(event_group_handle,1<<event_error);
        break;
    }
}

//Initiates the mqtt client, open the streams to publish the data and the telemetry, create a consumer task to handle the CMD stream 
//while listening to the command channel
esp_err_t transmission_manager(uint8_t sample_number, time_t boot_time, int32_t payload_group, time_t next_wakeup)
{    
    char data_str[MQTT_DATA_STR_SIZE]; 
    char telemetry_str[MQTT_TELEMETRY_STR_SIZE];
    QueueHandle_t mqtt_queue = NULL;
    esp_mqtt_client_handle_t mqtt_handle = NULL;
    TaskHandle_t consumer_task = NULL;
    EventGroupHandle_t event_group_handle = xEventGroupCreate();
    mqtt_cmd_handler_task_params_t* task_params = malloc(sizeof(mqtt_cmd_handler_task_params_t));
    esp_err_t ret = ESP_OK;
    EventBits_t ret_bits = 0;
    uint8_t MAC[6];
    char data_topic[32];
    char cmd_topic[32];
    char MAC_str[32];

    if(task_params == NULL)
    {
        ret = ESP_ERR_NO_MEM;
        goto cleanup;
    }

    if(esp_read_mac(MAC,ESP_MAC_WIFI_STA) != ESP_OK)
    {
        ret = ESP_ERR_INVALID_MAC;
        goto cleanup;
    }

    snprintf(data_topic, sizeof(data_topic),MACSTR "/DATA", MAC2STR(MAC));
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
        goto cleanup;
    }
    
    //Task and support data var init

    mqtt_queue = xQueueCreate(5, sizeof(queue_element_t*));  //Overkill, i have space for 5 messages
    
    if(mqtt_queue == NULL)
    {
        ret =  ESP_ERR_INVALID_RESPONSE;
        goto cleanup;
    }
    
    task_params->queue = mqtt_queue;
    task_params->event_group_handle = event_group_handle;
    task_params->mqtt_handle = mqtt_handle;
    memcpy(task_params->MAC,MAC,6);
    int gen_ret = 0;
    if(ret == ESP_OK) ret = esp_mqtt_client_register_event(mqtt_handle, MQTT_EVENT_DATA, cmd_topic_event_handler,(void*) task_params);
    if(ret == ESP_OK) xTaskCreate(mqtt_cmd_consumer_task,"MQTT cmd handler task",2048,(void*) task_params,2,&consumer_task);
    if(ret == ESP_OK) ret = esp_mqtt_client_start(mqtt_handle);
    if(ret == ESP_OK) gen_ret = esp_mqtt_client_subscribe_single(mqtt_handle,cmd_topic,1);
    
    if(ret == ESP_OK && (gen_ret == -1 || gen_ret == -2))
    {
        ret =  ESP_FAIL;
        goto cleanup;
    }

    rtc_data_t rtc_data[RTC_DATA_NOMINAL_SAMPLES_COUNT];
    time_t time_now;

    get_data_sample_number(rtc_data,RTC_DATA_NOMINAL_SAMPLES_COUNT);
    json_generate_data(data_str,3000,rtc_data,RTC_DATA_NOMINAL_SAMPLES_COUNT);

    time(&time_now);
    sprintf(MAC_str,MACSTR, MAC2STR(MAC));
    int Vbatt;
    int rssi;

    if(ret == ESP_OK) ret = read_battery_voltage(&Vbatt);
    esp_wifi_sta_get_rssi(&rssi);

    if(ret == ESP_OK) ret = json_generate_telemetry(telemetry_str, MQTT_TELEMETRY_STR_SIZE, time_now, MAC_str,Vbatt,rssi, boot_time,payload_group, next_wakeup);

    if(ret == ESP_OK) ret = esp_mqtt_client_publish(mqtt_handle,data_topic, data_str, 0, 1, 0);

    while(true)
    {
        EventBits_t ret_bits_local = xEventGroupWaitBits(
            event_group_handle,
            1<<event_ack_received |
            1<<event_data_retransmit_ok |
            1<<event_error |
            1<<event_handler_task_terminated,
            pdTRUE,
            pdFALSE,
            pdMS_TO_TICKS(MQTT_EVENT_GROUP_TIMEOUT));
        ret_bits |= ret_bits_local;

        if(ret_bits_local & 1<<event_ack_received)
            break;
    }

    //Function resources clearing
    
    esp_mqtt_client_unregister_event(mqtt_handle, MQTT_EVENT_DATA, cmd_topic_event_handler);

    if(ret_bits & 1<<event_error)
        ret = ESP_FAIL;
    else
        ret = ESP_OK;
    
cleanup:
    // Cleanup in REVERSE order of allocation
    if(mqtt_queue != NULL)
        xQueueDelete(mqtt_queue);
    
    if(event_group_handle != NULL)
        xEventGroupDelete(event_group_handle);
    
    if(mqtt_handle != NULL)
        esp_mqtt_client_destroy(mqtt_handle);
    
    if(task_params != NULL)
        free(task_params);
    
    return ret;
}