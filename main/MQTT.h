static void log_error_if_nonzero(const char *message, int error_code);

void MQTT_Config();

static void MQTT_Event_Handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);

int  MQTT_Deinit();

int MQTT_Subscribe(char *topic, int qos);

void MQTT_Unsubscribe(char *topic);

void MQTT_Publish(char *topic, char *payload,int qos);