/**
 * @file transmission_manager.h
 * @brief MQTT transmission manager for ESP32 sensor data acquisition board
 * 
 * This module handles bidirectional MQTT communication, managing:
 * - Publishing sensor data and telemetry to MQTT broker
 * - Receiving commands (ACK, RETRANSMIT) from broker
 * - Battery voltage monitoring
 * - Producer-consumer pattern for async message handling
 * 
 * @author
 * @date 2026
 * @version 1.0
 */

#ifndef TRANSMISSION_MANAGER_H
#define TRANSMISSION_MANAGER_H

#include <esp_err.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initiates MQTT transmission and waits for command responses
 * 
 * This function establishes an MQTT connection with the configured broker,
 * publishes sensor data and telemetry information, and waits for acknowledgment
 * or retransmission commands. It implements a producer-consumer pattern where:
 * - The event handler (producer) receives MQTT messages and queues them
 * - The consumer task processes commands (DATA_ACK, RETRANSMIT)
 * - The main function waits for task completion via event group signals
 * 
 * **Supported Commands:**
 * - `DATA_ACK`: Acknowledges successful data reception; terminates cleanly
 * - `RETRANSMIT <init_time> - <end_time>`: Requests retransmission of data
 *   within the specified time range; handles pagination for large datasets
 * 
 * **Resource Management:**
 * All dynamically allocated resources (MQTT client, queues, event groups,
 * task parameters) are properly cleaned up in reverse allocation order,
 * even if errors occur or the consumer task exits early.
 * 
 * @param[in] sample_number Number of samples to include in transmission
 * @param[in] boot_time Timestamp of device boot for telemetry
 * @param[in] payload_group Payload group identifier for tracking
 * @param[in] next_wakeup Scheduled time of next device wake/transmission
 * 
 * @return esp_err_t
 *   - @c ESP_OK:                   Transmission successful; DATA_ACK received
 *   - @c ESP_ERR_NO_MEM:           Failed to allocate task parameters
 *   - @c ESP_ERR_INVALID_MAC:      Failed to read device MAC address
 *   - @c ESP_ERR_INVALID_RESPONSE: Failed to initialize MQTT client
 *   - @c ESP_ERR_NOT_ALLOWED:      Failed to create queue or event group
 *   - @c ESP_FAIL:                 MQTT subscription failed or event error occurred
 * 
 * @note
 * - This is a blocking call that waits up to @c MQTT_EVENT_GROUP_TIMEOUT
 *   milliseconds for the consumer task to complete
 * - The function creates and manages a background FreeRTOS task
 * - Event handler registration/unregistration is handled automatically
 * - Safe for repeated calls; all resources are cleaned up before return
 * 
 * @see read_battery_voltage()
 * @see mqtt_cmd_consumer_task()
 * @see cmd_topic_event_handler()
 */
esp_err_t transmission_manager(uint8_t sample_number, time_t boot_time, 
                               int32_t payload_group, time_t next_wakeup);

/**
 * @brief Reads and calibrates the battery voltage via ADC
 * 
 * Performs single-shot ADC conversion on the battery voltage input (GPIO39/ADC1_CH3),
 * with hardware calibration when available. The raw readings are averaged over
 * 32 samples to improve accuracy, with 1ms delay between samples to allow
 * ADC stabilization.
 * 
 * **Calibration Strategy:**
 * - Prefers curve-fitting calibration (ESP32-S2/S3/C3 and newer)
 * - Falls back to line-fitting calibration (ESP32, ESP32-S2) if unavailable
 * - Applies calibration if available; uses raw calculation as fallback
 * 
 * **Hardware Notes:**
 * - Requires GPIO39 (SENSOR_VN) to be connected to battery voltage via divider
 * - Voltage divider output must be in ADC range (0-3100 mV after attenuation)
 * - ADC attenuation set to 12dB for full ESP32 rail voltage measurement
 * - On standard PCB, GPIO12 (3.3V_E) must be toggled high to enable divider
 * 
 * @param[out] voltage Pointer to int where voltage in millivolts is stored
 * 
 * @return esp_err_t
 *   - @c ESP_OK:                 Voltage successfully read and stored
 *   - @c ESP_ERR_NOT_SUPPORTED:  Hardware calibration not supported on this chip
 *   - Other ESP_ERR_* codes from ADC driver on configuration/read failure
 * 
 * @warning
 * - Caller must not call this from interrupt context (uses vTaskDelay)
 * - Output pointer must point to valid, writable memory
 * - ADC unit 1 is exclusively used; concurrent access from other tasks
 *   may cause conflicts or incorrect readings
 * 
 * @note
 * - Function is blocking; approximately 32ms delay for sample averaging
 * - All ADC resources (handle, calibration) are created and destroyed
 *   within this function; no persistent state
 * - Voltage calculation: `(accumulator / 32) * 2450 / 4096` or calibrated equivalent
 * 
 * @see transmission_manager()
 */
esp_err_t read_battery_voltage(int* voltage);

#ifdef __cplusplus
}
#endif

#endif /* TRANSMISSION_MANAGER_H */
