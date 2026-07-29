#ifndef SERIALIZER_H_
#define SERIALIZER_H_

#include <stdint.h>
#include <time.h>
#include <Static_data.h>

/**
 * @brief Generates a JSON payload containing sensor data arrays.
 * 
 * Serializes multiple sensor measurements into a JSON object with array fields.
 * Floating-point values (PM2.5, PM10, temperature, humidity) are multiplied by 100
 * and cast to integers to achieve 1/100 unit precision while avoiding decimal complexity.
 * 
 * @param[out] output_str Pointer to output buffer where JSON string will be written
 * @param[in] output_str_size Size of the output buffer in bytes
 * @param[in] data Pointer to array of sensor data structures to serialize
 * @param[in] members Number of data records to include in the JSON arrays
 * 
 * @return 0 on success, non-zero error code if JSON generation fails
 * 
 * @note The function creates arrays for: time, PM2p5, PM10p0, temperature, pressure,
 *       payload_group, errors, ambient_light, uvi, humidity, and board_temperature.
 *       Generation stops on first error and subsequent operations are skipped.
 * @note Floating-point fields are scaled by 100 before serialization:
 *       - To retrieve original value: stored_value / 100.0
 *       - Example: stored value 2350 = 23.50 (original units)
 */
uint8_t json_generate_data(char* output_str, int output_str_size, rtc_data* data, uint8_t members);

/**
 * @brief Generates a JSON object containing device telemetry information.
 * 
 * Serializes device metadata including timestamp, device ID, battery voltage,
 * signal strength, payload group counter, and next scheduled wake-up time.
 * All parameters are included as JSON object fields.
 * 
 * @param[out] output_str Pointer to output buffer where JSON string will be written
 * @param[in] output_str_size Size of the output buffer in bytes
 * @param[in] time Unix timestamp of the telemetry measurement
 * @param[in] ID Device identifier string (null-terminated)
 * @param[in] Vbatt Battery voltage in volts
 * @param[in] rssi Received Signal Strength Indicator in dBm
 * @param[in] payload_group Payload group sequence number
 * @param[in] next_wakeup Seconds until next device wake-up
 * 
 * @return 0 on success, non-zero error code if JSON generation fails
 * 
 * @note The JSON object contains fields: "time" (int64), "ID" (string), "Vbatt" (float),
 *       "rssi" (int), "payload_group" (int64), and "next_wakeup" (int64).
 *       Generation stops on first error; remaining fields are not added.
 */
int json_generate_telemetry(
    char* output_str,
    int output_str_size,
    time_t time,
    char* ID,
    float Vbatt,
    int rssi,
    time_t boot_time,
    int32_t payload_group,
    int16_t next_wakeup);

/**
 * @brief Adds a string element to a JSON array being constructed.
 * 
 * Appends a string value to the current JSON array in the generator context.
 * This function is used when building arrays of string values in JSON output.
 * 
 * @param[in,out] jstr Pointer to JSON generator string context
 * @param[in] input_str Null-terminated string to add to the JSON array
 * 
 * @return 0 on success, non-zero error code if adding string to array fails
 * 
 * @note This function must be called between json_gen_push_array() and 
 *       json_gen_pop_array() calls to properly add elements to an array.
 */
int json_generate_string(char* jstr, char* input_str);

#endif