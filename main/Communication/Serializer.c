#include <time.h>
#include <Static_data.h>
#include <json_generator.h>

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
uint8_t generate_data_json(char* output_str, int output_str_size, rtc_data* data, uint8_t members)
{
    json_gen_str_t jstr;
    uint8_t i;
    int ret;

    
    json_gen_str_start(&jstr,output_str,output_str_size,NULL,NULL);
    ret = json_gen_start_object(&jstr);
    
    //JSON payload generator. The floats value have been multiplied to 100 and casted to int
    //this allows me to reconstruct them to 1/100 of a unit precision, while saving me from dealing with all the floats decimals

    json_gen_push_array(&jstr, "time");
    for(i = 0; i < members; i++) if(ret == 0) ret = json_gen_arr_set_int64(&jstr, data[i].time);
    json_gen_pop_array(&jstr);

    json_gen_push_array(&jstr, "PM2p5");
    for(i = 0; i < members; i++) if(ret == 0) ret = json_gen_arr_set_int(&jstr, (int)(data[i].PM2p5*100));
    json_gen_pop_array(&jstr);

    json_gen_push_array(&jstr, "PM10p0");
    for(i = 0; i < members; i++) if(ret == 0) ret = json_gen_arr_set_int(&jstr, (int)(data[i].PM10p0*100));
    json_gen_pop_array(&jstr);

    json_gen_push_array(&jstr, "temperature");
    for(i = 0; i < members; i++) if(ret == 0) ret = json_gen_arr_set_int(&jstr, (int)(data[i].temperature*100));
    json_gen_pop_array(&jstr);

    json_gen_push_array(&jstr, "pressure");
    for(i = 0; i < members; i++) if(ret == 0) ret = json_gen_arr_set_int(&jstr, data[i].pressure);
    json_gen_pop_array(&jstr);

    json_gen_push_array(&jstr, "payload_group");
    for(i = 0; i < members; i++) if(ret == 0) ret = json_gen_arr_set_int(&jstr, data[i].payload_group);
    json_gen_pop_array(&jstr);

    json_gen_push_array(&jstr, "errors");
    for(i = 0; i < members; i++) if(ret == 0) ret = json_gen_arr_set_int(&jstr, data[i].errors);
    json_gen_pop_array(&jstr);
 
    json_gen_push_array(&jstr, "ambient_light");
    for(i = 0; i < members; i++) if(ret == 0) ret = json_gen_arr_set_int(&jstr, data[i].ambient_light);
    json_gen_pop_array(&jstr);
 
    json_gen_push_array(&jstr, "uvi");
    for(i = 0; i < members; i++) if(ret == 0) ret = json_gen_arr_set_int(&jstr, data[i].uvi);
    json_gen_pop_array(&jstr);
 
    json_gen_push_array(&jstr, "humidity");
    for(i = 0; i < members; i++) if(ret == 0) ret = json_gen_arr_set_int(&jstr, (int)(data[i].humidity*100));
    json_gen_pop_array(&jstr);
     
    json_gen_push_array(&jstr, "board_temperature");
    for(i = 0; i < members; i++) if(ret == 0) ret = json_gen_arr_set_int(&jstr, (int)data[i].board_temperature);
    json_gen_pop_array(&jstr);

    if(ret == 0) ret = json_gen_end_array(&jstr);
    if(ret == 0) ret = json_gen_end_object(&jstr);
 
    return ret;
}

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
int generate_telemetry_json(
    char* output_str,
    int output_str_size,
    time_t time,
    char* ID,
    float Vbatt,
    int rssi,
    int32_t payload_group,
    int16_t next_wakeup)
{
    json_gen_str_t jstr;
    int ret;
    
    json_gen_str_start(&jstr,output_str,output_str_size,NULL,NULL);
    ret = json_gen_start_object(&jstr);

    if(ret == 0) ret = json_gen_obj_set_int64(&jstr, "time", time);
    if(ret == 0) ret = json_gen_obj_set_string(&jstr, "ID", ID);
    if(ret == 0) ret = json_gen_obj_set_float(&jstr, "Vbatt", Vbatt);
    if(ret == 0) ret = json_gen_obj_set_int(&jstr, "rssi", rssi);
    if(ret == 0) ret = json_gen_obj_set_int64(&jstr, "payload_group", payload_group);
    if(ret == 0) ret = json_gen_obj_set_int(&jstr, "next_wakeup", next_wakeup);

    if(ret == 0) ret = json_gen_end_object(&jstr);

    return ret;
}
