#ifndef STATIC_DATA_H_
#define STATIC_DATA_H_

#include <stdio.h>
#include <string.h>
#include <synchronization.h>
#include <time.h>

struct {
time_t time;          
float PM2p5;          
float PM10p0;         
float temperature;    
float pressure;       
uint32_t payload_group;
uint32_t errors;       
uint16_t ambient_light;
uint16_t uvi;          
uint8_t humidity;     
float board_temperature;   
} typedef rtc_data_t;

void check_data();

rtc_data_t* get_raw_data();

/**
 * @brief Retrieves the most recent N sensor data samples.
 * 
 * Copies the specified number of most recent data records from the circular buffer
 * into the provided output buffer. Data is copied starting from the most recent sample.
 * Data entries with time=0 are included but implied to be discarded on the server side.
 * 
 * @param[out] rtc_data_pointer Pointer to pre-allocated buffer for storing results
 * @param[in] sample_number Number of samples to retrieve (must be < RTC_DATA_SIZE)
 * 
 * @return 1 on success, 0 if sample_number exceeds available buffer size (RTC_DATA_SIZE)
 * 
 * @note The requested samples are copied regardless of their timestamp validity.
 *       Samples with time=0 indicate empty/uninitialized entries.
 */
int get_data_sample_number(rtc_data_t* rtc_data_pointer, uint8_t sample_number);

/**
 * @brief Retrieves sensor data within a specified time interval.
 * 
 * Returns the data that fits between the specified time intervals, until it fills the buffer.
 * The caller must provide a pre-allocated buffer to store results.
 * 
 * @param[in] interval_t_beginning Start time of the interval (unix timestamp)
 * @param[in] interval_t_end End time of the interval (unix timestamp)
 * @param[out] rtc_data_pointer Pointer to pre-allocated buffer for storing results
 * @param[in] size Maximum number of records the buffer can hold
 * 
 * @return number of available samples contained in the period, including the ones not saved in the buffer.
 * 
 * @note Valid time range: 2026-2040 (1767222000 to 2208985200 unix timestamps)
 */
int get_data_interval(time_t interval_t_beginning, time_t interval_t_end, rtc_data_t* rtc_data_pointer, int size);

void store_data(sps30_task_param_t *sps30_task_param, ltr390_task_param_t *ltr390_task_param, sht40_task_param_t *sht40_task_param, bmp280_task_param_t *bmp280_task_param, EventBits_t error_mask, time_t time);


#endif