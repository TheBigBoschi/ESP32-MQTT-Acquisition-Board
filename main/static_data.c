#include <stdio.h>
#include <string.h>
#include <esp_sleep.h>
#include <synchronization.h>
#include <time.h>
#include <freertos/task.h>
#include <static_data.h>

// array used to store the readings, type defined in the header file.
// RTC_DATA_ATTR means that the data is saved in the RTC memory, that keeps it's value even in sleep.
// rtc_data[100] occupies around 4.8KB

#define RTC_DATA_SIZE 100

RTC_DATA_ATTR rtc_data_t rtc_data_array[RTC_DATA_SIZE];
static uint8_t rtc_data_array_latest_index;


/**
 * @brief Validates data integrity by checking the reset reason.
 * 
 * If the device was reset for any reason other than a deep sleep wake-up,
 * the data array is considered corrupted and is zeroed out. This ensures
 * that only data collected across consecutive sleep cycles is retained.
 * 
 * @return void
 * 
 * @note This function should be called during initialization to verify
 *       that the data array contains valid readings from the previous cycle.
 */
void check_data()
{
    // If the reset was triggered by any other reason than a deep sleep wake up the data is considered corrupted
    soc_reset_reason_t reset_reason = esp_rom_get_reset_reason(0);
    if(reset_reason != RESET_REASON_CORE_DEEP_SLEEP)
    {
        for(int i = 0;i<100;i++)
        {
            rtc_data_array[i].time = 0;
            rtc_data_array[i].PM2p5 = 0;
            rtc_data_array[i].PM10p0 = 0;
            rtc_data_array[i].temperature = 0;
            rtc_data_array[i].pressure = 0;
            rtc_data_array[i].payload_group = 0;
            rtc_data_array[i].errors = 0;
            rtc_data_array[i].ambient_light = 0;
            rtc_data_array[i].uvi = 0;
            rtc_data_array[i].humidity = 0;
            rtc_data_array[i].board_temperature = 0;
        }
    }
}

/**
 * @brief Retrieves the pointer to the static data array.
 * 
 * Provides access to the circular buffer containing sensor readings.
 * The array stores up to 100 data points and overwrites older entries
 * when the buffer is full.
 * 
 * @return rtc_data* Pointer to the first element of the static data array
 */
rtc_data_t* get_raw_data()
{
    return rtc_data_array;
}

//It's implied that data with time 0 will be discarded on the server side
int get_data_sample_number(rtc_data_t* rtc_data_pointer, uint8_t sample_number)
{
    if(sample_number >= RTC_DATA_SIZE)
        return 0;

    rtc_data_t* data[] = get_raw_data();
    memcpy(rtc_data_pointer,data,sizeof(rtc_data_t)*sample_number);
    return 1;
}

int get_data_interval(time_t interval_t_beginning, time_t interval_t_end, rtc_data_t* rtc_data_pointer, int size)
{
    rtc_data_t* data[] = get_raw_data();
    int16_t index = rtc_data_array_latest_index;
    time_t min_time = data[0]->time;
    time_t max_time = data[0]->time;
    uint8_t min_time_index = 255;
    uint8_t max_time_index = 255;
    uint8_t first_element;
    uint8_t last_element;

    if(interval_t_beginning >= interval_t_end || interval_t_end < 1767222000 || interval_t_beginning > 2208985200 )
        return -1;
    //Check if the data requested is available in memory
    for(uint8_t i = 0; i < 100; i++)
    {
        if(data[i]->time >= interval_t_beginning)
        {
            min_time = data[i]->time;
            min_time_index = i;
            break;
        }
        if(data[i]->time == 0)
            break;
    }

    for(uint8_t i = 0; i < 100; i++)
    {
        if(data[i]->time >= interval_t_end)
        {
            max_time = data[i]->time;
            max_time_index = i;
            break;
        }
        if(data[i]->time == 0)
            break;
            
    }

    if(min_time_index == 255 && max_time_index == 255)
        return -1;
    
    //At least some data available in memory. Looks for it and copies the values in the argument array

    uint8_t elements = max_time_index - min_time_index;
     
    memcpy(rtc_data_pointer,&data[min_time_index], sizeof(rtc_data_t)*(elements>size?size:elements));
    return elements;
}


/**
 * @brief Stores the current sensor readings into the data buffer.
 * 
 * Records measurements from all sensors into the buffer.
 * When the buffer is full (100 entries), the oldest data is discarded.
 * This implements a shifting buffer for continuous data acquisition.
 * 
 * @param[in] sps30_task_param Pointer to SPS30 sensor parameters (PM2.5 and PM10 readings)
 * @param[in] ltr390_task_param Pointer to LTR390 sensor parameters (ambient light and UV index)
 * @param[in] sht40_task_param Pointer to SHT40 sensor parameters (temperature and humidity)
 * @param[in] bmp280_task_param Pointer to BMP280 sensor parameters (pressure)
 * @param[in] error_mask Bit mask containing error flags from the acquisition cycle
 * @param[in] time Timestamp for the current measurement
 * 
 * @return void
 * 
 * @note The function uses a static counter that automatically wraps from 99 to 0,
 *       enabling the circular buffer behavior.
 */
void store_data(sps30_task_param_t *sps30_task_param, ltr390_task_param_t *ltr390_task_param, sht40_task_param_t *sht40_task_param, bmp280_task_param_t *bmp280_task_param, EventBits_t error_mask, time_t time)
{
    
    rtc_data_array_latest_index ++;
    rtc_data_t *data = get_raw_data();

    for(uint8_t i = 99; i>0;i--)
        memcpy(&data[i],&data[i-1], sizeof(rtc_data_t));
    
    data[0].time = time;
    data[0].PM2p5 = sps30_task_param->sps30_measurement.MC2p5;
    data[0].PM10p0 = sps30_task_param->sps30_measurement.MC10p0;;
    data[0].temperature = sht40_task_param->temperature;
    data[0].pressure = bmp280_task_param->avg_pressure;
    data[0].errors = error_mask;
    data[0].ambient_light = ltr390_task_param->ambient_light;
    data[0].uvi = ltr390_task_param->uvi;
    data[0].humidity = sht40_task_param->humidity;
    
}