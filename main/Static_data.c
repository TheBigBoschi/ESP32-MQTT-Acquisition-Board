#include <stdio.h>
#include <string.h>
#include <esp_sleep.h>
#include <syncronization.h>
#include <time.h>
#include <freertos/task.h>
#include <Static_data.h>

// array used to store the readings, type defined in the header file.
// RTC_DATA_ATTR means that the data is saved in the RTC memory, that keeps it's value even in sleep.
// rtc_data[100] occupies around 4.8KB

RTC_DATA_ATTR rtc_data rtc_data_array[100];

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
 * @return static_data* Pointer to the first element of the static data array
 */
rtc_data* get_data()
{
    return rtc_data_array;
}

/**
 * @brief Stores the current sensor readings into the circular data buffer.
 * 
 * Records measurements from all sensors into the next position of the circular buffer.
 * When the buffer is full (100 entries), the oldest data is overwritten.
 * This implements a rolling buffer pattern for continuous data acquisition.
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
    static uint8_t counter;
    
    counter ++;
    rtc_data *data = get_data();

    //rolling array. It fills up all the slots then it overwrites the older data.
    if(counter == 100)
        counter = 0;
    
    data[counter].time = time;
    data[counter].PM2p5 = sps30_task_param->sps30_measurement.MC2p5;
    data[counter].PM10p0 = sps30_task_param->sps30_measurement.MC10p0;;
    data[counter].temperature = sht40_task_param->temperature;
    data[counter].pressure = bmp280_task_param->avg_pressure;
    data[counter].errors = error_mask;
    data[counter].ambient_light = ltr390_task_param->ambient_light;
    data[counter].uvi = ltr390_task_param->uvi;
    data[counter].humidity = sht40_task_param->humidity;
    
}