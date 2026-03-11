/*
*   Sensiron sps30 library.
*/
#ifndef SPS30_H_
#define SPS30_H_

#include <stdint.h>
#include <stdbool.h>
#include <driver/i2c_master.h>


typedef struct {
    float MC1p0;
    float MC2p5;
    float MC4p0;
    float MC10p0;
    float NC0p5;
    float NC1p0;
    float NC2p5;
    float NC4p0;
    float NC10p0;
    float TypicalParticleSize;
} sps30_measurement_float_t;

typedef struct {
    uint16_t MC1p0;
    uint16_t MC2p5;
    uint16_t MC4p0;
    uint16_t MC10p0;
    uint16_t NC0p5;
    uint16_t NC1p0;
    uint16_t NC2p5;
    uint16_t NC4p0;
    uint16_t NC10p0;
    uint16_t TypicalParticleSize;
} sps30_measurement_uint16_t;

/**
 * @brief Starts measurement in float mode.
 *
 * @param dev_handle I2C device handle.
 * @return esp_err_t ESP_OK on success, otherwise error code.
 */
esp_err_t sps30_start_measurement_float(i2c_master_dev_handle_t dev_handle);

/**
 * @brief Starts measurement in uint16 mode.
 *
 * @param dev_handle I2C device handle.
 * @return esp_err_t ESP_OK on success, otherwise error code.
 */
esp_err_t sps30_start_measurement_uint16(i2c_master_dev_handle_t dev_handle);

/**
 * @brief Stops the measurement.
 *
 * @param dev_handle I2C device handle.
 * @return esp_err_t ESP_OK on success, otherwise error code.
 */
esp_err_t sps30_stop_measurement(i2c_master_dev_handle_t dev_handle);

/**
 * @brief Reads the data ready flag.
 *
 * @param dev_handle I2C device handle.
 * @param[out] stat Pointer to store the status (true if data ready).
 * @return esp_err_t ESP_OK on success, otherwise error code.
 */
esp_err_t sps30_read_data_ready_flag(i2c_master_dev_handle_t dev_handle, bool* stat);

/**
 * @brief Reads measured values in float format.
 *
 * @param dev_handle I2C device handle.
 * @param[out] measurement Pointer to structure to store float measurements.
 * @return esp_err_t ESP_OK on success, otherwise error code.
 */
esp_err_t sps30_read_measured_values_float(i2c_master_dev_handle_t dev_handle, sps30_measurement_float_t* measurement);

/**
 * @brief Reads measured values in uint16 format.
 *
 * @param dev_handle I2C device handle.
 * @param[out] measurement Pointer to structure to store uint16 measurements.
 * @return esp_err_t ESP_OK on success, otherwise error code.
 */
esp_err_t sps30_read_measured_values_uint16(i2c_master_dev_handle_t dev_handle, sps30_measurement_uint16_t* measurement);

/**
 * @brief Puts the SPS30 into sleep mode.
 *
 * @param dev_handle I2C device handle.
 * @return esp_err_t ESP_OK on success, otherwise error code.
 */
esp_err_t sps30_sleep(i2c_master_dev_handle_t dev_handle);

/**
 * @brief Wakes up the SPS30 from sleep mode.
 *
 * @param dev_handle I2C device handle.
 * @return esp_err_t ESP_OK on success, otherwise error code.
 */
esp_err_t sps30_wake_up(i2c_master_dev_handle_t dev_handle);

/**
 * @brief Starts the fan cleaning manually.
 *
 * @param dev_handle I2C device handle.
 * @return esp_err_t ESP_OK on success, otherwise error code.
 */
esp_err_t sps30_start_fan_cleaning(i2c_master_dev_handle_t dev_handle);

/**
 * @brief Reads the auto cleaning interval.
 *
 * @param dev_handle I2C device handle.
 * @param[out] interval Pointer to store the cleaning interval in seconds.
 * @return esp_err_t ESP_OK on success, otherwise error code.
 */
esp_err_t sps30_read_auto_cleaning_interval(i2c_master_dev_handle_t dev_handle, uint32_t* interval);

/**
 * @brief Writes the auto cleaning interval.
 *
 * @param dev_handle I2C device handle.
 * @param interval Cleaning interval in seconds.
 * @return esp_err_t ESP_OK on success, otherwise error code.
 */
esp_err_t sps30_write_auto_cleaning_interval(i2c_master_dev_handle_t dev_handle, uint32_t interval);

/**
 * @brief Reads the product type. Always return 00008000.
 *
 * @param dev_handle I2C device handle.
 * @param prod_type Product type. always 8 byte size, not null-terminated.
 * @return esp_err_t ESP_OK on success, otherwise error code.
 */
esp_err_t sps30_read_prod_type(i2c_master_dev_handle_t dev_handle, char prod_type[8]);

/**
 * @brief Reads the device serial number as a string. Null charachter terminated.
 *
 * @param dev_handle I2C device handle.
 * @param[out] SerialNumber Buffer to store the serial number (must be at least 32 bytes).
 * @return esp_err_t ESP_OK on success, otherwise error code.
 */
esp_err_t sps30_read_serial_number(i2c_master_dev_handle_t dev_handle, char SerialNumber[32]);

/**
 * @brief Reads the software version.
 *
 * @param dev_handle I2C device handle.
 * @param[out] FWmajor Pointer to store major version.
 * @param[out] FWminor Pointer to store minor version.
 * @return esp_err_t ESP_OK on success, otherwise error code.
 */
esp_err_t sps30_read_sw_version(i2c_master_dev_handle_t dev_handle, uint8_t* FWmajor, uint8_t* FWminor);

/**
 * @brief Reads the device status register.
 *
 * @param dev_handle I2C device handle.
 * @param[out] status Pointer to store the status register value.
 * @return esp_err_t ESP_OK on success, otherwise error code.
 */
esp_err_t sps30_read_status_register(i2c_master_dev_handle_t dev_handle, uint32_t* status);

/**
 * @brief Clears the SPS30 internal status register
 * 
 * @param dev_handle SPS30 I2C device handle.
 * @return esp_err_t ESP_OK on success, otherwise error code.
 */
esp_err_t sps30_clear_status_register(i2c_master_dev_handle_t dev_handle);

/**
 * @brief Send a device reset command to the SPS30
 * 
 * @param dev_handle SPS30 I2C device handle.
 * @return esp_err_t ESP_OK on success, otherwise error code.
 */
esp_err_t sps30_reset(i2c_master_dev_handle_t dev_handle);

/**
 * @brief Initialize the SPS30 sensor on the I2C master bus.
 * 
 * @note The I2C bus must be initialized using i2c_new_master_bus() before calling this function.
 * 
 * @param bus_handle  I2C bus handle.
 * @param[out] dev_handle  Pointer to store the SPS30 device handle.
 * @param I2C_CLK_FREQ_HZ Bus frequency in Hz (Max 100000).
 * @return esp_err_t ESP_OK on success, otherwise error code.
 */
esp_err_t sps30_init(
    i2c_master_bus_handle_t bus_handle,
    i2c_master_dev_handle_t *dev_handle,
    uint32_t I2C_CLK_FREQ_HZ
);

/**
 * @brief Deinitialize the SPS30 on the I2C master bus.
 * 
 * @param dev_handle SPS30 device handle.
 * @return esp_err_t ESP_OK on success, otherwise error code.
 */
esp_err_t sps30_deinit(i2c_master_dev_handle_t dev_handle);

#endif /* SPS30_H_ */
