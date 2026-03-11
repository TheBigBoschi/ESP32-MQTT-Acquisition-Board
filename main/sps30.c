/*
*   Sensiron sps30 library.
*/

#include <stdint.h>
#include "sps30.h"
#include <driver/i2c_master.h>
#include <string.h>


#define CMD_START_MEASUREMENT       0x0010
#define CMD_STOP_MEASUREMENT        0x0104
#define CMD_R_DATA_READY_FLAG       0x0202
#define CMD_R_MEASURED_VALUES       0x0300
#define CMD_SLEEP                   0x1001
#define CMD_WAKE_UP                 0x1103
#define CMD_START_FAN_CLEANING      0x5607
#define CMD_RW_CLEANING_INTERVAL    0x8004
#define CMD_R_PRODUCT_TYPE          0xD002
#define CMD_R_SN                    0xD033
#define CMD_R_VERSION               0xD100
#define CMD_R_DEVICE_STAT_REG       0xD206
#define CMD_CLR_DEVICE_STAT_REG     0xD210
#define CMD_RESET                   0xD304

/**
 * @brief Calculates the CRC for a 2-byte buffer using the Sensirion algorithm.
 *
 * @param buffer Array containing the 2 bytes to calculate CRC for.
 * @return uint8_t The calculated CRC.
 */
static uint8_t sps30_calculate_crc(uint8_t buffer[2])
{
    uint8_t crc = 0xFF;
    for(int i = 0; i < 2; i++) {
        crc ^= buffer[i];
        for(uint8_t bit = 8; bit > 0; --bit) {
            if(crc & 0x80)
                crc = (crc << 1) ^ 0x31u;
            else
                crc = (crc << 1);
        }
    }
    return crc;
}

/**
 * @brief Checks the CRC of a received buffer.
 *
 * @param buffer Buffer containing data and CRC checksums.
 * @param lenght Length of the buffer (should be multiple of 3).
 * @return uint8_t 0 if CRC is valid, 1 otherwise.
 */
static uint8_t sps30_check_crc(uint8_t buffer[], int length)
{
    for(int pos=0;pos*3<length;pos++)
        if(buffer[pos*3+2] != sps30_calculate_crc(&buffer[pos*3]))
            return 1;

    return 0;
}

/**
 * @brief Converts raw bytes from SPS30 frame to float.
 *
 * @param data Pointer to the data buffer (expects 2 bytes + CRC + 2 bytes).
 * @return float The converted float value.
 */
static float sps30_bytes_to_float(uint8_t* data) {
    uint32_t val = ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) | 
                   ((uint32_t)data[3] << 8)  | (uint32_t)data[4];
    float f;
    memcpy(&f, &val, 4);
    return f;
}

/**
 * @brief Starts measurement in float mode.
 *
 * @param dev_handle I2C device handle.
 * @return esp_err_t ESP_OK on success, otherwise error code.
 */
esp_err_t sps30_start_measurement_float(i2c_master_dev_handle_t dev_handle)
{
    uint8_t cmd[] = {
        CMD_START_MEASUREMENT >> 8,
        CMD_START_MEASUREMENT & 0xFF,
        0x03,
        0x00,
        0xAC
    };
    return i2c_master_transmit(dev_handle, cmd, 5,100);
}

/**
 * @brief Starts measurement in uint16 mode.
 *
 * @param dev_handle I2C device handle.
 * @return esp_err_t ESP_OK on success, otherwise error code.
 */
esp_err_t sps30_start_measurement_uint16(i2c_master_dev_handle_t dev_handle)
{
    uint8_t cmd[] = {
        CMD_START_MEASUREMENT >> 8,
        CMD_START_MEASUREMENT & 0xFF,
        0x05,
        0x00,
        0xF6
    };
    return i2c_master_transmit(dev_handle, cmd, 5,100);
}

/**
 * @brief Stops the measurement.
 *
 * @param dev_handle I2C device handle.
 * @return esp_err_t ESP_OK on success, otherwise error code.
 */
esp_err_t sps30_stop_measurement(i2c_master_dev_handle_t dev_handle)
{
    uint8_t cmd[2] = {
        CMD_STOP_MEASUREMENT >> 8,
        CMD_STOP_MEASUREMENT & 0xFF
    };
    return i2c_master_transmit(dev_handle, cmd, 2,100);
}

/**
 * @brief Reads the data ready flag.
 *
 * @param dev_handle I2C device handle.
 * @param[out] stat Pointer to store the status (true if data ready).
 * @return esp_err_t ESP_OK on success, otherwise error code.
 */
esp_err_t sps30_read_data_ready_flag(i2c_master_dev_handle_t dev_handle, bool* stat)
{
    esp_err_t ret;
    uint8_t data[3];
    uint8_t cmd[2] = {
        CMD_R_DATA_READY_FLAG >> 8,
        CMD_R_DATA_READY_FLAG & 0xFF
    };

    ret = i2c_master_transmit(dev_handle, cmd, 2,100);
    if(ret != ESP_OK)
        return ret;
    
    ret = i2c_master_receive(dev_handle,data,3,100);
    if(ret != ESP_OK)
        return ret;

    if(data[2] != sps30_calculate_crc(&data[0]))
        return ESP_ERR_INVALID_RESPONSE;
    if((((uint16_t) data[0] << 8) | (uint16_t) data[1]) != 0)
        *stat = true;
    else
        *stat = false;
    
    return ESP_OK;
}

/**
 * @brief Reads measured values in float format.
 *
 * @param dev_handle I2C device handle.
 * @param[out] measurement Pointer to structure to store float measurements.
 * @return esp_err_t ESP_OK on success, otherwise error code.
 */
esp_err_t sps30_read_measured_values_float(i2c_master_dev_handle_t dev_handle, sps30_measurement_float_t* measurement)
{
    esp_err_t ret;
    uint8_t data[60];
    uint8_t cmd[2] = {
        CMD_R_MEASURED_VALUES >> 8,
        CMD_R_MEASURED_VALUES & 0xFF
    };

    if(measurement == NULL)
        return ESP_ERR_INVALID_ARG;

    ret = i2c_master_transmit(dev_handle, cmd, 2,100);
    if( ret != ESP_OK)
        return ret;
    
    ret = i2c_master_receive(dev_handle,data,60,100);
    if( ret != ESP_OK)
        return ret;

    if(sps30_check_crc(data,60) == 1)
        return ESP_ERR_INVALID_RESPONSE;
    
    measurement->MC1p0 = sps30_bytes_to_float(&data[0]);
    measurement->MC2p5 = sps30_bytes_to_float(&data[6]);
    measurement->MC4p0 = sps30_bytes_to_float(&data[12]);
    measurement->MC10p0 = sps30_bytes_to_float(&data[18]);

    measurement->NC0p5 = sps30_bytes_to_float(&data[24]);  
    measurement->NC1p0 = sps30_bytes_to_float(&data[30]);   
    measurement->NC2p5 = sps30_bytes_to_float(&data[36]);   
    measurement->NC4p0 = sps30_bytes_to_float(&data[42]);   
    measurement->NC10p0 = sps30_bytes_to_float(&data[48]);

    measurement->TypicalParticleSize = sps30_bytes_to_float(&data[54]);   

    return ESP_OK;
}

/**
 * @brief Reads measured values in uint16 format.
 *
 * @param dev_handle I2C device handle.
 * @param[out] measurement Pointer to structure to store uint16 measurements.
 * @return esp_err_t ESP_OK on success, otherwise error code.
 */
esp_err_t sps30_read_measured_values_uint16(i2c_master_dev_handle_t dev_handle, sps30_measurement_uint16_t* measurement)
{
    esp_err_t ret;
    uint8_t data[30];
    uint8_t cmd[2] = {
        CMD_R_MEASURED_VALUES >> 8,
        CMD_R_MEASURED_VALUES & 0xFF
    };

    if(measurement == NULL)
        return ESP_ERR_INVALID_ARG;

    ret = i2c_master_transmit(dev_handle, cmd, 2,100);
    if( ret != ESP_OK)
        return ret;
    
    ret = i2c_master_receive(dev_handle,data,30,100);
    
    if( ret != ESP_OK)
        return ret;
    if(sps30_check_crc(data,30) == 1)
        return ESP_ERR_INVALID_RESPONSE;
    
    measurement->MC1p0 =
        ((uint16_t)data[0] << 8)  |
        ((uint16_t)data[1]);    

    measurement->MC2p5 =    
        ((uint16_t)data[3] << 8)  |
        ((uint16_t)data[4]);
    
    measurement->MC4p0 =    
        ((uint16_t)data[6] << 8)  |
        ((uint16_t)data[7]);
    
    measurement->MC10p0 =    
        ((uint16_t)data[9] << 8)  |
        ((uint16_t)data[10]); 
    
    measurement->NC0p5 =    
        ((uint16_t)data[12] << 8)  |
        ((uint16_t)data[13]);   

    measurement->NC1p0 =    
        ((uint16_t)data[15] << 8)  |
        ((uint16_t)data[16]);

    measurement->NC2p5 =    
        ((uint16_t)data[18] << 8)  |
        ((uint16_t)data[19]); 

    measurement->NC4p0 =    
        ((uint16_t)data[21] << 8)  |
        ((uint16_t)data[22]);

    measurement->NC10p0 =    
        ((uint16_t)data[24] << 8)  |
        ((uint16_t)data[25]);
            
    measurement->TypicalParticleSize =    
        ((uint16_t)data[27] << 8)  |
        ((uint16_t)data[28]);

    return ESP_OK;
}

/**
 * @brief Puts the SPS30 into sleep mode.
 *
 * @param dev_handle I2C device handle.
 * @return esp_err_t ESP_OK on success, otherwise error code.
 */
esp_err_t sps30_sleep(i2c_master_dev_handle_t dev_handle)
{
    uint8_t cmd[] = {
        CMD_SLEEP >> 8,
        CMD_SLEEP & 0xFF,
    };
    return i2c_master_transmit(dev_handle, cmd, 2,100);
}

/**
 * @brief Wakes up the SPS30 from sleep mode.
 *
 * @param dev_handle I2C device handle.
 * @return esp_err_t ESP_OK on success, otherwise error code.
 */
esp_err_t sps30_wake_up(i2c_master_dev_handle_t dev_handle)
{
    uint8_t cmd[] = {
        CMD_WAKE_UP >> 8,
        CMD_WAKE_UP & 0xFF,
    };
    return i2c_master_transmit(dev_handle, cmd, 2,100);
}

/**
 * @brief Starts the fan cleaning manually.
 *
 * @param dev_handle I2C device handle.
 * @return esp_err_t ESP_OK on success, otherwise error code.
 */
esp_err_t sps30_start_fan_cleaning(i2c_master_dev_handle_t dev_handle)
{
    uint8_t cmd[] = {
        CMD_START_FAN_CLEANING >> 8,
        CMD_START_FAN_CLEANING & 0xFF,
    };
    return i2c_master_transmit(dev_handle, cmd, 2,100);
}

/**
 * @brief Reads the auto cleaning interval.
 *
 * @param dev_handle I2C device handle.
 * @param[out] interval Pointer to store the cleaning interval in seconds.
 * @return esp_err_t ESP_OK on success, otherwise error code.
 */
esp_err_t sps30_read_auto_cleaning_interval(i2c_master_dev_handle_t dev_handle, uint32_t* interval)
{
    esp_err_t ret;
    uint8_t data[6];

    if (interval == NULL)
        return ESP_ERR_INVALID_ARG;

    uint8_t cmd[] = {
        CMD_RW_CLEANING_INTERVAL >> 8,
        CMD_RW_CLEANING_INTERVAL & 0xFF,
    };

    ret = i2c_master_transmit(dev_handle, cmd, 2,100);
    if( ret != ESP_OK)
        return ret;
    
    ret = i2c_master_receive(dev_handle,data,6,100);
    
    if( ret != ESP_OK)
        return ret;
    if(sps30_check_crc(data,6) == 1)
        return ESP_ERR_INVALID_CRC;
    
    *interval =
        ((uint32_t)data[0] << 24) |
        ((uint32_t)data[1] << 16) |
        (uint32_t)data[3] << 8  |
        (uint32_t)data[4];

    return ESP_OK;
}

/**
 * @brief Writes the auto cleaning interval.
 *
 * @param dev_handle I2C device handle.
 * @param interval Cleaning interval in seconds.
 * @return esp_err_t ESP_OK on success, otherwise error code.
 */
esp_err_t sps30_write_auto_cleaning_interval(i2c_master_dev_handle_t dev_handle, uint32_t interval)
{
    esp_err_t ret;
    uint8_t cmd[8];

    cmd[0] = CMD_RW_CLEANING_INTERVAL >> 8;
    cmd[1] = CMD_RW_CLEANING_INTERVAL & 0xFF;
    cmd[2] = (uint8_t)(interval >> 24);
    cmd[3] = (uint8_t)(interval >> 16);
    cmd[4] = sps30_calculate_crc(&cmd[2]);
    cmd[5] = (uint8_t)(interval >> 8);
    cmd[6] = (uint8_t)(interval);
    cmd[7] = sps30_calculate_crc(&cmd[5]);

    ret = i2c_master_transmit(dev_handle, cmd, 8, 100);
    if( ret != ESP_OK)
        return ret;

    return ESP_OK;
};

/**
 * @brief Reads the product type. Always return 00008000.
 *
 * @param dev_handle I2C device handle.
 * @param prod_type Product type. always 8 byte size, not null-terminated.
 * @return esp_err_t ESP_OK on success, otherwise error code.
 */
esp_err_t sps30_read_prod_type(i2c_master_dev_handle_t dev_handle, char prod_type[8])
{
    esp_err_t ret;
    uint8_t data[12];

    uint8_t cmd[] = {
        CMD_R_PRODUCT_TYPE >> 8,
        CMD_R_PRODUCT_TYPE & 0xFF,
    };
    
    ret = i2c_master_transmit(dev_handle, cmd, 2, 100);
    if( ret != ESP_OK)
        return ret;

    ret = i2c_master_receive(dev_handle, data, 12,100);
    if( ret != ESP_OK)
        return ret;
    
    if(sps30_check_crc(data,12) == 1)
        return ESP_ERR_INVALID_CRC;
    
    for(int i = 0, j = 0; i<8;i+=2, j+=3)
    {
        prod_type[i] = data[j];
        prod_type[i+1] = data[j+1];
    }   
    
    return ESP_OK;
}

/**
 * @brief Reads the device serial number as a string. Null charachter terminated.
 *
 * @param dev_handle I2C device handle.
 * @param[out] SerialNumber Buffer to store the serial number (must be at least 32 bytes).
 * @return esp_err_t ESP_OK on success, otherwise error code.
 */
esp_err_t sps30_read_serial_number(i2c_master_dev_handle_t dev_handle, char SerialNumber[32])
{
    esp_err_t ret;
    uint8_t data[48];

    uint8_t cmd[] = {
        CMD_R_SN >> 8,
        CMD_R_SN & 0xFF,
    };

    ret = i2c_master_transmit(dev_handle, cmd, 2, 100);
    if( ret != ESP_OK)
        return ret;

    ret = i2c_master_receive(dev_handle, data, 48,100);
    if( ret != ESP_OK)
        return ret;
    
    if(sps30_check_crc(data,48) == 1)
        return ESP_ERR_INVALID_CRC;
    
    for(uint8_t i = 0, j = 0; i<32;i+=2, j+=3){
        SerialNumber[i] = data[j];
        SerialNumber[i+1] = data[j+1]; 
    }

    return ESP_OK;
}

/**
 * @brief Reads the software version.
 *
 * @param dev_handle I2C device handle.
 * @param[out] FWmajor Pointer to store major version.
 * @param[out] FWminor Pointer to store minor version.
 * @return esp_err_t ESP_OK on success, otherwise error code.
 */
esp_err_t sps30_read_sw_version(i2c_master_dev_handle_t dev_handle, uint8_t* FWmajor, uint8_t* FWminor)
{
    esp_err_t ret;
    uint8_t data[3];

    uint8_t cmd[] = {
        CMD_R_VERSION >> 8,
        CMD_R_VERSION & 0xFF,
    };
    
    if(FWmajor == NULL || FWminor == NULL)
        return ESP_ERR_INVALID_ARG;
        
    ret = i2c_master_transmit(dev_handle, cmd, 2, 100);
    if( ret != ESP_OK)
        return ret;

    ret = i2c_master_receive(dev_handle, data, 3,100);
    if( ret != ESP_OK)
        return ret;
    
    if(sps30_check_crc(data,3) == 1)
        return ESP_ERR_INVALID_CRC;
    
    *FWmajor = data[0]; 
    *FWminor = data[1]; 

    return ESP_OK;
}

/**
 * @brief Reads the device status register.
 *
 * @param dev_handle I2C device handle.
 * @param[out] status Pointer to store the status register value.
 * @return esp_err_t ESP_OK on success, otherwise error code.
 */
esp_err_t sps30_read_status_register(i2c_master_dev_handle_t dev_handle, uint32_t* status)
{
    esp_err_t ret;
    uint8_t data[6];

    uint8_t cmd[] = {
        CMD_R_DEVICE_STAT_REG >> 8,
        CMD_R_DEVICE_STAT_REG & 0xFF,
    };
    
    if(status == NULL)
        return ESP_ERR_INVALID_ARG;

    ret = i2c_master_transmit(dev_handle, cmd, 2, 100);
    if( ret != ESP_OK)
        return ret;

    ret = i2c_master_receive(dev_handle, data, 6,100);
    if( ret != ESP_OK)
        return ret;
    
    if(sps30_check_crc(data,6) == 1)
        return ESP_ERR_INVALID_CRC;
    
    *status =
        ((uint8_t) data[0] << 24)  |
        ((uint8_t) data[1] << 16)  |
        ((uint8_t) data[3] << 8)   |
        (uint8_t) data[4];

    return ESP_OK;
}

/**
 * @brief Clears the SPS30 internal status register
 * 
 * @param dev_handle SPS30 I2C device handle.
 * @return esp_err_t ESP_OK on success, otherwise error code.
 */
esp_err_t sps30_clear_status_register(i2c_master_dev_handle_t dev_handle)
{

    uint8_t cmd[] = {
        CMD_CLR_DEVICE_STAT_REG >> 8,
        CMD_CLR_DEVICE_STAT_REG & 0xFF,
    };

    return i2c_master_transmit(dev_handle, cmd, 2, 100);
}

/**
 * @brief Send a device reset command to the SPS30
 * 
 * @param dev_handle SPS30 I2C device handle.
 * @return esp_err_t ESP_OK on success, otherwise error code.
 */
esp_err_t sps30_reset(i2c_master_dev_handle_t dev_handle)
{

    uint8_t cmd[] = {
        CMD_RESET >> 8,
        CMD_RESET & 0xFF,
    };

    return i2c_master_transmit(dev_handle, cmd, 2, 100);
}

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
)
{
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x69,
        .scl_speed_hz = I2C_CLK_FREQ_HZ,
    };

    return i2c_master_bus_add_device(bus_handle, &dev_config, dev_handle); 
}

/**
 * @brief Deinitialize the SPS30 on the I2C master bus.
 * 
 * @param dev_handle SPS30 device handle.
 * @return esp_err_t ESP_OK on success, otherwise error code.
 */
esp_err_t sps30_deinit(i2c_master_dev_handle_t dev_handle)
{
    return i2c_master_bus_rm_device(dev_handle);
}
