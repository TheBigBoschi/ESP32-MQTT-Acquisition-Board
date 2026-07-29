#ifndef WIFI_H_
#define WIFI_H_

#include "esp_err.h"

/**
 * @brief Initialize WiFi connection using provisioning method
 * 
 * This function initializes the WiFi provisioning service with a captive portal.
 * It sets up the WiFi configuration with default settings and starts the provisioning
 * process. The function blocks until a WiFi connection is established.
 * 
 * @details
 * - Creates a captive portal with SSID "CONFIGURATION_AP"
 * - Waits indefinitely for a successful WiFi connection
 * - Logs connection status updates
 * 
 * @return None
 * 
 * @note This is a blocking function that will wait until WiFi connection is complete
 */
void wifi_init_connection(void);

/**
 * @brief Retrieve time from SNTP server and initialize the RTC
 * 
 * This function initializes SNTP (Simple Network Time Protocol) to synchronize
 * the device's real-time clock with an NTP server. It will only attempt synchronization
 * if the device is connected to WiFi.
 * 
 * @details
 * - Configures SNTP to use "pool.ntp.org" as the primary server
 * - Accepts NTP offers from DHCP server if available
 * - Waits up to SNTP_TIMEOUT (30 seconds) for synchronization
 * - Cleans up SNTP resources after synchronization completes
 * 
 * @return esp_err_t
 *   - @c ESP_OK: Successfully synchronized time with NTP server
 *   - @c ESP_ERR_TIMEOUT: SNTP synchronization request timed out
 *   - @c ESP_ERR_INVALID_STATE: Device is not connected to WiFi
 * 
 * @note This is a blocking function that will wait up to 30 seconds for SNTP response
 * @see wifi_init_connection() for establishing WiFi connection first
 */
esp_err_t wifi_set_time(void);

#endif