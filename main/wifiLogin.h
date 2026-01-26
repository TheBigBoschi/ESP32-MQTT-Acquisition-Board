#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "esp_wifi_manager.h"

/**
 *  @brief Initialize and connect to WiFi. If no known networks, start captive portal. 
 */
void WiFi_Login(void);

/**
 * @brief Check if WiFi is connected
 * 
 * @return true if connected with IP, false otherwise
 */
bool WiFi_Connected(void);

/**
 * @brief Get full WiFi status
 * 
 * Returns IP, Gateway, Netmask, DNS, MAC, RSSI, channel, hostname, etc.
 * 
 * @param[out] status Output status structure
 * @return ESP_OK on success
 */
esp_err_t WiFi_Get_Status(wifi_status_t* status);