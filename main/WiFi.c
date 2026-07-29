#include <stdio.h>
#include <string.h>
#include "wifi_provisioner.h"
#include "esp_log.h"
#include "esp_sntp.h"
#include <time.h>
#include <sys/time.h>
#include <esp_sntp.h>
#include "esp_netif_sntp.h"
//#include "driver/gpio.h"


#define SNTP_TIMEOUT 30000

static const char *TAG = "WIFI";

static void on_connected(void)
{
    ESP_LOGI(TAG, "WiFi connected!");
}

static void on_portal_start(void)
{
    ESP_LOGI(TAG, "Captive portal started — connect to the AP to configure WiFi.");
}

void wifi_init_connection(void)
{
    wifi_prov_config_t config = WIFI_PROV_DEFAULT_CONFIG();
    config.ap_ssid        = "CONFIGURATION_AP";
    config.on_connected   = on_connected;
    config.on_portal_start = on_portal_start;

    ESP_ERROR_CHECK(wifi_prov_start(&config));

    /* Block until we have a WiFi connection */
    ESP_ERROR_CHECK(wifi_prov_wait_for_connection(portMAX_DELAY));

    ESP_LOGI(TAG, "Connected");
}
/*
* @brief retrieve the time from a SNTP server and initializes the RTC
*/
esp_err_t wifi_set_time()
{
    ESP_LOGI(TAG, "Initializing SNTP");
    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    config.server_from_dhcp = true;             // accept NTP offers from DHCP server, needs enabling the retrieval of server from the DHCP.
                                                // From menuconfig navigate to Component config > LWIP > SNTP > Request NTP servers from DHCP and enable it.
    config.renew_servers_after_new_IP = true;   // let esp-netif update configured SNTP server(s) after receiving DHCP lease
    config.index_of_first_server = 0;           // updates from server num 1, leaving server 0 (from DHCP) intact
    config.start = true;

    if(wifi_prov_is_connected() == true)
    {
        esp_netif_sntp_init(&config);

        if(esp_netif_sntp_sync_wait(pdMS_TO_TICKS(SNTP_TIMEOUT)) == ESP_ERR_TIMEOUT)
        {
            ESP_LOGI(TAG, "SNTP request timeout");
            return ESP_ERR_TIMEOUT;
        }
        else{
            esp_netif_sntp_deinit();
            return ESP_OK;    
        }
    }
    return ESP_ERR_INVALID_STATE;
}