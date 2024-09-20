#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#include "HAL_NET.h"
#include "HAL_LOGGER.h"

#define HAL_NET_sSSID       /*""*/""
#define HAL_NET_sPASS       /*""*/""
#define HAL_NET_nMAX_RETRY  (5)

static const char *TAG = "HAL_NET";

static void HAL_NET_l_vEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);

static uint8_t HAL_NET_u8RetryNum = 0;
esp_event_handler_instance_t HAL_NET_eInstanceAnyId;
esp_event_handler_instance_t HAL_NET_eInstanceGotId;
HAL_NET_pfvEventHandler HAL_NET_lpfvEventHandler;
bool HAL_NET_lbRegToEventHandler;

static void HAL_NET_l_vEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) 
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        HAL_LOGGER_vLog(W, TAG, "connect to the AP");
        if (HAL_NET_u8RetryNum < HAL_NET_nMAX_RETRY)
        {
            esp_wifi_connect();
            HAL_NET_u8RetryNum++;
            HAL_LOGGER_vLog(I, TAG, "retrying to connect to the AP");
        } 
        else
        {
            if(HAL_NET_lpfvEventHandler != NULL)
            {
               HAL_NET_lpfvEventHandler(HAL_NET_eDisconnected);
               HAL_NET_u8RetryNum = 0; 
            }
        } 
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        HAL_LOGGER_vLog(I, TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        HAL_NET_u8RetryNum = 0;
        if(HAL_NET_lpfvEventHandler != NULL)
        {
            HAL_NET_lpfvEventHandler(HAL_NET_eConnected); 
        }
    }
}

esp_err_t HAL_NET_eInit(void)
{
    esp_err_t eRetValue = ESP_FAIL;
    wifi_init_config_t eCfg = WIFI_INIT_CONFIG_DEFAULT();

    HAL_NET_lpfvEventHandler = NULL;
    HAL_NET_lbRegToEventHandler = false;

    eRetValue = esp_netif_init();
    if(ESP_OK == eRetValue)
    {
        eRetValue = esp_event_loop_create_default();
        if(ESP_OK == eRetValue)
        {
            esp_netif_create_default_wifi_sta();
            eRetValue = esp_wifi_init(&eCfg);
        }
    }

    return eRetValue;
}

bool HAL_NET_bEventRegister(HAL_NET_pfvEventHandler pfvEventHandler)
{ 
    HAL_NET_lbRegToEventHandler = false;
    esp_err_t eRegStatus = ESP_FAIL;

    if(NULL == HAL_NET_lpfvEventHandler && pfvEventHandler != NULL)
    {
        HAL_NET_lpfvEventHandler = pfvEventHandler;

        eRegStatus = esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &HAL_NET_l_vEventHandler,
                                                        NULL,
                                                        &HAL_NET_eInstanceAnyId);
        if (ESP_OK == eRegStatus)
        {
            HAL_LOGGER_vLog(I, TAG, "WiFi Event Handler registered");
            eRegStatus = esp_event_handler_instance_register(IP_EVENT,
                                                             IP_EVENT_STA_GOT_IP,
                                                             &HAL_NET_l_vEventHandler,
                                                             NULL,
                                                             &HAL_NET_eInstanceGotId);
            if(ESP_OK == eRegStatus)
            {
                HAL_LOGGER_vLog(I, TAG, "IP_EVENT_STA_GOT_IP REG");

                HAL_NET_lbRegToEventHandler = true;
            }
        }
    }

    return HAL_NET_lbRegToEventHandler;
}

void HAL_NET_vStart(const char *ssid, const char *pass)
{
    wifi_config_t wifi_config = {
        .sta = {

            .ssid            = HAL_NET_sSSID,
            .password        = HAL_NET_sPASS,

	     .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable  = true,
                .required = false
            },
        },
    };

    strcpy((char*)wifi_config.sta.ssid, ssid);
    strcpy((char*)wifi_config.sta.password, pass);
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    HAL_LOGGER_vLog(D, TAG, "wifi_init_sta finished.");
}

bool HAL_NET_bUnregister(HAL_NET_pfvEventHandler pfvEventHandler)
{
    bool bRetValue = false;

    if(HAL_NET_lpfvEventHandler == pfvEventHandler)
    {
        HAL_NET_lpfvEventHandler = NULL;
        HAL_NET_lbRegToEventHandler = false;

        ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, HAL_NET_eInstanceGotId));
        ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, HAL_NET_eInstanceAnyId));

        bRetValue = true;
    }

    return bRetValue;
}