#include "HAL_TWAI.h"
#include "HAL_LOGGER.h"

#define UNKNOWN_STATE (17)

static char* sTag = "TWAI";

static twai_status_info_t eStatus;

uint32_t HAL_TWAI_u32GetState(void)
{
    uint32_t u32Ret = UNKNOWN_STATE;
    if(twai_get_status_info(&eStatus) == ESP_OK)
    {
        u32Ret = (uint32_t)eStatus.state;
    }

    HAL_LOGGER_vLog(D, sTag, "State: %d", u32Ret);

    return u32Ret;
}

bool HAL_TWAI_bConfigureCan(int8_t i8TxPin, int8_t i8RxPin, 
                            HAL_TWAI_teCanSpeed eSpeed,
                            uint32_t i32AcceptanceCode, uint32_t i32AcceptancMask)
{
    bool bRetVal = true;
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_21, GPIO_NUM_22, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if((i8TxPin > GPIO_NUM_NC && i8TxPin < GPIO_NUM_MAX) &&
        (i8RxPin > GPIO_NUM_NC && i8RxPin < GPIO_NUM_MAX))
    {
        g_config.tx_io = i8TxPin;
        g_config.rx_io = i8RxPin;

        if(eSpeed != HAL_TWAI_e250KB && eSpeed != HAL_TWAI_e500KB)
        {
            bRetVal = false;
        }
        else
        {
            if(eSpeed == HAL_TWAI_e250KB)
            {
                t_config.brp = 16;
            }
            else
            {
                t_config.brp = 8;
            }

            f_config.acceptance_code = i32AcceptanceCode;
            f_config.acceptance_mask = i32AcceptancMask;

            if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) 
            {
                HAL_LOGGER_vLog(D, sTag, "Driver installed");
                
            } 
            else 
            {
                HAL_LOGGER_vLog(D, sTag, "Failed to install driver");
                bRetVal = false;
            }
        }
    }
    else
    {
        bRetVal = false;
    }

    return bRetVal;
}

bool HAL_TWAI_bStartCAN(void)
{
    bool bRetVal = true;
    if (twai_start() == ESP_OK)
    {
        HAL_LOGGER_vLog(D, sTag, "Driver started");
    }
    else 
    {
        HAL_LOGGER_vLog(D, sTag, "Failed to start driver");
        bRetVal = false;        
    }
    return bRetVal;
}

bool HAL_TWAI_bStopCAN(void)
{
    bool bRetVal = true;
    if (twai_stop() == ESP_OK)
    {
        HAL_LOGGER_vLog(D, sTag, "Driver stopped");
        if (twai_driver_uninstall() == ESP_OK) 
        {
            HAL_LOGGER_vLog(D, sTag, "Driver uninstalled");
        }   
        else 
        {
            HAL_LOGGER_vLog(D, sTag, "Failed to uninstall driver");
            bRetVal = false;
        }
    }
    else 
    {
        HAL_LOGGER_vLog(D, sTag, "Failed to stop driver");
        bRetVal = false;        
    }
    return bRetVal;
}

void HAL_TWAI_vCheckAlerts(void)
{
    uint32_t ui32AlertsTriggered;
    twai_read_alerts(&ui32AlertsTriggered, 0);
    HAL_LOGGER_vLog(D, sTag, "Alerts: %d", ui32AlertsTriggered);

    if (ui32AlertsTriggered & TWAI_ALERT_BUS_OFF)
    {
        HAL_LOGGER_vLog(D, sTag, "Bus off state");
        (void)twai_initiate_recovery();
    }
}

void HAL_TWAI_vCanMainTask(void)
{
    HAL_TWAI_vCheckAlerts();
}