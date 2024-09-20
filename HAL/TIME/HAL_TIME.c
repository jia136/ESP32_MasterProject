#include <string.h>
#include <time.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_attr.h"
#include "esp_sleep.h"
#include "nvs_flash.h"
#include "esp_sntp.h"

#include "HAL_LOGGER.h"

static const char *TAG = "HAL_TIME";

void time_sync_notification_cb(struct timeval *tv)
{
    HAL_LOGGER_vLog(I, TAG, "Notification of a time synchronization event");
    time_t now = 0;
    struct tm timeinfo = { 0 };
    char strftime_buf[64];
    time(&now);
    setenv("TZ", "CST-2", 1);
    tzset();
    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);    
}

void HAL_TIME_vGetCurrTime(char* strTimeBuf, int iStrLen)
{
    time_t now = 0;
    struct tm timeinfo = { 0 };
    time(&now);
    localtime_r(&now, &timeinfo);
    strftime(strTimeBuf, iStrLen, "%c", &timeinfo);
}

void HAL_TIME_vInit(void)
{
    HAL_LOGGER_vLog(I, TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    sntp_init();
}

