#ifndef HAL_NET_H
#define HAL_NET_H

typedef enum
{
    HAL_NET_eDisconnected,
    HAL_NET_eConnected,
} HAL_NET_teConStatus;

typedef void (*HAL_NET_pfvEventHandler)(HAL_NET_teConStatus eConStatus);

esp_err_t HAL_NET_eInit(void);
bool HAL_NET_bEventRegister(HAL_NET_pfvEventHandler pfvEventHandler);
bool HAL_NET_bUnregister(HAL_NET_pfvEventHandler pfvEventHandler);
void HAL_NET_vStart(const char *ssid, const char *pass);

#endif //HAL_NET_H