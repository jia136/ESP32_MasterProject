#ifndef HAL_TWAI_H
#define HAL_TWAI_H

#include "inttypes.h"
#include "driver/twai.h"
#include "driver/gpio.h"

typedef twai_message_t tCanFrame;

typedef void(*HAL_TWAI_pfvRxMsgNotif)(tCanFrame eCanFrame);

typedef enum
{
    HAL_TWAI_e250KB,
    HAL_TWAI_e500KB
} HAL_TWAI_teCanSpeed;

uint32_t HAL_TWAI_u32GetState(void);
bool HAL_TWAI_bConfigureCan(int8_t i8TxPin, int8_t i8RxPin, 
                            HAL_TWAI_teCanSpeed eSpeed,
                            uint32_t acceptance_code, uint32_t acceptance_mask);
bool HAL_TWAI_bStartCAN(void);
bool HAL_TWAI_bStopCAN(void);
void HAL_TWAI_vCheckAlerts(void);
void HAL_TWAI_vCanMainTask(void);

#endif //HAL_TWAI_H