#ifndef HAL_LOGGER_H
#define HAL_LOGGER_H

#include <stdarg.h>

typedef enum
{
    E, W, I, D, V, NUM_OF_LOG_LEVELS
} HAL_LOGGER_teLogLevel;

void HAL_LOGGER_vLog(HAL_LOGGER_teLogLevel elogLevel, const char *sTag, const char *sMsg, ...);

#endif //HAL_LOGGER_H
