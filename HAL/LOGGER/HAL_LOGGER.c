#include "HAL_LOGGER.h"
#include "HAL_TIME.h"
#include <stdio.h>
#include <string.h>

#define HAL_LOGGER_nTIME_BUF_STR_LEN (64)


char* sLogLevels[NUM_OF_LOG_LEVELS] = {"ERROR", "WARNING","INFO", "DEBUG", "VERBOSE"};
char* sFontColor[NUM_OF_LOG_LEVELS] = { "\x1b[31m", // red
                                        "\x1b[1;33m",// yellow
                                        "\x1b[32m", // green 
                                        "\x1b[0m", // white
                                        "\x1b[0m"}; // white

void HAL_LOGGER_vLog(HAL_LOGGER_teLogLevel elogLevel, const char *sTag, const char *sMsg, ...)
{
    va_list args;
    char sTimeBuf[HAL_LOGGER_nTIME_BUF_STR_LEN];
    HAL_TIME_vGetCurrTime(sTimeBuf, (int)HAL_LOGGER_nTIME_BUF_STR_LEN);
    printf("%s", sFontColor[elogLevel]);
    printf("%s: %s - %s ", sTimeBuf, sLogLevels[elogLevel], sTag);
    
    va_start(args, sMsg);
        vprintf(sMsg, args);
    va_end(args);
    printf("\n%s", sFontColor[D]);
}
