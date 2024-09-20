/**Libreries */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include "cJSON.h"
#include "driver/twai.h"

#include "HAL_NET.h"
#include "HAL_TIME.h"
#include "HAL_LOGGER.h"
#include "HAL_TWAI.h"

/** Macro functions */
#define GET_UPPER_NIBBLE(num)               ((num&0xF0) >> 4)
#define GET_LOWER_NIBBLE(num)               (num&0x0F)
#define ENG_SPEED_COVERSION_FORMULA(A,B)    ((256.0*A + B)/4)
#define FUEL_PRES_COVERSION_FORMULA(A)      (3*A)

/** Defines */
#define MAX_INIT_CONN_ERR_CNT               (5)
#define CYCLIC_PERIOD                       (100)
#define NUM_OF_CAN_PROTOCOLS                (2)
#define CAN_RX_TIMEOUT                      (1 * 1000)
#define VIN_LEN                             (20)
#define MAX_NUM_OF_SAMPLES                  (4)
#define DONOT_SAMPLE                        (-99)
#define ACCEPTANCE_CODE                     (0)
#define ACCEPTANCE_MASK                     (0xFFFFFFFF)
#define MAX_CAN_START_CNT                   (5)
#define MAX_SAMP_CNT                        (3)
#define MAX_CONFIG_RTR_CNT                  (3)

#define SERVICE_01                          (0X01)
#define SERVICE_09                          (0x09)

#define PID_00                              (0x00)
#define PID_02                              (0x02)
#define PID_0A                              (0x0A)
#define PID_0B                              (0x0B)
#define PID_0C                              (0x0C)
#define PID_0D                              (0x0D)

#define FUEL_PRESSURE                       (0x0A)
#define MAP                                 (0x0B)
#define ENGINE_SPEED                        (0x0C)
#define VEHICLE_SPEED                       (0x0D)

#define SERVICE_INDEX                       (1)
#define PID_INDEX                           (2)
#define ISOTP_POSITIVE_RESP                 (0x40)

#define SINGLE_FRAME                        (0)
#define FIRST_FRAME                         (1)
#define CONS_FRAME                          (2)
#define FLOW_CONTROL_FRAME                  (3)

#define FRAME_TYPE                          (0)
#define ISOTP_SF_RESP                       (1)
#define ISTOP_SF_PID                        (2)
#define ISOTP_FF_RESP                       (2)
#define ISTOP_SF_A                          (3)
#define ISTOP_SF_B                          (4)

#define FIRST_CF                            (1)
#define SECOND_CF                           (2)

#define VIN_INDEX_FF_START                  (0)
#define VIN_INDEX_FF_END                    (3)
#define VIN_RESP_INDEX_FF_START             (5)
#define VIN_INDEX_CF1_START                 (4)
#define VIN_INDEX_CF1_END                   (11)
#define VIN_RESP_INDEX_CF_START             (1)
#define VIN_INDEX_CF2_START                 (12)
#define VIN_INDEX_CF2_END                   (19)

#define FUEL_PRESSURE_INDEX                 (2)
#define MAP_INDEX                           (3)
#define ENGINE_SPEED_INDEX                  (1)
#define VEHICLE_SPEED_INDEX                 (0)

/** Enums */
typedef enum
{
    NET_IDLE,
    INIT_CONN,
    WAIT_CONN,
    CONN,
    DISCONN,
    NET_FATAL_ERROR,
    CONN_STATE_NUM
} teConnState;

typedef enum
{
    INIT_CONN_IDLE,
    NET_INIT,
    REG_NET_CB,
    NET_START,
    INIT_CONN_STATE_NUM
} teInitConnSubState;

typedef enum
{
    CAN_IDLE,
    CAN_INIT,
    CAN_START,
    CAN_SAMP,
    CAN_CLOSE,
    CAN_FATAL_ERROR,
    CAN_STATE_NUM
} teCanState;

typedef enum
{
    SAMP_PING,
    SAMP_DTCs,
    SAMP_CD,
    SAMP_WAIT,
    SAMP_VIN,
    SAMP_STATE_NUM
} teSampState;

/** Debug string values */
static const char *connState[CONN_STATE_NUM] = {"NET_IDLE", "INIT_CONN", "WAIT_CONN", "CONN", "DISCONN", "NET_FATAL_ERROR"};
static const char *InitConnState[INIT_CONN_STATE_NUM] = {"INIT_CONN_IDLE", "NET_INIT", "REG_NET_CB", "NET_START"};
static const char *canState[CAN_STATE_NUM] = {"CAN_IDLE", "CAN_INIT", "CAN_START", "CAN_SAMP", "CAN_CLOSE", "CAN_FATAL_ERROR"};                                         
static const char *sampState[SAMP_STATE_NUM] ={"SAMP_PING", "SAMP_DTCs", "SAMP_CD", "SAMP_WAIT", "SAMP_VIN"};

/** Local functions */
/** State functions */
static void vCanMainTask(void);
static void vNetMainTask(void);

/** Local functions used for Internet connection */
static void vInitMainApp(void);
static void vInitConn(void);
static void vNetCallback(HAL_NET_teConStatus eConStatus);
static void vCahngeConnState(teConnState eNewConnState);
static void vCahngeInitConnSubState(teInitConnSubState eNewInitConnSubState);

/** Local functions used for Can communication */
static void vCanClose();
static void vCanInit(void);
static void vCanStart(void);
static void vCanSamp(void);
static void vSampWaitResp(void);
static void vChangeCanState(teCanState eNewCanState);
static void vChangeSampState(teSampState eNewSampState);
static void vConstructCanMsg(int Ser, int Pid);
static void vCanPing(void);
static void vCheckRxCanMsg(int service);
static void vSampVin(void);
static void vTxFC(void);
static void vRXCDHandle(void);
static void vRXSFHandle(void);
static void vRXFFHandle(void);
static void vRXCFHandle(void);
static void vSampVinHandle(void);
static bool bDidWeReceive(int frame, int service);
static void vSampCD(void);
static void vStartSampling(void);
static void vTimerTickTock(void);

/** HTTP helper functions*/
static void vGetIntervalFromServer(char* sVin);
static void vSendSampledData(void);

static esp_err_t client_event_post_handler_intval(esp_http_client_event_handle_t eEvt);
static esp_err_t client_event_post_handler_samp_data(esp_http_client_event_handle_t eEvt);

/** Modul name used for debug */
static const char *sTag = "MainApp";

static teConnState eConnState = NET_IDLE;
static teInitConnSubState eInitConnSubState = INIT_CONN_IDLE;

static teCanState eCanState = CAN_IDLE;
static teSampState eSampState = SAMP_PING;
static teSampState ePrevSampState = SAMP_PING;

/** Global variables */
/** variables used between internet functions */
const char *ssid = "Petrovic";
const char *pass = "JelkaiVerka";
static int initConnErrCnt = 0;
static bool bGetInterval = false;
static int serviceTx = 0xFF;
static int timerCnt = 0;

/** Variables used for Can function */
static HAL_TWAI_teCanSpeed canProtocol[NUM_OF_CAN_PROTOCOLS] = {HAL_TWAI_e250KB, HAL_TWAI_e500KB};
static int selectedSpeed = 0;

static int vCanStartErrCnt = 0;

static char sVin[VIN_LEN] = "123-1234567-1234567";
static unsigned int prevRxMsgId = 0xFFFF;

static int sampTO = DONOT_SAMPLE;
static int sampTOCnt = 0;
static int sampList[MAX_NUM_OF_SAMPLES] = {PID_0D, PID_0C, PID_0A, PID_0B};
static char *sampleString[MAX_NUM_OF_SAMPLES] = {"VehSpeed", "Engine Speed", "Fuel pressure", "MAP"};
static float samples[MAX_NUM_OF_SAMPLES] = {0.0, 0.0, 0.0, 0.0};
static int data4Samp = 0;

/** Variables used for comunication between CAN and Net state machines */
static bool bSendSampls = false;
static bool bCanSend = true;

/** Variable used for sending OBDII frames */
static tCanFrame TxMsg = {
    /// Message type and format settings
    .extd = 0,              /// Standard vs extended format
    .rtr = 0,               /// Data vs RTR frame
    .ss = 0,                /// Whether the message is single shot (i.e., does not repeat on error)
    .self = 0,              /// Whether the message is a self reception request (loopback)
    .dlc_non_comp = 0,      /// DLC is less than 8
    /// Message ID and payload
    .identifier = 0x7E0,
    .data_length_code = 8,
    .data = {0x02, 0x01, 0x00, 0xAA, 0xAA, 0XAA, 0XAA, 0XAA},
};

/** Variable used for CF */
static tCanFrame TxCFMsg = {
    /// Message type and format settings
    .extd = 0,              /// Standard vs extended format
    .rtr = 0,               /// Data vs RTR frame
    .ss = 0,                /// Whether the message is single shot (i.e., does not repeat on error)
    .self = 0,              /// Whether the message is a self reception request (loopback)
    .dlc_non_comp = 0,      /// DLC is less than 8
    /// Message ID and payload
    .identifier = 0x7E0,
    .data_length_code = 8,
    .data = {0x30, 0x00, 0x00, 0x00, 0x00, 0X00, 0X00, 0X00},
};

/** Variable used to handle RX msgs*/
static tCanFrame RxMsg = {
    /// Message type and format settings
    .extd = 0,              /// Standard vs extended format
    .rtr = 0,               /// Data vs RTR frame
    .ss = 0,                /// Whether the message is single shot (i.e., does not repeat on error)
    .self = 0,              /// Whether the message is a self reception request (loopback)
    .dlc_non_comp = 0,      /// DLC is less than 8
    /// Message ID and payload
    .identifier = 0x7E0,
    .data_length_code = 8,
    .data = {0x02, 0x01, 0x00, 0xAA, 0xAA, 0XAA, 0XAA, 0XAA},
};

 /** CB for internet connection events notification*/
HAL_NET_pfvEventHandler pNetCb   = &vNetCallback;

void app_main(void)
{
    vInitMainApp();

    while (true)
    {
        vNetMainTask();
        vCanMainTask();
        vTaskDelay(CYCLIC_PERIOD / portTICK_RATE_MS);
    }
}

/** \brief Can state machine */
static void vCanMainTask(void)
{
    switch (eCanState)
    {
        case CAN_IDLE:
            vChangeCanState(CAN_INIT);
            break;

        case CAN_INIT:
            vCanInit();        
            break;

        case CAN_START:
            vCanStart();
            break;
        
        case CAN_SAMP:
            vCanSamp();            
            break;

        case CAN_CLOSE:
            vCanClose();
            break;

        case CAN_FATAL_ERROR:
            break;

        default:
            break;
    }
}

/** \brief Net state machine */
static void vNetMainTask(void)
{
    static bool bTimeSynced = false;
    switch (eConnState)
    {
        case NET_IDLE:
            vCahngeConnState(INIT_CONN);
            vCahngeInitConnSubState(NET_INIT);          
            break;

        case INIT_CONN: 
            if(initConnErrCnt < MAX_INIT_CONN_ERR_CNT)
            {
                vInitConn();
            }
            else
            {
                HAL_LOGGER_vLog(E, sTag, "Connection to internet cannot be init");
                vCahngeConnState(NET_FATAL_ERROR);
            }
            break;

        case WAIT_CONN:
            /** wait result from callback */
            break;

        case CONN:
            if (bGetInterval == true)
            {
                vGetIntervalFromServer(sVin);
                bGetInterval = false;
                HAL_LOGGER_vLog(I, sTag, "Request for interval sent");
            }

            if(bTimeSynced == false)
            {
                bTimeSynced = true;
                HAL_TIME_vInit();
            }

            if (bSendSampls == true && bCanSend == true)
            {
                bCanSend = false;
                bSendSampls = false;
                vSendSampledData();
            }           
            break;

        case DISCONN:
            (void)HAL_NET_bUnregister(pNetCb);
            vCahngeConnState(NET_FATAL_ERROR);
            break;

        default:
            break;
    }
}

/** Implementation of local functions */
/** \brief Initialization of Main app */
static void vInitMainApp(void)
{
    HAL_LOGGER_vLog(D, sTag, "ESP32 started");
    sVin[3]  = '-' ;
    sVin[11] = '-' ;
    sVin[19] = '\0';
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    esp_log_level_set("*", ESP_LOG_NONE);
}

/** \brief Init of internet connection state machine */
static void vInitConn(void)
{
    esp_err_t eInitRetVal;
    bool bNetEventReg;

    switch(eInitConnSubState)
    {
        case INIT_CONN_IDLE:
            break;

        case NET_INIT:
            eInitRetVal = HAL_NET_eInit();
            if(eInitRetVal == ESP_OK)
            {
                vCahngeInitConnSubState(REG_NET_CB);
                initConnErrCnt = 0;
            }
            else
            {
                initConnErrCnt++;
            }
            break;

        case REG_NET_CB:
            bNetEventReg = HAL_NET_bEventRegister(pNetCb);
            if(bNetEventReg == true)
            {
                vCahngeInitConnSubState(NET_START);

                initConnErrCnt = 0;
            }
            else
            {
                initConnErrCnt++;   
            }
                break;

        case NET_START:
            HAL_NET_vStart(ssid, pass);

            vCahngeInitConnSubState(INIT_CONN_IDLE);
            vCahngeConnState(WAIT_CONN);
            break;

        default:
            break;
    }
}

/** \brief Connection status update */
static void vNetCallback(HAL_NET_teConStatus eConStatus)
{   
    if (eConStatus == HAL_NET_eDisconnected)
    {
        HAL_LOGGER_vLog(D, sTag, "Internet disconnected");
        bCanSend = false;
        vCahngeConnState(DISCONN);
    }
    else if (eConStatus == HAL_NET_eConnected)
    {
        HAL_LOGGER_vLog(D, sTag, "Connected to internet");
        bCanSend = true;
        if(eConnState != CONN)
        {
            vCahngeConnState(CONN);
        }
    }
    else
    {
        HAL_LOGGER_vLog(E, sTag, "Unknown connection state");
        vCahngeConnState(NET_FATAL_ERROR);
    }
}

/** \brief Configure(used pins, protocol, filter) CAN connection */
static void vCanInit(void)
{
    static int rtrCnt = 0;
    if(selectedSpeed < NUM_OF_CAN_PROTOCOLS)
    {
        if(rtrCnt < MAX_CONFIG_RTR_CNT)
        {
            if(HAL_TWAI_bConfigureCan(GPIO_NUM_21, GPIO_NUM_22,
                                    canProtocol[selectedSpeed],
                                    ACCEPTANCE_CODE, (unsigned int)ACCEPTANCE_MASK))
            {
                vChangeCanState(CAN_START);
                rtrCnt = 0;
            }
            else
            {
                rtrCnt++;
            }
        }
        else
        {
            selectedSpeed++;
            rtrCnt = 0;
        }
    }
    else
    {
        vChangeCanState(CAN_FATAL_ERROR);
    }
}

/** \brief Open Can */
static void vCanStart(void)
{
    if(vCanStartErrCnt < MAX_CAN_START_CNT)
    {
        if(HAL_TWAI_bStartCAN())
        {
            vChangeCanState(CAN_SAMP);
            vChangeSampState(SAMP_PING);
            vCanStartErrCnt = 0;
        }
        else
        {
            vCanStartErrCnt++;
        }
    }
    else
    {
        vChangeCanState(CAN_FATAL_ERROR);
    }
}
 
/** \brief Sampling state machine */
static void vCanSamp(void)
{
    switch(eSampState)
    {
        case SAMP_PING:
            vCanPing();
            break;

        case SAMP_WAIT:
            vSampWaitResp();            
            break;

        case SAMP_VIN:
            vSampVin();
            break;

        case SAMP_CD:
            vSampCD();
            break;
        default:
            break;
    }
}

/** \brief Construct isotp msg*/
static void vConstructCanMsg(int Ser, int Pid)
{
    serviceTx = Ser;
    TxMsg.data[SERVICE_INDEX] = Ser;
    TxMsg.data[PID_INDEX] = Pid;
}

/** \brief Transmit supported live data request */
static void vCanPing(void)
{
    vConstructCanMsg(SERVICE_01, PID_00);
    
    if (twai_transmit(&TxMsg, 0) == ESP_OK)
    {
        HAL_LOGGER_vLog(D, sTag, "Message queued for transmission");
        timerCnt = 0;
        vChangeSampState(SAMP_WAIT);
    }
    else
    {
        HAL_LOGGER_vLog(W, sTag, "Failed to queue message for transmission");
    }
}

/** \brief Wait for resp until timer runs out and retry process MAX_SAMP_CNT */
static void vSampWaitResp(void)
{
    static int rtrCnt = 0;
    timerCnt += CYCLIC_PERIOD;
    if (timerCnt >= CAN_RX_TIMEOUT)
    {
        HAL_LOGGER_vLog(D, sTag, "Failed to receive message");
        if (ePrevSampState == SAMP_PING)
        {
            timerCnt = 0;
            rtrCnt = 0;
            vChangeCanState(CAN_CLOSE);
        }
        else if(ePrevSampState == SAMP_CD)
        {
            // just go to other current data
            vChangeSampState(SAMP_CD);
        }
        else
        {
            timerCnt = 0;
            rtrCnt++;
            if (rtrCnt >= MAX_SAMP_CNT)
            {
                vChangeCanState(ePrevSampState);
            }
            else
            {
                vChangeCanState(CAN_FATAL_ERROR);
            }   
        }
    }
    else
    {
        vCheckRxCanMsg(serviceTx);
    }
}

/** \brief Check and decode Rx msgs */
static void vCheckRxCanMsg(int service)
{
    if (twai_receive(&RxMsg, 0) == ESP_OK)
    {
        HAL_LOGGER_vLog(D, sTag, "Message received");

#ifdef DEBUG_RX_CAN_MG
        HAL_LOGGER_vLog(D, sTag, "ID is %d", RxMsg.identifier);
        if (!(RxMsg.rtr))
        {
            for (int i = 0; i < RxMsg.data_length_code; i++) {
                HAL_LOGGER_vLog(D, sTag, "Data byte %d = %d", i, RxMsg.data[i]);
            }
        }
#endif
        if(bDidWeReceive(SINGLE_FRAME, service)) 
        {
            vRXSFHandle();
        }
        else if(bDidWeReceive(FIRST_FRAME, service))
        {
            vRXFFHandle();
        }
        else if(bDidWeReceive(CONS_FRAME, service))
        {
            vRXCFHandle();
        }
        else
        {
            /* CF not needed for now */
        }
    }
}

/** \brief Determine which frame did we receive */
static bool bDidWeReceive(int frame, int service)
{
    bool bRetVal = false;
    switch (frame)
    {
        case SINGLE_FRAME:
            bRetVal = (SINGLE_FRAME == GET_UPPER_NIBBLE(RxMsg.data[FRAME_TYPE])) && 
                      ((service + ISOTP_POSITIVE_RESP) == RxMsg.data[ISOTP_SF_RESP]);
            break;
        case FIRST_FRAME:
            bRetVal = (FIRST_FRAME == GET_UPPER_NIBBLE(RxMsg.data[FRAME_TYPE])) &&
                    ((service + ISOTP_POSITIVE_RESP) == RxMsg.data[ISOTP_FF_RESP]);
            break;
        case CONS_FRAME:
            bRetVal = (CONS_FRAME == GET_UPPER_NIBBLE(RxMsg.data[FRAME_TYPE])) &&
                    (prevRxMsgId == RxMsg.identifier);
            break;
        default:
            break;
    }
    return bRetVal;
}

/** \brief Tx Vin Req - 09 02*/
static void vSampVin(void)
{
    vConstructCanMsg(SERVICE_09, PID_02);
    if (twai_transmit(&TxMsg, 0) == ESP_OK)
    {
        HAL_LOGGER_vLog(D, sTag, "Message queued for transmission");
        timerCnt = 0;
        vChangeSampState(SAMP_WAIT);
    }
    else
    {
        HAL_LOGGER_vLog(W, sTag, "Failed to queue message for transmission");
    }
}

/** \brief TX Flow Control Frame */
static void vTxFC(void)
{
    if (twai_transmit(&TxCFMsg, 0) == ESP_OK)
    {
        HAL_LOGGER_vLog(D, sTag, "CF Message queued for transmission");
    }
    else
    {
        HAL_LOGGER_vLog(W, sTag, "Failed to queue message for transmission");
    }
}

/** \brief Handle Consecutive Frame Rx msgs */
static void vRXCFHandle(void)
{
    switch(ePrevSampState)
    {
        case SAMP_VIN:
            vSampVinHandle();            
            break;
        default:
            break;
    }
}

/** \brief Handle First Frame RX msgs */
static void vRXFFHandle(void)
{
    switch(ePrevSampState)
    {
        case SAMP_VIN:
            prevRxMsgId = RxMsg.identifier;
        for(int i = VIN_INDEX_FF_START; i < VIN_INDEX_FF_END; i++)
        {
            sVin[i] =  RxMsg.data[VIN_RESP_INDEX_FF_START + i];
        }
        vTxFC();
            break;
        default:
            break;
    }
}

/** \brief Construct Vin from FF and two CF */
static void vSampVinHandle(void)
{
    if (FIRST_CF == GET_LOWER_NIBBLE(RxMsg.data[FRAME_TYPE]))
    {
        for(int i = VIN_INDEX_CF1_START; i < VIN_INDEX_CF1_END; i++)
        {
            sVin[i] = RxMsg.data[VIN_RESP_INDEX_CF_START+i-VIN_INDEX_CF1_START];
        }
    }
    else if(SECOND_CF == GET_LOWER_NIBBLE(RxMsg.data[FRAME_TYPE]))
    {
        for(int i = VIN_INDEX_CF2_START; i < VIN_INDEX_CF2_END; i++)
        {
            sVin[i] = RxMsg.data[VIN_RESP_INDEX_CF_START+i-VIN_INDEX_CF2_START];
        }
        vChangeSampState(SAMP_CD);
        bGetInterval = true;
        HAL_LOGGER_vLog(I, sTag, "VIN : %s", sVin);
    }
    else
    {
        /*nothing*/
    }
}

/** \brief Handle Single Frame Rx msgs */
static void vRXSFHandle(void)
{
    switch(ePrevSampState)
    {
        case SAMP_PING:
            vChangeSampState(SAMP_VIN);
            break;

        case SAMP_CD:
            vRXCDHandle();    
            vChangeSampState(SAMP_CD);
            break;

        default:
            break;
    }
}

/** \brief Coversion fuction from OBD-II data to useful info*/
static void vRXCDHandle(void)
{
    switch(RxMsg.data[ISTOP_SF_PID])
    {
        case FUEL_PRESSURE:
            samples[FUEL_PRESSURE_INDEX] = FUEL_PRES_COVERSION_FORMULA(RxMsg.data[ISTOP_SF_A]);
            break;
        case MAP:
            samples[MAP_INDEX] = RxMsg.data[ISTOP_SF_A];
            break;
        case ENGINE_SPEED:
            samples[ENGINE_SPEED_INDEX] = ENG_SPEED_COVERSION_FORMULA(RxMsg.data[ISTOP_SF_A],RxMsg.data[ISTOP_SF_B]);
            break;
        case VEHICLE_SPEED:
            samples[VEHICLE_SPEED_INDEX] = RxMsg.data[ISTOP_SF_A];
            break;
        default:
            break;
    }
} 

/** \brief Event handler for http req for sampling interval */
static esp_err_t client_event_post_handler_intval(esp_http_client_event_handle_t eEvt) 
{
    switch (eEvt->event_id) {
        case HTTP_EVENT_ON_DATA:
            //data ---> {"status":"success","data":"[{\"intVal\":\"300000\"}]"}
            HAL_LOGGER_vLog(D, sTag, "HTTP_EVENT_ON_DATA: %.*s", eEvt->data_len, (char *)eEvt->data);

            cJSON *jSonTmp = cJSON_Parse((char *)eEvt->data);

            if (cJSON_GetObjectItem(jSonTmp, "data"))
            {
		            char *pcDataOfEvent = cJSON_GetObjectItem(jSonTmp,"data")->valuestring;
                    HAL_LOGGER_vLog(D, sTag, "Data = %s", pcDataOfEvent);

                // remove [] from [{"intVal" : "xxx"}]
                pcDataOfEvent++;
                pcDataOfEvent[strlen(pcDataOfEvent)-1] = 0;

                // {"intVal" : "xxx"}
                jSonTmp = cJSON_Parse(pcDataOfEvent);

                if (cJSON_GetObjectItem(jSonTmp, "intVal"))
                {
                    char *pcIntVal = cJSON_GetObjectItem(jSonTmp,"intVal")->valuestring;
                    HAL_LOGGER_vLog(D, sTag, "intVal = %s", pcIntVal);
                    sampTO = atoi(pcIntVal) / CYCLIC_PERIOD;
                    HAL_LOGGER_vLog(D, sTag, "sampTO = %d", sampTO);
                }
                else
                {
                    HAL_LOGGER_vLog(E, sTag, "Problems parsing intVal part of JSON");
                }
	        }
            else
            {
                HAL_LOGGER_vLog(E, sTag, "Problems parsing data part of JSON");
            }
            cJSON_Delete(jSonTmp);
            break;

        default:
            break;
    }
    return ESP_OK;  
}

/** Send req to server for Interval sampling */
static void vGetIntervalFromServer(char* sVin)
{
    esp_http_client_config_t config_post = {
        .url = "http://192.168.1.7:3000/intInfo",
        .method = HTTP_METHOD_POST,
        .cert_pem = NULL,
        .event_handler = client_event_post_handler_intval};
    
    esp_http_client_handle_t client = esp_http_client_init(&config_post);

    char buf[100];
    snprintf(buf, 100, "{\"vinVal\":\"%s\"}",sVin);/*"{\"vinVal\":\"123-1234567-1234567\"}"*/
    HAL_LOGGER_vLog(D, sTag, "JSON -> %s", buf);

    char  *post_data = buf;
    esp_http_client_set_post_field(client, post_data, strlen(post_data));
    esp_http_client_set_header(client, "Content-Type", "application/json");

    esp_http_client_perform(client);
    esp_http_client_cleanup(client);
}

/** \brief Sample data change handler */
static esp_err_t client_event_post_handler_samp_data(esp_http_client_event_handle_t eEvt)
{
    switch (eEvt->event_id) {
        case HTTP_EVENT_ON_DATA:
            HAL_LOGGER_vLog(D, sTag, "CD sent succesfully");
            break;

        default:
            break;
    }
    bCanSend = true;
    return ESP_OK;
}

/** \brief Send Sampled data to server */
static void vSendSampledData(void)
{
    esp_http_client_config_t config_post = {
        .url = "http://192.168.1.7:3000/esp32DataPost",
        .method = HTTP_METHOD_POST,
        .cert_pem = NULL,
        .event_handler = client_event_post_handler_samp_data};
    
    esp_http_client_handle_t client = esp_http_client_init(&config_post);
    char buf[100];
    snprintf(buf, 100, "{\"vin\":\"%s\",\"data0\":\"%.2f\",\"data1\":\"%.2f\",\"data2\":\"%.2f\",\"data3\":\"%.2f\"}", sVin, samples[0], samples[1], samples[2], samples[3]);
    HAL_LOGGER_vLog(E, sTag, "JSON -> %s", buf);

    char  *post_data = buf;
    esp_http_client_set_post_field(client, post_data, strlen(post_data));
    esp_http_client_set_header(client, "Content-Type", "application/json");

    esp_http_client_perform(client);
    esp_http_client_cleanup(client);
}

/** \brief Sample Current data */
static void vSampCD(void)
{
    if(sampTO != DONOT_SAMPLE) /// Interval was not set
    {
        vTimerTickTock();
        vStartSampling();
    }
}

/** \brief Check if interval is set and start sampling if interval passed */
static void vStartSampling(void)
{
    if (sampTOCnt >= sampTO) /// if timeout has passed start sampeling
    {
        vConstructCanMsg(SERVICE_01, sampList[data4Samp]); /// costruct isotp msg for service 01 with pid from the list
        if (twai_transmit(&TxMsg, 0) == ESP_OK)            /// send CAN msg
        {
            HAL_LOGGER_vLog(D, sTag, "Message queued for transmission");
            data4Samp++;
            timerCnt = 0;
            vChangeSampState(SAMP_WAIT);
        }
        else
        {
            HAL_LOGGER_vLog(W, sTag, "Failed to queue message for transmission");
        }
    }
}

/** \brief Timer counter function, if all data is sampled it triggers send samples event */
static void vTimerTickTock(void)
{
    if (data4Samp == MAX_NUM_OF_SAMPLES) /// all date from the list is sampled, it is time to reset timer and return to the start of samp list
    {
        sampTOCnt = 0; /// interval timeout is set to 0
        data4Samp = 0; /// return to the start of sampling list

        for (int i = 0; i < MAX_NUM_OF_SAMPLES; i++) /// print sampled data
        {
            HAL_LOGGER_vLog(D, sTag, "%s -> %f", sampleString[i], samples[i]);
        }

        bSendSampls = true; /// set event which tells vNetMainTask to send sampled data to server
    }
    else
    {
        sampTOCnt++; /// cout number of cycles, sampTO is definened in number of cycles that need to pass
    }
}

/** Uninstall and clos Can */
static void vCanClose(void)
{
    if (HAL_TWAI_bStopCAN())
    {
        vChangeCanState(CAN_IDLE);
        selectedSpeed++;
    }
}

/** \brief Functions used for changing states od state machines */

static void vCahngeConnState(teConnState eNewConnState)
{
    HAL_LOGGER_vLog(D, sTag, "Conn state changed %s -> %s", connState[eConnState], connState[eNewConnState]);
    eConnState = eNewConnState;
}

static void vCahngeInitConnSubState(teInitConnSubState eNewInitConnSubState)
{
    HAL_LOGGER_vLog(D, sTag, "Init Conn substate changed %s -> %s", InitConnState[eInitConnSubState], InitConnState[eNewInitConnSubState]);
    eInitConnSubState = eNewInitConnSubState;
}

static void vChangeCanState(teCanState eNewCanState)
{
    HAL_LOGGER_vLog(D, sTag, "Can state changed %s -> %s", canState[eCanState], canState[eNewCanState]);
    eCanState = eNewCanState;
}

static void vChangeSampState(teSampState eNewSampState)
{
    HAL_LOGGER_vLog(D, sTag, "Init Samp substate changed %s -> %s", sampState[eSampState], sampState[eNewSampState]);
    ePrevSampState = eSampState;
    eSampState     = eNewSampState;
}