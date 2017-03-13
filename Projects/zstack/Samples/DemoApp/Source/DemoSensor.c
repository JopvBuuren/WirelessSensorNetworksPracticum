/**************************************************************************************************
  Filename:       DemoSensor.c

  Description:    Sensor application for the sensor demo utilizing the Simple API.

                  The sensor node is a ZigBee end device.
                  The sensor application binds to a gateway and will periodically
                  read temperature and supply voltage from the ADC and send report
                  towards the gateway node.

  Copyright 2009 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/******************************************************************************
 * INCLUDES
 */
#include "sapi.h"
#include "hal_key.h"
#include "hal_led.h"
#include "hal_adc.h"
#include "hal_uart.h"
#include "DemoApp.h"

/******************************************************************************
 * CONSTANTS
 */
#define REPORT_FAILURE_LIMIT                4
#define ACK_REQ_INTERVAL                    5    // each 5th packet is sent with ACK request

// Application osal event identifiers
// Bit mask of events ( from 0x0000 to 0x00FF )
#define MY_START_EVT                        0x0001
#define MY_FIND_COLLECTOR_EVT               0x0002
#define MY_DOOR_SET_TIMEOUT_EVT             0x0004

// ADC definitions for CC2430/CC2530 from the hal_adc.c file
#if defined (HAL_MCU_CC2530)
#define HAL_ADC_REF_125V    0x00    /* Internal 1.25V Reference */
#define HAL_ADC_DEC_064     0x00    /* Decimate by 64 : 8-bit resolution */
#define HAL_ADC_DEC_128     0x10    /* Decimate by 128 : 10-bit resolution */
#define HAL_ADC_DEC_512     0x30    /* Decimate by 512 : 14-bit resolution */
#define HAL_ADC_CHN_VDD3    0x0f    /* Input channel: VDD/3 */
#define HAL_ADC_CHN_TEMP    0x0e    /* Temperature sensor */
#endif // HAL_MCU_CC2530
   
// Port and pin for green LED
#define PORT_SIGNAL_LED                     1
#define PIN_SIGNAL_LED                      2

// Report failure related values
#define REPORT_FAILURE_LIMIT                4
#define BIND_RETRY_LIMIT                    2

/******************************************************************************
 * LOCAL VARIABLES
 */
static uint8 appState =           APP_INIT;

static uint8 reportFailureNr =    0;
static uint8 bindRetries =        0;

static uint16 myBindRetryDelay =  2000;        // milliseconds
static uint8 myStartRetryDelay =    10;        // milliseconds

static uint16 parentShortAddr;

// Value of the door limit switch on the coordinator
static int8 doorLimitSwitchVal = -1;

// Report failure related values
static uint8 myReportTimeout =      200;         // milliseconds

/******************************************************************************
 * GLOBAL VARIABLES
 */
// Inputs and Outputs for Sensor device
#define NUM_IN_CMD_SENSOR         1
#define NUM_OUT_CMD_SENSOR        1

// List of input commands for Sensor device
const cId_t zb_InCmdList[NUM_IN_CMD_SENSOR] =
{
  DOOR_REPORT_CMD_ID
};

// List of output commands for Sensor device
const cId_t zb_OutCmdList[NUM_OUT_CMD_SENSOR] =
{
  DOOR_SET_CMD_ID
};

// Define SimpleDescriptor for Sensor device
const SimpleDescriptionFormat_t zb_SimpleDesc =
{
  MY_ENDPOINT_ID,             //  Endpoint
  MY_PROFILE_ID,              //  Profile ID
  DEV_ID_SENSOR,              //  Device ID
  DEVICE_VERSION_SENSOR,      //  Device Version
  0,                          //  Reserved
  NUM_IN_CMD_SENSOR,          //  Number of Input Commands
  (cId_t *) zb_InCmdList,     //  Input Command List
  NUM_OUT_CMD_SENSOR,         //  Number of Output Commands
  (cId_t *) zb_OutCmdList     //  Output Command List
};

/******************************************************************************
 * LOCAL FUNCTIONS
 */
void uartRxCB( uint8 port, uint8 event );
static void updateSignalLed(void);
static void sendDoorVal( uint8 doorVal );

/******************************************************************************
 * GLOBAL FUNCTIONS
 */

/*****************************************************************************
 * @fn          zb_HandleOsalEvent
 *
 * @brief       The zb_HandleOsalEvent function is called by the operating
 *              system when a task event is set
 *
 * @param       event - Bitmask containing the events that have been set
 *
 * @return      none
 */
void zb_HandleOsalEvent( uint16 event )
{
  if( event & SYS_EVENT_MSG )
  {
  }

  if( event & ZB_ENTRY_EVENT )
  {
    // blind LED 1 to indicate joining a network
    HalLedBlink ( HAL_LED_1, 0, 50, 500 );

    // Start the device
    appState = APP_START;
    zb_StartRequest();
  }

  if ( event & MY_START_EVT )
  {
    zb_StartRequest();
  }

  if ( event & MY_FIND_COLLECTOR_EVT )
  {
    // blink LED 2 to indicate discovery and binding
    HalLedBlink ( HAL_LED_2, 0, 50, 500 );

    // Find and bind to a collector device
    appState = APP_BIND;
    zb_BindDevice( TRUE, DOOR_SET_CMD_ID, (uint8 *)NULL );
  }
  
  if ( event & MY_DOOR_SET_TIMEOUT_EVT ) 
  {
    /* TODO: Determine what to do on a timeout; handle like a report fail? */
  }
}

/******************************************************************************
 * @fn      zb_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 EVAL_SW4
 *                 EVAL_SW3
 *                 EVAL_SW2
 *                 EVAL_SW1
 *
 * @return  none
 */
void zb_HandleKeys( uint8 shift, uint8 keys )
{
  // shift is not used and keys HAL_KEY_SW_3 and HAL_KEY_SW_4 are not used, so 
  // removed code
  if ( keys & HAL_KEY_SW_1 )
  {
    // Do nothing
  }
  if ( keys & HAL_KEY_SW_2 )
  {
    if ( (appState == APP_RUN) && (doorLimitSwitchVal != -1) )
    {
      sendDoorVal( !doorLimitSwitchVal );
    }
  }
}

/******************************************************************************
 * @fn          zb_StartConfirm
 *
 * @brief       The zb_StartConfirm callback is called by the ZigBee stack
 *              after a start request operation completes
 *
 * @param       status - The status of the start operation.  Status of
 *                       ZB_SUCCESS indicates the start operation completed
 *                       successfully.  Else the status is an error code.
 *
 * @return      none
 */
void zb_StartConfirm( uint8 status )
{
  // If the device sucessfully started, change state to running
  if ( status == ZB_SUCCESS )
  {
    // Set LED 1 to indicate that node is operational on the network
    HalLedSet( HAL_LED_1, HAL_LED_MODE_ON );

    // Store parent short address
    zb_GetDeviceInfo(ZB_INFO_PARENT_SHORT_ADDR, &parentShortAddr);

    // Set event to bind to a collector
    osal_set_event( sapi_TaskID, MY_FIND_COLLECTOR_EVT );

    // Turn OFF Allow Bind mode infinitly
    zb_AllowBind( 0x00 );
    HalLedSet( HAL_LED_2, HAL_LED_MODE_OFF );
  }
  else
  {
    // Try again later with a delay
    osal_start_timerEx( sapi_TaskID, MY_START_EVT, myStartRetryDelay );
  }
}

/******************************************************************************
 * @fn          zb_BindConfirm
 *
 * @brief       The zb_BindConfirm callback is called by the ZigBee stack
 *              after a bind operation completes.
 *
 * @param       commandId - The command ID of the binding being confirmed.
 *              status - The status of the bind operation.
 *
 * @return      none
 */
void zb_BindConfirm( uint16 commandId, uint8 status )
{
  if( status == ZB_SUCCESS )
  {
    appState = APP_RUN;
    HalLedSet( HAL_LED_2, HAL_LED_MODE_OFF );
  }
  else
  {
    if ( ++bindRetries >= 2 ) {
      // Reset the system
      zb_SystemReset();
    }
    else
    {
      osal_start_timerEx( sapi_TaskID, MY_FIND_COLLECTOR_EVT, myBindRetryDelay );
    }
  }
}

/******************************************************************************
 * @fn          zb_SendDataConfirm
 *
 * @brief       The zb_SendDataConfirm callback function is called by the
 *              ZigBee after a send data operation completes
 *
 * @param       handle - The handle identifying the data transmission.
 *              status - The status of the operation.
 *
 * @return      none
 */
void zb_SendDataConfirm( uint8 handle, uint8 status )
{
  (void)handle;
  
  // Stop the report timeout timers
  osal_stop_timerEx( sapi_TaskID, MY_DOOR_SET_TIMEOUT_EVT );
  
  if(status != ZB_SUCCESS)
  {
    if ( ++reportFailureNr >= REPORT_FAILURE_LIMIT )
    {
       // Delete previous binding
       zb_BindDevice( FALSE, DOOR_SET_CMD_ID, (uint8 *)NULL );

       // Try binding to a new gateway
       osal_start_timerEx( sapi_TaskID, MY_FIND_COLLECTOR_EVT, myBindRetryDelay );
       reportFailureNr = 0;
    }
    else
    {
      // Send the door value again
      sendDoorVal(0);
    }
  }
  // status == SUCCESS
  else
  {
    // Reset failure counter
    reportFailureNr = 0;
  }
}

/******************************************************************************
 * @fn          zb_AllowBindConfirm
 *
 * @brief       Indicates when another device attempted to bind to this device
 *
 * @param
 *
 * @return      none
 */
void zb_AllowBindConfirm( uint16 source )
{
  (void)source;
}

/******************************************************************************
 * @fn          zb_FindDeviceConfirm
 *
 * @brief       The zb_FindDeviceConfirm callback function is called by the
 *              ZigBee stack when a find device operation completes.
 *
 * @param       searchType - The type of search that was performed.
 *              searchKey - Value that the search was executed on.
 *              result - The result of the search.
 *
 * @return      none
 */
void zb_FindDeviceConfirm( uint8 searchType, uint8 *searchKey, uint8 *result )
{
  (void)searchType;
  (void)searchKey;
  (void)result;
}

/******************************************************************************
 * @fn          zb_ReceiveDataIndication
 *
 * @brief       The zb_ReceiveDataIndication callback function is called
 *              asynchronously by the ZigBee stack to notify the application
 *              when data is received from a peer device.
 *
 * @param       source - The short address of the peer device that sent the data
 *              command - The commandId associated with the data
 *              len - The number of bytes in the pData parameter
 *              pData - The data sent by the peer device
 *
 * @return      none
 */
void zb_ReceiveDataIndication( uint16 source, uint16 command, uint16 len, uint8 *pData  )
{
  // Can only get the DOOR_REPORT_CMD_ID from the coordinator in our 
  // application, so don't have to check command id, source and length
  (void)source;
  (void)command;
  (void)len;
  
  // Store the new value of the door limit switch
  doorLimitSwitchVal = *pData;
  // Update the signal LED
  updateSignalLed();  
}

static void updateSignalLed(void) 
{
  // Set the signal LED to the value of the door limit switch
  MCU_IO_SET(
       PORT_SIGNAL_LED,
       PIN_SIGNAL_LED,
       doorLimitSwitchVal
  );
}

static void sendDoorVal( uint8 doorVal ) {
  // Data we will send 
  uint8 pData[DOOR_REPORT_LENGTH];
  uint8 txOptions;

  // Set the data
  pData[DOOR_STATE_OFFSET] = doorVal;
  txOptions = AF_MSG_ACK_REQUEST;
  
  // Send the data (destination address is set to previously established binding 
  // for the commandId)
  zb_SendDataRequest( ZB_BINDING_ADDR, DOOR_SET_CMD_ID, DOOR_REPORT_LENGTH, pData, 0, txOptions, 0 );
  
  // Set a timer to fire the MY_DOOR_SET_TIMEOUT_EVT (stopped as soon as we 
  // receive data)
  osal_start_timerEx( sapi_TaskID, MY_DOOR_SET_TIMEOUT_EVT, myReportTimeout );
}

/******************************************************************************
 * @fn          uartRxCB
 *
 * @brief       Callback function for UART
 *
 * @param       port - UART port
 *              event - UART event that caused callback
 *
 * @return      none
 */
void uartRxCB( uint8 port, uint8 event )
{
  (void)port;
  (void)event;
}
