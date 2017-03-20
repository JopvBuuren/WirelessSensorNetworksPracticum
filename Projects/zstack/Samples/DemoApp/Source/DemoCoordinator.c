/**************************************************************************************************
  Filename:       DemoCoordinator.c

  Description:    Coordinator application for the sensor demo utilizing Simple API.

                  The coordinator node functions as a gateway. The node accepts
                  incoming reports from the sensor nodes (router and end device)
                  and can send the reports via the UART to a PC tool.


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
#include "hal_uart.h"
#include "DemoApp.h"

/******************************************************************************
 * CONSTANTS
 */
// Stack Profile
#define ZIGBEE_2007                         0x0040
#define ZIGBEE_PRO_2007                     0x0041

#ifdef ZIGBEEPRO
#define STACK_PROFILE                       ZIGBEE_PRO_2007
#else
#define STACK_PROFILE                       ZIGBEE_2007
#endif

#define ZB_RECEIVE_DATA_INDICATION          0x8746

// Application osal event identifiers
#define MY_START_EVT                        0x0001
#define MY_DOOR_CHECK_EVT                   0x0002
#define MY_FIND_SENSOR_EVT                  0x0004
#define MY_FIND_ROUTER_EVT                  0x0008
#define MY_DOOR_REPORT_TIMEOUT_EVT          0x0016
#define MY_LIGHT_REPORT_TIMEOUT_EVT         0x0032

// Port and pin for door limit switch
#define PORT_DOOR_LIMIT_SWITCH              0
#define PIN_DOOR_LIMIT_SWITCH               5
// Port and pin for green LED
#define PORT_GREEN_LED                      0
#define PIN_GREEN_LED                       4
// Port and pin for door control
#define PORT_DOOR_CONTROL                   0
#define PIN_DOOR_CONTROL                    7

// Data handles
#define HANDLE_DOOR_CHECK                   0x0001
#define HANDLE_LIGHT_CHECK                  0x0002

// Report failure related values
#define REPORT_FAILURE_LIMIT                4

/******************************************************************************
 * TYPEDEFS
 */

/******************************************************************************
 * LOCAL VARIABLES
 */
static uint8 appState =             APP_INIT;
static uint8 myStartRetryDelay =    10;          // milliseconds
static uint16 myBindRetryDelay =    2000;        // milliseconds

static uint8 myDoorCheckDelay =     100;         // milliseconds
static uint8 prevDoorCheckVal;
static uint8 lightState =           0;
static uint8 doorRequest =          0;

// Report failure related values
static uint8 doorReportFailureNr =  0;
static uint8 lightReportFailureNr = 0;
static uint16 myReportTimeout =     5000;         // milliseconds

/******************************************************************************
 * LOCAL FUNCTIONS
 */
static void sendDoorReport(void);
static void sendLightReport(void);
static void sendLightRequest(void);

/******************************************************************************
 * GLOBAL VARIABLES
 */
// Inputs and Outputs for Collector device
#define NUM_IN_CMD_COLLECTOR            2
#define NUM_OUT_CMD_COLLECTOR           2
   
// List of input commands for Collector device
const cId_t zb_InCmdList[NUM_IN_CMD_COLLECTOR] =
{
   DOOR_SET_CMD_ID,
   LIGHT_SET_CMD_ID
};

// List of output commands for Collector device
const cId_t zb_OutCmdList[NUM_OUT_CMD_COLLECTOR] =
{
   DOOR_REPORT_CMD_ID,
   LIGHT_REPORT_CMD_ID
};

// Define SimpleDescriptor for Collector device
const SimpleDescriptionFormat_t zb_SimpleDesc =
{
  MY_ENDPOINT_ID,             //  Endpoint
  MY_PROFILE_ID,              //  Profile ID
  DEV_ID_COLLECTOR,           //  Device ID
  DEVICE_VERSION_COLLECTOR,   //  Device Version
  0,                          //  Reserved
  NUM_IN_CMD_COLLECTOR,       //  Number of Input Commands
  (cId_t *) zb_InCmdList,     //  Input Command List
  NUM_OUT_CMD_COLLECTOR,      //  Number of Output Commands
  (cId_t *) zb_OutCmdList     //  Output Command List
};

/******************************************************************************
 * GLOBAL FUNCTIONS
 */

/******************************************************************************
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
  if( event & ZB_ENTRY_EVENT )
  {
    /* If we get this event, then we can initialise things */
    
    // Initialise the door limit switch as input
    MCU_IO_DIR_INPUT( PORT_DOOR_LIMIT_SWITCH, PIN_DOOR_LIMIT_SWITCH );
    // Initialise the green LED as output
    MCU_IO_DIR_OUTPUT( PORT_GREEN_LED, PIN_GREEN_LED );
    // Initialise the door control as output
    MCU_IO_DIR_OUTPUT( PORT_DOOR_CONTROL, PIN_DOOR_CONTROL );
    
    // Set the previous door check value
    prevDoorCheckVal = MCU_IO_GET_SIMPLE( PORT_DOOR_LIMIT_SWITCH, PIN_DOOR_LIMIT_SWITCH );
    
    // Turn ON Allow Bind mode infinitly
    zb_AllowBind( 0xFF );

    // Blink LED 1 to indicate starting/joining a network
    HalLedBlink ( HAL_LED_1, 0, 50, 500 );
    HalLedSet( HAL_LED_2, HAL_LED_MODE_OFF );

    // Start the device
    appState = APP_START;
    zb_StartRequest();
  }

  if ( event & MY_START_EVT )
  {
    zb_StartRequest();
  }
  
  if ( event & MY_DOOR_CHECK_EVT ) 
  {
    // Check if the door limit switch has changed
    uint8 doorCheckVal = MCU_IO_GET_SIMPLE( PORT_DOOR_LIMIT_SWITCH, PIN_DOOR_LIMIT_SWITCH );
    if ( prevDoorCheckVal != doorCheckVal ) {      
      prevDoorCheckVal = doorCheckVal;
      
      // Send Light request if open
      if(doorCheckVal > 0){
        sendLightRequest();
      }
      
      // Door value changed, so send the door report to let the bound device 
      // know
      sendDoorReport();
      
      // No longer have to check whether or not the door limit switch changes, 
      // so stop the timer
      osal_stop_timerEx( sapi_TaskID, MY_DOOR_CHECK_EVT );
    }
  }
  
  if ( event & MY_FIND_SENSOR_EVT )
  {
    zb_BindDevice( TRUE, DOOR_REPORT_CMD_ID, (uint8 *)NULL );
  }
  
  if ( event & MY_FIND_ROUTER_EVT )
  {
    zb_BindDevice( TRUE, LIGHT_REPORT_CMD_ID, (uint8 *)NULL );
  }
  
  if ( event & MY_DOOR_REPORT_TIMEOUT_EVT )
  {
    // Door report timed out, so send it again
    sendDoorReport();
  }
  
  if ( event & MY_LIGHT_REPORT_TIMEOUT_EVT ) 
  {
    // Light report timed out, so send it again
    sendLightReport();
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
  static uint8 allowBind = FALSE;

  /* shift is not used and keys HAL_KEY_SW_3 and HAL_KEY_SW_4 are not used, so 
   * removed code
   */
  
  if ( keys & HAL_KEY_SW_1 )
  {    
    if ( appState == APP_RUN )
    {
      allowBind ^= 1;
      if ( allowBind )
      {
        // Turn ON Allow Bind mode infinitly
        zb_AllowBind( 0xFF );
        HalLedBlink ( HAL_LED_2, 0, 50, 500 );
      }
      else
      {
        // Turn OFF Allow Bind mode infinitly
        zb_AllowBind( 0x00 );
        HalLedSet( HAL_LED_2, HAL_LED_MODE_OFF );
      }
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
    // Init the network configuration
    initNwkConfig();
    
    // Set LED 1 to indicate that node is operational on the network
    HalLedSet( HAL_LED_1, HAL_LED_MODE_ON );

    // Change application state
    appState = APP_RUN;
    
    // Set events to bind to a sensor and a router
    osal_set_event( sapi_TaskID, MY_FIND_SENSOR_EVT );
    osal_set_event( sapi_TaskID, MY_FIND_ROUTER_EVT );
  }
  else
  {
    // Try again later with a delay
    osal_start_timerEx( sapi_TaskID, MY_START_EVT, myStartRetryDelay );
  }
}

/******************************************************************************
 * @fn          zb_SendDataConfirm
 *
 * @brief       The zb_SendDataConfirm callback function is called by the
 *              ZigBee stack after a send data operation completes
 *
 * @param       handle - The handle identifying the data transmission.
 *              status - The status of the operation.
 *
 * @return      none
 */
void zb_SendDataConfirm( uint8 handle, uint8 status )
{  
  // Stop the report timeout timers
  osal_stop_timerEx( sapi_TaskID, MY_DOOR_REPORT_TIMEOUT_EVT );
  osal_stop_timerEx( sapi_TaskID, MY_LIGHT_REPORT_TIMEOUT_EVT );
  
  if(status != ZB_SUCCESS)
  {
    // Check which data this is
    if ( handle == HANDLE_DOOR_CHECK )
    {
      if ( ++doorReportFailureNr >= REPORT_FAILURE_LIMIT )
      {
         // Delete previous binding
         zb_BindDevice( FALSE, DOOR_REPORT_CMD_ID, (uint8 *)NULL );

         // Try binding to a new gateway
         osal_start_timerEx( sapi_TaskID, MY_FIND_SENSOR_EVT, myBindRetryDelay );
         doorReportFailureNr = 0;
      }
      else
      {
        // Send door report again
        sendDoorReport();
      }
    }
    else if ( handle == HANDLE_LIGHT_CHECK ) 
    {
      if ( ++lightReportFailureNr >= REPORT_FAILURE_LIMIT )
      {
         // Delete previous binding
         zb_BindDevice( FALSE, LIGHT_REPORT_CMD_ID, (uint8 *)NULL );

         // Try binding to a new gateway
         osal_start_timerEx( sapi_TaskID, MY_FIND_ROUTER_EVT, myBindRetryDelay );
         lightReportFailureNr = 0;
      }
      else 
      {
        // Send light report again
        sendLightReport();
      }
    }
  }
  // status == SUCCESS
  else
  {
    if ( handle == HANDLE_DOOR_CHECK ) 
    {
      // Reset failure counter
      doorReportFailureNr = 0;
    }
    else if (handle == HANDLE_LIGHT_CHECK )
    {
      // Reset failure counter
      lightReportFailureNr = 0;
    }
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
    HalLedSet( HAL_LED_2, HAL_LED_MODE_ON );    
    
    // Check which command is now bound
    if ( commandId == DOOR_REPORT_CMD_ID )
    {
      // Send door report right away
      sendDoorReport();
    }
    else if ( commandId == LIGHT_REPORT_CMD_ID ) 
    {
      // Send light report right away
      sendLightReport();
    }
    
  }
  else
  {
    // Bind again after a given delay for given command id
    if ( commandId == DOOR_REPORT_CMD_ID )
    {
      osal_start_timerEx( sapi_TaskID, MY_FIND_SENSOR_EVT, myBindRetryDelay );
    }
    else if ( commandId == LIGHT_REPORT_CMD_ID ) 
    {
      osal_start_timerEx( sapi_TaskID, MY_FIND_ROUTER_EVT, myBindRetryDelay );        
    }
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
  // Check which command this is
  if ( command == DOOR_SET_CMD_ID )
  {    
    // Validate that the right data length has been sent
    if ( len == DOOR_REPORT_LENGTH ) 
    {
      // Get the target value for the door control
      uint8 targetDoorVal = *pData;
      // Set the door control value
      MCU_IO_SET(
           PORT_DOOR_CONTROL,
           PIN_DOOR_CONTROL,
           targetDoorVal
      );
      
      // Make sure there's a reload timer running for the MY_DOOR_CHECK_EVT so 
      // we can send the door report when door limit switch changes
      osal_start_reload_timer( sapi_TaskID, MY_DOOR_CHECK_EVT, myDoorCheckDelay );
    }
  } 
  else if ( command == LIGHT_SET_CMD_ID )
  {
    // Validate that the right data length has been sent
    if ( len == LIGHT_REPORT_LENGTH )
    {
      // Get the target value for the light
      uint8 targetLightVal = *pData;
      // Set the green LED value
      
      MCU_IO_SET(
           PORT_GREEN_LED,
           PIN_GREEN_LED,
           targetLightVal
      );   
      lightState = targetLightVal;
      sendLightReport();
    }
  }
  // Flash LED 2 once to indicate data reception
  HalLedSet ( HAL_LED_1, HAL_LED_MODE_FLASH );
}

static void sendDoorReport(void)
{
  // Data we will send 
  uint8 pData[DOOR_REPORT_LENGTH];
  uint8 txOptions;

  // Set the data
  pData[DOOR_STATE_OFFSET] = prevDoorCheckVal;
  txOptions = AF_MSG_ACK_REQUEST;
  
  // Send the data (destination address is set to previously established binding 
  // for the commandId)
  zb_SendDataRequest( ZB_BINDING_ADDR, DOOR_REPORT_CMD_ID, DOOR_REPORT_LENGTH, pData, HANDLE_DOOR_CHECK, txOptions, 0 );
  
  // Set a timer to fire the MY_DOOR_REPORT_TIMEOUT_EVT (stopped as soon as we 
  // receive data)
  osal_start_timerEx( sapi_TaskID, MY_DOOR_REPORT_TIMEOUT_EVT, myReportTimeout );
}

static void sendLightRequest(void){
  doorRequest = 1;
  sendLightReport();
}

static void sendLightReport(void)
{
  // get Door state
  uint8 doorState = MCU_IO_GET_SIMPLE( PORT_DOOR_LIMIT_SWITCH, PIN_DOOR_LIMIT_SWITCH );
  
  // Data we will send 
  uint8 pData[LIGHT_REPORT_LENGTH];
  uint8 txOptions;

  // Set the data
  pData[LIGHT_STATE_OFFSET] = lightState;
  pData[DOOR_OPENED_OFFSET] = doorRequest;
  if(doorRequest > 0){
    doorRequest = 0;
  }
  
  txOptions = AF_MSG_ACK_REQUEST;
  
  // Send the data (destination address is set to previously established binding 
  // for the commandId)
  zb_SendDataRequest( ZB_BINDING_ADDR, LIGHT_REPORT_CMD_ID, LIGHT_REPORT_LENGTH, pData, HANDLE_LIGHT_CHECK, txOptions, 0 );
  
  // Set a timer to fire the MY_LIGHT_REPORT_TIMEOUT_EVT (stopped as soon as we 
  // receive data)
  osal_start_timerEx( sapi_TaskID, MY_LIGHT_REPORT_TIMEOUT_EVT, myReportTimeout );
}