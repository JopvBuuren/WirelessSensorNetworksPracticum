/**************************************************************************************************
  Filename:       DemoApp.h

  Description:    Header file for the Sensor Demo application. This application
                  has two device types; Collectors and Sensors.

                  A collector device functions as router and may be configured
                  in gateway mode to accept binding requests from sensors. The
                  gateway node also forwards received sensor reports to its
                  serial port.

                  The sensors will periodically report sensor data to the
                  gateway in the system.


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
  PROVIDED �AS IS� WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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


#ifndef DEMO_APP_H
#define DEMO_APP_H

/******************************************************************************
 * CONSTANTS
 */

#define MY_PROFILE_ID                     0x0F20
#define MY_ENDPOINT_ID                    0x02

// Define devices
#define DEV_ID_SENSOR                     1
#define DEV_ID_ROUTER                     2
#define DEV_ID_COLLECTOR                  3

#define DEVICE_VERSION_SENSOR             1
#define DEVICE_VERSION_ROUTER             1
#define DEVICE_VERSION_COLLECTOR          1

// Define the Command ID's used in this application
#define DOOR_REPORT_CMD_ID                2
#define LIGHT_REPORT_CMD_ID               3
#define DOOR_SET_CMD_ID                   4
#define LIGHT_SET_CMD_ID                  5

// Door data format (both for report as for setting data)
#define DOOR_STATE_OFFSET                 0
#define DOOR_REPORT_LENGTH                1

// Light data format (both for report as for setting data)
#define LIGHT_STATE_OFFSET                0
#define DOOR_OPENED_OFFSET                1
#define LIGHT_REPORT_LENGTH               2

// Application States
#define APP_INIT                            0    // Initial state
#define APP_START                           1    // Node is starting
#define APP_BIND                            2    // Node is in process of binding
#define APP_RUN                             4    // Node is in running state

/******************************************************************************
 * GPIO PINS
 */
// Macro for getting a certain port/pin, simplified to 0 or 1 (as MCU_IO_GET can 
// return a value greater than 1) 
#define MCU_IO_GET_SIMPLE(port, pin) MCU_IO_GET(port, pin) > 0
   
/******************************************************************************
 * PUBLIC FUNCTIONS
 */

void initNwkConfig( void );

#endif // DEMO_APP_H