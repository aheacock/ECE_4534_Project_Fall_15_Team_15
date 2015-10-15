/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    sensors.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "sensors.h"
#include <stdlib.h> // For random

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

SENSORS_DATA sensorsData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback funtions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void SENSORS_Initialize ( void )

  Remarks:
    See prototype in sensors.h.
 */

void SENSORS_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    sensorsData.state = SENSORS_STATE_INIT;
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    sensorsData.xFakeSensorDataQueue = xQueueCreate( 10, sizeof( float ) );
    /* Enable the software interrupt and set its priority. */
    //prvSetupSoftwareInterrupt();
    sensorsData.index = 0;
}


/******************************************************************************
  Function:
    void SENSORS_Tasks ( void )

  Remarks:
    See prototype in sensors.h.
 */

void SENSORS_Tasks ( void )
{
    //long lValueToSend;
    portBASE_TYPE xStatus;
    const char* data[] ={"1.91", "2.34", "3.54", "4.88", "1.03", "0.19"};
   // int cycle = 0;   
    
    
    /* Check the application's current state. */
    switch ( sensorsData.state )
    {
        /* Application's initial state. */
        case SENSORS_STATE_INIT:
        {
                    // Sensor outputs -0.3 to Vcc+0.33. Vcc should  be 5. So the range is 
        // -0.3 to 5.3
        //lValueToSend =42; //= (rand()*53)/100 - 0.3;
        
        //xStatus = xQueueSendToBack( sensorsData.xFakeSensorDataQueue, &lValueToSend, 0 );
         
        if(sensorsData.index < 6){
            xStatus = xQueueSend( sensorsData.xFakeSensorDataQueue, &data[sensorsData.index], 0 );
            sensorsData.index++;
        }
        else {
            sensorsData.index = 0;
        }
      
        
        //xStatus = xQueueSend( sensorsData.xFakeSensorDataQueue, &data, 0 );
        //xStatus = xQueueSend( xFakeSensorDataQueue2, &bob, 0);
        if( xStatus == pdPASS )
        {
            // Successfully added data to queue
            PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_1);
        }
        
        /*
         * 
         * This code needs to go where sending data with wifly
         * 
         */
        /* Block on the queue to wait for data to arrive. */
        // --> xQueueReceive( sensorsData.xFakeSensorDataQueue, &pcString, portMAX_DELAY );
        
        /* Allow the other sender task to execute. taskYIELD() informs the
        scheduler that a switch to another task should occur now rather than
        keeping this task in the Running state until the end of the current time
        slice. */
        //taskYIELD();
            break;
        }

        /* TODO: implement your application state machine.*/

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}
 

/*******************************************************************************
 End of File
 */
