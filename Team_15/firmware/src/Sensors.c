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
#include "PacketManip.h"

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
    //sensorsData.xFakeSensorDataQueue = xQueueCreate( 10, sizeof( float ) );
    sensorsData.xSensorsToComsQueue = xQueueCreate( 10, 21);//sizeof( float ) );
    sensorsData.xSensorsToFnFQueue = xQueueCreate( 10, sizeof( float ) );
    sensorsData.xSensorsToMotorsQueue = xQueueCreate( 10, sizeof( float ) );
    /* Enable the software interrupt and set its priority. */
    //prvSetupSoftwareInterrupt();
    sensorsData.index = 0;
    sensorsData.index2 = 0;
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
    portBASE_TYPE xStatus2;
    char* data[] ={"1.914", "2.354", "3.534", "4.838", "1.023", "0.179"};
    //char* data = "9.874";
   // int cycle = 0;
    //char* ello = "hello";
    char ello[21] = "llllllllllllllllllll";
    //const char* wkki = "gwkki";
    char wkki[30] = "{\"S ns,23}";
    char wkki2[10] = "{BSens,34}";
    //char dest[30];
    //char* pntr = "hello";
    //int i;
    //int ii = 0;
    /*
    for (i=0;i<10;i++)
    {
        //des   t[i]=data[1][i];
        //*(ello+i) = *(wkki+i);
        ello[i] = wkki2[i];
        
    }
    //*
    for (i=10;i<20;i++)
    {
        //dest[i]=data[1][i];
        //*(ello+i) = *(wkki+i);
        ello[i] = wkki[ii];
        ii++;        
    }
    //*/
    //pntr = ello;
    //pntr = dest;
    
    //concatenate(wkki, wkki2, ello);
    char t[2] = "MT";
    char t2[3] = "220";
    char a[2] = "DA";
    char a2[3] = "978";
    char b[2] = "NU";
    char b2[3] = "001";    
    
    concatenate3(wkki, t, t2, a, a2, b, "222");
    wkki[0]='1';
    int x;
    if(isValidPacket(wkki))
    {
        wkki[0]='E';
    }
    else
    {
          wkki[0]='A';
    }
 
           
    /* Check the application's current state. */
    switch ( sensorsData.state )
    {
        /* Application's initial state. */
        case SENSORS_STATE_INIT:
        {
            //strcpy(dest, "yas");
            //strcat(dest, ello);
            
            if(sensorsData.index < 6){
                //xStatus = xQueueSend( sensorsData.xSensorsToComsQueue, &data[sensorsData.index], 0 );
                xStatus = xQueueSend( sensorsData.xSensorsToComsQueue, &wkki, 0 );
                sensorsData.index++;
            }
            else {
                sensorsData.index = 0;
            }


            //xStatus = xQueueSend( sensorsData.xFakeSensorDataQueue, &data, 0 );
            //xStatus = xQueueSend( xFakeSensorDataQueue2, &bob, 0);
//            if( xStatus == pdPASS )
//            {
//                // Successfully added data to queue
//                PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_1);
//            }

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
            
            
            // Second queue. Fill with fake data
            if(sensorsData.index2 < 6){
                xStatus2 = xQueueSend( sensorsData.xSensorsToFnFQueue, &data[sensorsData.index2], 0 );
                sensorsData.index2++;
            }
            else {
                sensorsData.index2 = 0;
            }
            
            
            
            // Third queue. Fill with fake data
            if(sensorsData.index3 < 6){
                xStatus2 = xQueueSend( sensorsData.xSensorsToMotorsQueue, &data[sensorsData.index3], 0 );
                sensorsData.index3++;
            }
            else {
                sensorsData.index3 = 0;
            }

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
