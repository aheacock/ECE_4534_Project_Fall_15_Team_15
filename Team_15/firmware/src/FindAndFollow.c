/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    findandfollow.c

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

#include "findandfollow.h"

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

FINDANDFOLLOW_DATA findandfollowData;

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
    void FINDANDFOLLOW_Initialize ( void )

  Remarks:
    See prototype in findandfollow.h.
 */

void FINDANDFOLLOW_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    findandfollowData.state = FINDANDFOLLOW_STATE_INIT;
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    findandfollowData.xFnFToMotorsQueue = xQueueCreate( 10, sizeof( float ) );
    findandfollowData.xFnFToComsQueue = xQueueCreate( 15, 45);
    findandfollowData.xFnFToSensorsQueue = xQueueCreate( 10, sizeof( float ) );
    
    findandfollowData.index = 0;
}


/******************************************************************************
  Function:
    void FINDANDFOLLOW_Tasks ( void )

  Remarks:
    See prototype in findandfollow.h.
 */

void FINDANDFOLLOW_Tasks ( void )
{
    
    //FindandFollow:<4f>RF:<string>,LF:<string>,CF:<string>
   
    
    char ello[40] = "FindandFollow:__,RF:333,LF:333,CF:333";
  //  const char* wkki = "gwkki";
    char errorcode[3] ="a12";
    char RF[3]="234";
    char LF[3]="234";
    char CF[3]="234";
   

    int i;
   
    //add error code
    for (i=14;i<16;i++)
    {
        ello[i] = errorcode[i-14];
        
    }
    //Add RF data
    for (i=20;i<23;i++)
    {
        ello[i] = RF[i-20];
        i++;        
    }
        //Add LF data
    for (i=27;i<31;i++)
    {
        ello[i] = LF[i-27];
        i++;        
    }
            //Add CF data
    for (i=35;i<37;i++)
    {
        ello[i] = CF[i-35];
        i++;        
    }
    
   // &ello
    portBASE_TYPE xStatus;
    const char* data[] ={"1.91", "2.34", "3.54", "4.88", "1.03", "0.19"};
    /* Check the application's current state. */
    switch ( findandfollowData.state )
    {
        /* Application's initial state. */
        case FINDANDFOLLOW_STATE_INIT:
        {
            //xStatus = xQueueSendToBack( sensorsData.xFakeSensorDataQueue, &lValueToSend, 0 );

            if(findandfollowData.index < 6){
                xStatus = xQueueSend( findandfollowData.xFnFToMotorsQueue, &data[findandfollowData.index], 0 );
                findandfollowData.index++;
            }
            else {
                findandfollowData.index = 0;
            }
            
            
            // Second queue. Fill with fake data
           // if(findandfollowData.index2 < 6){
                xStatus = xQueueSend( findandfollowData.xFnFToComsQueue, &ello, 0 );
             //   findandfollowData.index2++;
            //}
            //else {
             //   findandfollowData.index2 = 0;
            //}
            
            
            // Third queue. Fill with fake data
            if(findandfollowData.index3 < 6){
                xStatus = xQueueSend( findandfollowData.xFnFToSensorsQueue, &data[findandfollowData.index3], 0 );
                findandfollowData.index3++;
            }
            else {
                findandfollowData.index3 = 0;
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
