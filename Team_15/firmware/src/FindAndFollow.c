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
MOTORS_DATA motorsData;

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
    findandfollowData.xFnFToMotorsQueue = xQueueCreate( 15, 42);
    findandfollowData.xFnFToComsQueue = xQueueCreate( 15, 42);
    findandfollowData.xFnFToSensorsQueue = xQueueCreate( 15, 42);
       findandfollowData.xFnFToMotors = xQueueCreate( 15, 42);
    findandfollowData.index = 0;
}


int recievefrommotors()
{
    
    int x = 0;
    char lo[42];
    if(xQueueReceive(motorsData.xMotorsToFnFQueue, &lo, 0)) // working one
    {   
      //  stringPointer=lo;
        
        if(isValidPacket(lo))
        {
                if(xQueueSend( findandfollowData.xFnFToComsQueue, &lo, 0 ))
                {
                    findandfollowData.NUMBEROFPACKETSPLACEDINTHEQ=findandfollowData.NUMBEROFPACKETSPLACEDINTHEQ+1;
                }
                else 
                {
                   findandfollowData.NUMBEROFPACKETSDROPPEDBEFOREQ =findandfollowData.NUMBEROFPACKETSDROPPEDBEFOREQ+1;
                }
        }
     //   else 
        {
         //   lo[0]='6';
        }    
    }
    return x;
}




/******************************************************************************
  Function:
    void FINDANDFOLLOW_Tasks ( void )

  Remarks:
    See prototype in findandfollow.h.
 */

void FINDANDFOLLOW_Tasks ( void )
{
    

    
   
  

    
   // &ello
    
    /* Check the application's current state. */
    switch ( findandfollowData.state )
    {
        /* Application's initial state. */
        case FINDANDFOLLOW_STATE_INIT:
        {
            
            
             char ello[42];
            
            //xStatus = xQueueSendToBack( sensorsData.xFakeSensorDataQueue, &lValueToSend, 0 );
            //    xStatus = xQueueSend( findandfollowData.xFnFToMotorsQueue, &data[findandfollowData.index], 0 );   
         
                int RF = 234;
                int LF = 125;
                int CF = 114;
                
               char RF_c[3];
               char LF_c[3];
               char CF_c[3];
               char NumofPackets[3];
                
                snprintf(RF_c, 4,"%d", RF);
                snprintf(LF_c, 4,"%d", LF);
                snprintf(CF_c, 4,"%d", CF);
                snprintf(NumofPackets, 4,"%d",findandfollowData.NUMBEROFPACKETSPLACEDINTHEQ);
               if(uxQueueSpacesAvailable(findandfollowData.xFnFToComsQueue)==15)
                { 
                    recievefrommotors();
          
                 
                // concatenate6(, , , , , , , , ,, , char fiv[2], char fivNum[3])              
               concatenate6(ello,"FF","___","RF",RF_c,"LF",LF_c,"CF",CF_c,"NF",NumofPackets,"FF", "XXX");   
                //xStatus = xQueueSend( findandfollowData.xFnFToMotorsQueue, &data[findandfollowData.index], 0 );
             if(xQueueSend( findandfollowData.xFnFToComsQueue, &ello, 0 ))
                {
                    findandfollowData.NUMBEROFPACKETSPLACEDINTHEQ=findandfollowData.NUMBEROFPACKETSPLACEDINTHEQ+1;
                }
               else 
                {
                   findandfollowData.NUMBEROFPACKETSDROPPEDBEFOREQ =findandfollowData.NUMBEROFPACKETSDROPPEDBEFOREQ+1;
                }
           
                }
            // Third queue. Fill with fake data
           
           //     xStatus = xQueueSend( findandfollowData.xFnFToSensorsQueue, &data[findandfollowData.index3], 0 );
           
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
