/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    motors.c

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

#include "motors.h"

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

MOTORS_DATA motorsData;
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
    void MOTORS_Initialize ( void )

  Remarks:
    See prototype in motors.h.
 */

int recieveAKKK()
{
    int x=0;
     char lo[42];
    if(xQueueReceive(findandfollowData.xFnFToMotors, &lo, 0)) // working one
    {   
       if (lo[0]='A')
           motorsData.packetsrecievedinFindandFollow++;
        x=1;
    }
    else
    {
        x=0;
    }
     
     return 0;
}
int TalkToFnF()
{
    
    int x = 0;
    char lo[51];
    if(xQueueReceive(findandfollowData.xFnFToMotors, &lo, 0)) // working one
    {   
      //  stringPointer=lo;
        if(isValidPacket(lo))
        {
            lo[0]='A';
        }
        else 
        {
            lo[0]='6';
        }    
    }
    return x;
}
void MOTORS_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    motorsData.state = MOTORS_STATE_INIT;
      motorsData.NUMBEROFPACKETSPLACEDINTHEQ=0;
      motorsData.NUMBEROFPACKETSDROPPEDBEFOREQ=0;
      motorsData.packetsrecievedinFindandFollow=0;
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    
    motorsData.xMotorsToSensorsQueue = xQueueCreate( 15, 51 );
    motorsData.xMotorsToFnFQueue = xQueueCreate( 15, 51 );
    
    /* Enable the software interrupt and set its priority. */
    //prvSetupSoftwareInterrupt();
    motorsData.index = 0;
}


/******************************************************************************
 * 
 *
 * 
 * 
 * 
  Function:
    void MOTORS_Tasks ( void )

  Remarks:
    See prototype in motors.h.
 */

void MOTORS_Tasks ( void )
{
    switch ( motorsData.state )
    {
        /* Application's initial state. */
        case MOTORS_STATE_INIT:
        {
            
             char ello[42];
                
           //Grabs data from the 4 sensors
                int RS = 777;
                int LS = 777;
                int FS = 777;
                int BS = 777;
                
               char RS_c[3];
               char LS_c[3];
               char FS_c[3];
               char BS_c[3];
               char NumofPackets[3];
                
                snprintf(RS_c, 4,"%d", RS);
                snprintf(LS_c, 4,"%d", LS);
                snprintf(FS_c, 4,"%d", FS);
                 snprintf(BS_c, 4,"%d", BS);
                 
           
               snprintf(NumofPackets, 4,"%d",motorsData.NUMBEROFPACKETSPLACEDINTHEQ);
               concatenate6(ello,"MR","___","QW",RS_c,"QE",LS_c,"QT","QQQ","QY",BS_c,"NM", NumofPackets);
              
               
         if(uxQueueSpacesAvailable(motorsData.xMotorsToFnFQueue)==15)
            { 
               if(isValidPacket(ello))
               {
            
                    if( xQueueSend( motorsData.xMotorsToFnFQueue, &ello, 0 ))
                    {
                        motorsData.NUMBEROFPACKETSPLACEDINTHEQ=motorsData.NUMBEROFPACKETSPLACEDINTHEQ+1;
                    }
                    else 
                    {
                        motorsData.NUMBEROFPACKETSDROPPEDBEFOREQ=motorsData.NUMBEROFPACKETSDROPPEDBEFOREQ+1;
                    }
   
                    
                    if(((motorsData.NUMBEROFPACKETSPLACEDINTHEQ)%5)==0)
                    {
                        
                        char RF_c[3];
                        char PP[3];
                       int numpacketsdropped = motorsData.packetsrecievedinFindandFollow-motorsData.NUMBEROFPACKETSPLACEDINTHEQ;
                        snprintf(RF_c, 4,"%03d",findandfollowData.numdropped);
                        
                        char NumofPackets[3];
                        snprintf(PP, 4,"%03d",motorsData.packetsrecievedinFindandFollow);
                        snprintf(NumofPackets, 4,"%d",motorsData.NUMBEROFPACKETSPLACEDINTHEQ);
        
                        concatenate6(ello,"EM","XXX","XX",RS_c,"XX",LS_c,"Xd",RF_c,"XX",PP,"NP", NumofPackets);
                      if(xQueueSend( motorsData.xMotorsToFnFQueue, &ello, 0 ))
                      {
                             motorsData.NUMBEROFPACKETSPLACEDINTHEQ=motorsData.NUMBEROFPACKETSPLACEDINTHEQ+1;
                      }
                       
                        
                    }
               }
               else
               {
                   int i=0;
                   for(i=0; i<42; i++)
                    ello[i]='9';
                        // malformed packet exception
                    if( xQueueSend( motorsData.xMotorsToFnFQueue, &ello, 0 ))
                        {
                         motorsData.NUMBEROFPACKETSPLACEDINTHEQ=motorsData.NUMBEROFPACKETSPLACEDINTHEQ+1;
                        }
                    else 
                        {
                          motorsData.NUMBEROFPACKETSDROPPEDBEFOREQ=motorsData.NUMBEROFPACKETSDROPPEDBEFOREQ+1;
                        }
                
               }
         } 
            //    xStatus = xQueueSend( motorsData.xMotorsToSensorsQueue, &data[motorsData.index], 0 );
             
              
              
              // Second queue. Fill with fake data
            
            //    xStatus = xQueueSend( motorsData.xMotorsToFnFQueue, &data[motorsData.index2], 0 );
             
            
            
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
