/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    coms.c

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

#include "coms.h"
#include "sensors.h"

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
COMS_DATA comsData;
SENSORS_DATA sensorsData;
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
    void COMS_Initialize ( void )

  Remarks:
    See prototype in coms.h.
 */
 char *stringPointer="hello";
 char *tempPointer = "buffalo";
 
 void myPrintf(float fVal)
{
    char result[100];
    int dVal, dec, i;

    fVal += 0.005;   // added after a comment from Matt McNabb, see below.

    dVal = fVal;
    dec = (int)(fVal * 100) % 100;

    memset(result, 0, 100);
    result[0] = (dec % 10) + '0';
    result[1] = (dec / 10) + '0';
    result[2] = '.';

    i = 3;
    while (dVal > 0)
    {
        result[i] = (dVal % 10) + '0';
        dVal /= 10;
        i++;
    }
    tempPointer = stringPointer;
    for (i=strlen(result)-1; i>=0; i--){
        *stringPointer = result[i];
        stringPointer++;
    }
}
bool WriteString(void)
{
    if(stringPointer == '\0')
    {
        return true;
    }

    /* Write a character at a time, only if transmitter is empty */
    
    while(*stringPointer != '\0')
    {
        /* Send character */
        while (PLIB_USART_TransmitterIsEmpty(USART_ID_1))
        {
            PLIB_USART_TransmitterByteSend(USART_ID_1, *stringPointer);

        /* Increment to address of next character */
            stringPointer++;
        
            if(*stringPointer == '\0')
            {
                return true;
            }
        }
    }
        
    return false;
}

bool PutCharacter(const char character)
{
    //char *pcString;
    char lValueToSend[10]={'a','b','c','d','e','f','g','h','i','j'};
    //unsigned long lo;
    //char * lo;
    char lo[21];
    portBASE_TYPE xStatus;
    int i;
    
    
  
    /* Check if buffer is empty for a new transmission */
    if(PLIB_USART_TransmitterIsEmpty(USART_ID_1))
    {
       // PLIB_USART_TransmitterByteSend(USART_ID_1, character);   
        /* Send character */
        
        if(PLIB_USART_ReceiverDataIsAvailable(USART_ID_1))
            {           
               char x=PLIB_USART_ReceiverByteReceive(USART_ID_1);
               PLIB_USART_TransmitterByteSend(USART_ID_1, x);
             //  PLIB_USART_TransmitterByteSend(USART_ID_1, 's');   
            
               //xStatus = xQueueSendToBack( comsData.xFakeSensorDataQueue, &lValueToSend, 0 );
               //if(xQueueReceive( comsData.xFakeSensorDataQueue3, &lo, portMAX_DELAY))
               if(xQueueReceive( sensorsData.xSensorsToComsQueue, &lo, portMAX_DELAY)) // working one
               //if(xQueueReceive( findandfollowData.xFnFToComsQueue, &lo, portMAX_DELAY))
               {   
                   
                   PLIB_USART_TransmitterByteSend(USART_ID_1, '2');   
                   stringPointer=lo;
                   //unsigned char * p =(unsigned char*)&lo;
                 // PLIB_USART_TransmitterByteSend(USART_ID_1, lo);   
//                   float thetest=42.4;
//                   myPrintf(thetest);
                   if(WriteString())
                 //  PLIB_USART_TransmitterByteSend(USART_ID_1, 'x');     
                  // PLIB_USART_TransmitterByteSend(USART_ID_1, pcString[0]);
                  PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_1);
                   for(i=0;i<99999;i++){}
                  PLIB_USART_TransmitterByteSend(USART_ID_1, '3');   
               }
                
               PLIB_USART_TransmitterByteSend(USART_ID_1, 'k');   
             
            
            }
        
        return true;
  
    }
//    else
        return false;
}

void COMS_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    comsData.state = COMS_STATE_INIT;
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    //comsData.xFakeSensorDataQueue3 = xQueueCreate( 10, sizeof( char ) );
    comsData.xComsToFnFQueue = xQueueCreate( 10, sizeof( float ) );
    comsData.xComsToSensorsQueue = xQueueCreate( 10, sizeof( float ) );
}


/******************************************************************************
  Function:
    void COMS_Tasks ( void )

  Remarks:
    See prototype in coms.h.
 */

void COMS_Tasks ( void )
{
    /* Check the application's current state. */
    switch ( comsData.state )
    {
        /* Application's initial state. */
        case COMS_STATE_INIT:
        {
            PLIB_USART_Enable(USART_ID_1);
            comsData.state = COMS_STATE_RUN;
            break;
        }

        /* TODO: implement your application state machine.*/

        /* The default state should never be executed. */
        case COMS_STATE_RUN:
        {
            /*
//            float two = 42.4;
//            myPrintf(two);
            char *pcString;
            unsigned long lValueToSend;
            portBASE_TYPE xStatus;
            char bob ='a';
            lValueToSend =42; //= (rand()*53)/100 - 0.3;
            
            xStatus = xQueueSend( comsData.xFakeSensorDataQueue3, &bob, 0 );
            if( xStatus == pdPASS )
            {
                // Successfully added data to queue
                PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_1);
            }
            */
            if(PutCharacter('B'))
            {
               
            // Toggle pin for visual assurance


            }
            //PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_1);
            /* TODO: Handle error in application's state machine. */
            break;
        }
        default:
        {
            break;
        }
    }
}
 

/*******************************************************************************
 End of File
 */
