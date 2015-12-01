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
//extern 

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
    int notSent = 1;
    bool ret;
    
    if(stringPointer == '\0')
    {
        return true;
    }
    
    /* Write a character at a time, only if transmitter is empty */
    
    // Send beginning of "JSON" packet
    while(notSent == 1)
    {
        if (PLIB_USART_TransmitterIsEmpty(USART_ID_1))
        {
            PLIB_USART_TransmitterByteSend(USART_ID_1, '{');
            notSent = 0;    // { was sent
        }
    }
    
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
                ret = true;
            }
        }
    }
    
    // Sending ending packet
    notSent = 1;
    while(notSent == 1)
    {
        if (PLIB_USART_TransmitterIsEmpty(USART_ID_1))
        {
            //PLIB_USART_TransmitterByteSend(USART_ID_1, '}');
            PLIB_USART_TransmitterByteSend(USART_ID_1, '}');
            notSent = 0;    // } was sent
        }
    }
    if (ret == true)
        return true;
    else
        return false;
}


int TalkToFindAndFollow()
{
    int x = 0;
    int valid = 0;
//    char lo[42];
    char lo[51];
    char dest[3];
    char dest2[3];
    char temp[21];
    if(xQueueReceive( findandfollowData.xFnFToComsQueue, &lo, 0))
    {
        if (comsData.NumPacketsRecvFromFnFQ == 999)
            comsData.NumPacketsRecvFromFnFQ = 0;
        else
            comsData.NumPacketsRecvFromFnFQ += 1;
        valid = isValidPacket(&lo);
        if (valid == 0)
        {
            stringPointer=lo;
            if(WriteString())
            {
                x = getSequenceNumber(dest, &lo);
            }
        }
        else
        {
            //intTo3Char(stringPointer, valid);
            if (comsData.NumBadPacketsRecvFromFnFQ == 999)
                comsData.NumBadPacketsRecvFromFnFQ = 0;
            else
                comsData.NumBadPacketsRecvFromFnFQ += 1;
        }
    }
  /*  ack breaks the system idk why?
    // Ack here????????????
    intTo3Char(dest2, comsData.NumPacketsPutInFnFQ);
    concatenate3(temp, "AF", "000", "AN", "Te2", "NU", "Tes");
   // void concatenate3(char dest[21], char type[2], char typeNum[3], char one[2], char oneNum[3], char two[2], char twoNum[3] )
    if (xQueueSend( comsData.xComsToFnFQueue, temp, 0))
    {
        if (comsData.NumPacketsPutInFnFQ == 999)
            comsData.NumPacketsPutInFnFQ = 0;
        else
            comsData.NumPacketsPutInFnFQ += 1;
    }
    stringPointer = temp;
    WriteString();
   * */
    return x;
}

int TalkToSensors()
{
    int x = 0;
    int valid = 0;
    char lo[51];
    char dest[3];
    char dest2[3];
    char temp[21];
  

    if(xQueueReceive( sensorsData.xSensorsToComsQueue, &lo, 0)) // working one
    {
        if (comsData.NumPacketsRecvFromSensorsQ == 999)
            comsData.NumPacketsRecvFromSensorsQ = 0;
        else
            comsData.NumPacketsRecvFromSensorsQ += 1;
        valid = isValidPacket(&lo);
        if (valid == 0)
        {
            stringPointer=lo;
            if(WriteString())
            {
                x = getSequenceNumber(dest, &lo);
            }
        }
        else
        {
            //intTo3Char(stringPointer, valid);
            if (comsData.NumBadPacketsRecvFromSensorsQ == 999)
                comsData.NumBadPacketsRecvFromSensorsQ = 0;
            else
                comsData.NumBadPacketsRecvFromSensorsQ += 1;
        }
    }
    
    // Ack
    //getSequenceNumber(dest,&lo);
    intTo3Char(dest2, comsData.NumPacketsPutInSensorsQ);
    concatenate3(temp, "AS", "000", "AN", dest, "NU", dest2);
    if (xQueueSend( comsData.xComsToSensorsQueue, temp, 0))
    {
        if (comsData.NumPacketsPutInSensorsQ == 999)
            comsData.NumPacketsPutInSensorsQ = 0;
        else
            comsData.NumPacketsPutInSensorsQ += 1;
    }
    stringPointer = temp;
    WriteString();
    return x;
}

void checkreset()
{
         if(PLIB_USART_ReceiverDataIsAvailable(USART_ID_1))
            {   
            char x=PLIB_USART_ReceiverByteReceive(USART_ID_1);
            if(x=='?')
                {
                 stringPointer="XXXXXX";
                 WriteString();
                 int i=0;
                 while (i<1000)
                 {
                    i++;
                 }
                 SYS_RESET_SoftwareReset();   
                }
            }
}

bool PutCharacter(const char character)
{
   
    //char *pcString;
    //char lValueToSend[10]={'a','b','c','d','e','f','g','h','i','j'};
    //unsigned long lo;
    //char * lo;
    //char lo[21];
    //portBASE_TYPE xStatus;
    //int i;
    
    
  
    /* Check if buffer is empty for a new transmission */
    if(PLIB_USART_TransmitterIsEmpty(USART_ID_1))
    {
       // PLIB_USART_TransmitterByteSend(USART_ID_1, character);   
        /* Send character */
        
       //a if(PLIB_USART_ReceiverDataIsAvailable(USART_ID_1))
            {     
             
         // a     char x=PLIB_USART_ReceiverByteReceive(USART_ID_1);
           //a    PLIB_USART_TransmitterByteSend(USART_ID_1, x);
             //  PLIB_USART_TransmitterByteSend(USART_ID_1, 's');   
            
               //xStatus = xQueueSendToBack( comsData.xFakeSensorDataQueue, &lValueToSend, 0 );
               //if(xQueueReceive( comsData.xFakeSensorDataQueue3, &lo, portMAX_DELAY))
               //a if(xQueueReceive( sensorsData.xSensorsToComsQueue, &lo, portMAX_DELAY)) // working one
               //if(xQueueReceive( findandfollowData.xFnFToComsQueue, &lo, portMAX_DELAY))
               {   
                   
                   //PLIB_USART_TransmitterByteSend(USART_ID_1, '2');   
                   //a stringPointer=lo;
                   //unsigned char * p =(unsigned char*)&lo;
                 // PLIB_USART_TransmitterByteSend(USART_ID_1, lo);   
//                   float thetest=42.4;
//                   myPrintf(thetest);
       //if(WriteString())
                  TalkToFindAndFollow();
                 // PLIB_USART_TransmitterByteSend(USART_ID_1, ' ');
         //works         TalkToSensors();
                  
                 //  PLIB_USART_TransmitterByteSend(USART_ID_1, 'x');     
                  // PLIB_USART_TransmitterByteSend(USART_ID_1, pcString[0]);
                  //PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_1);
                  //for(i=0;i<99999;i++){}
                  //PLIB_USART_TransmitterByteSend(USART_ID_1, '3');
               }
                
               //PLIB_USART_TransmitterByteSend(USART_ID_1, 'k');   
             
            
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
    comsData.xComsToFnFQueue = xQueueCreate( 15, 42 );
    comsData.xComsToSensorsQueue = xQueueCreate( 15, 42 );
    
    comsData.NumPacketsPutInFnFQ = 0;
    comsData.NumPacketsPutInSensorsQ = 0;
    comsData.NumPacketsRecvFromFnFQ = 0;
    comsData.NumPacketsRecvFromSensorsQ = 0;
    comsData.NumBadPacketsRecvFromFnFQ = 0;
    comsData.NumBadPacketsRecvFromSensorsQ = 0;;
}


/******************************************************************************
  Function:
    void COMS_Tasks ( void )

  Remarks:
    See prototype in coms.h.
 */

void COMS_Tasks ( void )
{
    int ackFF;
    int ackSR;
 
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
            //calls functions that run the Coms thread
            ackFF = TalkToFindAndFollow();
        //    ackSR = TalkToSensors();
            // checks to see if the system reset button ( '?' ) has been pressed
            checkreset();
            
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
