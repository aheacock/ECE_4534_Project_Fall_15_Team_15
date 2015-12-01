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
int NUM_ENCODER_TICKS;

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

/*******************************************************************************
  Function:
    

  Summary:
    Extracts "CO" part of motor packet to get instruction
    Fromat expected -> MT:111,CO:000,NU:001
*/
int getSecondPacketValue(char* packet)
{
    char temp[4];
    temp[0] = packet[10];
    temp[1] = packet[11];
    temp[2] = packet[12];
    temp[3] = '\0';
    return atoi(temp);
}


/*******************************************************************************
  Function:
    

  Summary:
    Creates correctly formatted packet to be sent out (to find and follow).
*/
void createPacket(char ello[21], int command)
{
    char x[3];
    char num[3];
    snprintf(x, 4, "%03d", command);
    snprintf(num, 4, "%03d", motorsData.NUMBEROFPACKETSPLACEDINTHEQ);
    concatenate3(ello, "MT", "000", "CO", x, "NM", num);
}


/*******************************************************************************
  Function:
    

  Summary:
    Turns forward right
*/
void turnRight(int forwBack)
{
    int i;
    if (forwBack == 0)  // Moving forward
    {
        //PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_C, 0b00000000000000); //PORTS_BIT_POS_14); //Rev 1
        PORTCbits.RC14 = 1; // Rev 1
        //PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_G, !PORTS_BIT_POS_2); //Rev 2
        PORTGbits.RG1 = 0;  // Rev 2
        //PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_D, 0x3); // Move
        DRV_OC0_Start();
        DRV_OC1_Start();
        motorsData.motor1Counter = 0;
        //for (i=0; i<10000000; i++) {}
        //PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_D, 0x0); // Stop
    }
    else if (forwBack == 1) // Moving backward
    {
        PORTCbits.RC14 = 0; // Rev 1
        PORTGbits.RG1 = 1;  // Rev 2
        //PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_D, 0x3); // Move
        DRV_OC0_Start();
        DRV_OC1_Start();
        motorsData.motor1Counter = 0;
        //for (i=0; i<10000000; i++) {}
        //PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_D, 0x0); // Stop
    }
}


/*******************************************************************************
  Function:
    

  Summary:
    Turns forward left
*/
void turnLeft(int forwBack)
{
    int i;
    if (forwBack == 0)  // Moving forward
    {
        //PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_C, 0b00000000000000); //PORTS_BIT_POS_14); //Rev 1
        PORTCbits.RC14 = 0; // Rev 1
        //PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_G, !PORTS_BIT_POS_2); //Rev 2
        PORTGbits.RG1 = 1;  // Rev 2
        //PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_D, 0x3); // Move
        DRV_OC0_Start();
        DRV_OC1_Start();
        motorsData.motor1Counter = 0;
        //for (i=0; i<10000000; i++) {}
        //PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_D, 0x0); // Stop
    }
    else if (forwBack == 1) // Moving backward
    {
        PORTCbits.RC14 = 1; // Rev 1
        PORTGbits.RG1 = 0;  // Rev 2
        //PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_D, 0x3); // Move
        DRV_OC0_Start();
        DRV_OC1_Start();
        motorsData.motor1Counter = 0;
        //for (i=0; i<10000000; i++) {}
        //PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_D, 0x0); // Stop
    }
}


/*******************************************************************************
  Function:
    

  Summary:
    Moves forward
*/
void moveForward()
{
    int i;
    PORTCbits.RC14 = 0; // Rev 1
    PORTGbits.RG1 = 0;  // Rev 2
    //PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_D, 0x3); // Move
    DRV_OC0_Start();
    DRV_OC1_Start();
    motorsData.motor1Counter = 0;
    //for (i=0; i<10000000; i++) {}
    //PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_D, 0x0); // Stop
}


/*******************************************************************************
  Function:
    

  Summary:
    Moves backward
*/
void moveBackward()
{
    int i;
    PORTCbits.RC14 = 1; // Rev 1
    PORTGbits.RG1 = 1;  // Rev 2
    //PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_D, 0x3); // Move
    DRV_OC0_Start();
    DRV_OC1_Start();
    motorsData.motor1Counter = 0;
    //for (i=0; i<10000000; i++) {}
    //PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_D, 0x0); // Stop
}


/*******************************************************************************
  Function:
    

  Summary:
    Stops. This should only be called by the interrupt, but can be called
    externally if needed.
*/
void stop()
{
    //PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_D, 0x0); // Stop
    DRV_OC0_Stop();
    DRV_OC1_Stop();
}


/*******************************************************************************
  Function:
    

  Summary:
    Interrupt handler for motor1 encoder. Interrupts on rising edge
*/
void IntHandlerExternalInterruptInstance0(void)
{           
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_EXTERNAL_1);
    
    if (motorsData.motor1Counter >= 0 && motorsData.motor1Counter <= NUM_ENCODER_TICKS)
        motorsData.motor1Counter++;
    else if (motorsData.motor1Counter > NUM_ENCODER_TICKS)
    {
        stop();
        motorsData.motor1Counter = -1;
        motorsData.shouldSendNow = 1;
    }
    PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_1);
}


/*******************************************************************************
  Function:
    

  Summary:
    Interrupt handler for motor2 encoder. Interrupts on rising edge.
    Not currently used.
*/
void IntHandlerExternalInterruptInstance1(void)
{           
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_EXTERNAL_2);
    motorsData.motor2Counter++;
}


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

void MOTORS_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    motorsData.state = MOTORS_STATE_INIT;
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    // State var
    motorsData.state = MOTORS_STATE_INIT;
    
    // Error vars
    motorsData.NUMBEROFPACKETSPLACEDINTHEQ=0;
    motorsData.NUMBEROFPACKETSDROPPEDBEFOREQ=0;
    motorsData.packetsrecievedinFindandFollow=0;
    
    // Create motor queues
    motorsData.xMotorsToSensorsQueue = xQueueCreate( 15, 51 );
    motorsData.xMotorsToFnFQueue = xQueueCreate( 15, 51 );
    
    motorsData.index = 0;
    
    // Packet sending
    motorsData.shouldSendNow = 0;
    motorsData.nextPacketToSend;
    createPacket(motorsData.nextPacketToSend, 0);
    
    // Start PWM components
    DRV_TMR0_Start();
    DRV_OC0_Enable();
    DRV_OC1_Enable();
    PLIB_OC_PulseWidth16BitSet(OC_ID_1, 125);   // 125=top, 63=half
    PLIB_OC_PulseWidth16BitSet(OC_ID_2, 125);
    stop();     // Make sure initially stopped
    
    // Set counter variables for quadrature encoder
    motorsData.motor1Counter = -1;
    motorsData.motor2Counter = -1;
    
    // Set constant value
    NUM_ENCODER_TICKS = 110;
    // 540 ticks per 90 degrees, so 90 ticks per 15 degrees
    // Without 1000 task delay in system_tasks.c
    // 660 ticks per 90 degrees, so 110 ticks per 15 degrees
    
    // This line can be used for debugging
    //PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_1);
    
    // Init testing data
    //*
    strcpy(motorsData.listOfCommands[0], "MT:111,CO:002,NU:001");
    strcpy(motorsData.listOfCommands[1], "MT:111,CO:002,NU:001");
    strcpy(motorsData.listOfCommands[2], "MT:111,CO:002,NU:001");
    strcpy(motorsData.listOfCommands[3], "MT:111,CO:002,NU:001");
    strcpy(motorsData.listOfCommands[4], "MT:111,CO:002,NU:001");
    strcpy(motorsData.listOfCommands[5], "MT:111,CO:002,NU:001");
    strcpy(motorsData.listOfCommands[6], "MT:111,CO:002,NU:001");
    strcpy(motorsData.listOfCommands[7], "MT:111,CO:002,NU:001");
    strcpy(motorsData.listOfCommands[8], "MT:111,CO:002,NU:001");
    strcpy(motorsData.listOfCommands[9], "MT:111,CO:002,NU:001");
    strcpy(motorsData.listOfCommands[10], "MT:111,CO:002,NU:001");   // 1
    strcpy(motorsData.listOfCommands[11], "MT:111,CO:002,NU:001");
    strcpy(motorsData.listOfCommands[12], "MT:111,CO:002,NU:001");
    strcpy(motorsData.listOfCommands[13], "MT:111,CO:002,NU:001");
    strcpy(motorsData.listOfCommands[14], "MT:111,CO:002,NU:001");
    strcpy(motorsData.listOfCommands[15], "MT:111,CO:002,NU:001");   // 6
    strcpy(motorsData.listOfCommands[16], "MT:111,CO:003,NU:001");   // 1
    strcpy(motorsData.listOfCommands[17], "MT:111,CO:003,NU:001");
    strcpy(motorsData.listOfCommands[18], "MT:111,CO:003,NU:001");
    strcpy(motorsData.listOfCommands[19], "MT:111,CO:003,NU:001");
    strcpy(motorsData.listOfCommands[20], "MT:111,CO:003,NU:001");
    strcpy(motorsData.listOfCommands[21], "MT:111,CO:003,NU:001");   // 6
    strcpy(motorsData.listOfCommands[22], "MT:111,CO:001,NU:001");
    strcpy(motorsData.listOfCommands[23], "MT:111,CO:001,NU:001");
    strcpy(motorsData.listOfCommands[24], "MT:111,CO:011,NU:001");
    strcpy(motorsData.listOfCommands[25], "MT:111,CO:011,NU:001");
    strcpy(motorsData.listOfCommands[26], "MT:111,CO:012,NU:001");
    strcpy(motorsData.listOfCommands[27], "MT:111,CO:012,NU:001");
    strcpy(motorsData.listOfCommands[28], "MT:111,CO:013,NU:001");
    strcpy(motorsData.listOfCommands[29], "MT:111,CO:013,NU:001");
    strcpy(motorsData.listOfCommands[30], "MT:111,CO:011,NU:001");
    //*/
    /*
    int ar[87] = {1,1,1,1,1,1,1,1,1,1,
                2,2,2,2,2,2,2,2,2,2,2,2,
                1,1,1,1,1,1,1,1,1,1,
                3,3,3,3,3,3,
                1,1,1,1,1,1,1,1,1,1,
                2,2,2,
                1,1,1,1,1,1,1,1,1,1,
                2,2,2,2,2,2,2,2,2,2,2,2,
                1,1,1,1,1,
                3,3,3,
                1,1,1,1,1,1,1,1,1,1,
                2,2,2,
                1,1,1,1,1};
    int i;
    for (i=0; i<87; i++)
        createPacket(motorsData.listOfCommands[i], ar[i]);
    */
    motorsData.indexCom = 0;
}


/******************************************************************************
  Function:
    void MOTORS_Tasks ( void )

  Remarks:
    See prototype in motors.h.
 */

void MOTORS_Tasks ( void )
{
    /* Check the application's current state. */
    switch ( motorsData.state )
    {
        /* Application's initial state. */
        case MOTORS_STATE_INIT:
        {
            if(motorsData.shouldSendNow == 1)
            {
                if (PLIB_USART_TransmitterIsEmpty(USART_ID_1))
                {
                    PLIB_USART_TransmitterByteSend(USART_ID_1, motorsData.nextPacketToSend[12]);
                }
                motorsData.shouldSendNow = 0;
            }
            //*
            // Send error and ack packets 
            if(motorsData.shouldSendNow == 1 && uxQueueSpacesAvailable(motorsData.xMotorsToFnFQueue)==15)    // Check if queue is filled
            {
                if(isValidPacket(motorsData.nextPacketToSend))
                {
                    motorsData.shouldSendNow = 0;
                    // Send dummy data over uart for testing
                    if( xQueueSend( motorsData.xMotorsToFnFQueue, &motorsData.nextPacketToSend, 0 ))
                    {
                        motorsData.NUMBEROFPACKETSPLACEDINTHEQ=motorsData.NUMBEROFPACKETSPLACEDINTHEQ+1;
                    }
                    else 
                    {
                        motorsData.NUMBEROFPACKETSDROPPEDBEFOREQ=motorsData.NUMBEROFPACKETSDROPPEDBEFOREQ+1;
                    }
                    
                    // Send error packets every 5 packets
                    if(((motorsData.NUMBEROFPACKETSPLACEDINTHEQ)%5)==0)
                    {
                        char ello[42];
                        char RF_c[3];
                        char PP[3];
                        char num[3];
                        int numpacketsdropped = motorsData.packetsrecievedinFindandFollow-motorsData.NUMBEROFPACKETSPLACEDINTHEQ;
                        snprintf(RF_c, 4,"%03d",findandfollowData.numdropped);
                        snprintf(PP, 4,"%03d",motorsData.packetsrecievedinFindandFollow);
                        snprintf(num, 4,"%03d",motorsData.NUMBEROFPACKETSPLACEDINTHEQ);
        
                        // Create packet
                        concatenate5(ello,"EM","000","RM",RF_c,"NS",num,"NR",PP,"NP", num);
                        
                        // Put in queue
                        if(xQueueSend( motorsData.xMotorsToFnFQueue, &ello, 0 ))
                        {
                            motorsData.NUMBEROFPACKETSPLACEDINTHEQ=motorsData.NUMBEROFPACKETSPLACEDINTHEQ+1;
                        }
                        else 
                        {
                            motorsData.NUMBEROFPACKETSDROPPEDBEFOREQ=motorsData.NUMBEROFPACKETSDROPPEDBEFOREQ+1;
                        }
                    }
               }
               else
               {
                    // If the packet formation was invalid, send all 9s
                    int i=0;
                    char ello[42];
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
            //*/
            // Receive commands from FnF
            if (motorsData.motor1Counter == -1) // Motors aren't moving
            {
                int x;
                char lo[42];
                //strcpy(lo, motorsData.listOfCommands[motorsData.indexCom]);
                motorsData.indexCom++;
                if (motorsData.indexCom == 31)
                    motorsData.indexCom = 0;
                //char loo[42] = "MT:111,CO:002,NU:001";
                if(xQueueReceive(findandfollowData.xFnFToMotorsQueue, &lo, 0))
                {
                    // Convert received packet to motor instruction
                 //  if(isValidPacket(lo))
                    {
                        x = getSecondPacketValue(lo);
                        /*
                        if (PLIB_USART_TransmitterIsEmpty(USART_ID_1))
                        {
                            PLIB_USART_TransmitterByteSend(USART_ID_1, loo[12]);
                        }
                        */
                        //*
                        if (x == 0)
                        {
                            stop();
                            createPacket(motorsData.nextPacketToSend, 0);
                        }
                        else if (x == 1)
                        {
                            moveForward();
                            createPacket(motorsData.nextPacketToSend, 1);
                        }
                        else if (x == 2)
                        {
                            turnRight(0);
                            createPacket(motorsData.nextPacketToSend, 2);
                        }
                        else if (x == 3)
                        {
                            turnLeft(0);
                            createPacket(motorsData.nextPacketToSend, 3);
                        }
                        else if (x == 11)
                        {
                            moveBackward();
                            createPacket(motorsData.nextPacketToSend, 11);
                        }
                        else if (x == 12)
                        {
                            turnRight(1);
                            createPacket(motorsData.nextPacketToSend, 12);
                        }
                        else if (x == 13)
                        {
                            turnLeft(1);
                            createPacket(motorsData.nextPacketToSend, 13);
                        }
                        //*/
                    }
                    
                    // Send Ack
                    if(uxQueueSpacesAvailable(motorsData.xMotorsToFnFQueue)==15)
                    {
                        char temp[21];
                        char num[3];
                        snprintf(num, 4, "%03d", motorsData.NUMBEROFPACKETSPLACEDINTHEQ);
                        concatenate3(temp, "AM", "000", "AN", num, "NU", num);
                        if (xQueueSend(motorsData.xMotorsToFnFQueue, temp, 0))
                        {
                            motorsData.NUMBEROFPACKETSPLACEDINTHEQ += 1;
                        }
                        else 
                        {
                            motorsData.NUMBEROFPACKETSDROPPEDBEFOREQ=motorsData.NUMBEROFPACKETSDROPPEDBEFOREQ+1;
                        }
                    }
                }
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
