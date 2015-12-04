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
COMS_DATA comsData;
// vars for testing
    int RS;
    int LS;
    int FS;
    int BS;
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



void APP_ReadComplete (void *handle)
{
    sensorsData.rdComplete = true;
}

void APP_WriteComplete (void *handle)
{
    sensorsData.wrComplete = true;
}

void APP_Reset ()
{
    sensorsData.rdComplete = true;
    sensorsData.wrComplete = true;
}


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
    
    // Set up the queues
    //sensorsData.xFakeSensorDataQueue = xQueueCreate( 10, sizeof( float ) );
    sensorsData.xSensorsToComsQueue = xQueueCreate( 15, 51);//sizeof( float ) );
    sensorsData.xSensorsToFnFQueue = xQueueCreate( 15, 51 );
    sensorsData.xSensorsToMotorsQueue = xQueueCreate( 15, 51 );
    
    /* Enable the software interrupt and set its priority. */
    //prvSetupSoftwareInterrupt();
    
    //Sensor Data Acquisition
    sensorsData.wrComplete = true;
    sensorsData.rdComplete = true; 
    
    //sensorsData.xFakeSensorDataQueue = xQueueCreate( 10, sizeof( float ) );
    sensorsData.xSensorsToComsQueue = xQueueCreate( 10, 51);//sizeof( float ) );
    sensorsData.xSensorsToFnFQueue = xQueueCreate( 10, 51 );
    sensorsData.xSensorsToFnFQueueE = xQueueCreate( 10, 51 );
    sensorsData.xSensorsToMotorsQueue = xQueueCreate( 10, 51 );    
    
    //May need to add channles to scan mode
//    DRV_ADC_ChannelScanInputsAdd(ADC_INPUT_SCAN_AN0);
//    DRV_ADC_ChannelScanInputsAdd(ADC_INPUT_SCAN_AN1);
    DRV_ADC_Open();//Opens ADC
    
    // Fake sensor data vars
    sensorsData.index = 0;
    sensorsData.index2 = 0;
    RS = 10;
    LS = 10;
    FS = 5;
    BS = 123;
}


/******************************************************************************
  Function:
    void SENSORS_Tasks ( void )

  Remarks:
    See prototype in sensors.h. 
*/

void SENSORS_Tasks ( void )
{
  
   // char t[2] = "MT";
   // char t2[3] = "220";
   // char a[2] = "DA";
   // char a2[3] = "978";
   // char b[2] = "NU";
   // char b2[3] = "001";    
    
   // concatenate3(wkki, t, t2, a, a2, b, "222");

 
           
    /* Check the application's current state. */
    switch ( sensorsData.state )
    {
        /* Application's initial state. */
        case SENSORS_STATE_INIT:
        {
            SENSORS_Initialize();
//            PLIB_ADC_SampleAcquisitionTimeSet(DRV_ADC_ID_1, 31);
            sensorsData.state = APP_STATE_START;
            break;
        }
        case APP_STATE_START:
        {
            DRV_ADC_Start();//Starts converting adc values
//            DRV_ADC_Start();
            sensorsData.state = APP_STATE_WAIT;
            PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_1);
            break;
        }
        case APP_STATE_WAIT:
        {
            PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_2);
//            PLIB_ADC_ConversionStart(DRV_ADC_ID_1);
            if (sensorsData.dataReady)
            {
                sensorsData.state = APP_STATE_SEND_RESULTS;
                
            }
            break;
        }
        case APP_STATE_SEND_RESULTS:
        {
           char ello[42];
           char lo[51];
                
           //____ Forward
           /*
             RS = 19;
             LS = 19;
             FS = 29;
            */ 
            
           //____ Backward
             
            // RS = 10;
           //  LS = 10;
           //  FS = 5;
           
           //____ Forward Right
             /*
             RS = 23;
             LS = 27;
             FS = 21;
           */
           //____ Forward Left
             /*
             RS = 27;
             LS = 24;
             FS = 21;
           */
           //____ Right
               /*
             RS = 10;
             LS = 30;
             FS = 10;
           */
           //____ Left
             /*
             RS = 30;
             LS = 10;
             FS = 10;
          */
           //Needs to grab the data from the 4 sensors here--------------------------
                //Turn Right Check
                FS++;
                if(FS>15)
                {
                    if(LS>25)
                    {
                        LS=0;
                    }
                    if(LS==0)
                    {
                        RS=RS+1;
                    }
                    else
                    {
                        LS++;
                    }
                }
                if(FS>30)
                {
                    RS = 10;
                    LS = 23;
                    FS = 11;
                }
                   
                //----------------------------------------------------------
               char RS_c[3];
               char LS_c[3];
               char FS_c[3];
               char BS_c[3];
               char NumofPackets[3];
               // converts ints to 3 bite char arrays 
                snprintf(RS_c, 4,"%03d", RS);
                snprintf(LS_c, 4,"%03d", LS);
                snprintf(FS_c, 4,"%03d", FS);
                snprintf(BS_c, 4,"%03d", BS);
                snprintf(NumofPackets, 4,"%03d",sensorsData.NUMBEROFPACKETSPLACEDINTHEQ);
               // builds packet
                concatenate6(ello,"SR","000","RS",RS_c,"LS",LS_c,"FS",FS_c,"BS",BS_c,"NP", NumofPackets);
             
              //------------------------------------------------------------------------
               // Greg's Test Code
               // This code was written because 
                if(uxQueueSpacesAvailable(sensorsData.xSensorsToFnFQueue)==15)
                {
                 xQueueSend( sensorsData.xSensorsToFnFQueue, &ello, 0 );
                 sensorsData.NUMBEROFPACKETSPLACEDINTHEQ=sensorsData.NUMBEROFPACKETSPLACEDINTHEQ+1;
                }
                //-----------------------------------------
               /*    
            
               if(isValidPacket(ello))
               {
            
                    if(xQueueSend( sensorsData.xSensorsToComsQueue, &ello, 0 ))
                    {
                        sensorsData.NUMBEROFPACKETSPLACEDINTHEQ=sensorsData.NUMBEROFPACKETSPLACEDINTHEQ+1;
                    }
                    else 
                    {
                        sensorsData.NUMBEROFPACKETSDROPPEDBEFOREQ=sensorsData.NUMBEROFPACKETSDROPPEDBEFOREQ+1;
                    }
               }
               //else
               {
                int i=0;
                   for(i=0; i<42; i++)
                    ello[i]='9';
                        // malformed packet exception
                    if(xQueueSend( sensorsData.xSensorsToComsQueue, &ello, 0 ))
                        {
                         sensorsData.NUMBEROFPACKETSPLACEDINTHEQ=sensorsData.NUMBEROFPACKETSPLACEDINTHEQ+1;
                        }
                    else 
                        {
                          sensorsData.NUMBEROFPACKETSDROPPEDBEFOREQ=sensorsData.NUMBEROFPACKETSDROPPEDBEFOREQ+1;
                        }
              
               }

            // Second queue. Fill with fake data
        
        //        xStatus2 = xQueueSend( sensorsData.xSensorsToFnFQueue, &data[sensorsData.index2], 0 );
            
            
            
            // Third queue. Fill with fake data
          
          //      xStatus2 = xQueueSend( sensorsData.xSensorsToMotorsQueue, &data[sensorsData.index3], 0 );
          
                // Receive data from coms and forward back
                if(xQueueReceive( comsData.xComsToSensorsQueue, &lo, 0))
                {
                    xQueueSend( sensorsData.xSensorsToComsQueue, &lo, 0 );
                }
                */
			sensorsData.dataReady = false;
            sensorsData.state = APP_STATE_START;
            break;
        }
        case APP_STATE_SPIN:
        {
            if (sensorsData.tick > 1250)
            {
                sensorsData.tick = 0;
                sensorsData.state = APP_STATE_START;
            }
            break;
        }
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
