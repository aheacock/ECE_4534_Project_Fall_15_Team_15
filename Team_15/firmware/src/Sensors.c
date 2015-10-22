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

#define ADC_REF_VOLT            (float)3.33
#define ADC_MAX_READING         (float)4096        // bits
#define ADC_CONV_FACT           (float)(ADC_MAX_READING/ADC_REF_VOLT)   // bits/V

/* definitions for temperature sensor */
#define MCP9700_COEFF               (float)0.01         // V/degC
#define MCP9700_COEFF_SCALED        (int)(MCP9700_COEFF * ADC_CONV_FACT)   // bits/degC
#define MCP9700_0DEG_OUT            (float)0.5          // V @ 0degC
#define MCP9700_0DEG_OUT_SCALED     (int)(MCP9700_0DEG_OUT * ADC_CONV_FACT)   // bits/degC

/* definitions for accelerometer */
#define ADXL325_SENSITIVITY         (float)0.174        // V/g
#define ADXL325_COEFF               (int)(ADXL325_SENSITIVITY *ADC_CONV_FACT)   // bits/g

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
void getSensorData(){
    char ello[42];
    switch ( sensorsData.state )
        {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            sensorsData.state = APP_STATE_WAIT;
            break;
        }
        case APP_STATE_WAIT:
        {
//            PLIB_INT_SourceFlagSet(INT_ID_0, INT_SOURCE_ADC_1);
//            sensorsData.tempSensor = DRV_ADC_SamplesRead(1);
            if (true == sensorsData.dataReady)
            {
                sensorsData.state = APP_STATE_SEND_RESULTS;
            }
        }
        break;
        
        case APP_STATE_SEND_RESULTS:
        {
            if(true == sensorsData.dataReady)
            {
                char NumofPackets[3];
                snprintf(NumofPackets, 4,"%03d",sensorsData.NUMBEROFPACKETSPLACEDINTHEQ);
                concatenate6(ello,"SR","000","RS",sensorsData.tempSensor,"LS",sensorsData.tempSensor,"FS",sensorsData.tempSensor,"BS",sensorsData.tempSensor,"NP", NumofPackets);
               //if(isValidPacket(ello))
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
                /*   int i=0;
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
               */
               }
                /*
                sensorsData.tempDegC = (sensorsData.tempSensor - MCP9700_0DEG_OUT_SCALED)/MCP9700_COEFF_SCALED;
                
                sensorsData.fltxAxis = (float)sensorsData.xAxis/ADXL325_COEFF;
                sensorsData.fltyAxis = (float)sensorsData.yAxis/ADXL325_COEFF;
                sensorsData.fltzAxis = (float)sensorsData.zAxis/ADXL325_COEFF;
                
                if(sensorsData.fltxAxis > 1.0)
                    sensorsData.fltxAxis = 1.0;
                else if(sensorsData.fltxAxis < -1.0)
                    sensorsData.fltxAxis = -1.0;
                
                if(sensorsData.fltyAxis > 1.0)
                    sensorsData.fltyAxis = 1.0;
                else if(sensorsData.fltyAxis < -1.0)
                    sensorsData.fltyAxis = -1.0;
                
                if(sensorsData.fltzAxis > 1.0)
                    sensorsData.fltzAxis = 1.0;
                else if(sensorsData.fltzAxis < -1.0)
                    sensorsData.fltzAxis = -1.0;
                
                sensorsData.fltxAxisSqr = sensorsData.fltxAxis * sensorsData.fltxAxis;
                sensorsData.fltyAxisSqr = sensorsData.fltyAxis * sensorsData.fltyAxis;
                sensorsData.fltzAxisSqr = sensorsData.fltzAxis * sensorsData.fltzAxis;
                
                sensorsData.xySqr = sqrtf(sensorsData.fltxAxisSqr + sensorsData.fltyAxisSqr);
                sensorsData.yzSqr = sqrtf(sensorsData.fltyAxisSqr + sensorsData.fltzAxisSqr);
                sensorsData.zxSqr = sqrtf(sensorsData.fltzAxisSqr + sensorsData.fltxAxisSqr);
                
                sensorsData.xRatio = (sensorsData.fltxAxis/sensorsData.yzSqr);
                sensorsData.yRatio = (sensorsData.fltyAxis/sensorsData.zxSqr);
                sensorsData.zRatio = (sensorsData.fltzAxis/sensorsData.xySqr);
                
                sensorsData.xAngle = atanf(sensorsData.xRatio);
                sensorsData.yAngle = atanf(sensorsData.yRatio);
                sensorsData.zAngle = atanf(sensorsData.zRatio); 
                */
            }
			sensorsData.dataReady = false;
            sensorsData.state = APP_STATE_WAIT;
        }
        break;
        case APP_STATE_SPIN:
        {
            if (sensorsData.tick)
            {
                sensorsData.tick = false;
                sensorsData.state = APP_STATE_WAIT;
            }
        }
        break;
    }
}

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
void SENSORS_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    sensorsData.state = SENSORS_STATE_INIT;
    
    //Sensor Data Acquisition
    sensorsData.wrComplete = true;
    sensorsData.rdComplete = true; 
    DRV_ADC_Open();//Opens ADC
    DRV_ADC_Start();//Starts converting adc values
    //sensorsData.xFakeSensorDataQueue = xQueueCreate( 10, sizeof( float ) );
    sensorsData.xSensorsToComsQueue = xQueueCreate( 10, 51);//sizeof( float ) );
    sensorsData.xSensorsToFnFQueue = xQueueCreate( 10, 51 );
    sensorsData.xSensorsToMotorsQueue = xQueueCreate( 10, 51 );
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
            char ello[42];
            char lo[51];

            //Gets Sensor Data
            getSensorData();
            /*
            //Grabs data from the 4 sensors
            int RS = 266;
            int LS = 167;
            int FS = 168;
            int BS = 123;

            char RS_c[3];
            char LS_c[3];
            char FS_c[3];
            char BS_c[3];
            char NumofPackets[3];

            snprintf(RS_c, 4,"%d", RS);
            snprintf(LS_c, 4,"%d", LS);
            snprintf(FS_c, 4,"%d", FS);
            snprintf(BS_c, 4,"%d", BS);

            snprintf(NumofPackets, 4,"%03d",sensorsData.NUMBEROFPACKETSPLACEDINTHEQ);
            concatenate6(ello,"SR","000","RS",RS_c,"LS",LS_c,"FS","000","BS",BS_c,"NP", NumofPackets);
            //if(isValidPacket(ello))
               {
            
                    if(xQueueSend( sensorsData.xSensorsToComsQueue, &ello, 0 ))
                    {
                        sensorsData.NUMBEROFPACKETSPLACEDINTHEQ=sensorsData.NUMBEROFPACKETSPLACEDINTHEQ+1;
                    }
                    else 
                    {
                        sensorsData.NUMBEROFPACKETSDROPPEDBEFOREQ=sensorsData.NUMBEROFPACKETSDROPPEDBEFOREQ+1;
                    }
               }//*/
               //else
               {
                /*   int i=0;
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
               */
               }

            // Second queue. Fill with fake data
        
        //        xStatus2 = xQueueSend( sensorsData.xSensorsToFnFQueue, &data[sensorsData.index2], 0 );
            
            
            
            // Third queue. Fill with fake data
          
          //      xStatus2 = xQueueSend( sensorsData.xSensorsToMotorsQueue, &data[sensorsData.index3], 0 );
          
                // Receive data from coms
                if(xQueueReceive( comsData.xComsToSensorsQueue, &lo, 0))
                {
                    //xQueueSend( sensorsData.xSensorsToComsQueue, &lo, 0 );
                    if (!isValidPacket(lo))
                        sensorsData.NumBadPacketsRecvFromComsQ += 1;
                    else if (isAckPacket(lo))
                        sensorsData.NumPacketsRecvFromComsQ += 1;
                    else
                    {
                        // Ack the packet
                        
                        // Then do stuff
                        
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
