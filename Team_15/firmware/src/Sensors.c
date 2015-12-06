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
    
    float FrontRightSensor = 0;
    int   FrontRightSensorCM = 0;
    
    float FrontLeftSensor = 0;
    int   FrontLeftSensorCM = 0;
    
    float BackRightSensor = 0;
    int   BackRightSensorCM = 0;
    
    float BackLeftSensor = 0;
    int   BackLeftSensorCM = 0;
    
    float ForwardLeftSensor = 0;
    int   ForwardLeftSensorCM = 0;
    
    float ForwardCenterSensor = 0;
    int   ForwardCenterSensorCM = 0;
    
    float ForwardRightSensor = 0;
    int   ForwardRightSensorCM = 0;
    
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
    sensorsData.xSensorsToComsQueue = xQueueCreate( 15, 51);//sizeof( float ) );
    sensorsData.xSensorsToFnFQueue = xQueueCreate( 15, 51 );
    sensorsData.xSensorsToMotorsQueue = xQueueCreate( 15, 51 );
    
    //Sensor Data Acquisition
    sensorsData.wrComplete = true;
    sensorsData.rdComplete = true; 
    
   // sensorsData.xSensorsToComsQueue = xQueueCreate( 15, 51);//sizeof( float ) );
   // sensorsData.xSensorsToFnFQueue = xQueueCreate( 15, 51 );
    sensorsData.xSensorsToFnFQueueE = xQueueCreate( 15, 51 );
   // sensorsData.xSensorsToMotorsQueue = xQueueCreate( 15, 51 );    
    
    DRV_ADC_Open();//Opens ADC
    
//    // Fake sensor data vars
//    sensorsData.index = 0;
//    sensorsData.index2 = 0;
//    RS = 10;
//    LS = 10;
//    FS = 5;
//    BS = 123;
}

// The structure of the parameters of the IR distance sensors
typedef const struct
{
	const signed short a;
	const signed short b;
	const signed short k;
}
ir_distance_sensor;
 
// The object of the parameters of GP2Y0A21YK sensor
const ir_distance_sensor MID_RANGE_SENSOR = { 5461, -17, 3 };
const ir_distance_sensor LONG_RANGE_SENSOR = { 10181, -17, -20 };


// Converting the values of the IR distance sensor to centimeters
// Returns -1, if the conversion did not succeed
signed short ir_distance_calculate_cm(ir_distance_sensor sensor, unsigned short adc_value)
{
	if (adc_value + sensor.b <= 0)
	{
		return -1;
	}
	return sensor.a / (adc_value + sensor.b) - sensor.k;
}

int edgeDetected(){
    if(FrontRightSensorCM > 12 ||
       FrontLeftSensorCM > 12  ||
       BackRightSensorCM > 12  ||
       BackLeftSensorCM > 12){
        return 1;
    }
    return 0;
}
/******************************************************************************
  Function:
    void SENSORS_Tasks ( void )

  Remarks:
    See prototype in sensors.h. 
*/

void SENSORS_Tasks ( void )
{
    /* Check the application's current state. */
    switch ( sensorsData.state )
    {
        /* Application's initial state. */
        case SENSORS_STATE_INIT:
        {
            SENSORS_Initialize();
            sensorsData.state = APP_STATE_START;
            break;
        }
        case APP_STATE_START:
        {
            DRV_ADC_Start();//Starts converting adc values
            sensorsData.state = APP_STATE_WAIT;
            PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_1);
            break;
        }
        case APP_STATE_WAIT:
        {
            PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_2);
            if (sensorsData.dataReady)
            {
                sensorsData.state = APP_STATE_SEND_RESULTS;
                
            }
            break;
        }
        case APP_STATE_SEND_RESULTS:
        {
            char ello[42];
           //Needs to grab the data from the 4 sensors here--------------------------
           FrontRightSensor = sensorsData.frontRightEdgeSensor*.5 + FrontRightSensor * .5;
           FrontRightSensorCM = ir_distance_calculate_cm(MID_RANGE_SENSOR, FrontRightSensor);
           
           FrontLeftSensor = sensorsData.frontLeftEdgeSensor*.5 + FrontLeftSensor * .5;
           FrontLeftSensorCM = ir_distance_calculate_cm(MID_RANGE_SENSOR, FrontLeftSensor);
           
           BackRightSensor = sensorsData.backRightEdgeSensor*.5 + BackRightSensor * .5;
           BackRightSensorCM = ir_distance_calculate_cm(MID_RANGE_SENSOR, BackRightSensor);
           
           BackLeftSensor = sensorsData.backLeftEdgeSensor*.5 + BackLeftSensor * .5;
           BackLeftSensorCM = ir_distance_calculate_cm(MID_RANGE_SENSOR, BackLeftSensor);
           

           ForwardLeftSensor = sensorsData.leftWhiskerSensor*.5 + ForwardLeftSensor * .5;
           ForwardLeftSensorCM = ir_distance_calculate_cm(LONG_RANGE_SENSOR, ForwardLeftSensor);
           
           ForwardCenterSensor = sensorsData.centerWhiskerSensor*.5 + ForwardCenterSensor * .5;
           ForwardCenterSensorCM = ir_distance_calculate_cm(LONG_RANGE_SENSOR, ForwardCenterSensor);
           
           ForwardRightSensor = sensorsData.rightWhiskerSensor*.5 + ForwardRightSensor * .5;
           ForwardRightSensorCM = ir_distance_calculate_cm(LONG_RANGE_SENSOR, ForwardRightSensor);
           
                     
           if(edgeDetected()){
               char RFS_c[3];
               char LFS_c[3];
               char RBS_c[3];
               char LBS_c[3];
               char NumofPackets[3];
               // converts ints to 3 bite char arrays 
                snprintf(RFS_c, 4,"%03d", FrontRightSensorCM);
                snprintf(LFS_c, 4,"%03d", FrontLeftSensorCM);
                snprintf(RBS_c, 4,"%03d", BackRightSensorCM);
                snprintf(LBS_c, 4,"%03d", BackLeftSensorCM);
                snprintf(NumofPackets, 4,"%03d",sensorsData.NUMBEROFPACKETSPLACEDINTHEQ);
               // builds packet
                concatenate6(ello,"SR","000","RS",RFS_c,"LS",LFS_c,"FS",RBS_c,"BS",LBS_c,"NP", NumofPackets);
                
                if(uxQueueSpacesAvailable(sensorsData.xSensorsToFnFQueue)==15)
                {
                 xQueueSend( sensorsData.xSensorsToFnFQueueE, &ello, 0 );
                 sensorsData.NUMBEROFPACKETSPLACEDINTHEQ=sensorsData.NUMBEROFPACKETSPLACEDINTHEQ+1;
                }
           }
           else{       
               char RS_c[3];
               char LS_c[3];
               char FS_c[3];
               char BS_c[3]; // unused
               char NumofPackets[3];
               // converts ints to 3 bite char arrays 
                snprintf(RS_c, 4,"%03d", ForwardRightSensorCM);
                snprintf(LS_c, 4,"%03d", ForwardLeftSensorCM);
                snprintf(FS_c, 4,"%03d", ForwardCenterSensorCM);
                snprintf(BS_c, 4,"%03d", BS);
                snprintf(NumofPackets, 4,"%03d",sensorsData.NUMBEROFPACKETSPLACEDINTHEQ);
                
                // builds packet
                concatenate6(ello,"SR","000","RS",RS_c,"LS",LS_c,"FS",FS_c,"BS",BS_c,"NP", NumofPackets);
                
                if(uxQueueSpacesAvailable(sensorsData.xSensorsToFnFQueue)==15)
                {
                 xQueueSend( sensorsData.xSensorsToFnFQueue, &ello, 0 );
                 sensorsData.NUMBEROFPACKETSPLACEDINTHEQ=sensorsData.NUMBEROFPACKETSPLACEDINTHEQ+1;
                }
           }
			sensorsData.dataReady = false;
            sensorsData.state = APP_STATE_START;
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
