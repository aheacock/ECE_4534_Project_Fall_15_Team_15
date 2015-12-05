/*******************************************************************************
 System Interrupts File

  File Name:
    system_int.c

  Summary:
    Raw ISR definitions.

  Description:
    This file contains a definitions of the raw ISRs required to support the
    interrupt sub-system.

  Summary:
    This file contains source code for the interrupt vector functions in the
    system.

  Description:
    This file contains source code for the interrupt vector functions in the
    system.  It implements the system and part specific vector "stub" functions
    from which the individual "Tasks" functions are called for any modules
    executing interrupt-driven in the MPLAB Harmony system.

  Remarks:
    This file requires access to the systemObjects global data structure that
    contains the object handles to all MPLAB Harmony module objects executing
    interrupt-driven in the system.  These handles are passed into the individual
    module "Tasks" functions to identify the instance of the module to maintain.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2011-2014 released Microchip Technology Inc.  All rights reserved.

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

#include <xc.h>
#include <sys/attribs.h>
#include "coms.h"
#include "findandfollow.h"
#include "sensors.h"
#include "motors.h"
#include "system_definitions.h"

int index = 0;

// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************
extern SENSORS_DATA sensorsData;

void IntHandlerDrvAdc(void)
{
    /* Clear ADC Interrupt Flag */
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_ADC_1);

    switch(index){
        int dataAvg = 0;
        dataAvg += DRV_ADC_SamplesRead(0);
        dataAvg += DRV_ADC_SamplesRead(1);
        dataAvg += DRV_ADC_SamplesRead(2);
        dataAvg += DRV_ADC_SamplesRead(3);

        dataAvg += DRV_ADC_SamplesRead(4);
        dataAvg += DRV_ADC_SamplesRead(5);
        dataAvg += DRV_ADC_SamplesRead(6); 
        case 0:
            sensorsData.frontRightEdgeSensor = dataAvg;
            index++;
            break;
        case 1:
            sensorsData.frontLeftEdgeSensor = dataAvg;
            index++;
            break;
        case 2:
            sensorsData.backRightEdgeSensor = dataAvg;
            index++;
            break;
        case 3:
            sensorsData.backLeftEdgeSensor = dataAvg;
            index++;
            break;
        case 4:
            sensorsData.leftWhiskerSensor = dataAvg;
            index++;
            break;
        case 5:
            sensorsData.centerWhiskerSensor = dataAvg;
            index++;
            break;
        case 6:
            sensorsData.rightWhiskerSensor = dataAvg;
            index = 0;
            break;
        default:
            break;
            
//    sensorsData.frontRightEdgeSensor = DRV_ADC_SamplesRead(0);
//    sensorsData.frontLeftEdgeSensor = DRV_ADC_SamplesRead(1);
//    sensorsData.backRightEdgeSensor = DRV_ADC_SamplesRead(2);
//    sensorsData.backLeftEdgeSensor = DRV_ADC_SamplesRead(3);
//
//    sensorsData.leftWhiskerSensor = DRV_ADC_SamplesRead(4);
//    sensorsData.centerWhiskerSensor = DRV_ADC_SamplesRead(5);
//    sensorsData.rightWhiskerSensor = DRV_ADC_SamplesRead(6);
    } 
    sensorsData.dataReady = true;
    PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_6);
    /* Clear ADC Interrupt Flag */
//    DRV_ADC_Stop();
}


/*


void IntHandlerExternalInterruptInstance0(void)
{           
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_EXTERNAL_1);

}

void IntHandlerExternalInterruptInstance1(void)
{           
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_EXTERNAL_2);

}

*/
void IntHandlerDrvTmrInstance0(void)

{

    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_TIMER_3);

}
  
 
/*******************************************************************************
 End of File
*/

