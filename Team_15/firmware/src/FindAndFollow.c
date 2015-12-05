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

int leadermodeflag;
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
    leadermodeflag=0;
     findandfollowData.numdropped=0;
     findandfollowData.packetsrecievedinComs=0;
    findandfollowData.xFnFToMotorsQueue = xQueueCreate( 15, 42);
    findandfollowData.xFnFToComsQueue = xQueueCreate( 15, 42);
    findandfollowData.xFnFToSensorsQueue = xQueueCreate( 15, 42);
       findandfollowData.xFnFToMotors = xQueueCreate( 15, 42);
    findandfollowData.index = 0;
   
}

char recievecommand()
{
     int x=0;
     char lo[42];
    if(xQueueReceive(comsData.xComsToFnFQueue, &lo, 0)) // working one
    { 
        
      
            if(lo[1]=='w')
                {// forward
                return 'w';
                }
            else if(lo[0]=='s')
                {// back
                return 's';
                }
            else if(lo[0]=='d')
                {// fr
                return 'd';
                }
            else if (lo[0]=='a')
                {// FL
                return 'a';             
                }
            else if(lo[0]=='2')
                {//right
               return '2';      
                }
            else if(lo[0]=='3')
                {//left
                return '3';
                }
            else 
            {
                return 'X';
            }
    }
     return 'X';
}

int recievefrommotors()
{
    
    int x = 0;
    char lo[42];
    if(xQueueReceive(motorsData.xMotorsToFnFQueue, &lo, 0)) // working one
    {   
          if(uxQueueSpacesAvailable(findandfollowData.xFnFToComsQueue)>10)
                {       
                    xQueueSend( findandfollowData.xFnFToComsQueue, &lo, 0 );
             
                }
        
    }
      //  stringPointer=lo;
        
     /*   if(isValidPacket(lo))
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
       if(uxQueueSpacesAvailable(findandfollowData.xFnFToMotors)==15)
                {
                char temp[21];
                concatenate3(temp, "AM", "000", "AN", "000", "NU", "000");
                    if (xQueueSend( findandfollowData.xFnFToMotors, temp, 0))
                        {
                            findandfollowData.NUMBEROFPACKETSPLACEDINTHEQ += 1;
                        }
                }
    }
      */
    return x;
}




/******************************************************************************
  Function:
    void FINDANDFOLLOW_Tasks ( void )

  Remarks:
    See prototype in findandfollow.h.
 */
int sensorstofnf()
{  
      int rightsensor=0;
        int leftsensor=0;
        int forwardsensor=0;
        int backsensor =0;
        
        int rightfloorsensor=0;
        int leftfloorsensor=0;
        int forwardfloorsensor=0;
        int backfloorsensor =0;
    
    int x = 0;
    char lo[42];
    // 3 Modes Emergency, Control, Sensor Modes 
    if(xQueueReceive(sensorsData.xSensorsToFnFQueueE, &lo, 0)) 
    {  
        int nothingthereconst=12;
        //Emergency mode
        // takes the four sensors 
           rightfloorsensor=((lo[10]-'0')*100)+((lo[11]-'0')*10)+(lo[12]-'0');
           leftfloorsensor=((lo[17]-'0')*100)+((lo[18]-'0')*10)+(lo[19]-'0');
           forwardfloorsensor=((lo[24]-'0')*100)+((lo[25]-'0')*10)+(lo[26]-'0');
           backfloorsensor=((lo[31]-'0')*100)+((lo[32]-'0')*10)+(lo[33]-'0');
           
            // 4 for left
           // 8 for right
           // +1 for forward 
           // -1 for back   
           // brace for impact
           if(rightfloorsensor>nothingthereconst)
                x=4;
           else if(leftfloorsensor>nothingtherefconst)    
                x=8;
           else if(forwardfloorsensor>nothingthereconst)
                x=1;
           else if(backfloorsensor>nothingthereconst)
                x=7;
                       
               
               
        
   
    }
    else(xQueueReceive(sensorsData.xSensorsToFnFQueue, &lo, 0)) // working one
    {   
        int nothingtherefrontsensors;
        
           // lo is sensor data
           //packet deconstruction here       
           rightsensor=((lo[10]-'0')*100)+((lo[11]-'0')*10)+(lo[12]-'0');
           leftsensor=((lo[17]-'0')*100)+((lo[18]-'0')*10)+(lo[19]-'0');
           forwardsensor=((lo[24]-'0')*100)+((lo[25]-'0')*10)+(lo[26]-'0');
           backsensor=((lo[31]-'0')*100)+((lo[32]-'0')*10)+(lo[33]-'0');
           
           
         
           
           // 4 for left
           // 8 for right
           // +1 for forward 
           // -1 for back   
           if(rightsensor<45)
           {
               // turn right
               x=8;
           }
           if(leftsensor<45)
           {
                // turn left
               x=4;
              
           }
           if(forwardsensor<45)
           {
               // move forward
               x=x+1;   
           }
           else if(forwardsensor<35)
           { //move back
               x=7;
           }  
          // {SR:000,RS:266,LS:167,FS:000,BS:123,NP:010}
       if(uxQueueSpacesAvailable(findandfollowData.xFnFToComsQueue)==15)
                {
                    // debug send: sends Sensor data to Coms 
                    xQueueSend( findandfollowData.xFnFToComsQueue, &lo, 0 );           
                }
    }
    return x;
}// combo for locker 9244
void FINDANDFOLLOW_Tasks ( void )
{
    /* Check the application's current state. */
    switch ( findandfollowData.state )
    {
        /* Application's initial state. */
        case FINDANDFOLLOW_STATE_INIT:
        {
            char b='b';
          recievefrommotors();
          int x=sensorstofnf();
          char y=recievecommand();
          if (y=='X' && leadermodeflag==0)
          {      // 4 for left
           // 8 for right
           // +1 for forward 
           // -1 for back
          if(x!=0)
          {
              if(x==1)
              {  // send move forward +
                  if(uxQueueSpacesAvailable(findandfollowData.xFnFToMotorsQueue)==15)
                {
                    char temp[20];
                    concatenate3(temp, "FF", "000", "FF", "001", "FF", "001");
                    xQueueSend( findandfollowData.xFnFToMotorsQueue, temp, 0);
                }
              }
              else if(x==8)
              {
                  if(uxQueueSpacesAvailable(findandfollowData.xFnFToMotorsQueue)==15)
                {//TURN RIGHT
                    char temp[20];
                    concatenate3(temp, "ES", "000", "RR", "002", "NU", "002");
                    xQueueSend( findandfollowData.xFnFToMotorsQueue, temp, 0);
                }
              }
            else if(x==9)
              {
               if(uxQueueSpacesAvailable(findandfollowData.xFnFToMotorsQueue)==15)
                {//TURN RIGHT and Forward
                    char temp[20];
                    concatenate3(temp, "EF", "000", "RR", "002", "NU", "000");
                    xQueueSend( findandfollowData.xFnFToMotorsQueue, temp, 0);
                }            
              }
            else if (x==4)
               { // just left
                 if(uxQueueSpacesAvailable(findandfollowData.xFnFToMotorsQueue)==15)
                {
                    char temp[20];
                    concatenate3(temp, "ES", "000", "LL", "003", "NU", "000");
                    xQueueSend( findandfollowData.xFnFToMotorsQueue, temp, 0);
                }               
               }
              else if(x==5)
              {
                  // forward and left
                if(uxQueueSpacesAvailable(findandfollowData.xFnFToMotorsQueue)==15)
                {
                    char temp[20];
                    concatenate3(temp, "EF", "000", "LL", "003", "NU", "000");
                    xQueueSend( findandfollowData.xFnFToMotorsQueue, temp, 0);
                }               
                  
              }
              else if(x==9)
              {
                if(uxQueueSpacesAvailable(findandfollowData.xFnFToMotorsQueue)==15)
                {
                    char temp[20];
                    concatenate3(temp, "EM", "000", "AN", "002", "NU", "000");
                    xQueueSend( findandfollowData.xFnFToMotorsQueue, temp, 0);
                }
                  // forward and right
              }
              
            }
               else if(x==3 || x==7 || x==-1)
              {
                if(uxQueueSpacesAvailable(findandfollowData.xFnFToMotorsQueue)==15)
                {//"MT:111,CO:002,NU:001
                    char temp[20];
                    concatenate3(temp, "EB", "000", "BB", "011", "BB", "000");
                    xQueueSend( findandfollowData.xFnFToMotorsQueue, temp, 0);
                }
             // move back 
              //(reverse is not a priority)
              }
    }
    else 
    {
        leadermodeflag=1; // sets robot into command mode in order to switch out of command mode you have to reset the system
                if(y=='w')
                {// forward
                if(uxQueueSpacesAvailable(findandfollowData.xFnFToMotorsQueue)==15)
                    {
                        char temp[20];
                        concatenate3(temp, "FF", "000", "FF", "001", "FF", "001");
                        xQueueSend( findandfollowData.xFnFToMotorsQueue, temp, 0);
                    }
                }
            else if(y=='s')
                {// back
                    if(uxQueueSpacesAvailable(findandfollowData.xFnFToMotorsQueue)==15)
                    {
                        char temp[20];
                        concatenate3(temp, "FF", "000", "FF", "011", "FF", "001");
                        xQueueSend( findandfollowData.xFnFToMotorsQueue, temp, 0);
                    }
                }
            else if(y=='d')
                {// fr
                   if(uxQueueSpacesAvailable(findandfollowData.xFnFToMotorsQueue)==15)
                    {
                        char temp[20];
                        concatenate3(temp, "FF", "000", "FF", "004", "FF", "001");
                        xQueueSend( findandfollowData.xFnFToMotorsQueue, temp, 0);
                    }
                }
            else if (y=='a')
                {// FL
                   if(uxQueueSpacesAvailable(findandfollowData.xFnFToMotorsQueue)==15)
                    {
                        char temp[20];
                        concatenate3(temp, "FF", "000", "FF", "005", "FF", "001");
                        xQueueSend( findandfollowData.xFnFToMotorsQueue, temp, 0);
                    }              
                }
            else if(y=='2')
                {//right
               if(uxQueueSpacesAvailable(findandfollowData.xFnFToMotorsQueue)==15)
                    {
                        char temp[20];
                        concatenate3(temp, "FF", "000", "FF", "002", "FF", "001");
                        xQueueSend( findandfollowData.xFnFToMotorsQueue, temp, 0);
                    }
                }
            else if(y=='3')
                {//left
               if(uxQueueSpacesAvailable(findandfollowData.xFnFToMotorsQueue)==15)
                    {
                        char temp[20];
                        concatenate3(temp, "FF", "000", "FF", "003", "FF", "001");
                        xQueueSend( findandfollowData.xFnFToMotorsQueue, temp, 0);
                    }
                }
            }
                
        }
                
  
            break;
    }
}

 

/*******************************************************************************
 End of File
 */
