/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/*******************************************************************************
  This source file has NOT been generated by the MHC
 *******************************************************************************/

/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "PacketManip.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************



// *****************************************************************************
// *****************************************************************************
// Section: User Functions
// *****************************************************************************
// *****************************************************************************

// Comment a function definition and leverage automatic documentation 
/**
  <p><b>Function:</b></p>

  <p><b>Summary:</b></p>

  <p><b>Description:</b></p>

  <p><b>Remarks:</b></p>
 */
// TODO Insert function definitions (right here) to leverage live documentation


void concatenate3(char dest[21], char type[2], char typeNum[3], char one[2], char oneNum[3], char two[2], char twoNum[3] )
{
    int i = 0;
    
    for (i=0;i<2;i++)
    {
        dest[i] = type[i];
        
    }
    dest[i] = ':';
    i++;
    for (i=3;i<6;i++)
    {
        dest[i] = typeNum[i-3];
    }
    dest[i] = ',';
    i++;
    
    for (i=7;i<9;i++)
    {
        dest[i] = one[i-7];
        
    }
    dest[i] = ':';
    i++;
    for (i=10;i<13;i++)
    {
        dest[i] = oneNum[i-10];
    }
    dest[i] = ',';
    i++;
    
    for (i=14;i<16;i++)
    {
        dest[i] = two[i-14];
        
    }
    dest[i] = ':';
    i++;
    for (i=17;i<20;i++)
    {
        dest[i] = twoNum[i-17];
    }
    dest[20] = '\0';    
    
    /*
    Doesn't work:
    strcpy(dest,a);
    strcat(dest,b);
    */
}

void concatenate5(char dest[21], char type[2], char typeNum[3], char one[2], char oneNum[3], char two[2], char twoNum[3], char thr[2], char thrNum[3], char fou[2], char fouNum[3])
{
    int i = 0;
    
    for (i=0;i<2;i++)
    {
        dest[i] = type[i];
        
    }
    dest[i] = ':';
    i++;
    for (i=3;i<6;i++)
    {
        dest[i] = typeNum[i-3];
    }
    dest[i] = ',';
    i++;
    
    for (i=7;i<9;i++)
    {
        dest[i] = one[i-7];
        
    }
    dest[i] = ':';
    i++;
    for (i=10;i<13;i++)
    {
        dest[i] = oneNum[i-10];
    }
    dest[i] = ',';
    i++;
    
    for (i=14;i<16;i++)
    {
        dest[i] = two[i-14];
        
    }
    dest[i] = ':';
    i++;
    for (i=17;i<20;i++)
    {
        dest[i] = twoNum[i-17];
    }
    dest[i] = ',';
    i++;
    
    for (i=22;i<24;i++)
    {
        dest[i] = thr[i-22];        
    }
    dest[i] = ':';
    i++;
    for (i=25;i<28;i++)
    {
        dest[i] = thrNum[i-25];
    }
    dest[i] = ',';
    i++;
    
    for (i=29;i<31;i++)
    {
        dest[i] = fou[i-29];        
    }
    dest[i] = ':';
    i++;
    for (i=32;i<34;i++)
    {
        dest[i] = fouNum[i-32];
    }
    
    dest[34] = '\0';
}

void concatenate6(char dest[21], char type[2], char typeNum[3], char one[2], char oneNum[3], char two[2], char twoNum[3], char thr[2], char thrNum[3], char fou[2], char fouNum[3], char fiv[2], char fivNum[3])
{
    int i = 0;
    
    for (i=0;i<2;i++)
    {
        dest[i] = type[i];
        
    }
    dest[i] = ':';
    i++;
    for (i=3;i<6;i++)
    {
        dest[i] = typeNum[i-3];
    }
    dest[i] = ',';
    
    i++;
    for (i=7;i<9;i++)
    {
        dest[i] = one[i-7];
        
    }
    dest[i] = ':';
    i++;
    for (i=10;i<13;i++)
    {
        dest[i] = oneNum[i-10];
    }
    dest[i] = ',';
    i++;
    
    for (i=14;i<16;i++)
    {
        dest[i] = two[i-14];
        
    }
    dest[i] = ':';
    i++;
    for (i=17;i<20;i++)
    {
        dest[i] = twoNum[i-17];
    }
    dest[i] = ',';
    i++;
    
    for (i=22;i<24;i++)
    {
        dest[i] = thr[i-22];        
    }
    dest[i] = ':';
    i++;
    for (i=25;i<28;i++)
    {
        dest[i] = thrNum[i-25];
    }
    dest[i] = ',';
    i++;
    
    for (i=29;i<31;i++)
    {
        dest[i] = fou[i-29];        
    }
    dest[i] = ':';
    i++;
    for (i=32;i<34;i++)
    {
        dest[i] = fouNum[i-32];
    }
    dest[i] = ',';
    i++;
    
    for (i=35;i<37;i++)
    {
        dest[i] = fou[i-35];        
    }
    dest[i] = ':';
    i++;
    for (i=38;i<41;i++)
    {
        dest[i] = fouNum[i-38];
    }
    
    dest[41] = '\0';
}