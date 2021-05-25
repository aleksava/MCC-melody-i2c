/*
Copyright (c) [2012-2020] Microchip Technology Inc.  

    All rights reserved.

    You are permitted to use the accompanying software and its derivatives 
    with Microchip products. See the Microchip license agreement accompanying 
    this software, if any, for additional info regarding your rights and 
    obligations.
    
    MICROCHIP SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT 
    WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT 
    LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT 
    AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP OR ITS
    LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT, NEGLIGENCE, STRICT 
    LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE 
    THEORY FOR ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES INCLUDING BUT NOT 
    LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT OR CONSEQUENTIAL DAMAGES, 
    OR OTHER SIMILAR COSTS. 
    
    To the fullest extend allowed by law, Microchip and its licensors 
    liability will not exceed the amount of fees, if any, that you paid 
    directly to Microchip to use this software. 
    
    THIRD PARTY SOFTWARE:  Notwithstanding anything to the contrary, any 
    third party software accompanying this software is subject to the terms 
    and conditions of the third party's license agreement.  To the extent 
    required by third party licenses covering such third party software, 
    the terms of such license will apply in lieu of the terms provided in 
    this notice or applicable license.  To the extent the terms of such 
    third party licenses prohibit any of the restrictions described here, 
    such restrictions will not apply to such third party software.
*/
#include "mcc_generated_files/system/system.h"

/*
    Main application
*/
//uint16_t i2c_read2ByteRegister(i2c1_address_t address, uint8_t reg);
//I2C Temp sensor: AT30TSE758
#define I2C_TEMP_SENS_ADDRESS      0x4F      
#define I2C_TEMPERATURE_REGISTER   0x00

uint8_t count = 0;
uint16_t volatile adcVal = 0;  //ADC
uint16_t volatile i2cTemp = 0; //I2C 
uint8_t len = 2; //I2C read buffer 

void TC_overflow_cb(void){

    LED_RE0_Toggle();
    DebugIO_RE2_Toggle();

    ADCC_DischargeSampleCapacitor();
    adcVal = ADCC_GetSingleConversion(channel_ANA0);

    i2cTemp = i2c_read2ByteRegister(I2C_TEMP_SENS_ADDRESS, I2C_TEMPERATURE_REGISTER);
 
    variableWrite_sendFrame(count++, adcVal, i2cTemp);
    
}

int main(void)
{
    SYSTEM_Initialize();

    Timer0.TimeoutCallbackRegister(TC_overflow_cb);

    
    // Enable the Global Interrupts
    INTERRUPT_GlobalInterruptEnable();

    // Enable the Peripheral Interrupts
    INTERRUPT_PeripheralInterruptEnable();

    while(1)
    {
    }    
}