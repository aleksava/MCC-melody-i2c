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
#include "mcc_generated_files/i2c_host/i2c_simple_host.h"
#include "mcc_generated_files/data_streamer/data_streamer.h"

#define I2C_SLAVE_ADDR          0x4D
#define MCP3221_REG_ADDR        0x00


void TC_overflow_cb(void){
    LED_RE0_Toggle();
    DebugIO_RE2_Toggle();
}

float ADC_value;
uint16_t ADC_read;
float Vdd = 3.3;

int main(void)
{
    SYSTEM_Initialize();
//
    Timer0.TimeoutCallbackRegister(TC_overflow_cb);
//
//    // Enable the Global Interrupts
    INTERRUPT_GlobalInterruptEnable();
//
//    // Enable the Peripheral Interrupts
    INTERRUPT_PeripheralInterruptEnable();
//
    while(1)
    {   
        /*Read ADC value*/
        ADC_read = i2c_read2ByteRegister(I2C_SLAVE_ADDR, MCP3221_REG_ADDR);
        
        /*Convert value to float*/
        ADC_value = ADC_read*(Vdd/4096);
        
        /*Write to data visualizer*/
        variableWrite_SendFrame(ADC_value);
        
        /*Delay 100ms*/
        __delay_ms(100);
    }    
}