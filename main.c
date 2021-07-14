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

/*
    Main application
*/


#define I2C_MCP3221_CLIENT_ADDR             0x4D /*7-bit Address*/
#define I2C_TC1321_CLIENT_ADDR              0x48 /*7-bit Address*/  
#define TC1321_REG_ADDR                     0x00

volatile uint8_t TC_flag = 0;

void TC_overflow_cb(void){
    
    LED_RE0_Toggle();
    DebugIO_RE2_Toggle();
    TC_flag = 1;
}

int main(void)
{
    SYSTEM_Initialize();

    Timer0.TimeoutCallbackRegister(TC_overflow_cb);

    // Enable the Global Interrupts
    INTERRUPT_GlobalInterruptEnable();

    // Enable the Peripheral Interrupts
    INTERRUPT_PeripheralInterruptEnable();
    
    float DACVoltage = 0;
    float VrefDAC = 2.5;
    uint16_t ADCRead = 0;
    uint16_t ADCRightShift;
    uint16_t DACWrite;
    uint16_t resolution = 1024;
    uint8_t dataWrite[3];
    uint8_t dataRead[2];
    
    dataWrite[0] = TC1321_REG_ADDR;

    while(1)
    {
        if(TC_flag)
        {
            /*Read data from ADC*/
            i2c_readNBytes(I2C_MCP3221_CLIENT_ADDR, dataRead, 2);
            __delay_ms(10);

            /*Make one 16-bit value from the 2 bytes read from ADC*/
            ADCRead = (uint16_t) ((dataRead[0] << 8) | (dataRead[1] & 0xff));
            /*Right-shift value by 2, to obtain a 10-bit resolution*/
            ADCRightShift = ADCRead >> 2;

            /*Calculate the voltage output from the DAC*/
            DACVoltage = ADCRightShift * (VrefDAC/resolution);
            /*Send data to variable streamer*/
            variableWrite_SendFrame(DACVoltage);

            /*Left-shift by 6 - to write the data to the DAC. The datasheet
             defines how the 10-bit value should be written to the 0x00 register.*/
            DACWrite = ADCRightShift << 6;

            /*Split into 2 single bytes*/
            dataWrite[1] = (uint8_t) (DACWrite >> 8);
            dataWrite[2] = (uint8_t) (DACWrite & 0xFF);

            /*Write data to the DATA register (0x00)*/
            i2c_writeNBytes(I2C_TC1321_CLIENT_ADDR, dataWrite, 3);
            __delay_ms(10);
            
            TC_flag = 0;
        }
    }    
    
}