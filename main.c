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

#define I2C_MCP3221_CLIENT_ADDR          0x4D /*7-bit Address*/
#define I2C_MCP23008_CLIENT_ADDR         0x20 /*7-bit Address*/
#define MCP23008_REG_ADDR_IODIR          0x00
#define MCP23008_REG_ADDR_GPIO           0x09
#define PINS_DIGITAL_OUTPUT              0x00

volatile uint8_t TC_flag = 0; 

uint8_t mapADCToGPIO(uint16_t rawADCValue);

void TC_overflow_cb(void){
    LED_RE0_Toggle();
    DebugIO_RE2_Toggle();
    TC_flag = 1;
}

int main(void)
{
    /* Initialize the device */
    SYSTEM_Initialize();
    
    Timer0.TimeoutCallbackRegister(TC_overflow_cb);

    /* Enable the Global Interrupts */
    INTERRUPT_GlobalInterruptEnable();

    /* Enable the Peripheral Interrupts */
    INTERRUPT_PeripheralInterruptEnable();
    
    /* Declear variables */
    float ADCVoltage;
    uint16_t rawADCValue;
    uint8_t dataRead[2];
    uint8_t dataWrite[2];
    float Vdd = 3.3;
    uint16_t resolution = 4096;
    
    /* Set the extended pins of I/O expander as digital output */
    i2c_write1ByteRegister(I2C_MCP23008_CLIENT_ADDR, MCP23008_REG_ADDR_IODIR, PINS_DIGITAL_OUTPUT);

    while(1)
    {   
        if(TC_flag)
        {
            /*Read data from ADC*/
            i2c_readNBytes(I2C_MCP3221_CLIENT_ADDR, dataRead, 2);

            /*Make one 16-bit value from the 2 bytes read from ADC*/
            rawADCValue = (uint16_t) ((dataRead[0] << 8) | (dataRead[1] & 0xff));
            
            /*Write to I/O Expander based on raw ADC voltage value*/
            dataWrite[0] = MCP23008_REG_ADDR_GPIO;
            dataWrite[1] = mapADCToGPIO(rawADCValue);
            i2c_writeNBytes(I2C_MCP23008_CLIENT_ADDR, dataWrite, 2);
            
            /*Convert value to float*/
            ADCVoltage = rawADCValue*(Vdd/resolution);

            /*Write to data visualizer*/
            variableWrite_SendFrame(ADCVoltage);
            
            TC_flag = 0;
        }
    }    
}

/*
 *Decided how many LEDs should be turned on based on the raw voltage value of read from the ADC.
 *Then writes the corresponding value to the I/O expander.
 */
uint8_t mapADCToGPIO(uint16_t rawADCValue){
    uint16_t interval = 512; /*4096/8 = 512*/
    uint16_t i = rawADCValue/interval;
    if (i > 8) i = 8;
    switch(i)
    {
        case 0: return 0x01; /*0V*/
        case 1: return 0x03; /*0.4125V*/
        case 2: return 0x07; /*0.825V*/
        case 3: return 0x0F; /*1.2375*/
        case 4: return 0x1F; /*1.65V*/
        case 5: return 0x3F; /*2.0625V*/
        case 6: return 0x7F; /*2.475V*/
        case 7: return 0xFF; /*2.8875V*/           
    }
}