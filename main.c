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

#define I2C_MCP9800_ADDRESS             0x49
#define I2C_MCP9800_TEMP_REG            0x00
#define I2C_MCP9800_CONFIG_REG          0x01
#define I2C_MCP9800_CONFIG_DATA         0x60

#define I2C_MCP23008_ADDRESS            0x20
#define I2C_MCP23008_DIR_REG            0x00
#define I2C_MCP23008_GPIO_REG           0x09
#define I2C_MCP23008_DIR_DATA           0x00 

uint8_t dataRead[2];
uint8_t dataWrite[4];
uint16_t tempRaw;
float tempCelsius = 0.5;
bool TC_overflow_flag = false;

uint8_t mapTempToGPIO(uint16_t tempRaw);
void GPIO_init(void);
void tempSensor_init(void);

void TC_overflow_cb(void){
    TC_overflow_flag = true;
    LED_RE0_Toggle();
    DebugIO_RE2_Toggle();
}

int main(void)
{
    SYSTEM_Initialize();
    Timer0.TimeoutCallbackRegister(TC_overflow_cb);
    // Enable the Global Interrupts
    INTERRUPT_GlobalInterruptEnable();
    // Enable the Peripheral Interrupts
    INTERRUPT_PeripheralInterruptEnable();
    
    GPIO_init();
    tempSensor_init();

    while(1)
    {
        if(TC_overflow_flag)
        {
            TC_overflow_flag = false;
            
            /* Reading the temp sensor */
            i2c_readNBytes(I2C_MCP9800_ADDRESS, dataRead, 2);
            tempRaw = (dataRead[0] << 4) + (dataRead[1] >> 4);
            
            /* Displaying the temperature on the GPIO LEDs */
            dataWrite[0] = I2C_MCP23008_GPIO_REG;
            dataWrite[1] = mapTempToGPIO(tempRaw);
            i2c_writeNBytes(I2C_MCP23008_ADDRESS,dataWrite,2);
            
            /* Converting the raw temperature to degrees Celsius */
            tempCelsius = tempRaw / 16.0;
            variableWrite_SendFrame(tempCelsius);
        }
    }    
}


void tempSensor_init(void)
{
        /* Enabling 12-bit resolution on temperature sensor */
        dataWrite[0] = I2C_MCP9800_CONFIG_REG;
        dataWrite[1] = I2C_MCP9800_CONFIG_DATA;
        i2c_writeNBytes(I2C_MCP9800_ADDRESS,dataWrite,2);
        __delay_ms(50);

        dataWrite[0] = I2C_MCP9800_TEMP_REG;
        i2c_writeNBytes(I2C_MCP9800_ADDRESS,dataWrite,1);
        __delay_ms(50);
}

void GPIO_init(void)
{
    /* Setting GPIO pins as outputs */
    dataWrite[0] = I2C_MCP23008_DIR_REG;
    dataWrite[1] = I2C_MCP23008_DIR_DATA;
    i2c_writeNBytes(I2C_MCP23008_ADDRESS,dataWrite,2);
    __delay_ms(50);
}


uint8_t mapTempToGPIO(uint16_t tempRaw){
    uint16_t minTemp = 384; // Degrees Celsius: 384/16 = 24
    uint8_t resolution = 8;         // 1*C = 16.
    tempRaw = tempRaw - minTemp;
    uint8_t i = tempRaw/resolution;
    if (i > 8) i = 8;
    switch(i)
    {
        case 0: return 0x00;    //24,00 *C
        case 1: return 0x01;    //24,25 *C
        case 2: return 0x03;    //24,50 *C
        case 3: return 0x07;    //24,75 *C
        case 4: return 0x0F;    //25,00 *C
        case 5: return 0x1F;    //25,25 *C
        case 6: return 0x3F;    //25,50 *C
        case 7: return 0x7F;    //25,75 *C
        case 8: return 0xFF;    //26,00 *C
    }
}

