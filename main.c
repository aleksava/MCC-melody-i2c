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
#include "mcc_generated_files/data_streamer/data_streamer.h"
#include "mcc_generated_files/i2c_host/i2c_simple_host.h"
#include "mcc_generated_files/eeprom-at24cm02/at24cm02.h"


void EEPROM_Temp_Read(uint8_t EEPROM_lastRegWritten);
uint8_t EEPROM_Temp_Write(uint8_t EEPROM_ADDRESS, uint8_t EEPROM_REGISTER_ADDRESS);
static uint8_t i2c_writeNBytesEEPROM(i2c1_address_t address, uint8_t memoryAddress, uint8_t* data, uint8_t dataLength, uint8_t EEPROMPagesize);
#define MIN(x,y) (((x)<(y)) ? (x) : (y))
bool I2C_ACK_POLL(uint8_t address);

/*
    Main application
*/

#define I2C_TEMP_CLIENT_ADR         0x49
#define I2C_TEMP_READ_REG           0x00
#define I2C_TEMP_CONFIG_REG         0x01
#define I2C_TEMP_CONFIG_DATA        0x60

uint8_t TC_flag = 0;
uint8_t dataRead[5];
uint8_t dataWrite[6];
uint16_t rawTempData;
float   celciusTemp = 0.5;
uint16_t rawTempHigh = 0;
uint16_t rawTempLow = 0xFFFF;


#define I2C_EEPROM_CLIENT_ADR       0x50
#define I2C_EEPROM_REG_POINTER      0xFF
#define PAGESIZE                       8

uint8_t EEPROM_lastRegWritten = 0;
bool    gate_open = true;

#define I2C_GPIO_CLIENT_ADR         0x20
#define I2C_GPIO_DIR_REG            0x00
#define I2C_GPIO_PIN_REG            0x09

uint8_t gpio_data = 0xFF;


void TC_overflow_cb(void){
    
    TC_flag++;
    LED_RE0_Toggle();
    DebugIO_RB6_Toggle();
}

int main(void)
{
    SYSTEM_Initialize();

    Timer0.TimeoutCallbackRegister(TC_overflow_cb);

    // Enable the Global Interrupts
    INTERRUPT_GlobalInterruptEnable();

    // Enable the Peripheral Interrupts
    INTERRUPT_PeripheralInterruptEnable();
    
    /*Set up temp sensor config*/
    dataWrite[0] = I2C_TEMP_CONFIG_REG;
    dataWrite[1] = I2C_TEMP_CONFIG_DATA;
    i2c_writeNBytes(I2C_TEMP_CLIENT_ADR, dataWrite, 2);
    
    dataWrite[0] = I2C_TEMP_READ_REG;
    i2c_writeNBytes(I2C_TEMP_CLIENT_ADR, dataWrite, 1);
    
    /*Set up gpio config*/
    dataWrite[0] = I2C_GPIO_DIR_REG;
    dataWrite[1] = 0x00;
    i2c_writeNBytes(I2C_GPIO_CLIENT_ADR, dataWrite, 2);
 
    /*Set up memory pointer for EEPROM*/
    dataWrite[0] = I2C_EEPROM_REG_POINTER;
    dataWrite[1] = EEPROM_lastRegWritten;
    i2c_writeNBytes(I2C_EEPROM_CLIENT_ADR, dataWrite, 2);

    __delay_ms(10);
    EEPROM_lastRegWritten = i2c_read1ByteRegister(I2C_EEPROM_CLIENT_ADR, I2C_EEPROM_REG_POINTER);
    printf("\r\nLast register written to is: %i\r\n", EEPROM_lastRegWritten);
    
    while(1)
    {
        /*Note the in 12-bit mode, the update time of the temp sensor is 240ms conversion time*/
        if(TC_flag % 10 == 0)
        {
            i2c_readNBytes(I2C_TEMP_CLIENT_ADR, dataRead, 2);
            //i2c_readDataBlock(I2C_TEMP_CLIENT_ADR,I2C_TEMP_READ_REG, dataRead, 2);
            /*Calculate the temperature in celsius*/
            rawTempData = (dataRead[0] << 4) + (dataRead[1] >> 4);
            celciusTemp = (float) (rawTempData / 16.0);
            printf("\r\nRaw temperature data: %i \t garbage: ", rawTempData);
            variableWrite_SendFrame(celciusTemp);
            
            /*Tracking highest and lowest temperature*/
            if(rawTempData > rawTempHigh)
            {
                rawTempHigh = rawTempData;
            }
            if(rawTempData < rawTempLow)
            {
                rawTempLow = rawTempData; 
            }
            
            /*Blink the leds on I2C board*/
            dataWrite[0] = I2C_GPIO_PIN_REG;
            dataWrite[1] = gpio_data;
            i2c_writeNBytes(I2C_GPIO_CLIENT_ADR, dataWrite, 2);
            gpio_data = ~gpio_data;
            
            gate_open = true;
            
        }
        
        if(TC_flag >= 100)
        {
            TC_flag = 0;
            EEPROM_Temp_Read(EEPROM_lastRegWritten);
        }
        
        if(!SW0_RE2_GetValue() & gate_open)
        {
            gate_open = false;
            dataWrite[0] = ((rawTempLow & 0xFF00) >> 8);
            dataWrite[1] = (rawTempLow) & 0x00FF;
            dataWrite[2] = ((rawTempHigh & 0xFF00) >> 8);
            dataWrite[3] = (rawTempHigh) & 0x00FF;
            EEPROM_lastRegWritten = i2c_writeNBytesEEPROM(I2C_EEPROM_CLIENT_ADR, EEPROM_lastRegWritten, dataWrite, 4, PAGESIZE);
            printf("\r\n Last eeprom register: %i \t low: %i \t high: %i ",EEPROM_lastRegWritten, rawTempLow, rawTempHigh);
            rawTempLow = 0xFFFF;
            rawTempHigh = 0;
        }
    }    
}


void EEPROM_Temp_Read(uint8_t EEPROM_lastRegWritten){
    uint8_t i = 0;
    uint8_t block_size = 4;
    uint16_t rawTempPrint[2];
    printf("\r\n-----------------------------------");
    while(i < (EEPROM_lastRegWritten/block_size))
    {
        i2c_readDataBlock(I2C_EEPROM_CLIENT_ADR, i*block_size, dataRead, 4);
        __delay_ms(10);
        rawTempPrint[0] = (dataRead[0] << 8) + dataRead[1];
        rawTempPrint[1] = (dataRead[2] << 8) + dataRead[3];
        printf("\r\ntemperature log number: %i \t rawTempLow: %i \t rawTempHigh: %i", i, rawTempPrint[0], rawTempPrint[1]);
        i++;
    }
    printf("\r\n-----------------------------------");
}

//

/* This function enables the user to write N bytes to an EEPROM without having to think about pagesize and pagebuffer.
 * However, it does not take care of end of memory space issues. E.g. what happens when we try to write past the last memory address.
 * Returns a value that corresponds to the last memory address written to */
static uint8_t i2c_writeNBytesEEPROM(i2c1_address_t address, uint8_t memoryAddress, uint8_t* data, uint8_t dataLength, uint8_t EEPROMPagesize)
{
    uint8_t pageCounter = memoryAddress/EEPROMPagesize;
    uint8_t pageEnd = pageCounter + dataLength / EEPROMPagesize;
    uint8_t dataPerIteration = MIN(EEPROMPagesize - (memoryAddress%EEPROMPagesize) ,dataLength);
    uint8_t dataBuffer[8 + 1];  /* PAGESIZE + memoryAddress */
    
    /* Storing the memory pointer */
    dataBuffer[0] = memoryAddress;

    while(pageCounter <= pageEnd)
    {
        /* Loading the desired data onto the buffer */
        for (uint8_t i = 0; i < dataPerIteration; i++)
        {
            dataBuffer[i+1] = *data++;
        } 
        /* Writing the memory address and data to EEPROM */
        i2c_writeNBytes(address, dataBuffer, dataPerIteration + 1);

        /* Updating variables for next iteration */
        dataLength -= dataPerIteration;
        dataBuffer[0] += dataPerIteration;
        dataPerIteration = MIN(EEPROMPagesize,dataLength); 
        pageCounter++;
       
        /* ACK polling to wait for page write buffer */
        while (I2C_ACK_POLL(address));
    }  
    return dataBuffer[0];
}

bool I2C_ACK_POLL(uint8_t address)
{
    while(!I2C1_Open(address)); // sit here until we get the bus..
    I2C1_HostRead();
    if(!(SSP1CON2 & _SSP1CON2_ACKSTAT_MASK))
    {
        return true;
    }
    while(I2C1_BUSY == I2C1_Close()); // sit here until finished.
    return false;
}


//uint8_t EEPROM_Temp_Write(uint8_t EEPROM_ADDRESS, uint8_t EEPROM_REGISTER_ADDRESS)
//{
//        /*Save temperature data to EEPROM*/
//    EEPROM_lastRegWritten = i2c_read1ByteRegister(I2C_EEPROM_CLIENT_ADR, I2C_EEPROM_REG_POINTER);
//    printf("\r\n Last eeprom register: %i \t low: %i \t high: %i ",EEPROM_lastRegWritten, rawTempLow, rawTempHigh);
//    __delay_ms(50);
//
//    /*In order keep our register pointer at 255 i need to set the limit here. */
//    if (EEPROM_lastRegWritten == 251) EEPROM_lastRegWritten = 0;
//
//    dataWrite[0] = EEPROM_lastRegWritten;
//    dataWrite[1] = ((rawTempLow & 0xFF00) >> 8);
//    dataWrite[2] = (rawTempLow) & 0x00FF;
//    dataWrite[3] = ((rawTempHigh & 0xFF00) >> 8);
//    dataWrite[4] = (rawTempHigh) & 0x00FF;
//    i2c_writeNBytes(I2C_EEPROM_CLIENT_ADR, dataWrite, 5);
//    __delay_ms(50);
//    EEPROM_lastRegWritten += 4;
//    dataWrite[0] = I2C_EEPROM_REG_POINTER;
//    dataWrite[1] = EEPROM_lastRegWritten;
//    i2c_writeNBytes(I2C_EEPROM_CLIENT_ADR, dataWrite, 2);
//    __delay_ms(50);
//    return EEPROM_lastRegWritten;
//}