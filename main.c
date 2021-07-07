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

#define I2C_24LC02B_ADDRESS             0x50
#define EEPROM_ADDRESS_POINTER          0xFF
#define EEPROM_SAMPLE_TIMER             0x7F
#define PAGESIZE                        8


uint8_t dataRead[4];
uint8_t dataWrite[6];
uint16_t tempRaw;
uint16_t rawTempHigh = 0;
uint16_t rawTempLow = 0xFFFF;
float tempCelsius = 0.5;
bool TC_overflow_flag = false;
uint8_t EEPROM_memory_pointer;
uint8_t EEPROM_write_counter = 0;

#define MIN(x,y) (((x)<(y)) ? (x) : (y))
uint8_t mapTempToGPIO(uint16_t tempRaw);
void GPIO_init(void);
void tempSensor_init(void);
static uint8_t i2c_writeNBytesEEPROM(i2c1_address_t address, uint8_t memory_address, uint8_t* data, size_t data_length, uint8_t EEPROM_Pagesize);
void EEPROM_reset(void);
uint8_t EEPROM_Temp_Write(uint8_t EEPROM_ADDRESS, uint8_t EEPROM_REGISTER_ADDRESS);
void EEPROM_Temp_Read(uint8_t EEPROM_lastRegWritten);
void GPIO_Blink(void);

void TC_overflow_cb(void){
    TC_overflow_flag = true;
    EEPROM_write_counter++;
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
    EEPROM_reset();
    
    while(1)
    {
        if(TC_overflow_flag)
        {
            TC_overflow_flag = false;
            
            /* Reading the temp sensor */
            i2c_readNBytes(I2C_MCP9800_ADDRESS, dataRead, 2);
            tempRaw = (dataRead[0] << 4) + (dataRead[1] >> 4);
            
            /*Tracking highest and lowest temperature*/
            if(tempRaw > rawTempHigh)
            {
                rawTempHigh = tempRaw;
            }
            if(tempRaw < rawTempLow)
            {
                rawTempLow = tempRaw; 
            }
            
            
            /* Displaying the temperature on the GPIO LEDs */
            dataWrite[0] = I2C_MCP23008_GPIO_REG;
            dataWrite[1] = mapTempToGPIO(tempRaw);
            i2c_writeNBytes(I2C_MCP23008_ADDRESS,dataWrite,2);
            
            /* Converting the raw temperature to degrees Celsius */
            tempCelsius = tempRaw / 16.0;
            printf("\r\n rawTemp: %i \t dataStreamer: ", tempRaw);
            variableWrite_SendFrame(tempCelsius);
        }
        
        if(EEPROM_write_counter >= EEPROM_SAMPLE_TIMER)
        {
            EEPROM_write_counter = 0;
            EEPROM_memory_pointer = EEPROM_Temp_Write(I2C_24LC02B_ADDRESS, EEPROM_ADDRESS_POINTER);
            EEPROM_Temp_Read(EEPROM_memory_pointer);
            GPIO_Blink();
            rawTempLow = 0xFFFF;
            rawTempHigh = 0;
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


void GPIO_Blink(void)
{
    dataWrite[0] = I2C_MCP23008_GPIO_REG;
    dataWrite[1] = 0xFF;
    i2c_writeNBytes(I2C_MCP23008_ADDRESS,dataWrite,2);
    __delay_ms(500);
}


uint8_t mapTempToGPIO(uint16_t tempRaw){
    uint16_t minTemp = 384;//384; // Degrees Celsius: 384/16 = 24
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
        default: return 0xAA;
    }
}


static uint8_t i2c_writeNBytesEEPROM(i2c1_address_t address, uint8_t memory_address, uint8_t* data, size_t data_length, uint8_t EEPROM_Pagesize)
{
    uint8_t page_counter = memory_address/EEPROM_Pagesize;
    uint8_t page_end = page_counter + data_length / EEPROM_Pagesize;
    uint8_t data_length_iteration = MIN(EEPROM_Pagesize - (memory_address%EEPROM_Pagesize) ,data_length);
    uint8_t dataBuffer[8 + 1];  //PAGESIZE + memory_address

    
    /* Storing the memory pointer */
    dataBuffer[0] = memory_address;

    while(page_counter <= page_end)
    {
        /* Loading the desired data onto the buffer */
        for (uint8_t i = 0; i < data_length_iteration; i++)
        {
            dataBuffer[i+1] = *data++;
        } 
        /* Writing the memory address and data to EEPROM */
        i2c_writeNBytes(address, dataBuffer, data_length_iteration + 1);

        /* Updating variables for next iteration */
        data_length -= data_length_iteration;
        dataBuffer[0] += data_length_iteration; 
        data_length_iteration = MIN(EEPROM_Pagesize,data_length);
        page_counter++;
        __delay_ms(20);
    }  
    return dataBuffer[0];
}

void EEPROM_reset(void)
{
    dataWrite[0] = EEPROM_ADDRESS_POINTER;
    dataWrite[1] = 0x00;
    i2c_writeNBytes(I2C_24LC02B_ADDRESS, dataWrite, 2);
}


uint8_t EEPROM_Temp_Write(uint8_t EEPROM_ADDRESS, uint8_t EEPROM_REGISTER_ADDRESS)
{
    /*Save temperature data to EEPROM*/
    EEPROM_memory_pointer = i2c_read1ByteRegister(I2C_24LC02B_ADDRESS, EEPROM_ADDRESS_POINTER);
    printf("\r\n Last EEPROM register: %i \t low: %i \t high: %i ",EEPROM_memory_pointer, rawTempLow, rawTempHigh);
    __delay_ms(50);

    /*In order keep our register pointer at 255 i need to set the limit here. */
    if (EEPROM_memory_pointer == 251) EEPROM_memory_pointer = 0;

    //dataWrite[0] = EEPROM_lastRegWritten;
    dataWrite[0] = ((rawTempLow & 0xFF00) >> 8);
    dataWrite[1] = (rawTempLow) & 0x00FF;
    dataWrite[2] = ((rawTempHigh & 0xFF00) >> 8);
    dataWrite[3] = (rawTempHigh) & 0x00FF;
    
    EEPROM_memory_pointer = i2c_writeNBytesEEPROM(I2C_24LC02B_ADDRESS, EEPROM_memory_pointer, dataWrite, 4, PAGESIZE);
    __delay_ms(20);
    dataWrite[0] = EEPROM_ADDRESS_POINTER;
    dataWrite[1] = EEPROM_memory_pointer;
    i2c_writeNBytes(I2C_24LC02B_ADDRESS, dataWrite, 2);
    __delay_ms(20);
    return EEPROM_memory_pointer;
}


void EEPROM_Temp_Read(uint8_t EEPROM_memory_pointer){
    uint8_t i = 0;
    uint8_t block_size = 4;
    uint16_t rawTempPrint[2];
    printf("\r\n-----------------------------------");
    while(i < (EEPROM_memory_pointer/block_size))
    {
        dataWrite[0] = i*block_size;
        i2c_writeNBytes(I2C_24LC02B_ADDRESS,dataWrite,1);
        __delay_ms(10);
        i2c_readNBytes(I2C_24LC02B_ADDRESS,dataRead,4);
        __delay_ms(10);
        rawTempPrint[0] = (dataRead[0] << 8) + dataRead[1];
        rawTempPrint[1] = (dataRead[2] << 8) + dataRead[3];
        printf("\r\ntemperature log number: %i \t rawTempLow: %i \t rawTempHigh: %i", i, rawTempPrint[0], rawTempPrint[1]);
        i++;
    }
    printf("\r\n-----------------------------------");
}