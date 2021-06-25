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
 
#include "../../eeprom-at24cm02/at24cm02.h"
#include "../../eeprom-at24cm02/at24cm02_config.h"
#include "../../system/system.h"
#include <stdlib.h>
 
#define MIN(a,b) (a<b?a:b)
 
typedef struct
{
    uint16_t wordAdress;
    uint8_t *data;
    uint16_t dataSize;
} block_t;
 
typedef struct
{
    uint8_t deviceAddrs;
    uint16_t wordAddrs;
} addrs_t;
 
// Local APIs Declaration
static uint8_t AT24CM02_MSBAddressGet(uint32_t address);
static uint16_t AT24CM02_WordAddressGet(uint32_t address);
static uint8_t AT24CM02_DeviceAddressGet(uint8_t MSBAddress);
static addrs_t AT24CM02_ParsedAddressGet(uint32_t address);
static void AT24CM02_DataToSendSet(block_t *block, uint8_t *dataToSend);
 
/*
* Local API Definitions
*/
 
static uint8_t AT24CM02_MSBAddressGet(uint32_t address)
{
    uint8_t deviceAddress;
 
    deviceAddress = (address & 0xF0000) >> 16;
 
    return (deviceAddress);
}
 
static uint16_t AT24CM02_WordAddressGet(uint32_t address)
{
    return (uint16_t) (address & 0x00FFFF);
}
 
static uint8_t AT24CM02_DeviceAddressGet(uint8_t MSBAddress)
{
    MSBAddress = ((MSBAddress << 1) & 0x06);
 
    return ((DEVICE_ADDRS_BYTE | MSBAddress) >> 1);
}
 
static addrs_t AT24CM02_ParsedAddressGet(uint32_t address)
{
    addrs_t addrs;
 
    uint8_t msbAddrs = AT24CM02_MSBAddressGet(address);
    addrs.deviceAddrs = AT24CM02_DeviceAddressGet(msbAddrs);
    addrs.wordAddrs = AT24CM02_WordAddressGet(address);
 
    return addrs;
}
 
static void AT24CM02_DataToSendSet(block_t *block, uint8_t *dataToSend)
{
    dataToSend[0] = (block->wordAdress >> 8) &0xFF;
    dataToSend[1] = block->wordAdress & 0xFF;
 
    for (uint16_t index = 0; index < block->dataSize; ++index) {
        dataToSend[index + 2] = *(block->data + index);
    }
}
 
 
/*
* Global API Definitions
*/
 
void AT24CM02_WriteOneByte(uint32_t address, uint8_t data)
{
    AT24CM02_Write(address, &data, 1);
}
 
uint8_t AT24CM02_ReadOneByte(uint32_t address)
{
    uint8_t readData = 0;
 
    AT24CM02_Read(address, &readData, 1);
 
    return readData;
}
 
void AT24CM02_Write(uint32_t startAddress, void *data, uint16_t byteCount)
{
    addrs_t parsedAddrs;
    block_t block = { 0 };
    uint8_t dataToSend[PAGESIZE];
 
    uint16_t blockSize = (uint16_t) PAGESIZE - (startAddress & (PAGESIZE - 1));
    uint8_t *blockData = data;
 
    do
    {
        blockSize = MIN(blockSize,byteCount);
 
        parsedAddrs = AT24CM02_ParsedAddressGet(startAddress);
 
        block.wordAdress = parsedAddrs.wordAddrs;
        block.data = blockData;
        block.dataSize = blockSize;
 
        AT24CM02_DataToSendSet(&block, dataToSend);
        I2C1_Write(parsedAddrs.deviceAddrs, dataToSend, (blockSize + 2));
 
        startAddress = (startAddress + blockSize) & 0x3FFFF;
        blockData = blockData + blockSize;
        byteCount = byteCount - blockSize;
        blockSize = PAGESIZE;
    } while(byteCount > 0);
}
 
void AT24CM02_Read(uint32_t startAddress, void* data, uint16_t byteCount)
{
    addrs_t parsedAddrs;
    block_t block = { 0 };
    uint8_t dataToSend[PAGESIZE];
 
    parsedAddrs = AT24CM02_ParsedAddressGet(startAddress);
 
    block.wordAdress = parsedAddrs.wordAddrs;
    block.data = NULL;
    block.dataSize = 0;
 
    AT24CM02_DataToSendSet(&block, dataToSend);
    I2C1_WriteRead(parsedAddrs.deviceAddrs, dataToSend, 2, data, byteCount);
}