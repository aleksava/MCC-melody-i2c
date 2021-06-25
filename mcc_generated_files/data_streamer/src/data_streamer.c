// Generate variable frames for sending data to the MPLAB Data Visualizer, 
// e.g. to be display on a graph.
#include <stdint.h>
#include "../../uart/eusart2.h"
#include "../data_streamer.h"

#define DATA_STREAMER_START_BYTE 3  //trivial Data Streamer Protocol start of frame token
#define DATA_STREAMER_END_BYTE (255 - DATA_STREAMER_START_BYTE)  
// Data Streamer Protocol end byte is the One's compliment (~) of startByte, 
// e.g. startByte: 0b0000 0011, endByte: 0b1111 1100

static void variableWrite_SendValue(uint8_t* byte_ptr, uint8_t num_bytes)
 {
      for(uint8_t i = 0; i < num_bytes; i++)
      {
         EUSART2_Write(byte_ptr[i]);
      }
}

void variableWrite_SendFrame(float count_f)
{
   EUSART2_Write(DATA_STREAMER_START_BYTE);  

   variableWrite_SendValue((uint8_t *) &count_f, 4);         // float

   EUSART2_Write(DATA_STREAMER_END_BYTE);  

   while(!EUSART2_IsTxDone());
}
