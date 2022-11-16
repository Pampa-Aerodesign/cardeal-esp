#pragma once
#include <stdio.h>

// full data structure
typedef struct data_packet_t{
  int64_t timestamp;
  uint8_t packetid;
  int baro;
  float temp;
} DataPacket;

void logWriteHeader(FILE** file);
void logWrite(FILE** file, DataPacket* sdpacket);