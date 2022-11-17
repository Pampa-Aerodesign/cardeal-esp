#pragma once
#include <stdio.h>

#define SDTAG "SD"

// full data structure
typedef struct data_packet_t{
  int64_t timestamp;
  uint8_t packetid;
  uint8_t logging;
  int baro;
  float temp;
} DataPacket;

void logWriteHeader(FILE* file);
void logWrite(FILE* file, DataPacket* datapacket);
void taskSD(void *datapacket);