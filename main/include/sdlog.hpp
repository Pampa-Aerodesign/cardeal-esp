#pragma once
#include <stdio.h>
#include "freertos/event_groups.h"

#define SDTAG "SD"

// full data structure
typedef struct data_packet_t{
  EventGroupHandle_t* eventWrite;
  uint16_t databits;  // bits corresponding to eventgroup bits
                    // 1 = sensor reading updated; 0 = sensor not updated
  int64_t timestamp;
  uint8_t packetid;
  uint8_t logging;
  int baro;
  float temp;
  float bat_amp, elev_amp, ail_amp, rud_amp;
} DataPacket;

void logWriteHeader(FILE* file);
void logWrite(FILE* file, DataPacket* datapacket);
void taskSD(void *datapacket);