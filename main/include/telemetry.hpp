#pragma once

#define LORATAG "LORA"

// LoRa packet struct
typedef struct lora_packet{
  uint8_t packetid;
  uint8_t logging;
  int baro;
  float temp;
  float bat_amp, elev_amp, ail_amp, rud_amp;
} LoraPacket;

void taskLoRa_tx(void *pvParameters);