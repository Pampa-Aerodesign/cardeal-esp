#pragma once

#define LORATAG "LORA"

// LoRa packet struct
typedef struct lora_packet{
  uint8_t packetid;
  int baro;
  float temp;
} LoraPacket;

void taskLoRa_tx(void *pvParameters);