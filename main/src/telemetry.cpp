// This file contains functions and tasks related to the LoRa SX1276 board
// used for real-time telemetry

// CardealESP config header
#include "include/config.hpp"

// FreeRTOS
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

// DataPacket
#include "include/sdlog.hpp"

// LoRa communication via SX1276 chips
#include "lora.h"
#include "include/telemetry.hpp"

// LoRa telemetry task
void taskLoRa_tx(void *pvParameters){
  ESP_LOGI(LORATAG, "Initializing LoRa transmitter");
  lora_init();
  /* Exact same configuration as the receiver chip: */
  lora_set_frequency(915e6);  // 915 MHz, unchanged
  lora_set_tx_power(LORA_TX_POWER);
  lora_set_spreading_factor(LORA_SPREAD_FACT);
  lora_set_coding_rate(LORA_CODING_RATE);
  lora_set_preamble_length(LORA_PREAMBLE_LEN);
  // lora_explicit_header_mode();
  lora_set_sync_word(LORA_SYNC_WORD);
  lora_disable_crc();
  ESP_LOGI(LORATAG, "LoRa TX initialized");
  ESP_LOGI(LORATAG, "Starting the loop");

  while (1) {
    // Build LoRa Packet
    LoraPacket lorapacket;
    lorapacket.packetid = ((DataPacket *)pvParameters)->packetid;
    lorapacket.logging = ((DataPacket *)pvParameters)->logging;
    lorapacket.baro = ((DataPacket *)pvParameters)->baro;
    lorapacket.temp = ((DataPacket *)pvParameters)->temp;
    lorapacket.bat_amp = ((DataPacket *)pvParameters)->bat_amp;
    lorapacket.elev_amp = ((DataPacket *)pvParameters)->elev_amp;
    lorapacket.ail_amp = ((DataPacket *)pvParameters)->ail_amp;
    lorapacket.rud_amp = ((DataPacket *)pvParameters)->rud_amp;

    // Transmit packet
    lora_send_packet((uint8_t *)&lorapacket, sizeof(LoraPacket));
    ESP_LOGI(LORATAG, "Packet %d sent", lorapacket.packetid);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}