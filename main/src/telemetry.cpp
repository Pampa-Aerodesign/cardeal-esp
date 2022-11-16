// CardealESP config header
#include "include/config.h"

// FreeRTOS
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

// DataPacket
#include "include/sdlog.hpp"

// LoRa communication via SX1276 chips
#include "lora.h"
#include "include/telemetry.hpp"

void taskLoRa_tx(void *pvParameters){
  lora_init();
  /* Exact same configuration as the receiver chip: */
  lora_set_frequency(915e6);
  lora_set_tx_power(17);
  lora_set_spreading_factor(8);
  lora_set_coding_rate(5);
  lora_set_preamble_length(8);
  // lora_explicit_header_mode();
  lora_set_sync_word(0x12);
  lora_disable_crc();

  while (1) {
    // Build LoRa Packet
    LoraPacket lorapacket;
    lorapacket.packetid = ((DataPacket *)pvParameters)->packetid;
    lorapacket.baro = ((DataPacket *)pvParameters)->baro;
    lorapacket.temp = ((DataPacket *)pvParameters)->temp;

    lora_send_packet((uint8_t *)&lorapacket, sizeof(LoraPacket));
    printf("Packet %d sent...\n", lorapacket.packetid);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}