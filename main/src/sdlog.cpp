// This file contains functions and tasks related to the SD Card module

// Standard libraries
#include <stdio.h>
#include <string.h>
#include <string>

// CardealESP config header
#include "include/config.hpp"

// FreeRTOS
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

// SD Card
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "include/sdlog.hpp"

// Write CSV file header
void logWriteHeader(FILE* file){
  fprintf(file, "PacketID,Timestamp,Baro,Temp\n");
}

// Write data into CSV file
void logWrite(FILE* file, DataPacket* datapacket){
  // get timestamp in miliseconds
  ((DataPacket *)datapacket)->timestamp = esp_timer_get_time()/1000;

  // Retrieve data from DataPacket
  fprintf(file, "%d,", datapacket->packetid);
  fprintf(file, "%lld,", datapacket->timestamp);
  fprintf(file, "%d,", datapacket->baro);
  fprintf(file, "%lf", datapacket->temp);

  // Print CRLF
  fprintf(file, "\n");
}

// Initialize SPI and mount SD card
sdmmc_card_t* sd_card_init(void){
  // ESP LOG error code and tag
  esp_err_t ret;

  // Options for mounting the filesystem.
  // If format_if_mount_failed is set to true, SD card will be partitioned and
  // formatted in case when mounting fails.
  esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED
    .format_if_mount_failed = true,
#else
    .format_if_mount_failed = false,
#endif // EXAMPLE_FORMAT_IF_MOUNT_FAILED
    .max_files = 5,
    .allocation_unit_size = 512 //16 * 1024
  };
  sdmmc_card_t *card;

  // Use settings defined above to initialize SD card and mount FAT filesystem.
  // Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.
  // Please check its source code and implement error recovery when developing
  // production applications.

  ESP_LOGI(SDTAG, "Using SPI peripheral");

  // replaced host.slot everywhere with this line, probably not ideal
  spi_host_device_t hostslot = SPI2_HOST;

  sdmmc_host_t host = SDSPI_HOST_DEFAULT();
  host.max_freq_khz = MAX_FREQ_SPI_SDCARD;
  spi_bus_config_t bus_cfg = {
    .mosi_io_num = PIN_NUM_MOSI,
    .miso_io_num = PIN_NUM_MISO,
    .sclk_io_num = PIN_NUM_CLK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 4000,
  };
  ret = spi_bus_initialize(hostslot, &bus_cfg, SPI_DMA_CHAN);
  if (ret != ESP_OK) {
    ESP_LOGE(SDTAG, "Failed to initialize bus.");
    // return;
  }

  // This initializes the slot without card detect (CD) and write protect (WP) signals.
  // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
  sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
  slot_config.gpio_cs = PIN_NUM_CS;
  slot_config.host_id = hostslot;

  ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card);

  if (ret != ESP_OK) {
    if (ret == ESP_FAIL) {
      ESP_LOGE(SDTAG, "Failed to mount filesystem. "
                "If you want the card to be formatted, set the EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
    } else {
      ESP_LOGE(SDTAG, "Failed to initialize the card (%s). "
                "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
    }
    // return;
  }

  ESP_LOGI(SDTAG, "Filesystem mounted");
  // SD card finished initializing

  return card;
}

// Unmount SD card and deinitialize SPI
void sd_card_unmount(sdmmc_card_t* card){
  // replaced host.slot everywhere with this line, probably not ideal
  spi_host_device_t hostslot = SPI2_HOST;

  // Close and unmount
  // All done, unmount partition and disable SDMMC or SPI peripheral
  esp_vfs_fat_sdcard_unmount(MOUNT_POINT, card);
  ESP_LOGI(SDTAG, "Card unmounted");
#ifdef USE_SPI_MODE
  // deinitialize the bus after all devices are removed
  spi_bus_free(hostslot);
#endif
}

// SD Card logging task
void taskSD(void *datapacket){
  ESP_LOGI(SDTAG, "Initializing SD card");
  sdmmc_card_t* card = sd_card_init();

  // check for "start logging" signal (aux3 on rx, GPIO 16)
  gpio_set_direction(PIN_SDLOG, GPIO_MODE_INPUT);
  gpio_pullup_en(PIN_SDLOG);

  // File name
  int attempt = 0;
  std::string strattempt;
  char fname[32] = "\0";
  FILE* file = NULL;

  bool logging = false; // flag to check if it file was already created
  ((DataPacket*) datapacket)->packetid = 0; // start packetid at zero
  ((DataPacket *)datapacket)->logging = 0; // update logging status

  ESP_LOGI(SDTAG, "Starting the loop");

  while(1){
    // start logging once the signal has been received from the RX
    if(!gpio_get_level(PIN_SDLOG)){
      // try to open file in read mode. if it opens, the file already exists,
      // increment number in filename and try to open again.
      // when it fails, that means the file does not exist, so open it
      // in write mode to create the file and start logging
      if(!logging){
        do{
          fclose(file);
          attempt++;
          strcpy(fname, MOUNT_POINT "/" FILENAME);
          strattempt = std::to_string(attempt);
          strcat(fname, strattempt.c_str());
          strcat(fname, "." FILETYPE);
          file = fopen(fname, "r");
        } while(file != NULL && attempt < 255);

        ESP_LOGI(SDTAG, "Attempting to create file %s", fname);

        // open file in write mode
        if(file == NULL){
          file = fopen(fname, "w");

          // not elegant but whatever, none of this is so far
          if(file == NULL){
            ESP_LOGE(SDTAG, "File %s failed to open. Killing task", fname);
            sd_card_unmount(card);
            vTaskSuspend(NULL);
          }

          // Write header in CSV file
          logWriteHeader(file);
          ESP_LOGI(SDTAG, "File %s created", fname);
        }

        logging = true;
        ((DataPacket *)datapacket)->packetid = 0;   // start packetid at zero
        ((DataPacket *)datapacket)->logging = attempt; // update logging status
        ESP_LOGI(SDTAG, "Created file %s, starting log", fname);
      }
      
      // write packet to SD card
      logWrite(file, (DataPacket *)datapacket);
    }
    else{ // Stop logging (GPIO16 == HIGH)
      // check if a file is open and close it
      if(file != NULL){
        fclose(file);
        file = NULL;

        logging = false;
        ((DataPacket*) datapacket)->packetid = 0; // reset packetid to zero
        ((DataPacket *)datapacket)->logging = 0; // update logging status
        ESP_LOGI(SDTAG, "Logging stopped.");
      }

      // keep looping every second checking for logging signal
      vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ((DataPacket*) datapacket)->packetid++; // increment packetid
    vTaskDelay(pdMS_TO_TICKS(100)); // Refresh rate for logging (100 ms)
  }

  // shouldn't ever reach this point, not sure if that's bad or not
  sd_card_unmount(card);
}