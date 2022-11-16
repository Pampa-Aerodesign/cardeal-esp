/* Cardeal ESP

   This is an embedded system developed for Pampa Aerodesign's aircraft.

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/* clang-format off */
#include <stdio.h>
#include <string.h>
#include <string>

// CardealESP config header
#include "src/config.h"

// FreeRTOS
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// SD Card
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"

#include "src/sdlog.hpp"

// Current Sensor (INA219)
#include "ina219.h"

// Voltage Measurement (ADC)
#include "src/VoltageSensor.hpp"
#include "driver/adc.h"

// Temperature Sensor (BMP280)
#include "bmp280.h"

// LoRa communication via SX1276 chips
#include "lora.h"

struct params_taskVoltage_t {
  adc1_channel_t adc1_channel;
  adc_atten_t adc_atten_db;
  float R1, R2;  // if there's no resistors, both 0
};

// xQueueHandle interputQueue;

// const int blades = 1;
// static int pulses = 0;

// ISR
// static void IRAM_ATTR gpio_isr_handler(void *args) {
//     // pulse
//     if (!gpio_get_level(PIN_SWITCH)) {
//         pulses++;
//     }

//     // xQueueOverwriteFromISR(interputQueue, &pulses, NULL);
//     vTaskDelay(50 / portTICK_PERIOD_MS);
// }

// LoRa packet struct
typedef struct lora_packet{
  uint8_t packetid;
  int baro;
  float temp;
} LoraPacket;

void taskCurrent(void *params_i2c_address) {
  ina219_t sensor;  // device struct
  memset(&sensor, 0, sizeof(ina219_t));

  ESP_ERROR_CHECK(ina219_init_desc(
      &sensor, (int)params_i2c_address, I2C_PORT, SDA_GPIO,
      SCL_GPIO));  // if fails (invalid address), aborts execution
  ESP_LOGI("INA219", "Initializing INA219");

  // attempt initialization 5 times
  uint8_t attempt = 1;
  while (ina219_init(&sensor) != ESP_OK && attempt <= 5) {
    ESP_LOGE("INA219",
              "Failed to initialize 0x%x. Is the wiring connected? (attempt "
              "%d/5)",
              (int)params_i2c_address, attempt);
    vTaskDelay(pdMS_TO_TICKS(5000));
    attempt++;
  }
  if (attempt > 5) {
    ESP_LOGE("INA219", "Failed to initialize 0x%x, suspending task",
             (int)params_i2c_address);
    vTaskSuspend(NULL);
  }
  // if successful, take readings
  else {
    ESP_LOGI("INA219", "Configuring INA219");
    ESP_ERROR_CHECK_WITHOUT_ABORT(
        ina219_configure(&sensor, INA219_BUS_RANGE_16V, INA219_GAIN_0_125,
                          INA219_RES_12BIT_1S, INA219_RES_12BIT_1S,
                          INA219_MODE_CONT_SHUNT_BUS));

    float current;

    ESP_LOGI("INA219", "Starting the loop");
    while (1) {
      ina219_get_current(&sensor, &current);
      printf("Current: %.04f mA, address: 0x%x\n", current * 1000,
             (int)params_i2c_address);

      vTaskDelay(500 / portTICK_PERIOD_MS);
    }
  }
}

void taskVoltage(void *pvParameters) {
  params_taskVoltage_t *params = (params_taskVoltage_t *)pvParameters;

  VoltageSensor sensor;

  sensor.setup(params->adc1_channel, params->adc_atten_db, params->R1,
               params->R2);
  sensor.calibLog();

  while (true) {
    int value = sensor.read_mV(50);
    printf("Voltage: %d mV, multiplier: %f\n", value,
           (params->R1 + params->R2) / params->R2);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void taskLoRa_tx(void *pvParameters) {
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

void taskBMP280(void *pvParameters) {
  // setup
  ESP_LOGI("BMP280", "Setting up BMP280");
  bmp280_params_t params;
  bmp280_init_default_params(&params);
  bmp280_t dev;
  memset(&dev, 0, sizeof(bmp280_t));

  // initializing
  ESP_LOGI("BMP280", "Initializing BMP280");
  ESP_ERROR_CHECK(
      bmp280_init_desc(&dev, BMP280_I2C_ADDRESS_0, 0, SDA_GPIO, SCL_GPIO));

  // attempt initialization 5 times
  int attempt = 1;
  do {
    ESP_LOGE("BMP280", "Failed to initialize (attempt %d/5)", attempt);
    attempt++;
    vTaskDelay(pdMS_TO_TICKS(5000));
  } while (bmp280_init(&dev, &params) != ESP_OK && attempt < 5);

  // suspend task after 5 attempts
  if (attempt >= 5) {
    ESP_LOGE("BMP280", "Failed to initialize, suspending task");
    vTaskSuspend(NULL);
  }

  bool bme280p = dev.id == BME280_CHIP_ID;
  ESP_LOGI("BMP280", "Found %s", bme280p ? "BME280" : "BMP280");

  float pressure, temperature, humidity;

  // loop
  ESP_LOGI("BMP280", "Starting the loop");
  while (1) {
    // reading temp, pressure and humidity (if available)
    if(bmp280_read_float(&dev, &temperature, &pressure, &humidity) != ESP_OK){
      printf("Temperature/pressure reading failed\n");
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    // printing readings
    // printf("Pressure: %.2f Pa, Temperature: %.2f C", pressure, temperature);
    // if (bme280p)  // print humidity if available
    //   printf(", Humidity: %.2f\n", humidity);
    // else
    //   printf("\n");

    // update lora packet values
    ((DataPacket*) pvParameters)->baro = pressure;
    ((DataPacket*) pvParameters)->temp = temperature;

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void taskRPM(void *pvParameters) {
  // // int pulses = 0;
  // int rpm = 0;
  // // int pinNumber;

  // // GPIO setup
  // gpio_pad_select_gpio(PIN_SWITCH);
  // gpio_set_direction(PIN_SWITCH, GPIO_MODE_INPUT);
  // gpio_pulldown_en(PIN_SWITCH);
  // gpio_pullup_dis(PIN_SWITCH);
  // gpio_set_intr_type(PIN_SWITCH, GPIO_INTR_NEGEDGE);

  // // Creating queue
  // interputQueue = xQueueCreate(1, sizeof(int));

  // // ISR setup
  // gpio_install_isr_service(0);
  // gpio_isr_handler_add(PIN_SWITCH, gpio_isr_handler, NULL);

  // while (true) {
  //     // if (xQueueReceive(interputQueue, &(pulses), 0) == pdTRUE){
  //     rpm = pulses;  //(pulses * 60) / blades;
  //     pulses = 0;

  //     ESP_LOGI("RPM", "%d", rpm * 60);
  //     vTaskDelay(1000 / portTICK_PERIOD_MS);
  //     //}
  //     // else{
  //     //	ESP_LOGI("RPM", "Queue Failed");
  //     //	vTaskDelay(1000 / portTICK_PERIOD_MS);
  //     //}
  // }
}

void taskSD(void *datapacket){
  vTaskDelay(1500 / portTICK_PERIOD_MS);

  esp_err_t ret;
  static const char *TAG = "SD";

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
  ESP_LOGI(TAG, "Initializing SD card");

  // Use settings defined above to initialize SD card and mount FAT filesystem.
  // Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.
  // Please check its source code and implement error recovery when developing
  // production applications.

  ESP_LOGI(TAG, "Using SPI peripheral");

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
    ESP_LOGE(TAG, "Failed to initialize bus.");
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
      ESP_LOGE(TAG, "Failed to mount filesystem. "
                "If you want the card to be formatted, set the EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
    } else {
      ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
    }
    // return;
  }

  ESP_LOGI(TAG, "Filesystem mounted");
  // SD card finished initializing

  // check for "start logging" signal (aux3 on rx, GPIO 16)
  gpio_set_direction(PIN_SDLOG, GPIO_MODE_INPUT);
  gpio_pullup_en(PIN_SDLOG);

  int attempt = 1;
  std::string strattempt;
  char fname[32] = "\0";
  FILE* file = NULL;

  bool logging = false; // flag to check if it file was already created
  ((DataPacket*) datapacket)->packetid = 0; // start packetid at zero

  ESP_LOGI(TAG, "Starting SD loop");

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
          strcpy(fname, MOUNT_POINT "/" FILENAME);
          strattempt = std::to_string(attempt);
          strcat(fname, strattempt.c_str());
          strcat(fname, "." FILETYPE);
          file = fopen(fname, "r");
          attempt++;
        } while(file != NULL);

        ESP_LOGI(TAG, "Attempting to create file %s", fname);

        // open file in write mode
        if(file == NULL){
          file = fopen(fname, "w");

          // not elegant but whatever, none of this is so far
          if(file == NULL){
            ESP_LOGE(TAG, "File %s failed to open. Killing task", fname);
            vTaskSuspend(NULL);
          }

          ESP_LOGI(TAG, "File %s created", fname);
          // logWriteHeader(&file); // undefined reference ???
          fprintf(file, "PacketID,Timestamp,Baro,Temp\n");
        }

        logging = true;
        ((DataPacket*) datapacket)->packetid = 0; // start packetid at zero
        ESP_LOGI(TAG, "Created file %s, starting log", fname);
      }
      

      // write packet to SD card
      // get timestamp in miliseconds
      ((DataPacket*) datapacket)->timestamp = esp_timer_get_time()/1000;
      // logWrite(&file, (SDPacket*) sdpacket); // undefined reference ???
      fprintf(file, "%d,", ((DataPacket*) datapacket)->packetid);
      fprintf(file, "%lld,", ((DataPacket*) datapacket)->timestamp);
      fprintf(file, "%d,", ((DataPacket*) datapacket)->baro);
      fprintf(file, "%lf\n", ((DataPacket*) datapacket)->temp);

      // log data at 10 Hz (100 ms interval)
      // vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    else{
      // check if a file is open and close it
      if(file != NULL){
        fclose(file);
        file = NULL;
        ((DataPacket*) datapacket)->packetid = 0; // reset packetid to zero
        logging = false;
        ESP_LOGI(TAG, "Not logging");
      }

      vTaskDelay(pdMS_TO_TICKS(1000));
      // keep looping every second checking for logging signal
    }

    ((DataPacket*) datapacket)->packetid++; // increment packetid
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  // Close and unmount
  // All done, unmount partition and disable SDMMC or SPI peripheral
  esp_vfs_fat_sdcard_unmount(MOUNT_POINT, card);
  ESP_LOGI(TAG, "Card unmounted");
#ifdef USE_SPI_MODE
  //deinitialize the bus after all devices are removed
  spi_bus_free(hostslot);
#endif
}

extern "C" void app_main(void) {
  // Task parameters
  // static const struct params_taskVoltage_t BatteryElec = {
  //     ADC1_CHANNEL_0, ADC_ATTEN_DB_11, 1,
  //     0.47};  // BAT_ELEC (adc range: 470-7660mV)
  // static const struct params_taskVoltage_t BuckBoostElec = {
  //     ADC1_CHANNEL_3, ADC_ATTEN_DB_11, 1,
  //     0.47};  // BUCK (adc range: 470-7660mV)
  // static const struct params_taskVoltage_t BatteryDAQ = {
  //     ADC1_CHANNEL_5, ADC_ATTEN_DB_11, 0,
  //     0};  // BAT_ESP (adc range: 150-2450mV)
  // static const struct params_taskVoltage_t StepUpDAQ = {
  //     ADC1_CHANNEL_6, ADC_ATTEN_DB_11, 1,
  //     0.47};  // STEPUP (adc range: 470-7660mV)

  // Packet to be logged into SD card
  DataPacket datapacket;
  datapacket.packetid = 0;

  // start i2cdev library, dependency for esp-idf-lib libraries
  ESP_ERROR_CHECK(i2cdev_init());

  //
  // Tasks --------------------------------------------------------------------

  // INA219 current measuring tasks
  // xTaskCreate(&taskCurrent, "INA219 battery", configMINIMAL_STACK_SIZE * 8,
  //             (void *)INA219_ADDR_GND_GND, 2, NULL);  // A1 open A0 open
  // xTaskCreate(&taskCurrent, "INA219 right aileron",
  //             configMINIMAL_STACK_SIZE * 8, (void *)INA219_ADDR_GND_VS, 2,
  //             NULL);  // A1 open A0 bridged
  // xTaskCreate(&taskCurrent, "INA219 right rudder",
  //             configMINIMAL_STACK_SIZE * 8, (void *)INA219_ADDR_VS_GND, 2,
  //             NULL);  // A1 bridged A0 open
  // xTaskCreate(&taskCurrent, "INA219 right elevator",
  //             configMINIMAL_STACK_SIZE * 8, (void *)INA219_ADDR_VS_VS, 2,
  //             NULL);  // A1 bridged A0 bridged

  // Voltage measuring tasks
  // xTaskCreate(&taskVoltage, "read battery voltage",
  //             configMINIMAL_STACK_SIZE * 8, (void *)&BatteryElec, 2,
  //             NULL);  // GPIO36 (= VP)
  // xTaskCreate(&taskVoltage, "read regulator voltage",
  //             configMINIMAL_STACK_SIZE * 8, (void *)&BuckBoostElec, 2,
  //             NULL);  // GPIO39 (= VN)
  // xTaskCreate(&taskVoltage, "read DAQ battery voltage",
  //             configMINIMAL_STACK_SIZE * 8, (void *)&BatteryDAQ, 2,
  //             NULL);  // GPIO33
  // xTaskCreate(&taskVoltage, "read DAQ regulator voltage",
  //             configMINIMAL_STACK_SIZE * 8, (void *)&StepUpDAQ, 2,
  //             NULL);  // GPIO34

  // Wheel RPM hall effect sensor task
  // xTaskCreate(&taskRPM, "wheel rpm", configMINIMAL_STACK_SIZE * 8, NULL, 1,
  //             NULL);

  // BMP280 task (baro, temp)
  xTaskCreate(&taskBMP280, "BMP280 read pressure temp",
              configMINIMAL_STACK_SIZE * 8, (void *)&datapacket, 2, NULL);

  // SD logging task
  xTaskCreatePinnedToCore(&taskSD, "write SD log", 8192, (void *)&datapacket, 3, NULL, 1);

  // LoRa telemetry task
  xTaskCreate(&taskLoRa_tx, "send LoRa packets", 2048, (void *)&datapacket, 4, NULL);
}
