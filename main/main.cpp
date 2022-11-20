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
#include "include/config.hpp"

// FreeRTOS
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// SD Card (CATALEX MicroSD module)
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "include/sdlog.hpp"

// Current Sensor (INA219)
#include "ina219.h"
#include "include/ina219.hpp"

// Voltage Measurement (ADC)
#include "driver/adc.h"
#include "include/voltage.hpp"

// Temperature Sensor (BMP280)
#include "bmp280.h"
#include "include/bmp280.hpp"

// LoRa communication (SX1276)
#include "lora.h"
#include "include/telemetry.hpp"

extern "C" void app_main(void) {
  ESP_LOGI("MAIN", "Starting CardealESP");
  
  // Task parameters ----------------------------------------------------------
  // start i2cdev library, dependency for esp-idf-lib libraries
  ESP_ERROR_CHECK(i2cdev_init());

  // Packet to be logged into SD card
  DataPacket datapacket;
  datapacket.packetid = 0;

  // INA219
  // INA_BATTERY
  static const struct params_taskINA_t INA_BAT = {
    INA219_ADDR_VS_VS, &datapacket};

  //INA_ELEVATOR
  static const struct params_taskINA_t INA_ELEV = {
    INA219_ADDR_VS_GND, &datapacket};

  //INA_AILERON
  static const struct params_taskINA_t INA_AIL = {
    INA219_ADDR_GND_VS, &datapacket};

  //INA_RUDDER
  static const struct params_taskINA_t INA_RUD = {
    INA219_ADDR_GND_GND, &datapacket};

  // Voltage ADC
  // BAT_ELEC (adc range: 470-7660mV)
  static const struct params_taskVoltage_t BatteryElec = {
    ADC1_CHANNEL_0, ADC_ATTEN_DB_11, 1, 0.47};
    
  // BUCK (adc range: 470-7660mV)
  static const struct params_taskVoltage_t BuckBoostElec = {
    ADC1_CHANNEL_3, ADC_ATTEN_DB_11, 1, 0.47};

  // BAT_ESP (adc range: 150-2450mV)
  static const struct params_taskVoltage_t BatteryDAQ = {
    ADC1_CHANNEL_6, ADC_ATTEN_DB_11, 0, 0};

  // STEPUP (adc range: 470-7660mV)
  static const struct params_taskVoltage_t StepUpDAQ = {
    ADC1_CHANNEL_7, ADC_ATTEN_DB_11, 1, 0.47};


  // Tasks --------------------------------------------------------------------
  // INA219 current measuring tasks
  xTaskCreate(&taskCurrent, "INA219 battery", configMINIMAL_STACK_SIZE * 8,
             (void *)&INA_BAT, 2, NULL);  // A1 bridged A0 bridged
  xTaskCreate(&taskCurrent, "INA219 elevator", configMINIMAL_STACK_SIZE * 8,
             (void *)&INA_ELEV, 2, NULL);  // A1 open A0 open
  xTaskCreate(&taskCurrent, "INA219 aileron", configMINIMAL_STACK_SIZE * 8,
             (void *)&INA_AIL, 2, NULL);  // A1 open A0 bridged
  xTaskCreate(&taskCurrent, "INA219 rudder", configMINIMAL_STACK_SIZE * 8,
             (void *)&INA_RUD, 2, NULL);  // A1 bridged A0 open

  // Voltage measuring tasks (disabled due to sharing pins with LoRa)
  xTaskCreate(&taskVoltage, "read battery voltage",        // GPIO36 (= VP)
              configMINIMAL_STACK_SIZE * 8, (void *)&BatteryElec, 2, NULL);
  xTaskCreate(&taskVoltage, "read regulator voltage",      // GPIO39 (= VN)
              configMINIMAL_STACK_SIZE * 8, (void *)&BuckBoostElec, 2, NULL);
  xTaskCreate(&taskVoltage, "read DAQ battery voltage",    // GPIO33
              configMINIMAL_STACK_SIZE * 8, (void *)&BatteryDAQ, 2, NULL);
  xTaskCreate(&taskVoltage, "read DAQ regulator voltage",  // GPIO34
              configMINIMAL_STACK_SIZE * 8, (void *)&StepUpDAQ, 2, NULL);

  // BMP280 task (baro, temp)
  xTaskCreate(&taskBMP280, "BMP280 read", configMINIMAL_STACK_SIZE * 8,
             (void *)&datapacket, 2, NULL);

  // SD logging task
  xTaskCreate(&taskSD, "write SD log", 4096, (void *)&datapacket, 3, NULL);

  // LoRa telemetry task
  xTaskCreate(&taskLoRa_tx, "send LoRa packets", 2048, (void *)&datapacket, 4, NULL);
}
