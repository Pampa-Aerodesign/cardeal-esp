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
#include "include/config.h"

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

#include "include/sdlog.hpp"

// Current Sensor (INA219)
#include "ina219.h"
#include "include/taskina.hpp"

// Voltage Measurement (ADC)
#include "include/VoltageSensor.hpp"
#include "driver/adc.h"

// Temperature Sensor (BMP280)
#include "bmp280.h"
#include "include/taskbmp.hpp"

// LoRa communication via SX1276 chips
#include "lora.h"
#include "include/telemetry.hpp"

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
  xTaskCreate(&taskCurrent, "INA219 battery", configMINIMAL_STACK_SIZE * 8,
             (void *)INA219_ADDR_GND_GND, 2, NULL);  // A1 open A0 open
  xTaskCreate(&taskCurrent, "INA219 aileron", configMINIMAL_STACK_SIZE * 8,
             (void *)INA219_ADDR_GND_VS, 2, NULL);  // A1 open A0 bridged
  xTaskCreate(&taskCurrent, "INA219 rudder", configMINIMAL_STACK_SIZE * 8,
             (void *)INA219_ADDR_VS_GND, 2, NULL);  // A1 bridged A0 open
  xTaskCreate(&taskCurrent, "INA219 elevator", configMINIMAL_STACK_SIZE * 8,
             (void *)INA219_ADDR_VS_VS, 2, NULL);  // A1 bridged A0 bridged

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
  xTaskCreate(&taskBMP280, "BMP280 read", configMINIMAL_STACK_SIZE * 8,
             (void *)&datapacket, 2, NULL);

  // SD logging task
  xTaskCreatePinnedToCore(&taskSD, "write SD log", 8192, (void *)&datapacket, 3, NULL, 1);

  // LoRa telemetry task
  xTaskCreate(&taskLoRa_tx, "send LoRa packets", 2048, (void *)&datapacket, 4, NULL);
}
