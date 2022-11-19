// This file contains functions and tasks related to the INA219 current sensor

// Standard libraries
#include <string.h>

// CardealESP config header
#include "include/config.hpp"

// FreeRTOS
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

// Current Sensor (INA219)
#include "ina219.h"
#include "include/ina219.hpp"

// DataPacket
#include "include/sdlog.hpp"

// INA219 Current Sensor task
void taskCurrent(void *params_ina219) {
  ina219_t sensor;  // device struct
  memset(&sensor, 0, sizeof(ina219_t));

  // retrieve data from parameter
  int i2c_address = ((struct params_taskINA_t *)params_ina219)->INA219_ADDR;
  DataPacket* datapacket = ((struct params_taskINA_t *)params_ina219)->data;

  ESP_ERROR_CHECK(ina219_init_desc( &sensor,
                 i2c_address, I2C_PORT,
                 SDA_GPIO, SCL_GPIO));  // if fails (invalid address), aborts execution
  ESP_LOGI(INATAG, "Initializing INA219");

  // attempt initialization 5 times
  uint8_t attempt = 1;
  while (ina219_init(&sensor) != ESP_OK && attempt <= 5) {
    ESP_LOGE(INATAG,
              "Failed to initialize 0x%x. Is the wiring connected? "
              "(attempt %d/5)",
              i2c_address, attempt);
    vTaskDelay(pdMS_TO_TICKS(5000));
    attempt++;
  }
  if (attempt > 5) {
    ESP_LOGE(INATAG, "Failed to initialize 0x%x, suspending task",
             i2c_address);
    vTaskSuspend(NULL);
  }
  // if successful, take readings
  else {
    ESP_LOGI(INATAG, "Configuring INA219");
    ESP_ERROR_CHECK_WITHOUT_ABORT(
        ina219_configure(&sensor, INA219_BUS_RANGE_16V, INA219_GAIN_0_125,
                          INA219_RES_12BIT_1S, INA219_RES_12BIT_1S,
                          INA219_MODE_CONT_SHUNT_BUS));

    float current;

    ESP_LOGI(INATAG, "Starting the loop");
    while (1) {
      ina219_get_current(&sensor, &current);
      printf("Current: %.04f mA, address: 0x%x\n", current * 1000,
             i2c_address);

    // write data to DataPacket
    datapacket->bat_amp = current * 1000;

    vTaskDelay(pdMS_TO_TICKS(100));
    }
  }
}