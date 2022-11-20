// This file contains functions and tasks related to the BMP280 sensor
// built in the GY-91 IMU module

// Standard libraries
#include <string.h>

// CardealESP config header
#include "include/config.hpp"

// FreeRTOS
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

// Temperature Sensor (BMP280)
#include "bmp280.h"
#include "include/bmp280.hpp"

// DataPacket
#include "include/sdlog.hpp"

// BMP280 Baro and Temp task
void taskBMP280(void *pvParameters) {
  // setup
  ESP_LOGI(BMPTAG, "Setting up BMP280");
  bmp280_params_t params;
  bmp280_init_default_params(&params);
  bmp280_t dev;
  memset(&dev, 0, sizeof(bmp280_t));

  // initializing
  ESP_LOGI(BMPTAG, "Initializing BMP280");
  ESP_ERROR_CHECK(
      bmp280_init_desc(&dev, BMP280_I2C_ADDRESS_0, I2C_PORT, SDA_GPIO, SCL_GPIO));

  // attempt initialization 5 times
  int attempt = 1;
  do {
    ESP_LOGE(BMPTAG, "Failed to initialize (attempt %d/5)", attempt);
    attempt++;
    vTaskDelay(pdMS_TO_TICKS(5000));
  } while (bmp280_init(&dev, &params) != ESP_OK && attempt < 5);

  // suspend task after 5 attempts
  if (attempt >= 5) {
    ESP_LOGE(BMPTAG, "Failed to initialize, suspending task");
    vTaskSuspend(NULL);
  }

  bool bme280p = dev.id == BME280_CHIP_ID;
  ESP_LOGI(BMPTAG, "Found %s", bme280p ? "BME280" : "BMP280");

  float pressure, temperature, humidity;

  // loop
  ESP_LOGI(BMPTAG, "Starting the loop");
  while (1) {
    // reading temp, pressure and humidity (if available)
    if(bmp280_read_float(&dev, &temperature, &pressure, &humidity) != ESP_OK){
      printf("Temperature/pressure reading failed\n");
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    // update DataPacket
    ((DataPacket*) pvParameters)->baro = pressure;
    ((DataPacket*) pvParameters)->temp = temperature;

    // printing readings
    // printf("Pressure: %.2f Pa, Temperature: %.2f C", pressure, temperature);
    // if (bme280p)  // print humidity if available
    //   printf(", Humidity: %.2f\n", humidity);
    // else
    //   printf("\n");

    // set EventGroup bit
    xEventGroupSetBits(*(((DataPacket*)pvParameters)->eventWrite), EG_BMP280_BIT);
    // what the actual f*
    // cast pvParameters back into DataPacket pointer
    // dereference with -> to get eventWrite, which is an address to eventHandle
    // dereference that to get the eventHandle
    // probably works by just passing the handle itself? maybe, i dont know, im tired

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}