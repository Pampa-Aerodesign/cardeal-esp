// Standard libraries
#include <string.h>

// CardealESP config header
#include "include/config.h"

// FreeRTOS
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

// Temperature Sensor (BMP280)
#include "bmp280.h"
#include "include/bmp280.hpp"

#include "include/sdlog.hpp"

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