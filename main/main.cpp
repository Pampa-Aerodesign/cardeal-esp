/* Cardeal ESP

   This is an embedded system developed for Pampa Aerodesign's aircraft.

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/* clang-format off */
#include <stdio.h>
#include <string.h>

#include "src/config.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// SD Card
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

// Current Sensor (INA219)
#include "ina219.h"

// Voltage Measurement (ADC)
#include "src/VoltageSensor.hpp"
#include "driver/adc.h"

// Temperature Sensor (BMP280)
#include "bmp280.h"

// LoRa communication via SX1276 chips
#include "lora.h"
/* clang-format on */

struct params_taskVoltage_t {
    adc1_channel_t adc1_channel;
    adc_atten_t adc_atten_db;
    float R1, R2;  // if there's no resistors, both 0
};

void taskCurrent(void *params_i2c_address) {
    ina219_t sensor;  // device struct
    memset(&sensor, 0, sizeof(ina219_t));

    ESP_ERROR_CHECK(ina219_init_desc(&sensor, (int)params_i2c_address, I2C_PORT,
                                     SDA_GPIO,
                                     SCL_GPIO));  // if fails, aborts execution
    ESP_LOGI("INA219", "Initializing INA219");
    ESP_ERROR_CHECK(ina219_init(&sensor));

    ESP_LOGI("INA219", "Configuring INA219");
    ESP_ERROR_CHECK(ina219_configure(
        &sensor, INA219_BUS_RANGE_16V, INA219_GAIN_0_125, INA219_RES_12BIT_1S,
        INA219_RES_12BIT_1S, INA219_MODE_CONT_SHUNT_BUS));

    float current;

    ESP_LOGI("INA219", "Starting the loop");
    while (1) {
        ESP_ERROR_CHECK(ina219_get_current(&sensor, &current));
        printf("Current: %.04f mA, address: 0x%x\n", current * 1000,
               (int)params_i2c_address);

        vTaskDelay(500 / portTICK_PERIOD_MS);
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
    lora_set_tx_power(2);
    lora_set_spreading_factor(8);
    lora_set_coding_rate(5);
    lora_set_preamble_length(8);
    lora_explicit_header_mode();
    lora_set_sync_word(0x12);
    lora_disable_crc();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        lora_send_packet((uint8_t *)"cardeal-esp", sizeof("cardeal-esp"));
        printf("Packet sent...\n");
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
        vTaskDelay(5000 / portTICK_PERIOD_MS);
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
        if (bmp280_read_float(&dev, &temperature, &pressure, &humidity) !=
            ESP_OK) {
            printf("Temperature/pressure reading failed\n");
            continue;
        }

        // printing readings
        printf("Pressure: %.2f Pa, Temperature: %.2f C", pressure, temperature);
        if (bme280p)  // print humidity if available
            printf(", Humidity: %.2f\n", humidity);
        else
            printf("\n");

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

extern "C" void app_main(void) {
    // Task parameters
    static const struct params_taskVoltage_t BatteryElec = {
        ADC1_CHANNEL_0, ADC_ATTEN_DB_11, 1,
        0.47};  // BAT_ELEC (adc range: 470-7660mV)
    static const struct params_taskVoltage_t BuckBoostElec = {
        ADC1_CHANNEL_3, ADC_ATTEN_DB_11, 1,
        0.47};  // BUCK (adc range: 470-7660mV)
    static const struct params_taskVoltage_t BatteryDAQ = {
        ADC1_CHANNEL_5, ADC_ATTEN_DB_11, 0,
        0};  // BAT_ESP (adc range: 150-2450mV)
    static const struct params_taskVoltage_t StepUpDAQ = {
        ADC1_CHANNEL_6, ADC_ATTEN_DB_11, 1,
        0.47};  // STEPUP (adc range: 470-7660mV)

    // start i2cdev library, dependency for esp-idf-lib libraries
    ESP_ERROR_CHECK(i2cdev_init());

    // Tasks
    xTaskCreate(&taskCurrent, "INA219 battery", configMINIMAL_STACK_SIZE * 8,
                (void *)INA219_ADDR_GND_GND, 2, NULL);  // A1 open A0 open
    xTaskCreate(&taskCurrent, "INA219 right aileron",
                configMINIMAL_STACK_SIZE * 8, (void *)INA219_ADDR_GND_VS, 2,
                NULL);  // A1 open A0 bridged
    xTaskCreate(&taskCurrent, "INA219 right rudder",
                configMINIMAL_STACK_SIZE * 8, (void *)INA219_ADDR_VS_GND, 2,
                NULL);  // A1 bridged A0 open
    xTaskCreate(&taskCurrent, "INA219 right elevator",
                configMINIMAL_STACK_SIZE * 8, (void *)INA219_ADDR_VS_VS, 2,
                NULL);  // A1 bridged A0 bridged

    xTaskCreate(&taskBMP280, "BMP280 read pressure temp",
                configMINIMAL_STACK_SIZE * 8, NULL, 3, NULL);

    xTaskCreate(&taskVoltage, "read battery voltage",
                configMINIMAL_STACK_SIZE * 8, (void *)&BatteryElec, 2,
                NULL);  // GPIO36 (= VP)
    xTaskCreate(&taskVoltage, "read regulator voltage",
                configMINIMAL_STACK_SIZE * 8, (void *)&BuckBoostElec, 2,
                NULL);  // GPIO39 (= VN)
    xTaskCreate(&taskVoltage, "read DAQ battery voltage",
                configMINIMAL_STACK_SIZE * 8, (void *)&BatteryDAQ, 2,
                NULL);  // GPIO33
    xTaskCreate(&taskVoltage, "read DAQ regulator voltage",
                configMINIMAL_STACK_SIZE * 8, (void *)&StepUpDAQ, 2,
                NULL);  // GPIO34

    xTaskCreate(&taskLoRa_tx, "send LoRa packets", 2048, NULL, 5, NULL);
}
