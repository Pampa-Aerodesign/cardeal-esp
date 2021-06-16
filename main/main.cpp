/* Cardeal ESP

   This is an embedded system developed for Pampa Aerodesign's aircraft.

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>

#include "I2Cbus.hpp"
#include "MPU.hpp"
#include "mpu/math.hpp"
#include "mpu/types.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "src/config.h"

MPU_t MPU;  // MPU object

void startMPU(MPU_t *MPU) {
    MPU->setBus(i2c0);  // sets MPU bus to i2c0
    MPU->setAddr(
        mpud::MPU_I2CADDRESS_AD0_LOW);  // sets MPU i2c slave address to 0x68

    // Tries to connect to the MPU repeatedly
    while (esp_err_t err = MPU->testConnection()) {
        ESP_LOGE("mpu", "Failed to connect to the MPU, error=%#X", err);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    MPU->initialize();  // initializes chip
    MPU->setSampleRate(
        MPU_SAMPLE_RATE);  // update rate of sensor register (not the esp!)
    MPU->setAccelFullScale(ACCELEROMETER_SCALE);
    // MPU.setDigitalLowPassFilter(mpud::DLPF_42HZ)
}

void taskAccel(void *params) {
    while (true) {
        /* Acquiring raw acceleratioon data */
        mpud::raw_axes_t accelRaw;    // holds x, y, z axes as int16
        MPU.acceleration(&accelRaw);  // fetch raw data from the registers
        ESP_LOGI("mpu", "accel-raw: %+d %+d %+d\n", accelRaw.x, accelRaw.y,
                 accelRaw.z);

        /* Converting raw data to value in [g] */
        mpud::float_axes_t accelG = mpud::accelGravity(
            accelRaw, ACCELEROMETER_SCALE);  // raw data to gravity
        ESP_LOGI("mpu", "accel: %+.2f %+.2f %+.2f\n", accelG.x, accelG.y,
                 accelG.z);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

extern "C" void app_main(void) {
    // Initialize I2C on port 0 using I2Cbus interface
    i2c0.begin(SDA, SCL, CLOCK_SPEED);
    startMPU(&MPU);

    xTaskCreate(&taskAccel, "read accelerometer data", 2048, NULL, 2, NULL);
}
