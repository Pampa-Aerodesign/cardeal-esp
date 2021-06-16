#include "IMUaccel.hpp"

/* Code for working with 'GY-91' IMU's accelerometer, InvenSense MPU9255 */

#include "I2Cbus.hpp"
#include "MPU.hpp"
#include "freertos/task.h"

void startMPU(MPU_t &MPU) {
    MPU.setBus(i2c0);  // sets MPU bus to i2c0
    MPU.setAddr(
        mpud::MPU_I2CADDRESS_AD0_LOW);  // sets MPU i2c slave address to 0x68

    // Tries to connect to the MPU repeatedly
    while (esp_err_t err = MPU.testConnection()) {
        ESP_LOGE("mpu", "Failed to connect to the MPU, error=%#X", err);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    MPU.initialize();  // initializes chip
    MPU.setSampleRate(
        MPU_SAMPLE_RATE);  // update rate of sensor register (not the esp!)
    MPU.setAccelFullScale(ACCELEROMETER_SCALE);
    // MPU.setDigitalLowPassFilter(mpud::DLPF_42HZ)
}