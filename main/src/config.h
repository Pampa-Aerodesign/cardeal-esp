/* Configuration file for settings and pinouts */
/* Futurely could be implemented into menuconfig for the project */

#ifndef CARDEAL_CONFIG_H_
#define CARDEAL_CONFIG_H_

#include "MPU.hpp"
#include "mpu/types.hpp"

/* Settings */

// IMU
static constexpr uint32_t CLOCK_SPEED = 200000;   // range from 100 kHz ~ 400kHz
static constexpr uint16_t MPU_SAMPLE_RATE = 200;  // range from 4 Hz ~ 1000 Hz
static constexpr mpud::accel_fs_t ACCELEROMETER_SCALE =
    mpud::ACCEL_FS_4G;  // +- 2g, 4g, 8g or 16g

// SD Card
#define MOUNT_POINT "/sdcard"
#define MAX_FREQ_SPI_SDCARD 10000 // in kHz

/* Pinouts */

// I2Cbus lib (IMU dependency)
static constexpr gpio_num_t SDA = GPIO_NUM_21;
static constexpr gpio_num_t SCL = GPIO_NUM_22;

// SPI (SD Card)
static constexpr gpio_num_t PIN_NUM_MISO = GPIO_NUM_2;
static constexpr gpio_num_t PIN_NUM_MOSI = GPIO_NUM_15;
static constexpr gpio_num_t PIN_NUM_CLK  = GPIO_NUM_14;
static constexpr gpio_num_t PIN_NUM_CS   = GPIO_NUM_13;

#endif  // CARDEAL_CONFIG_H_