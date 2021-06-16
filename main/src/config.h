/* Configuration file for settings and pinouts */
/* Futurely could be implemented into menuconfig for the project */

#ifndef CARDEAL_CONFIG_H_
#define CARDEAL_CONFIG_H_

/* Settings */

// IMU
static constexpr uint32_t CLOCK_SPEED = 400000;   // range from 100 KHz ~ 400Hz
static constexpr uint16_t MPU_SAMPLE_RATE = 100;  // range from 4 Hz ~ 1000 Hz
static constexpr mpud::accel_fs_t ACCELEROMETER_SCALE =
    mpud::ACCEL_FS_4G;  // +- 2g, 4g, 8g or 16g

/* Pinouts */

// I2Cbus lib (IMU dependency)
static constexpr gpio_num_t SDA = GPIO_NUM_21;
static constexpr gpio_num_t SCL = GPIO_NUM_22;

#endif  // CARDEAL_CONFIG_H_