/* Configuration file for settings and pinouts */
/* Futurely could be implemented into menuconfig for the project */

#ifndef CARDEAL_CONFIG_H_
#define CARDEAL_CONFIG_H_

#include "ina219.h"

/* Settings */

// SD Card
#define MOUNT_POINT "/sdcard"
#define MAX_FREQ_SPI_SDCARD 10000 // in kHz

/* Pinouts */

// I2C port 0
static constexpr gpio_num_t SDA_GPIO = GPIO_NUM_21;
static constexpr gpio_num_t SCL_GPIO = GPIO_NUM_22;
#define I2C_PORT 0
#define I2C_ADDR INA219_ADDR_GND_GND // A1 A0

// SPI (SD Card)
static constexpr gpio_num_t PIN_NUM_MISO = GPIO_NUM_2;
static constexpr gpio_num_t PIN_NUM_MOSI = GPIO_NUM_15;
static constexpr gpio_num_t PIN_NUM_CLK  = GPIO_NUM_14;
static constexpr gpio_num_t PIN_NUM_CS   = GPIO_NUM_13;

// ADC1 (for voltage measurement)
#define DEFAULT_VREF 1100 // nominal value, can be measured with adc_vref_to_gpio()

#endif  // CARDEAL_CONFIG_H_