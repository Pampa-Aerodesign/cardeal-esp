/* Configuration file for settings and pinouts */
/* Futurely could be implemented into menuconfig for the project */

#ifndef CARDEAL_CONFIG_H_
#define CARDEAL_CONFIG_H_

#include "ina219.h"

/* Settings */

// SD Card
#define USE_SPI_MODE
#define CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED
#define MOUNT_POINT "/sdcard"
#define MAX_FREQ_SPI_SDCARD 10000  // in kHz
// DMA channel to be used by the SPI peripheral
#define SPI_DMA_CHAN    1

/* Pinouts */

// I2C port 0
static constexpr gpio_num_t SDA_GPIO = GPIO_NUM_21;
static constexpr gpio_num_t SCL_GPIO = GPIO_NUM_22;
#define I2C_PORT 0

// SPI (SD Card)
static const gpio_num_t PIN_NUM_MISO = GPIO_NUM_19;
static const gpio_num_t PIN_NUM_MOSI = GPIO_NUM_23;
static const gpio_num_t PIN_NUM_CLK = GPIO_NUM_18;
static const gpio_num_t PIN_NUM_CS = GPIO_NUM_5;
// Start Logging input
static const gpio_num_t PIN_SDLOG = GPIO_NUM_16;


// ADC1 (for voltage measurement)
#define DEFAULT_VREF \
    1100  // nominal value, can be measured with adc_vref_to_gpio()

// LoRa pins (configurable via 'idf.py menuconfig')
//
// default: CS=15, RST=32, MISO=13, MOSI=12, SCK=14

#endif  // CARDEAL_CONFIG_H_