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

// LoRa communication via SX1276 chips
#include "lora.h"
/* clang-format on */

void startSDCard() {
    esp_err_t ret;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,   // won't format card if mount fails
        .max_files = 5,                    // max number of open files
        .allocation_unit_size = 16 * 1024  // 16 kb allocation unit size
    };

    sdmmc_card_t *card;
    const char mount_point[] = MOUNT_POINT;
    ESP_LOGI("SD", "Initializing SD card");
    ESP_LOGI("SD", "Using SPI peripheral");

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.max_freq_khz =
        MAX_FREQ_SPI_SDCARD;  // for some reason, the default frequency value
                              // (20 MHz) issues errors when running

    spi_bus_config_t bus_cfg = {.mosi_io_num = PIN_NUM_MOSI,
                                .miso_io_num = PIN_NUM_MISO,
                                .sclk_io_num = PIN_NUM_CLK,
                                .quadwp_io_num = -1,
                                .quadhd_io_num = -1,
                                .max_transfer_sz = 4000,
                                .flags = 0,
                                .intr_flags = 0};

    ret = spi_bus_initialize((spi_host_device_t)host.slot, &bus_cfg, 1);
    if (ret != ESP_OK) {
        ESP_LOGE("SD", "Failed to initialize bus.");
        return;
    }

    // This initializes the slot without card detect (CD -> slot_config.gpio_cd)
    // and write protect (WP -> slot_config.gpio_wp) signals
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;  // CS pin
    slot_config.host_id = (spi_host_device_t)host.slot;

    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config,
                                  &mount_config, &card);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE("SD",
                     "Failed to mount filesystem. "
                     "If you want the card to be formatted, set the "
                     "EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE("SD",
                     "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.",
                     esp_err_to_name(ret));
        }
        return;
    }

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);
}

void openfileSDCard(FILE *&f, const char *file_name) {
    char file_dir[30] = MOUNT_POINT "/";
    strcat(file_dir, file_name);
    ESP_LOGI("SD", "Opening file %s", file_dir);
    f = fopen(file_dir, "a");
    if (f == NULL) {
        ESP_LOGE("SD", "Failed to open file for writing");
        return;
    }
}

/*void writeinSDCard(FILE *&f, char * string) {
    fprintf(f, string);
    ESP_LOGI("SD", "File written");
}*/

void closefileSDCard(FILE *&f) {
    fclose(f);
    ESP_LOGI("SD", "File closed");
}

void taskCurrent(void *params) {
    ina219_t sensor;  // device struct
    memset(&sensor, 0, sizeof(ina219_t));

    ESP_ERROR_CHECK(ina219_init_desc(&sensor, I2C_ADDR, I2C_PORT, SDA_GPIO,
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
        printf("Current: %.04f mA\n", current * 1000);

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void taskVoltage(void *params) {
    VoltageSensor AileronServo;

    AileronServo.setup(ADC1_CHANNEL_0, ADC_ATTEN_DB_11, 1.0, 0.470);
    AileronServo.calibLog();

    while (true) {
        int value = AileronServo.read_mV(50);
        printf("voltage is %dmV\n", value);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void taskLoRa_tx(void *params) {
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

extern "C" void app_main(void) {
    /*startSDCard();
    FILE *f;
    openfileSDCard(f, "accel.txt");
    fprintf(f, "x(g)     y(g)     z(g)    t(us)\n");
    closefileSDCard(f);*/

    ESP_ERROR_CHECK(i2cdev_init());  // start i2cdev library, dependency for
                                     // esp-idf-lib libraries

    // xTaskCreate(&taskCurrent, "read INA219 data", configMINIMAL_STACK_SIZE*8,
    // NULL, 2, NULL); xTaskCreate(&taskVoltage, "read voltage measurement",
    // configMINIMAL_STACK_SIZE*8, NULL, 2, NULL);
    xTaskCreate(&taskLoRa_tx, "send LoRa packets", 2048, NULL, 5, NULL);
}
