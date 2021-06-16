/* Cardeal ESP

   This is an embedded system developed for Pampa Aerodesign's aircraft.

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/* clang-format off */
#include <stdio.h>

#include "src/config.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// IMU
#include "I2Cbus.hpp"
#include "MPU.hpp"
#include "mpu/math.hpp"
#include "mpu/types.hpp"

// SD Card
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
/* clang-format on */

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
                                .intr_flags = 0
    };

    ret = spi_bus_initialize((spi_host_device_t)host.slot, &bus_cfg, 1);
    if (ret != ESP_OK) {
        ESP_LOGE("SD", "Failed to initialize bus.");
        return;
    }

    // This initializes the slot without card detect (CD -> slot_config.gpio_cd) and write protect (WP -> slot_config.gpio_wp) signals
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS; // CS pin
    slot_config.host_id = (spi_host_device_t)host.slot;

    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE("SD", "Failed to mount filesystem. "
                     "If you want the card to be formatted, set the EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE("SD", "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
        return;
    }

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);
}

void openfileSDCard(FILE *&f, const char *file_name) {
    char file_dir[30] = MOUNT_POINT"/";
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

void taskAccel(void *params) {
    FILE *f;
    while (true) {
        /* Acquiring raw acceleratioon data */
        mpud::raw_axes_t accelRaw;    // holds x, y, z axes as int16
        MPU.acceleration(&accelRaw);  // fetch raw data from the registers
        long time_us = (long)esp_timer_get_time();
        ESP_LOGI("mpu", "accel-raw: %+d %+d %+d\n", accelRaw.x, accelRaw.y,
                 accelRaw.z);

        /* Converting raw data to value in [g] */
        mpud::float_axes_t accelG = mpud::accelGravity(
            accelRaw, ACCELEROMETER_SCALE);  // raw data to gravity
        ESP_LOGI("mpu", "accel: %+.2f %+.2f %+.2f\n", accelG.x, accelG.y,
                 accelG.z);

        /* Writes values to SD Card */
        openfileSDCard(f, "accel.txt");
        fprintf(f, "%+.5f,%+.5f,%+.5f,%ld\n", accelG.x, accelG.y,
                 accelG.z, time_us);
        closefileSDCard(f);

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

extern "C" void app_main(void) {
    // Initialize I2C on port 0 using I2Cbus interface
    i2c0.begin(SDA, SCL, CLOCK_SPEED);
    startMPU(&MPU);
    startSDCard();

    FILE *f;
    openfileSDCard(f, "accel.txt");
    fprintf(f, "x(g)     y(g)     z(g)    t(us)\n");
    closefileSDCard(f);

    xTaskCreate(&taskAccel, "read accelerometer data", 8*1024, NULL, 2, NULL);
}
