// This file contains functions and tasks related to the MS4525DO differential pressure
// sensor used on the pitot tube

// CardealESP config header
#include "include/config.hpp"

// FreeRTOS
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "driver/i2c.h"

// DataPacket
#include "include/sdlog.hpp"

#include "include/pitot.hpp"

void taskPitot(void* pvParameters){
  // I2C initialization
	i2c_config_t conf = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = 2,
		.scl_io_num = 4,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master = {
      .clk_speed = I2C_MASTER_FREQ_HZ,
    }
	};
	i2c_param_config(I2C_NUM_1, &conf);
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_1, I2C_MODE_MASTER, 0, 0, 0));

  // Data received from sensor via I2C (4 bytes)
  uint8_t rx_data[4];

  ESP_LOGI(PITOTTAG, "I2C reading start");

  while(1){
    // send read command
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
    //i2c_master_write_byte(cmd, 0x3F, true);
    i2c_master_stop(cmd);
    if(i2c_master_cmd_begin(I2C_NUM_1, cmd, pdMS_TO_TICKS(1000)) == ESP_OK){
      ESP_LOGI(PITOTTAG, "send read command\n");
    }
    i2c_cmd_link_delete(cmd);

    // read bytes
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_SENSOR_ADDR << 1) | I2C_MASTER_READ, true);
    for(int i = 0; i < 4; i++){
      if(i == 3)
        i2c_master_read_byte(cmd, &rx_data[i], I2C_MASTER_NACK); 

      i2c_master_read_byte(cmd, &rx_data[i], I2C_MASTER_ACK);
    }
    i2c_master_stop(cmd);
    if(i2c_master_cmd_begin(I2C_NUM_1, cmd, pdMS_TO_TICKS(1000)) == ESP_OK){
      printf("read bytes\n");
    }
    i2c_cmd_link_delete(cmd);

    // Assembling pressure data
    // data comes in two unsigned 8-bit integers from rx_data
    // assemble data into unsigned 16-bit integer pres_data
    uint16_t pres_data = (uint16_t)((rx_data[0] & 0x3F) << 8 | rx_data[1]);

    // this equation is an inversion of the equation in the
    // pressure transfer function figure on page 4 of the datasheet

    // We negate the result so that positive differential pressures
    // are generated when the bottom port is used as the static
    // port on the pitot and top port is used as the dynamic port
    float pres_psi = (float)(((pres_data) - C_ * P_CNT_) * ((P_MAX_ - P_MIN_) / (D_ * P_CNT_))) + P_MIN_;

    // Convert PSI to Pascal
    float pres_pa = pres_psi * PSI_TO_PA;

    // Assembling temperature data
    uint16_t temp_data = (uint16_t)((rx_data[2] << 8) + (0b11100000 & rx_data[3])) / (1 << 5);
    float temp_c = (float)((200.f * temp_data) / 2047) - 50.f;

    printf("diff_pres_pa: %.2f\n", pres_pa);
    printf("temp: %.2f\n", temp_c);

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}