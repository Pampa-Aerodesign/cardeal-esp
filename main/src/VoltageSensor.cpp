// CardealESP config header
#include "include/config.h"

#include "include/VoltageSensor.hpp"

#include "driver/adc.h"
#include "esp_adc_cal.h"

void VoltageSensor::setup(adc1_channel_t ADC1_CHANNEL_num,
                          adc_atten_t ADC_ATTEN_DB_num, float R1 /*= 0*/,
                          float R2 /*= 0*/) {
    m_channel = ADC1_CHANNEL_num;
    m_atten = ADC_ATTEN_DB_num;
    m_R1 = R1;
    m_R2 = R2;

    adc1_config_width(m_resolution);
    adc1_config_channel_atten(m_channel, m_atten);
}

esp_adc_cal_value_t VoltageSensor::calib() {
    m_calib_chars = (esp_adc_cal_characteristics_t*)calloc(
        1, sizeof(esp_adc_cal_characteristics_t));

    return esp_adc_cal_characterize(m_unit, m_atten, m_resolution, DEFAULT_VREF,
                                    m_calib_chars);
}

void VoltageSensor::calibLog() {
    esp_adc_cal_value_t val_type = this->calib();

    if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("eFuse Vref\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Two Point\n");
    } else {
        printf("Default\n");
    }
}

int VoltageSensor::read_mV(int number_of_samples /*= 1*/) {
    int adc_reading = 0;
    int measured_mV = 0;
    // Multisampling
    for (int i = 0; i < number_of_samples; i++) {
        adc_reading += adc1_get_raw(m_channel);
    }
    adc_reading /= number_of_samples;

    // Convert value to mV
    measured_mV = esp_adc_cal_raw_to_voltage(adc_reading, m_calib_chars);

    // Apply voltage divider scaling (if configured)
    if (m_R1 && m_R2) {
        return (int)((float)measured_mV * (m_R1 + m_R2) / m_R2);
    } else {
        return measured_mV;
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