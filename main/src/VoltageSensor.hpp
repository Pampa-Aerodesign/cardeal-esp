#ifndef CARDEAL_VOLTAGE_H_  // include guard
#define CARDEAL_VOLTAGE_H_

#include "driver/adc.h"
#include "esp_adc_cal.h"

class VoltageSensor {
   private:
    adc_unit_t m_unit = ADC_UNIT_1;
    adc_bits_width_t m_resolution = ADC_WIDTH_BIT_12;
    adc1_channel_t m_channel;
    adc_atten_t m_atten;

    esp_adc_cal_characteristics_t
        *m_calib_chars;  // calibration characteristics

    float m_R1, m_R2;  // in kOhms

    /*  o V_real
        |
        Z R1
        |
        o V_measured ---> ESP32
        |
        Z R2
        |
        o GND
    */

    /* To minimize noise, users may connect a 0.1 µF capacitor to the ADC input
     * pad in use. (Espressif Docs) */

   public:
    void setup(adc1_channel_t ADC1_CHANNEL_num, adc_atten_t ADC_ATTEN_DB_num,
               float R1 = 0, float R2 = 0);
    esp_adc_cal_value_t calib();
    void calibLog();
    int read_mV(int number_of_samples = 1);
};

#endif  // CARDEAL_VOLTAGE_H_
