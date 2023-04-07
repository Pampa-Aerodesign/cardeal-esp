#pragma once

// great part of this code has been taken from PX4 Autopilot GitHub, more specifically
// the differential pressure airspeed sensor driver:
// https://github.com/PX4/PX4-Autopilot/blob/main/src/drivers/differential_pressure/ms4525do/MS4525DO.cpp

#include "include/sdlog.hpp"

#define PITOTTAG "INA219"

#define I2C_SENSOR_ADDR	0x28
#define I2C_MASTER_FREQ_HZ 100000

// Output is proportional to the difference between Port 1 and Port 2. Output swings
// positive when Port 1> Port 2. Output is 50% of supply voltage when Port 1=Port 2.

// Calculate differential pressure. As its centered around 8000
// and can go positive or negative
static constexpr float P_MIN_ = -1.f; // -1 PSI
static constexpr float P_MAX_ = 1.f;  // +1 PSI

// this equation is an inversion of the equation in the
// pressure transfer function figure on page 4 of the datasheet

// We negate the result so that positive differential pressures
// are generated when the bottom port is used as the static
// port on the pitot and top port is used as the dynamic port
const float C_ = 0.1f;
const float D_ = 0.8f;
const float P_CNT_ = 16383.f;
const float PSI_TO_PA = 6894.757f;

// Temperature constants
const float T_MAX_ = 150;
const float T_MIN_ = -50;

void taskPitot(void* pvParameters);