#pragma once

#include "include/sdlog.hpp"

#define INATAG "INA219"

struct params_taskINA_t {
  const int INA219_ADDR;
  DataPacket* data;
};

void taskCurrent(void *params_ina219);