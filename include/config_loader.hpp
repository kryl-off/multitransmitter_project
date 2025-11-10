#pragma once
#include <vector>
#include <string>
#include "stm32f4xx_hal.h"
#include "sensor.hpp"

struct UartConfig { uint32_t baud = 115200; };

struct DeviceConfig {
  UartConfig uart;
  // глобальные дефолты для EOL
  float   eol_vref   = 3.3f;
  float   eol_rpull  = 10000.0f;
  uint16_t eol_adcmax= 4095;

  struct SensorItem {
    std::string  name;
    SensorConfig cfg;
  };
  std::vector<SensorItem> sensors;
};

bool load_config_from_json(const char* json, DeviceConfig& out);

// helpers
GPIO_TypeDef* port_from_char(char p);
uint32_t adc_channel_from_number(int ch);
SensorMode sensor_mode_from_str(const char* s);
