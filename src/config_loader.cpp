#include "config_loader.hpp"
#include "cJSON.h"
#include <cstring>

/* --------- HAL utils --------- */
GPIO_TypeDef* port_from_char(char p) {
  switch (p) {
    case 'A': return GPIOA;
    case 'B': return GPIOB;
    case 'C': return GPIOC;
    case 'D': return GPIOD;
    case 'E': return GPIOE;
    default:  return nullptr;
  }
}

uint32_t adc_channel_from_number(int ch) {
  switch (ch) {
    case 0:  return ADC_CHANNEL_0;  case 1:  return ADC_CHANNEL_1;
    case 2:  return ADC_CHANNEL_2;  case 3:  return ADC_CHANNEL_3;
    case 4:  return ADC_CHANNEL_4;  case 5:  return ADC_CHANNEL_5;
    case 6:  return ADC_CHANNEL_6;  case 7:  return ADC_CHANNEL_7;
    case 8:  return ADC_CHANNEL_8;  case 9:  return ADC_CHANNEL_9;
    case 10: return ADC_CHANNEL_10; case 11: return ADC_CHANNEL_11;
    case 12: return ADC_CHANNEL_12; case 13: return ADC_CHANNEL_13;
    case 14: return ADC_CHANNEL_14; case 15: return ADC_CHANNEL_15;
    default: return ADC_CHANNEL_0;
  }
}

SensorMode sensor_mode_from_str(const char* s) {
  if (!s) return SensorMode::NO;
  if (strcmp(s, "NO")==0)  return SensorMode::NO;
  if (strcmp(s, "NC")==0)  return SensorMode::NC;
  if (strcmp(s, "EOL")==0) return SensorMode::EOL;
  return SensorMode::NO;
}

/* --------- JSON helpers (минимально) --------- */
static inline bool j_is_num(cJSON* o, const char* k) {
  cJSON* x = cJSON_GetObjectItemCaseSensitive(o, k);
  return cJSON_IsNumber(x);
}
static inline double j_num(cJSON* o, const char* k, double def=0.0) {
  cJSON* x = cJSON_GetObjectItemCaseSensitive(o, k);
  return cJSON_IsNumber(x) ? x->valuedouble : def;
}
static inline bool j_is_bool(cJSON* o, const char* k) {
  cJSON* x = cJSON_GetObjectItemCaseSensitive(o, k);
  return cJSON_IsBool(x);
}
static inline bool j_bool(cJSON* o, const char* k, bool def=false) {
  cJSON* x = cJSON_GetObjectItemCaseSensitive(o, k);
  return cJSON_IsBool(x) ? cJSON_IsTrue(x) : def;
}
static inline const char* j_str(cJSON* o, const char* k) {
  cJSON* x = cJSON_GetObjectItemCaseSensitive(o, k);
  return (cJSON_IsString(x) && x->valuestring) ? x->valuestring : nullptr;
}

/* --------- blocks --------- */
static bool parse_uart(cJSON* root, UartConfig& u) {
  cJSON* uart = cJSON_GetObjectItemCaseSensitive(root, "uart");
  if (!cJSON_IsObject(uart)) return true;
  cJSON* baud = cJSON_GetObjectItemCaseSensitive(uart, "baud");
  if (cJSON_IsNumber(baud)) u.baud = (uint32_t)baud->valuedouble;
  return true;
}

static void apply_globals(DeviceConfig& cfg, cJSON* root) {
  // Глобалки для EOL/ADC
  cJSON* globals = cJSON_GetObjectItemCaseSensitive(root, "globals");
  if (!cJSON_IsObject(globals)) return;
  cJSON* eol = cJSON_GetObjectItemCaseSensitive(globals, "eol");
  if (!cJSON_IsObject(eol)) return;
  cJSON* vref   = cJSON_GetObjectItemCaseSensitive(eol, "vref");
  cJSON* rpull  = cJSON_GetObjectItemCaseSensitive(eol, "r_pull");
  cJSON* adcmax = cJSON_GetObjectItemCaseSensitive(eol, "adc_max");
  if (cJSON_IsNumber(vref))   cfg.eol_vref   = (float)vref->valuedouble;
  if (cJSON_IsNumber(rpull))  cfg.eol_rpull  = (float)rpull->valuedouble;
  if (cJSON_IsNumber(adcmax)) cfg.eol_adcmax = (uint16_t)adcmax->valuedouble;
}

/* Заполняет общие для АЦП поля (vref/r_pull/adc_max/avg/period/debounce) + пороги.
   Возвращает true, если АЦП «включён» (нашли channel). */
static bool apply_adc_fields_if_present(SensorConfig& cfg, cJSON* it, ADC_HandleTypeDef* hadc1_ptr, const DeviceConfig& globals) {
  // есть ли канал?
  int ch = (int)j_num(it, "channel", -1);
  if (ch < 0) return false;            // канала нет — работаем по GPIO

  // есть канал → включаем АЦП-режим
  cfg.hadc        = hadc1_ptr;
  cfg.adc_channel = adc_channel_from_number(ch);

  // глобальные дефолты
  cfg.vref    = globals.eol_vref;
  cfg.r_pull  = globals.eol_rpull;
  cfg.adc_max = globals.eol_adcmax;

  // локальные переопределения (если есть)
  if (j_is_num(it, "vref"))     cfg.vref     = (float)j_num(it, "vref");
  if (j_is_num(it, "r_pull"))   cfg.r_pull   = (float)j_num(it, "r_pull");
  if (j_is_num(it, "adc_max"))  cfg.adc_max  = (uint16_t)j_num(it, "adc_max");
  if (j_is_num(it, "avg"))      cfg.avg_samples = (uint8_t)j_num(it, "avg");

  // интервалы опроса/дребезга (общие)
  if (j_is_num(it, "debounce_ms"))  cfg.debounce_ms      = (uint32_t)j_num(it, "debounce_ms");
  if (j_is_num(it, "period_ms"))    cfg.sample_period_ms = (uint32_t)j_num(it, "period_ms");

  // пороги (универсальные — работают для EOL/NO/NC)
  if (j_is_num(it, "v_alarm_max")) cfg.v_alarm_max = (float)j_num(it, "v_alarm_max");
  if (j_is_num(it, "v_fault_min")) cfg.v_fault_min = (float)j_num(it, "v_fault_min");
  if (j_is_num(it, "v_alarm_min")) cfg.v_alarm_min = (float)j_num(it, "v_alarm_min");
  if (j_is_num(it, "v_fault_max")) cfg.v_fault_max = (float)j_num(it, "v_fault_max");

  return true;
}

static bool parse_sensors(DeviceConfig& out, cJSON* root, ADC_HandleTypeDef* hadc1_ptr) {
  cJSON* arr = cJSON_GetObjectItemCaseSensitive(root, "sensors");
  if (!cJSON_IsArray(arr)) return false;

  cJSON* it = nullptr;
  cJSON_ArrayForEach(it, arr) {
    if (!cJSON_IsObject(it)) continue;

    DeviceConfig::SensorItem item;
    SensorConfig cfg{};

    // name/type
    const char* nm = j_str(it, "name");
    const char* tp = j_str(it, "type");
    if (nm) item.name = nm;
    cfg.mode = sensor_mode_from_str(tp);

    // дефолты опроса
    cfg.debounce_ms       = 30;
    cfg.sample_period_ms  = 5;

    // для всех режимов: пусть по умолчанию подставятся глобальные значения АЦП (если пригодятся)
    cfg.vref    = out.eol_vref;
    cfg.r_pull  = out.eol_rpull;
    cfg.adc_max = out.eol_adcmax;
    cfg.tol = 0.25f;
    cfg.avg_samples = 8;

    if (cfg.mode == SensorMode::NO || cfg.mode == SensorMode::NC) {
      // цифровые поля (fallback)
      const char* p = j_str(it, "port");
      if (p && p[0]) cfg.port = port_from_char(p[0]);
      if (j_is_num(it, "pin"))        cfg.pin  = (uint16_t)j_num(it, "pin");
      if (j_is_bool(it, "pullup"))    cfg.enable_pullup = j_bool(it, "pullup", cfg.enable_pullup);
      if (j_is_bool(it, "active_low"))cfg.active_low    = j_bool(it, "active_low", cfg.active_low);
      if (j_is_num(it, "debounce_ms"))cfg.debounce_ms   = (uint32_t)j_num(it, "debounce_ms");
      if (j_is_num(it, "period_ms"))  cfg.sample_period_ms = (uint32_t)j_num(it, "period_ms");

      // опционально включаем аналоговый режим, если задан channel (и т.д.)
      bool analog_enabled = apply_adc_fields_if_present(cfg, it, hadc1_ptr, out);
      // если analog_enabled == false, датчик останется цифровым (readDigital)
      // если true — будет читать по АЦП и использовать пороги, заданные выше
    }

    if (cfg.mode == SensorMode::EOL) {
      // В EOL аналог обязателен — используем globals + локальные
      cfg.hadc    = hadc1_ptr;
      cfg.debounce_ms = (uint32_t)j_num(it, "debounce_ms", 50);
      cfg.sample_period_ms = (uint32_t)j_num(it, "period_ms", 10);

      // канал обязателен для EOL
      if (j_is_num(it, "channel"))
        cfg.adc_channel = adc_channel_from_number((int)j_num(it, "channel", 0));

      // номиналы и параметры
      if (j_is_num(it, "r_eol")) cfg.r_eol = (float)j_num(it, "r_eol");
      if (j_is_num(it, "tol"))   cfg.tol   = (float)j_num(it, "tol");
      if (j_is_num(it, "avg"))   cfg.avg_samples = (uint8_t)j_num(it, "avg");
      if (j_is_num(it, "vref"))  cfg.vref  = (float)j_num(it, "vref");
      if (j_is_num(it, "r_pull"))cfg.r_pull= (float)j_num(it, "r_pull");
      if (j_is_num(it, "adc_max")) cfg.adc_max = (uint16_t)j_num(it, "adc_max");

      // универсальные пороги (если заданы, перекрывают «стандартную» eol-логику)
      if (j_is_num(it, "v_alarm_max")) cfg.v_alarm_max = (float)j_num(it, "v_alarm_max");
      if (j_is_num(it, "v_fault_min")) cfg.v_fault_min = (float)j_num(it, "v_fault_min");
      if (j_is_num(it, "v_alarm_min")) cfg.v_alarm_min = (float)j_num(it, "v_alarm_min");
      if (j_is_num(it, "v_fault_max")) cfg.v_fault_max = (float)j_num(it, "v_fault_max");
    }

    // power/reset
    const char* pwr = j_str(it, "power");
    if (pwr) {
      if (strcmp(pwr, "FIRE")==0) cfg.power = PowerGroup::FIRE;
      else cfg.power = PowerGroup::AUX;
    }
    if (j_is_num(it, "reset_ms")) cfg.reset_ms = (int32_t)j_num(it, "reset_ms");

    out.sensors.push_back({ item.name.empty() ? std::string("sensor") : item.name, cfg });
  }
  return true;
}

/* --------- entrypoint --------- */
bool load_config_from_json(const char* json, DeviceConfig& out)
{
  cJSON* root = cJSON_Parse(json);
  if (!root) return false;

  apply_globals(out, root);
  if (!parse_uart(root, out.uart)) { cJSON_Delete(root); return false; }

  extern ADC_HandleTypeDef hadc1;
  bool ok = parse_sensors(out, root, &hadc1);

  cJSON_Delete(root);
  return ok;
}
