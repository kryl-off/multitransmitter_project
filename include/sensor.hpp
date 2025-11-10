#pragma once
#include <cstdint>
#include <cmath>
#include <string>
#include "stm32f4xx_hal.h"

/* Моды шлейфа */
enum class SensorMode : uint8_t { NO, NC, EOL };

/* Состояние шлейфа */
enum class SensorState : uint8_t { NORMAL, ALARM, FAULT };

/* Группа питания (обычные / пожарные) */
enum class PowerGroup : uint8_t { AUX, FIRE };

/* Конфиг датчика */
struct SensorConfig {
  std::string name;
  SensorMode mode = SensorMode::NO;

  // Цифровой вход
  GPIO_TypeDef* port = nullptr;
  uint16_t      pin  = 0;
  bool          active_low = true;
  bool          enable_pullup = true;

  // Фильтры/опрос
  uint32_t debounce_ms      = 30;
  uint32_t sample_period_ms = 5;

  // EOL / ADC
  ADC_HandleTypeDef* hadc = nullptr;
  uint32_t adc_channel = 0;
  float   r_pull = 10000.0f; // подтяжка на плате (к 3.3В)
  float   r_eol  = 4700.0f;  // оконечник в шлейфе (типично)
  float   tol    = 0.25f;    // допуск нормы (±)
  float   vref   = 3.3f;
  uint16_t adc_max = 4095;
  uint8_t  avg_samples = 8;

  // Питание/сброс FIRE
  PowerGroup power = PowerGroup::AUX;
  int32_t    reset_ms = -1;  // -1 => не сбрасывать питание после тревоги

  // Кастомные пороги (опционально). <0 => не использовать
  float v_alarm_max = -1.0f;  // ALARM, если v < v_alarm_max    (классика «вниз»)
  float v_fault_min = -1.0f;  // FAULT, если v > v_fault_min     (классика «вверх»)
  float v_alarm_min = -1.0f;  // ALARM, если v > v_alarm_min     (твой случай «вверх»)
  float v_fault_max = -1.0f;  // FAULT, если v < v_fault_max     (твой случай «низ»)
};

class Sensor {
public:
  float lastVoltage() const { return last_v_; }

  explicit Sensor(const SensorConfig& cfg)
  : cfg_(cfg),
    last_tick_(HAL_GetTick()),
    last_sample_(0),
    edge_stamp_(0),
    stable_state_(SensorState::NORMAL),
    raw_state_(SensorState::NORMAL)
  {}


  // вызывать часто; вернёт true при изменении стабильного состояния
  bool update() {
    uint32_t now = HAL_GetTick();
    if (now - last_sample_ < cfg_.sample_period_ms) return false;
    last_sample_ = now;

    SensorState s = readOnce();

    if (s != raw_state_) {
      raw_state_ = s;
      edge_stamp_ = now;
      return false;
    }
    if ((now - edge_stamp_) >= cfg_.debounce_ms && stable_state_ != raw_state_) {
      stable_state_ = raw_state_;
      return true;
    }
    return false;
  }

  SensorState state() const { return stable_state_; }
  bool isAlarm() const { return stable_state_ == SensorState::ALARM; }
  bool isFault() const { return stable_state_ == SensorState::FAULT; }
  const SensorConfig& cfg() const { return cfg_; }

private:
  SensorState readOnce() {
    switch (cfg_.mode) {
      case SensorMode::NO:
        if (cfg_.hadc) return readNoNcAnalog(false);
        else           return readDigital();
      case SensorMode::NC:
        if (cfg_.hadc) return readNoNcAnalog(true);
        else           return readDigital();
      case SensorMode::EOL:
        return readEol();
    }
    return SensorState::FAULT;
  }


  SensorState readDigital() {
    if (!cfg_.port) return SensorState::FAULT;
    GPIO_PinState v = HAL_GPIO_ReadPin(cfg_.port, cfg_.pin);
    bool active = cfg_.active_low ? (v == GPIO_PIN_RESET) : (v == GPIO_PIN_SET);
    if (cfg_.mode == SensorMode::NO) {
      return active ? SensorState::ALARM : SensorState::NORMAL;
    } else { // NC
      return active ? SensorState::NORMAL : SensorState::ALARM;
    }
  }

  static uint16_t readAdcSingle(ADC_HandleTypeDef* hadc, uint32_t channel) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
    HAL_ADC_ConfigChannel(hadc, &sConfig);
    HAL_ADC_Start(hadc);
    HAL_ADC_PollForConversion(hadc, 10);
    uint16_t value = (uint16_t)HAL_ADC_GetValue(hadc);
    HAL_ADC_Stop(hadc);
    return value;
  }

  SensorState readEol() {
    if (!cfg_.hadc) return SensorState::FAULT;

      // усреднение
      uint32_t acc = 0;
      for (uint8_t i = 0; i < cfg_.avg_samples; i++) {
        acc += readAdcSingle(cfg_.hadc, cfg_.adc_channel);
      }
      const uint32_t raw = acc / (cfg_.avg_samples ? cfg_.avg_samples : 1);
      const float v = (cfg_.vref * raw) / cfg_.adc_max;
      last_v_ = v;

    // вычислим состояние
      SensorState out;

  // a) Классический вариант (если заданы v_alarm_max & v_fault_min)
      if (cfg_.v_alarm_max >= 0.0f && cfg_.v_fault_min >= 0.0f) {
        if (v < cfg_.v_alarm_max)      out = SensorState::ALARM;
        else if (v > cfg_.v_fault_min) out = SensorState::FAULT;
        else                           out = SensorState::NORMAL;

      // b) Твой текущий случай: ALARM-верх и FAULT-низ
      } else if (cfg_.v_alarm_min >= 0.0f && cfg_.v_fault_max >= 0.0f) {
        if (v > cfg_.v_alarm_min)      out = SensorState::ALARM;  // тревога выше нормы
        else if (v < cfg_.v_fault_max) out = SensorState::FAULT;  // «обрыв» у тебя внизу
        else                           out = SensorState::NORMAL;

      // c) Иначе — стандартная EOL-логика по номиналам
      } else {
        const float v_norm = cfg_.vref * (cfg_.r_eol / (cfg_.r_pull + cfg_.r_eol));
        const float band   = cfg_.tol * v_norm;
        const float v_low  = 0.10f * cfg_.vref;
        const float v_high = 0.90f * cfg_.vref;

        if (v <= v_low)                     out = SensorState::ALARM;
        else if (v >= v_high)               out = SensorState::FAULT;
        else if (fabsf(v - v_norm) <= band) out = SensorState::NORMAL;
        else                                 out = SensorState::FAULT;
      }
    return out; 
}

// NO/NC по АЦП: только NORMAL/ALARM, без FAULT
SensorState readNoNcAnalog(bool is_nc) {
  if (!cfg_.hadc) return SensorState::FAULT; // без АЦП не измерим

  // усреднение АЦП как в EOL
  uint32_t acc = 0;
  const uint8_t n = (cfg_.avg_samples ? cfg_.avg_samples : 1);
  for (uint8_t i = 0; i < n; ++i) acc += readAdcSingle(cfg_.hadc, cfg_.adc_channel);
  const uint32_t raw = acc / n;
  const float v = (cfg_.vref * raw) / cfg_.adc_max;
  last_v_ = v;

  SensorState out = SensorState::NORMAL;

  if (is_nc) {
    // NC: по умолчанию "ALARM вниз", но если задан v_alarm_min — разрешаем "вверх".
    if (cfg_.v_alarm_max >= 0.0f) {
      if (v < cfg_.v_alarm_max) out = SensorState::ALARM;
    } else if (cfg_.v_alarm_min >= 0.0f) {
      if (v > cfg_.v_alarm_min) out = SensorState::ALARM;
    } else {
      // фоллбек от номиналов: ниже коридора — ALARM
      const float v_norm = cfg_.vref * (cfg_.r_eol / (cfg_.r_pull + cfg_.r_eol));
      const float band   = cfg_.tol * v_norm;
      if (v < (v_norm - band)) out = SensorState::ALARM;
    }
  } else {
    // NO: твой случай — "ALARM вниз" (норма ~2.6 В, тревога ~0 В)
    if (cfg_.v_alarm_max >= 0.0f) {
      if (v < cfg_.v_alarm_max) out = SensorState::ALARM;
    } else if (cfg_.v_alarm_min >= 0.0f) {
      // альтернативно можно включить "ALARM вверх", если нужно
      if (v > cfg_.v_alarm_min) out = SensorState::ALARM;
    } else {
      // фоллбек: тоже "вниз"
      const float v_norm = cfg_.vref * (cfg_.r_eol / (cfg_.r_pull + cfg_.r_eol));
      const float band   = cfg_.tol * v_norm;
      if (v < (v_norm - band)) out = SensorState::ALARM;
    }
  }

  return out; // никогда не возвращаем FAULT для NO/NC
}



private:
  float last_v_ = 0.0f; 
  uint32_t dbg_stamp_ = 0;
  SensorConfig cfg_;
  uint32_t last_tick_;
  uint32_t last_sample_;
  uint32_t edge_stamp_ = 0;
  SensorState stable_state_;
  SensorState raw_state_;
};
