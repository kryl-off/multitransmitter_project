#include <cstdarg>
#include <cstdio>
#include <vector>
#include <string>
#include "main.h"
#include "sensor.hpp"
#include "config_loader.hpp"

/* HAL handles */
ADC_HandleTypeDef   hadc1;
UART_HandleTypeDef  huart2;

/* --- FIRE power line control (shared line for all FIRE sensors) --- */
#define FIRE_PWR_GPIO      GPIOA
#define FIRE_PWR_PIN       GPIO_PIN_8
#define FIRE_PWR_ACTIVE_ON 1  // 1 если лог.1 включает питание

static inline void fire_power_write(bool on) {
#if FIRE_PWR_ACTIVE_ON
  HAL_GPIO_WritePin(FIRE_PWR_GPIO, FIRE_PWR_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
  HAL_GPIO_WritePin(FIRE_PWR_GPIO, FIRE_PWR_PIN, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
#endif
}

struct FirePowerCtrl {
  bool powered = true;
  bool in_reset = false;
  uint32_t reset_deadline = 0;
  uint32_t min_gap_ms = 1000;
  uint32_t last_reset_done = 0;

  void init_gpio() {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef io = {0};
    io.Pin = FIRE_PWR_PIN;
    io.Mode = GPIO_MODE_OUTPUT_PP;
    io.Pull = GPIO_NOPULL;
    io.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(FIRE_PWR_GPIO, &io);
    fire_power_write(true);
    powered = true;
  }
  void request_reset(uint32_t duration_ms) {
    uint32_t now = HAL_GetTick();
    if (in_reset) return;
    if (now - last_reset_done < min_gap_ms) return;
    fire_power_write(false);
    powered = false;
    in_reset = true;
    reset_deadline = now + duration_ms;
  }
  void update() {
    if (!in_reset) return;
    uint32_t now = HAL_GetTick();
    if ((int32_t)(now - reset_deadline) >= 0) {
      fire_power_write(true);
      powered = true;
      in_reset = false;
      last_reset_done = now;
    }
  }
} firePwr;

/* ---- UART printf ---- */
static void uart_printf(const char* fmt, ...) {
  char buf[256];
  va_list args;
  va_start(args, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  if (n < 0) return;
  if (n >= (int)sizeof(buf)) n = (int)sizeof(buf) - 1;  // << было sizeof(buf)
  HAL_UART_Transmit(&huart2, (uint8_t*)buf, (uint16_t)n, 50);
}
static const char* stateToStr(SensorState s) {
  switch (s) {
    case SensorState::NORMAL: return "NORMAL";
    case SensorState::ALARM:  return "ALARM";
    case SensorState::FAULT:  return "FAULT";
  }
  return "UNKNOWN";
}

/* ---------- HELPER: map ADC channel -> pin ---------- */
static bool adc_channel_to_gpio(uint32_t ch, GPIO_TypeDef** port, uint16_t* pin) {
  switch (ch) {
    case ADC_CHANNEL_0: *port = GPIOA; *pin = GPIO_PIN_0; return true;
    case ADC_CHANNEL_1: *port = GPIOA; *pin = GPIO_PIN_1; return true;
    case ADC_CHANNEL_3: *port = GPIOA; *pin = GPIO_PIN_4; return true;
    case ADC_CHANNEL_4: *port = GPIOB; *pin = GPIO_PIN_0; return true;
    case ADC_CHANNEL_5: *port = GPIOC; *pin = GPIO_PIN_0; return true;
    case ADC_CHANNEL_6: *port = GPIOC; *pin = GPIO_PIN_1; return true;
    default: return false;
  }
}

/* ---------- APPLY GPIO CONFIG FROM JSON ---------- */
/* ---------- APPLY GPIO CONFIG FROM JSON (полностью) ---------- */
static void ApplyGpioFromConfig(const DeviceConfig& dev)
{
  auto enable_port = [](GPIO_TypeDef* p){
    if (p == GPIOA)      __HAL_RCC_GPIOA_CLK_ENABLE();
    else if (p == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
    else if (p == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();
    else if (p == GPIOD) __HAL_RCC_GPIOD_CLK_ENABLE();
    else if (p == GPIOE) __HAL_RCC_GPIOE_CLK_ENABLE();
  };

  GPIO_InitTypeDef io = {0};

  for (const auto& s : dev.sensors) {
    const auto& c = s.cfg;

    // --- АНАЛОГОВЫЕ: EOL ИЛИ NO/NC С АЦП ---
    if (c.hadc) {
      GPIO_TypeDef* aport = nullptr; 
      uint16_t      apin  = 0;
      if (adc_channel_to_gpio(c.adc_channel, &aport, &apin)) {
        enable_port(aport);
        io.Pin   = apin;
        io.Mode  = GPIO_MODE_ANALOG;
        io.Pull  = GPIO_NOPULL;           // важно: без внутренних подтяжек!
        io.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(aport, &io);
      }
      // не продолжаем цифровой настройкой того же датчика
      continue;
    }

    // --- ЦИФРОВЫЕ: NO/NC БЕЗ АЦП ---
    if (c.port) {
      enable_port(c.port);
      io.Pin   = c.pin;
      io.Mode  = GPIO_MODE_INPUT;
      io.Pull  = c.enable_pullup ? GPIO_PULLUP : GPIO_NOPULL;
      io.Speed = GPIO_SPEED_FREQ_LOW;
      HAL_GPIO_Init(c.port, &io);
    }
  }
}


/* ====== Clock 84 MHz ====== */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|
                                RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ====== GPIO ====== */
static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitTypeDef io = {0};

  // LED PA5
  io.Pin = GPIO_PIN_5; io.Mode = GPIO_MODE_OUTPUT_PP; io.Pull = GPIO_NOPULL; io.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &io);

  // Примеры цифровых входов (если используешь NO/NC на PB3/PB4):
  io.Pin = GPIO_PIN_3 | GPIO_PIN_4;
  io.Mode = GPIO_MODE_INPUT;
  io.Pull = GPIO_PULLUP; // шлейф к GND
  HAL_GPIO_Init(GPIOB, &io);

  // FIRE_PWR пин настраивается в firePwr.init_gpio()
}


static void MX_ADC1_Init(void)
{
  __HAL_RCC_ADC1_CLK_ENABLE();

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();
}

/* ====== USART2 (PA2/PA3, 115200 8N1) ====== */
static void MX_USART2_UART_Init(void)
{
  __HAL_RCC_USART2_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitTypeDef io = {0};
  io.Pin = GPIO_PIN_2 | GPIO_PIN_3;  // TX/RX
  io.Mode = GPIO_MODE_AF_PP;
  io.Pull = GPIO_NOPULL;
  io.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  io.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &io);

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

void Error_Handler(void)
{
  while (1) {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    HAL_Delay(100);
  }
}


static const char* kJsonConfig = R"JSON(
{
  "uart": { "baud": 115200 },
  "globals": {
    "eol": { "vref": 3.3, "r_pull": 10000, "adc_max": 4095 }
  },
  "sensors": [
    {
      "name": "smoke1",
      "type": "EOL",
      "channel": 0,
      "debounce_ms": 120,
      "power": "FIRE",
      "reset_ms": 5000,
      "v_alarm_max": 1.20,
      "v_fault_min": 2.35
    },
    {
      "name": "button1",
      "type": "NO",
      "channel": 1,
      "debounce_ms": 120,
      "reset_ms": -1,
      "v_alarm_max": 1.20
    }
  ]
}
)JSON";


extern "C" int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();

  firePwr.init_gpio();

  uart_printf("\r\n=== start ===\r\n");

  // Load config
  DeviceConfig cfg;
  if (!load_config_from_json(kJsonConfig, cfg)) {
    uart_printf("CONFIG ERROR: bad JSON\r\n");
    Error_Handler();
  }

  ApplyGpioFromConfig(cfg);

  // Build sensors
  std::vector<Sensor> sensors;
  std::vector<std::string> names;
  sensors.reserve(cfg.sensors.size());
  names.reserve(cfg.sensors.size());
  for (auto& it : cfg.sensors) {
    it.cfg.name = it.name;
    sensors.emplace_back(it.cfg);
    names.push_back(it.name);
  } 

  
  

  // Track previous states for "front" detection
  std::vector<SensorState> prev_states(sensors.size());
  for (size_t i=0;i<sensors.size();++i) prev_states[i] = sensors[i].state();

  // Print initial
  uart_printf("INIT:");
  for (size_t i=0;i<sensors.size();++i)
    uart_printf(" %s=%s", names[i].c_str(), stateToStr(prev_states[i]));
  uart_printf("\r\n");

  uint32_t lastBlink=0, lastPing=0; bool led=false;

  while (1)
  {
    bool anyAlarm=false, anyFault=false;
    uint32_t now = HAL_GetTick();

    for (size_t i=0;i<sensors.size();++i) {
      SensorState before = prev_states[i];
      bool changed = sensors[i].update();
      SensorState cur = sensors[i].state();
      if (changed) {
        prev_states[i] = cur;
        uart_printf("EVENT: sensor='%s' state=%s\r\n", names[i].c_str(), stateToStr(cur));

        // FIRE power reset on ALARM front
        const auto& sc = sensors[i].cfg();
        if (sc.power == PowerGroup::FIRE && sc.reset_ms >= 0) {
          if (before != SensorState::ALARM && cur == SensorState::ALARM) {
            firePwr.request_reset((uint32_t)sc.reset_ms);
          }
        }
      }
      if (cur == SensorState::ALARM) anyAlarm = true;
      if (cur == SensorState::FAULT) anyFault = true;
    }

    // Ping once per second
    if (now - lastPing >= 1000) {
      lastPing = now;

      // --- первая строка: сводка состояний ---
      uart_printf("PING t=%lu", (unsigned long)now);
      for (size_t i=0;i<sensors.size();++i)
        uart_printf(" | %s=%s", names[i].c_str(), stateToStr(sensors[i].state()));
      uart_printf("\r\n");

      // --- следующие строки: подробности по EOL ---
      for (size_t i=0;i<sensors.size();++i) {
        const auto& sc = sensors[i].cfg();
        

        int v_mv   = (int)(sensors[i].lastVoltage() * 1000.0f);
        int a_mv   = (int)(sc.v_alarm_max * 1000.0f);
        int f_mv   = (int)(sc.v_fault_min * 1000.0f);

      uart_printf("%-8s: raw=%4u v=%4dmV | alm<%4dmV flt>%4dmV | state=%d\r\n",
                  sc.name.c_str(),
                  (unsigned)(v_mv * sc.adc_max / (int)(sc.vref * 1000.0f)),  // raw при желании
                  v_mv, a_mv, f_mv, (int)sensors[i].state());
    
  }
}

    // LED (PA5): NORMAL slow blink, ALARM solid, FAULT fast blink
    if (anyFault) {
      if (now - lastBlink > 100) { lastBlink = now; led = !led; }
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, led ? GPIO_PIN_SET : GPIO_PIN_RESET);
    } else if (anyAlarm) {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    } else {
      if (now - lastBlink > 500) {
        lastBlink = now; led = !led;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, led ? GPIO_PIN_SET : GPIO_PIN_RESET);
      }
    }

    firePwr.update();
  }
}

