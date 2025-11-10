#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

void SystemClock_Config(void);
void Error_Handler(void);

/* HAL handles, visible from C++ files */
extern ADC_HandleTypeDef   hadc1;
extern UART_HandleTypeDef  huart2;

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
