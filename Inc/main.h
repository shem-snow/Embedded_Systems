#pragma once
#include <stdint.h>
#include <stm32f0xx_hal.h> // Ensure HAL is included

// Declare the main functions of each lab.
int lab1_main(void);

// Functionality we added
void HAL_RCC_GPIOC_CLK_Enable(void); // Enables GPIOC
void HAL_RCC_GPIOA_CLK_Enable(void); // Enables GPIOA

// Other system stuff you need.
void SystemClock_Config(void);
