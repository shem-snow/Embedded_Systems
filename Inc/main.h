#pragma once
#include <stdint.h>
#include <stm32f0xx_hal.h> // Ensure HAL is included

/*
Idealy, I could just do 

#define assert(expr) ((expr) ? 0 : assert_failed())

but the assert_failed() method isn't triggered when expressions are false. Instead the program just freezes.
*/

// Declare the main functions of each lab.
int lab1_main(void);

// Functionality we added
void HAL_RCC_GPIOC_CLK_Enable(void); // Enables GPIOC
void HAL_RCC_GPIOA_CLK_Enable(void); // Enables GPIOA

// Other system stuff you need.
void SystemClock_Config(void);
