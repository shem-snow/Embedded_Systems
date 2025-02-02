#pragma once
#include <stdint.h>
#include <stm32f0xx_hal.h> // Ensure HAL is included

/*
Idealy, I could just do 
#define assert(expr) ((expr) ? 0 : assert_failed())
but the assert_failed() method isn't triggered when expressions are false. Instead the program just freezes.
*/
void assert_failed(uint8_t* file, uint32_t line);
#define assert(expr) \
    do { \
        if (!(expr)) { \
            assert_failed((uint8_t*)__FILE__, __LINE__); \
        } \
    } while (0)

// Declare the main functions of each lab.
int lab1_main(void);
void Checkoff_1_2(void);
void Checkoff_1_3(void);

// Functionality in the self-implemented HAL
void HAL_RCC_GPIOX_CLK_Enable(char GPIOx);

// Other system stuff you need.
void SystemClock_Config(void);
