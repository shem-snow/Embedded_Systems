/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stm32f0xx_hal.h>
#include <stm32f0xx_hal_gpio.h>
#include "main.h"
#include "My_HAL.h"
#include <stdio.h>
#include <stdlib.h>
#include "stm32f0xx.h"

#include "motor.h"
#include "SEGGER_RTT.h"

// Local prototypes -----------------------------------------
void ungate_clocks(void);

// Global variables -----------------------------------------

volatile uint32_t debouncer;

/* -------------------------------------------------------------------------------------------------------------
 *  Miscellaneous Core Functions
 *  -------------------------------------------------------------------------------------------------------------
 */

void LED_init(void) {
    // Initialize PC8 and PC9 for LED's
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;                                          // Enable peripheral clock to GPIOC
    GPIOC->MODER |= GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0;                  // Set PC8 & PC9 to outputs
    GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9);                    // Set to push-pull output type
    GPIOC->OSPEEDR &= ~((GPIO_OSPEEDR_OSPEEDR8_0 | GPIO_OSPEEDR_OSPEEDR8_1) |
                        (GPIO_OSPEEDR_OSPEEDR9_0 | GPIO_OSPEEDR_OSPEEDR9_1));   // Set to low speed
    GPIOC->PUPDR &= ~((GPIO_PUPDR_PUPDR8_0 | GPIO_PUPDR_PUPDR8_1) |
                      (GPIO_PUPDR_PUPDR9_0 | GPIO_PUPDR_PUPDR9_1));             // Set to no pull-up/down
    GPIOC->ODR &= ~(GPIO_ODR_8 | GPIO_ODR_9);                                   // Shut off LED's
}

void  button_init(void) {
    // Initialize PA0 for button input
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;                                          // Enable peripheral clock to GPIOA
    GPIOA->MODER &= ~(GPIO_MODER_MODER0_0 | GPIO_MODER_MODER0_1);               // Set PA0 to input
    GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR0_0 | GPIO_OSPEEDR_OSPEEDR0_1);     // Set to low speed
    GPIOA->PUPDR |= GPIO_PUPDR_PUPDR0_1;                                        // Set to pull-down
}

/* Called by SysTick Interrupt
 * Performs button debouncing, changes wave type on button rising edge
 * Updates frequency output from ADC value
 */
void HAL_SYSTICK_Callback(void) {
    // Remember that this function is called by the SysTick interrupt
    // You can't call any functions in here that use delay

    debouncer = (debouncer << 1);
    if(GPIOA->IDR & (1 << 0)) { // This is PA0 (the button).
        debouncer |= 0x1;
    }


    if(debouncer == 0x7FFFFFFF) {

        // Begin critical section
        __disable_irq();
        switch(rpm_flag) {
            case 0:
                target_rpm = 4; // 80
                rpm_flag++;
                GPIOC->ODR ^= RED;
                break;
            case 1:
                target_rpm = 16; // 50
                rpm_flag++;
                GPIOC->ODR ^= RED;
                break;
            case 2:
                target_rpm = 64; // 81
                rpm_flag++;
                GPIOC->ODR ^= RED;
                break;
            case 3:
                target_rpm = 80; // 0
                rpm_flag = 0;
                GPIOC->ODR ^= RED;
                break;
            default:
                target_rpm = 0;
                rpm_flag = 0;
                break;
        }
        __enable_irq(); // End critical section
    }

   
  // Blink the blue LED every 200 loops to ensure this is being called.
  static int Lab7_SysTick_accumulator;
  if(Lab7_SysTick_accumulator >= 200) {
    GPIOC->ODR ^= BLUE;
    Lab7_SysTick_accumulator = 0;
  }
  else
    Lab7_SysTick_accumulator += 1;
}

/* -------------------------------------------------------------------------------------------------------------
 * Main Program Code
 *
 * Starts initialization of peripherals
 * Blinks green LED (PC9) in loop as heartbeat
 * -------------------------------------------------------------------------------------------------------------
 */
volatile uint32_t encoder_count = 0;

int lab7_main(void) {

    // Initializations of global variables
    debouncer = 0;

    // HAL setup
	HAL_Init();

    // Peripheral initializations
    Init_LEDs();
    button_init();
    motor_init();

    while (1) {
        encoder_count = TIM2->CNT;
        HAL_GPIO_TogglePin(GPIOC, GREEN); // Toggle green LED (heartbeat)
        HAL_Delay(128); // Delay 1/8 second
    }


    return -1;
}
