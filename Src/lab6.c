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
// #include <assert.h>
#include "main.h"
#include "My_HAL.h"

// Local prototypes -----------------------------------------
void ungate_clocks(void);
void Checkoff_6_1(void);
void Checkoff_6_2(void);

// Global variables -----------------------------------------
int table_index = 0;
const uint8_t sine_table[32] = {127,151,175,197,216,232,244,251,254,251,244,
232,216,197,175,151,127,102,78,56,37,21,9,2,0,2,9,21,37,56,78,102}; // Sine Wave: 8-bit, 32 samples/cycle

const uint8_t triangle_table[32] = {0,15,31,47,63,79,95,111,127,142,158,174,
190,206,222,238,254,238,222,206,190,174,158,142,127,111,95,79,63,47,31,15};// Triangle Wave: 8-bit, 32 samples/cycle

const uint8_t sawtooth_table[32] = {0,7,15,23,31,39,47,55,63,71,79,87,95,103,
111,119,127,134,142,150,158,166,174,182,190,198,206,214,222,230,238,246};// Sawtooth Wave: 8-bit, 32 samples/cycle

const uint8_t square_table[32] = {254,254,254,254,254,254,254,254,254,254,
254,254,254,254,254,254,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // Square Wave: 8-bit, 32 samples/cycle (Don't use)


int lab6_main(void) {
    // Initializations
    HAL_Init();
    SystemClock_Config();
	
	// Clock enables
    ungate_clocks();
	
	Init_LEDs();

	// Configure PC0 (ON CHANNEL 10 as shown on page 37 in the M-0 manual) as ADC input and PA4 as DAC output (channel 1 page 38).
	Init_ADC(GPIOC, 0);
	Init_DAC(GPIOA, 4);
    
	// Self-calibrate the ADC.
	Calibrate_and_start_ADC();
	
    while (1) {
        Checkoff_6_1();
		//Checkoff_6_2();
    }
    return 0;
}

void Checkoff_6_1(void) {
	
	// Store the analog signal into a variable
	int16_t analog_signal = ADC1->DR;
	
	// Use the 4 LEDs as threshold values to determine its magnitude
	if(analog_signal < 8) // Keep all the LEDs off
		GPIOC->ODR &= ~(RED | GREEN | BLUE | ORANGE);
	else if ( (8 < analog_signal) & (analog_signal < 16) ) {
		// Turn RED on
		GPIOC->ODR &= ~(GREEN | BLUE | ORANGE);
		GPIOC->ODR |= RED;
	}
	else if ( (16 < analog_signal) & (analog_signal < 32) ) {
		// Turn GREEN on
		GPIOC->ODR &= ~(BLUE | ORANGE);
		GPIOC->ODR |= (RED | GREEN);
	}
	else if ( (32 < analog_signal) & (analog_signal < 64) ) {
		// Turn BLUE on
		GPIOC->ODR &= ~ORANGE;
		GPIOC->ODR |= (RED | GREEN | BLUE);
	}
	else// 64 < sig
		// Turn ORANGE on
		GPIOC->ODR |= (RED | GREEN | BLUE | ORANGE);
}
	
void Checkoff_6_2(void) {
	
	// Write the chosen table into the data holding register one value at a time with a 100 ms delay.
	DAC1->DHR8R1 = sine_table[table_index];
	//DAC1->DHR8R1 = triangle_table[table_index];
	//DAC1->DHR8R1 = sawtooth_table[table_index];
	
	// Reset the index
	table_index = (table_index = 31)? 0 : table_index + 1;
	
	HAL_Delay(100);
}


void ungate_clocks(void) {
    HAL_RCC_CLK_Enable('A', 0); // GPIOA
    HAL_RCC_CLK_Enable('C', 0); // GPIOC
    HAL_RCC_CLK_Enable('A', 1); // ADC
	HAL_RCC_CLK_Enable('D', 1); // DAC
}