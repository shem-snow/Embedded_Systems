/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This program was my first "Hello_World" for embedded systems. It was completed in two parts.
  *
  * Part 1: Learning to toggle two LEDs with the HAL_Delay() function.
  *
  * Part 2: Learning to toggle those same two LEDs but only when the user presses a button.
  *     This method required the use of a 'de-bouncer' since button presses are not stable.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stm32f0xx_hal.h> // Ensure HAL is included

/* Function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Setup(void);
void Checkoff1(void);
void Checkoff2(void);

int lab1_main(void){

	// Reset of all peripherals, init the Flash and Systick
	HAL_Init();

	// Configure the system clock
	SystemClock_Config();

	// Enable the GPIOC clock in the RCC (OR with itself and all zeros except the bit to enable)
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // on lines 449 and 7869
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // on line 7863

	Setup();

	//Checkoff1();
	Checkoff2();

	// To avoid getting the "control reaches end of non-void function" error, return an int.
	return 0;
}

void Setup(void) {

	// _____________________________ Push Button ________________________________________

	// 1: Set the MODER pin to input mode (00).
	GPIOA->MODER &= ~(3); // 111100

	// 2: Set the OSPEEDR register to low speed (x0).
	GPIOA->OSPEEDR &= ~1;

	// 3: Set the PUPDR register to pull-down (10).
	GPIOA->PUPDR &= ~3; // xx..00
	GPIOA->PUPDR |= 2; // xx..10




	/* _____________________________ LEDs ________________________________________

		Mappings:
			uint32_t RED = 1 << 6;
			uint32_t BLUE = 1 << 7;
			uint32_t ORANGE = 1 << 8;
			uint32_t GREEN = 1 << 9;
	*/

	// 1: Set the MODER pins PC6 (blue) and PC7 (red) to general purpose output mode (01).

	// Step one: clear the existing bits
	uint32_t mask = 15; // 1111
	mask = mask << (2*6); // 0000..1111..0000
	GPIOC->MODER &= ~mask; // xxxx..0000..xxxx

	// Step two: set the bits to their new value.
	mask = 5; // 0101
	mask = mask << (2 * 6); // 0000..0101..0000
	GPIOC->MODER |= mask; // xxxx..0101..xxxx

	// 2: Set the LED OTYPER pins to push-pull output type (single-bit 0).
	mask = 3 << 6; // 00..11..00
	GPIOC->OTYPER &= ~mask; // xx..00..xx

	// 3: Set the OSPEEDR register to 'low speed' (x0).
	mask = 5 << (2*6); // 00..0101..00
	GPIOC->OSPEEDR &= ~mask; // xx..x0x0..xx

	// 4: Set the PUPDR register to 'No pull-up, pull-down' (00).
	mask = 15 << (2*6); // 00..1111..00
	GPIOC->PUPDR &= ~mask; // xx..0000..xx

	// 5: Initialize PC6 (blue) high (1) and PC7 (red) to low (0) => set it to 10.

	// Step one: clear xx..00..xx
	mask = 3 << 6; // 00..11..00
	GPIOC->ODR &= ~mask; // 11..00..11

	// Step two: set to 10
	mask = 2 << 6; // 00..10..00
	GPIOC->ODR |= mask; // xx..10..xx
}


void Checkoff1() {

	// Use this loop to make the red and blue LEDs toggle automatically.
	uint32_t mask = (3 << 6); // 00..11..00
	while (1) {
		HAL_Delay(200);
		GPIOC->ODR ^= mask; // xx..~(xx)..xx
	}
}


void Checkoff2() {
		// Toggle the output state of both PC6 (blue) and PC7 (red) every time the user button is pressed.
	uint32_t debouncer = 0;
	uint32_t mask = (3 << 6); // 00..11..00
	while (1) {

		// Shift the debouncer left at every loop iteration
		debouncer = debouncer << 1;

		// If the user button is pressed, set the LSB to 1.
		if(GPIOA->IDR & 1)
			debouncer |= 1;

		// Do nothing when the button is 'steady high' or 'steady low'.
		if( (debouncer == 0xFFFFFFFF) || (debouncer == 0) )
			continue;

		// When transitioning from 'steady low' to 'steady high', toggle the LEDs.
		else if(debouncer == 0x1) {
			GPIOC->ODR ^= mask; // xx..~(xx)..xx
		}
	}
}