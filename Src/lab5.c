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

/* Local prototypes ------------------------------------------------------------------------------------------------------------------*/
void Ungate_Clocks(void);
void Error_loop(int LED_color, int frequency);

void Checkoff_5_1(void);
void Checkoff_5_2(void);

/* Global Variables ------------------------------------------------------------------------------------------------------------------*/
static int16_t x_total = 0;
static int16_t y_total = 0;

int lab5_main(void) {

	// Initializations and Instantiations
	HAL_Init();
	SystemClock_Config();
    
    Ungate_Clocks();
	Init_LEDs();
	Init_I2C2();

	// The instructions said to clear the number of bits and slave address
	I2C2->CR2 &= ~( (0x7F << 16) | (0x3FF << 0) );
	
	//Checkoff_5_1();
	Checkoff_5_2();

	return 0;
}

/* Helper Methods ------------------------------------------------------------------------------------------------------------------*/

/*
	Uses a Gyroscope and LEDs to indicate the physical orientation of the microcontroller. 

	Note that the gyroscope measures CHANGES to its orientation. Therefor you should save its current position and 
	add each change you read.
*/
void Checkoff_5_2(void) {

	Init_Gyroscope();
	
	while(1) {

		// Insert a delay so orientation is read only once in an observable period.
		HAL_Delay(100);
		
		// Read the gyroscope output for the X and Y axes. Each axis is done in two bytes because the gyroscope uses 8-bit registers for 16-bit data.
		int16_t x_change = Read_Gyroscope_Output('X'); // MSB
		x_change = x_change << 8;
		x_change |= Read_Gyroscope_Output('x'); // LSB

		x_total += x_change;

		int16_t y_change = Read_Gyroscope_Output('Y'); // MSB
		y_change = y_change << 8;
		y_change |= Read_Gyroscope_Output('y'); // LSB

		y_total += y_change;

	
		// If the x position is negative, turn off the orange LED and turn on the green LED. Note that this is opposite of what you see on the board.
		// The instructions also said to use a threshold value for more accuracy. I'll try that
		int neg_thresh = -10;
		int pos_thresh = 10;
		
		if (x_total < neg_thresh) {
			GPIOC->ODR &= ~ORANGE;
			GPIOC->ODR |= GREEN;
		}
		// Otherwise the x position is positive. Turn on orange and turn off green.
		else if(x_total > pos_thresh) {
			GPIOC->ODR |= ORANGE;
			GPIOC->ODR &= ~GREEN;
		}
	
		// If the y position is negative, turn blue off and red on.
		if (y_total < neg_thresh) {
			GPIOC->ODR &= ~BLUE;
			GPIOC->ODR |= RED;
		}
		// Otherwise the y position is positive. Turn on blue and turn off red.
		else if(y_total > pos_thresh) {
			GPIOC->ODR |= BLUE;
			GPIOC->ODR &= ~RED;
		}
	}
}

/*
	Reads a memory-mapped register on the gyroscope to ensure you are communicating with it. Success is indicated by turning the GREEN lED on.
	Failure is indicated by toggling various LEDs.
*/
void Checkoff_5_1(void) {

	// Configure parameters for the current transaction (CR2).
	I2C2->CR2 |= (0x69 << 1); // Slave (gyroscope) address is 0x69. Shift 1 because the 0th and 9th bits are don't cares.
	I2C2->CR2 |= (1 << 16); // Number of bytes to send = 1.
	I2C2->CR2 &= ~(1 << 10); // Set the read/write direction to write (relative to the master device).
	I2C2->CR2 |= I2C_CR2_START; // Start is the 13th bit.

	// Wait for communication to be established.
	while(!( (I2C2->ISR) & ( (I2C_ISR_TXIS) | (I2C_ISR_NACKF) ) )) {} // Not-Acknowledge status (which means transmission failed) is bit 4. Transmit interrupt status (which means transmission succeeded) is bit 1.

	// Then check if the slave acknowledged the master.
	if( (I2C2->ISR) & (I2C_ISR_NACKF) )
		Error_loop(RED, 100);
	else {

		// Now that we have a medium to communicate, send a message by writing it to the Transmit Data Register.
		I2C2->TXDR |= 15; // 0xF is the address of the "whoamI" (out_x_L) register.

		// Wait for the transmission to complete.
		while(!( (I2C2->ISR) & ( (I2C_ISR_TC) | (I2C_ISR_NACKF)) )) {} // Transfer Complete is the 6th bit.

		// Enter an error loop on failure
		if( (I2C2->ISR) & (I2C_ISR_NACKF) )
			Error_loop(ORANGE, 100);

		else {
			
			// On success, re-configure the parameters for this transaction. This time, set the read/write direction to read.
			I2C2->CR2 |= 0x69 << 1; // Slave (gyroscope) address is 0x69. Shift 1 because the 0th and 9th bits are don't cares.
			I2C2->CR2 |= 1 << 16; // Number of bytes to send = 1.
			I2C2->CR2 |= 1 << 10; // Set the read/write direction to read (relative to the master device).
			I2C2->CR2 |= I2C_CR2_START; // 1<<13

			// Just like before, wait for success or failure in establishing the connection.
			while(!((I2C2->ISR) & ((I2C_ISR_RXNE) | (I2C_ISR_NACKF)))) {} // But this time, since we're reading instead of writing, Check the Receive data register Not Empty flag (2nd bit) which indicates a successful read.

			// Enter an error loop on failure
			if( (I2C2->ISR) & (I2C_ISR_NACKF) )
				Error_loop(RED, 1000);
	
			else {
				
				// Wait for the transmission to complete.
				while(!( (I2C2->ISR) & ( (I2C_ISR_TC) | (I2C_ISR_NACKF)) )) {} // Transfer Complete is the 6th bit.
	
				// Enter an error loop on failure
				if( (I2C2->ISR) & (I2C_ISR_NACKF) )
					Error_loop(ORANGE, 1000);

				// Indicate corrupt ID error by toggling the BLUE LED.
				if( I2C2->RXDR != 0xD3)
					Error_loop(BLUE, 100);
	
				// The transaction was successful. Release the I2C BUS.
				I2C2->CR2 |= I2C_CR2_STOP; // 1<<14

				// Indicate success by turning the green LED on.
				HAL_GPIO_WritePin(GPIOC, GREEN, GPIO_PIN_SET);
			}
		}
	}
}

/*
	Gets you stuck in an infinite loop that toggles an LED periodically to indicate errors.
*/
void Error_loop(int LED_color, int frequency) {
	while(1) {
		HAL_Delay(frequency);
		GPIOC->ODR ^= LED_color;
	}
}

void Ungate_Clocks(void) {
    HAL_RCC_CLK_Enable('B', 0); // GPIOB
    HAL_RCC_CLK_Enable('C', 0); // GPIOC
    HAL_RCC_CLK_Enable('I', 2); // I2C2
}

void Init_I2C2(void) {
	
	// PB11 AF1 is I2C2_SDA
	GPIOB->MODER |= (2 << 2*11); // GPIOB->MODER |= GPIO_MODER_MODER11_1;
	GPIOB->OTYPER |= (1 << 11); // GPIOB->OTYPER |= GPIO_OTYPER_OT_11;
	GPIOB->AFR[1] |= (1 << 12); // GPIOB->AFR[1] |= (0x1 << GPIO_AFRH_AFSEL11_Pos);

	// PB13 AF5 is I2C2_CLK
	GPIOB->MODER |= (2 << 2*13); // GPIOB->MODER |=GPIO_MODER_MODER13_1;
	GPIOB->OTYPER |= (1 << 13); // GPIOB->OTYPER |= GPIO_OTYPER_OT_13;
	GPIOB->AFR[1] |= (5 << 20); // GPIOB->AFR[1] |= (0x5 << GPIO_AFRH_AFSEL13_Pos);

	// Pin PC0 is connected to the Gyroscope's mode select line between SPI and I2C.
	GPIOC->MODER &= ~(2 << 0); 
	GPIOC->MODER |= (1 <<0); // GPIOB->MODER |= GPIO_MODER_MODER0_0;
	GPIOC->OTYPER &= ~(1 << 0); // GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_0);
	GPIOC->ODR |= (1 << 0); // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);

	// Setting the 0'th bit high selects I2C. Setting it low selects SPI. 0 means no action (use default which is SPI). 1 means do the other thing.
	GPIOC->BSRR = (1 << 0); // I think this is the same as writing to ODR
	
	// PB14 controls the slave address when in I2C mode.
	GPIOB->MODER &= ~(2 << 2*14);
	GPIOB->MODER |= (1 << 2*14); // GPIOB->MODER |= GPIO_MODER_MODER14_0;
	GPIOB->OTYPER &= ~(1 << 14); // GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_14);
	GPIOB->ODR |= (1 << 14); // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	
	// Configure the I2C2 peripheral to operate at 100 kHz (standard-mode).
	I2C2->TIMINGR |= 1 << 28; // Prescaler = 1  // I2C2->TIMINGR |= (0x1 << I2C_TIMINGR_PRESC_Pos);
	I2C2->TIMINGR |= 0x13 << 0; // SCLL = 0x13  // I2C2->TIMINGR |= (0x13 << I2C_TIMINGR_SCLL_Pos);
	I2C2->TIMINGR |= 0xF << 8; // SCLH = 0xF	// I2C2->TIMINGR |= (0xF << I2C_TIMINGR_SCLH_Pos);
	I2C2->TIMINGR |= 0x2 << 16; // SCADEL = 0x2 // I2C2->TIMINGR |= (0x2 << I2C_TIMINGR_SDADEL_Pos);
	I2C2->TIMINGR |= 0x4 << 20; // SCLDEL = 0x4 // I2C2->TIMINGR |= (0x4 << I2C_TIMINGR_SCLDEL_Pos);

	// Enable the I2C peripheral in CR1
	I2C2->CR1 |= I2C_CR1_PE; // 1;
}
