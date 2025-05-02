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
#include "main.h"

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Init_LEDs(void);
void Init_ADC(void);
void Init_DAC(void);

void Calibrate_and_start_ADC(void);

void Checkoff_1(void);
void Checkoff_2(void);

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

int main(void)
{
  // Initializations
  HAL_Init();
  SystemClock_Config();
	
	// Clock enables
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;
	
	// Configure PC0 (ON CHANNEL 10 as shown on page 37 in the M-0 manual) as ADC input and PA4 as DAC output (channel 1 page 38).
	Init_ADC();
	Init_DAC();
	
	// Self-calibrate the ADC.
	Calibrate_and_start_ADC();
	
  while (1) {
		Checkoff_1();
		//Checkoff_2();
  }
}

// __________________________________________________ Lab checkoffs _______________________________________________________________
void Checkoff_1(void) {
	
	int32_t RED = 1 << 6;
	int32_t GREEN = 1 << 9;
	int32_t BLUE = 1 << 7;
	int32_t ORANGE = 1 << 8;
	
	// Store the analog signal into a variable
	int16_t sig = ADC1->DR;
	
	// Use the 4 LEDs as threshold values to determine its magnitude
	if(sig < 8) // Keep all the LEDs off
		GPIOC->ODR &= ~(RED | GREEN | BLUE | ORANGE);
	else if (8 < sig & sig < 16) {
		// Turn RED on
		GPIOC->ODR &= ~(GREEN | BLUE | ORANGE);
		GPIOC->ODR |= RED;
	}
	else if (16 < sig & sig < 32) {
		// Turn GREEN on
		GPIOC->ODR &= ~(BLUE | ORANGE);
		GPIOC->ODR |= (RED | GREEN);
	}
	else if (32 < sig & sig < 64) {
		// Turn BLUE on
		GPIOC->ODR &= ~ORANGE;
		GPIOC->ODR |= (RED | GREEN | BLUE);
	}
	else// 64 < sig
		// Turn ORANGE on
		GPIOC->ODR |= (RED | GREEN | BLUE | ORANGE);
}
	
void Checkoff_2(void) {
	
	// Write the chosen table into the data holding register one value at a time with a 100 ms delay.
	DAC1->DHR8R1 = sine_table[table_index];
	//DAC1->DHR8R1 = triangle_table[table_index];
	//DAC1->DHR8R1 = sawtooth_table[table_index];
	
	// Reset the index
	table_index = (table_index = 31)? 0 : table_index + 1;
	
	HAL_Delay(100);
}

// __________________________________________________ Helper methods _______________________________________________________________
void Init_LEDs(void) {
	
	// Set the MODER to output mode (01)
	GPIOC->MODER &= ~(1<< (2*6 +1) );
	GPIOC->MODER &= ~(1<< (2*7 +1) );
	GPIOC->MODER &= ~(1<< (2*8 +1) );
	GPIOC->MODER &= ~(1<< (2*9 +1) );
	
	GPIOC->MODER |= 1 << 2*6;
	GPIOC->MODER |= 1 << 2*7;
	GPIOC->MODER |= 1 << 2*8;
	GPIOC->MODER |= 1 << 2*9;
	
	// Set output type register to push-pull (0)
	GPIOC->OTYPER &= ~(1<<6);
	GPIOC->OTYPER &= ~(1<<7);
	GPIOC->OTYPER &= ~(1<<8);
	GPIOC->OTYPER &= ~(1<<9);
	
	// Set output speed register to low speed (x0)
	GPIOC->OSPEEDR &= ~(1 << 2*6);
	GPIOC->OSPEEDR &= ~(1 << 2*7);
	GPIOC->OSPEEDR &= ~(1 << 2*8);
	GPIOC->OSPEEDR &= ~(1 << 2*9);
	
	// Set the pins to no pull-up, pull-down (00)
	GPIOC->PUPDR &= ~(3<<2*6);
	GPIOC->PUPDR &= ~(3<<2*7);
	GPIOC->PUPDR &= ~(3<<2*8);
	GPIOC->PUPDR &= ~(3<<2*9);
	
	// Initialize each light to be off
	GPIOC->BSRR = GPIO_BSRR_BR_6;
	GPIOC->BSRR = GPIO_BSRR_BR_7;
	GPIOC->BSRR = GPIO_BSRR_BR_8;
	GPIOC->BSRR = GPIO_BSRR_BR_9;
}


void Init_ADC(void) {
	
	GPIOC->MODER |= 3 << 0; // Analog mode (11)
	GPIOC->PUPDR &= ~(3 << 0); // No pull-up, pull down (00)
	ADC1->CHSELR |= (1 << 10 ); // Configure the pin for ADC conversion on channel 10
	
	// 8-bit resolution (10)
	ADC1->CFGR1 |= (2 << 3);
	ADC1->CFGR1 &= ~(1 << 3);
	
	// Continuous conversion mode
	ADC1->CFGR1 |= (1 << 13);
	
	// Hardware triggers disabled (software-triggered only)
	ADC1->CFGR1 &= ~(3 << 10);
}

void Init_DAC(void) {
	// Configure PA4
	GPIOA->MODER |= 3 << 8; // Analog mode (11)
	GPIOA->PUPDR &= ~(3 << 8); // No pull-up, pull down (00)
	GPIOA->OTYPER &= ~(3 <<8); // Push/Pull (00)
	GPIOA->OSPEEDR &= ~(3 <<8); // Low speed (00)
	
	// Actually initialize the DAC
	DAC1->CR &= ~(7 << 19); // Software-triggered (111)
	DAC1->CR |= 1; // Enable the DAC Channel 1
}

void Calibrate_and_start_ADC(void) {
	
	// ___________________Calibrate (reference appendix A.7.1)___________________
	// Calibration is initialted when ADEN = 1. So initialize it to zero/disable it.
	if( (ADC1->CR & ADC_CR_ADEN) !=0)
		ADC1->CR |= 1 << 1;
	// Wait for the action to complete.
	while ( (ADC1->CR & ADC_CR_ADEN) != 0) {
	}
	// Clear the DMA bit so 
	ADC1->CFGR1 &= ~(1);
	// Trigger the calibration in the control register
	ADC1->CR |= (1 << 31);
	// Wait for the action to complete.
	while ( (ADC1->CR & ADC_CR_ADCAL) != 0) {
	}
	
	
	// ___________________ Enable Sequence code (reference appendix A.7.1)___________________
	if( (ADC1->ISR & ADC_ISR_ADRDY) != 0)
		ADC1->ISR |= ADC_ISR_ADRDY;
	ADC1->CR |= ADC_CR_ADEN;
	// Wait for the action to complete
	while ( (ADC1->ISR & ADC_ISR_ADRDY) == 0 ) {
	}
	
	// _____________________ Start _____________________
	ADC1->CR |= (1 << 2);
	
}
// __________________________________________________ System _______________________________________________________________
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
