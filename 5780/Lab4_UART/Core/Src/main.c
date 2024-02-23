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
#include "main.h"

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Transmit_Char(char c);


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
	
	// Enable all the peripherals we're using.
	RCC->AHBENR |= RCC_AHBENR_CRCEN; // Port C: LEDs and the port C pins.
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN; // USART3
	
	// Set the discovery board pins (connected to USART3) to alternate function mode (10)
	GPIOC->MODER |= 2 << (2*4); // pin 4
	GPIOC->MODER &= ~(1<<2*4);
	
	GPIOC->MODER |= 2 << (2*5); // pin 5
	GPIOC->MODER &= ~(1<<2*5);
	
	// Multiplex to alternate function mode 1 ([3:0] = 0001)
	GPIOC->AFR[0] &= ~(14 << 4*4); // pin 4
	GPIOC->AFR[0] |= (1 << 4*4);
	
	GPIOC->AFR[0] &= ~(14 << 4*5); // pin 5
	GPIOC->AFR[0] |= (1 << 4*5);
	
  // Set the Baud rate for communcation to be 115,200 bits/second. The system clock is 8 MHz.
	USART3->BRR = HAL_RCC_GetHCLKFreq() / 69; // nice
	
	// Enable USART in the control register.
	USART3->CR1 |= 6; // ..110, bit 2 enables the receiver and bit 3 enables the transmitter.
	USART3->CR1 |= 1;


  while (1)
  {
  }
}

// __________________________________________________________________ Helper methods _______________________________________________

/*
* Does nothing while the transmit data register is empty.
* When it's not empty, it transmits the character.
*/
void Transmit_Char(char c) {
	
	while( !(USART3->ISR & (1<<7) ) ) {
	}
	USART3->TDR = c;
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
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

  /** Initializes the CPU, AHB and APB buses clocks
  */
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

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
  __disable_irq();
  while (1) {
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line) {
}
#endif
