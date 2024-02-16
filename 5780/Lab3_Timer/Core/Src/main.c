/* USER CODE BEGIN Header */
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes and definitions ----------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* Reset and initializes. */
  HAL_Init();
	
	// Enable the peripherals we will use (reference the "reset and clock control" section of the peripheral datasheet).
	RCC->APB1ENR |=  RCC_APB1ENR_TIM2EN;// Timer 2
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // Timer 3
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // LEDs
	
	// Configure and initialize the green (PC8) and orange (PC9) LEDs
	GPIOC->MODER &= ~(3<<16); // input mode 00
	GPIOC->MODER &= ~(3<<18);
	GPIOC->OTYPER &= ~(3<<8); // push-pull 0
	GPIOC->OSPEEDR &= ~(1<<16); // low speed x0
	GPIOC->OSPEEDR &= ~(1<<18);
	GPIOC->PUPDR &= ~(3<<16); // no pull, pull-down 00
	GPIOC->PUPDR &= ~(3<<18);
	GPIOC->ODR |= (1<<8); // Green high
	GPIOC->ODR &= ~(1<<9); // orange low
	
	// Set the PSC and ARR to obtain a target clock frequency of 4 Hz (timer 2 is 32-bit and timer 3  is 16-bit)
	TIM2->PSC = 7999;
	TIM2->ARR = 250;
	
	// Configure the timer to generate an interrrupt on the UEV event.ADC
	TIM2->DIER |= 1;
	
	// Configure the timer and start it.
	TIM2->CR1 |=1;
	
	
  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  

  /* Infinite loop */
  while (1)
  {
  }
}


/*
* Interrupt handler for timer 2
*/
void TIM2_IRQHandler(void) {
	// Toggle between the green (PC8) and orange (PC9) LEDs
	
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
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
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
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
