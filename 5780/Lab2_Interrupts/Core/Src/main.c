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


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();
	
	// Enable the peripheral clocks
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	
  /* Initialize all configured peripherals _______________________________________*/
	
	// Set the MODER pins to general purpose output mode (01) 
	
	// The green LED (PC9)
	GPIOC->MODER &= ~(1 << 19); // 0
	GPIOC->MODER |= 1 << 18; // 1
	
	// The red LED (PC6)
	GPIOC->MODER &= ~(1 << 13); // 0
	GPIOC->MODER |= 1 << 12; // 1
	
	// The blue LED (PC7)
	GPIOC->MODER &= ~(1 << 15); // 0
	GPIOC->MODER |= 1 << 14; // 1
	
	// The oragne LED (PC8)
	GPIOC->MODER &= ~(1 << 17); // 0
	GPIOC->MODER |= 1 << 16; // 1
	
	// Set the MODER pins to input mode (00) for the push button 
	GPIOA->MODER &= ~(3); // 00
	
	
	// Set the OTYPER register pins to push-pull (0)
	GPIOC->OTYPER &= ~(1 << 9); // Green
	GPIOC->OTYPER &= ~(1 << 6); // Red
	GPIOC->OTYPER &= ~(1 << 7); // Blue
	GPIOC->OTYPER &= ~(1 << 8); // Orange
	
	// Set the OSPEEDR register to 'low speed' (x0).
	GPIOC->OSPEEDR &= ~(1 << 18); // Green
	GPIOC->OSPEEDR &= ~(1 << 12); // Red
	GPIOC->OSPEEDR &= ~(1 << 14); // Blue
	GPIOC->OSPEEDR &= ~(1 << 16); // Orange
	GPIOA->OSPEEDR &= ~(1); // Push-button
	
	// Set the PUPDR register to 'No pull-up, pull-down' (00).
	GPIOC->PUPDR &= ~(3 << 18); // Green
	GPIOC->PUPDR &= ~(3 << 12); // Red
	GPIOC->PUPDR &= ~(3 << 14); // Blue
	GPIOC->PUPDR &= ~(3 << 16); // Orange
	
	// Set the PUPDR register to 'pull-down' (10) for the push-button.
	GPIOA->PUPDR |= 2; // 1
	GPIOA->PUPDR &= ~(1); // 0
	
	// Initialize the LEDs to high or low.
	GPIOC->ODR |= (1 << 9); // Green
	GPIOC->ODR |= (1 << 6); // Red
	GPIOC->ODR |= (1 << 7); // Blue
	GPIOC->ODR |= (1 << 8); // Orange
	
	// Configure the EXTI peripheral to generate interrupts on the rising edge of the push-button.
	EXTI->IMR |= 1; // Unmask the associated bit (page 219 and example code on page 947)
	EXTI->RTSR |= 1; // Configure the trigger selection bit of the interrupt line on the rising edge
	// EXTI->FTSR |= 1; // Configure the trigger selection bits of the interrupt line on the falling edge
	
	RCC->APB2RSTR |= 1; // Enable the SYSCFGRST clock (peripheral manual page 117).
	SYSCFG->EXTICR[0] &= ~(7); // (pages 169 and 170) clear the bottom three bits of PA to multiplex the push-button into EXTI0
	
	NVIC_EnableIRQ(EXTI0_1_IRQn); // Enable Interrupt on EXTIO_1
	NVIC_SetPriority(EXTI0_1_IRQn, 0); // Make it more important than the other interrupts.
	
	
  /* Infinite loop */
  while (1) {
		HAL_Delay(500);
		
		GPIOC->ODR ^= (1 << 6); // toggle red
  }
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK |RCC_CLOCKTYPE_PCLK1;
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
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
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
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
