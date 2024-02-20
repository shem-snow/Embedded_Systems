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
	
	// Enable the peripherals we will use (reference the "reset and clock control" section of the peripheral manual).
	RCC->APB1ENR |=  RCC_APB1ENR_TIM2EN;// Timer 2
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // Timer 3
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // LEDs
	
	// Configure and initialize the green (PC8) and orange (PC9) LEDs
	GPIOC->MODER &= ~(1<<17); // input mode 01
	GPIOC->MODER &= ~(1<<19);
	GPIOC->MODER |= (1<<16);
	GPIOC->MODER |= (1<<18);
	GPIOC->OTYPER &= ~(3<<8); // push-pull 0
	GPIOC->OTYPER &= ~(3<<9);
	GPIOC->OSPEEDR &= ~(1<<16); // low speed x0
	GPIOC->OSPEEDR &= ~(1<<18);
	GPIOC->PUPDR &= ~(3<<16); // no pull, pull-down 00
	GPIOC->PUPDR &= ~(3<<18);
	GPIOC->ODR |= (1<<8); // Green high
	GPIOC->ODR &= ~(1<<9); // orange low
	
	// Configure and initialize the red (PC6) and blue (PC7) LEDs.
	GPIOC->MODER |= (2<<2*6); // red Alternate function mode: 10
	GPIOC->MODER &= ~(1<<2*6);
	GPIOC->MODER |= (2<<2*7); // blue Alternate function mode: 10
	GPIOC->MODER &= ~(1<<2*7);
	GPIOC->OTYPER &= ~(3<<6); // red push-pull: 0
	GPIOC->OTYPER &= ~(3<<7); // blue push-pull: 0
	GPIOC->OSPEEDR &= ~(1<<2*6); // red low speed: x0
	GPIOC->OSPEEDR &= ~(1<<2*7); // blue low speed: x0
	GPIOC->PUPDR &= ~(3<<2*6); // red no pull, pull-down: 00
	GPIOC->PUPDR &= ~(3<<2*7); // blue no pull, pull-down: 00
	
	// Set the PSC and ARR to obtain target clock frequencies of 4 Hz in timer 2 and 800 Hz in timer 3.
	TIM2->PSC = 7999;
	TIM2->ARR = 250;
	
	// For the lab check-off. Show that as the ratio changes, the duty cycle changes => the blue LED's brightness changes
	
	// blue is bright like red
	TIM3->PSC = 1249;
	TIM3->ARR = 8;
	
	// blue is visible but not as bright as red.
	TIM3->PSC = 1999;
	TIM3->ARR = 5;
	
	// blue is less visible and obviously blinks
	//TIM3->PSC = 7999;
	//TIM3->ARR = 1250;
	
	// blue is almost non-visible
	//TIM3->PSC = 7;
	//TIM3->ARR = 1250;
	
	// Configure the timer to generate an interrrupt on the UEV event. DIER enables direct memory access for a given timer.
	TIM2->DIER |= 1;
	// TIM3->DIER |= 1;
	
	// Configure timer 2 to start.
	TIM2->CR1 |= 1;
	
	// Set the Capture/Compare Mode Registers to put the output channels in PWM mode.
	TIM3->CCMR1 &= ~(3); // 00 indicates that the channel is configured as an output.
	TIM3->CCMR1 |= (7<<4); // bits 4, 5, and 6 set the mode (we want PWM mode 2: 111).
	
	TIM3->CCMR2 &= ~(3);
	
	// Set channel 2 to PWM Mode 1 (110). Bits [14:12] do the same thing as bits [6:4]
	TIM3->CCMR1 |= (3<<13); // 11x
	TIM3->CCMR1 &= ~(1<<12); // xx0
	
	// Enable the output compare preload for both channels (bits 11 and 3).
	TIM3->CCMR1 |= (1<<11);
	TIM3->CCMR1 |= (1<<3);
	
	// Set the output enable bits for channels 1 (bit 0) and 2 (bit 4) in the Capture/Compare Enable Register.
	TIM3->CCER |= 17; // 10001
	
	// Set the capture/compare registers for both channels to 20% of my ARR
	TIM3->CCR1 = 1;
	TIM3->CCR2 = 1;
	
	//TIM3->CCR1 = 20;
	//TIM3->CCR2 = 20;
	
	// Configure the alternate function register for port C (LEDs) 
	GPIOC->AFR[0] &= ~(15); // 1111
	
	// Configure timer 3 to start
	TIM3->CR1 |= 1;
	
	
  /* Configure the system clock */
  SystemClock_Config();

  // Enable the interrupt in the NVIC
	NVIC_EnableIRQ(TIM2_IRQn);
	NVIC_SetPriority(TIM2_IRQn, 1); // A priority is set by default if you don't set one
	
	
	
  

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
	GPIOC->ODR ^= (1<<8);
	GPIOC->ODR ^= (1<<9);
	
	// Clear the pending flag for the interrrup status register
	TIM2->SR ^= 1;
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
