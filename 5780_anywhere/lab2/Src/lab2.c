#include "main.h"
#include "stm32f0xx_hal.h"
#include "My_HAL.h"
#include "System_Setup.h"

// Local prototypes
void SystemClock_Config(void);
void Checkoff_2_1(void);
void Checkoff_2_2(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  // Enable the clocks for each peripheral of interest.
  HAL_RCC_CLK_Enable('C', 0); // GPIOC for LEDs
  HAL_RCC_CLK_Enable('A', 0); // GPIOA for the push-button
  HAL_RCC_CLK_Enable('S', 0); // SYSCFG for selecting external interrupts.

  // Initialize the pins to be used.
  Init_LEDs();
  Init_Button();

  // Select which external interrupt the EXTI will 'multiplex' into the NVIC.
  Reset_Interrupt('A'); // PinA is the push-button.

  // Set the priorities of each task how you want them.
  NVIC_SetPriority(EXTI0_1_IRQn, 3); // Button toggles orange and green.
  NVIC_SetPriority(SysTick_IRQn, 1); // SysTick toggles blue.

  // Toggle the red LED every 500 ms.
  while (1) {
    HAL_Delay(500);
    My_HAL_GPIO_TogglePin(GPIOC, RED);
  }
  return -1;
}

/*
* Pre-defined interrupt handler for External Interrupts.
*
* The "EXTernal Interrupt and events controller (EXTI) allows us to 'multiplex' non-peripheral
* wires so they cause interrupts. In this case, the push button will cause the interrupt.
*/
void EXTI0_1_IRQHandler(void) {
	//Checkoff_2_1();
  Checkoff_2_2();

  // Clear the pending bit to prevent the same interrupt from being re-triggered.
	EXTI->PR |= 1; // Clears the pending bit so execution can leave the interrupt.
	//EXTI->PR &= ~1; // gets execution stuck in the interrupt.
}

void Checkoff_2_1(void) {
  // Toggle the orange and green LEDs every time this interrupt is triggered.
  HAL_GPIO_TogglePin(GPIOC, GREEN);
	HAL_GPIO_TogglePin(GPIOC, ORANGE);
}

void Checkoff_2_2(void) {

  // Toggle the LEDs once
  HAL_GPIO_TogglePin(GPIOC, GREEN);
	HAL_GPIO_TogglePin(GPIOC, ORANGE);

	// Insert a delay (without using the hardware abstraction layer) so we can see the lower priority tasks stop.
	volatile uint32_t accumulator = 0;
	while(accumulator < 999900)
		accumulator++;

	// Toggle the LEDs again and watch the lower priority tasks resume.
  My_HAL_GPIO_TogglePin(GPIOC, GREEN);
	My_HAL_GPIO_TogglePin(GPIOC, ORANGE);
}

// ########################## Build-in nonsense ##########################

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
  /* User can add their own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add their own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */
