/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This program was an introduction to interrupts.
  *
  * The lesson in it was to perform multiple (three) different tasks (blinking LEDs),
  * each with a different priority, and watching the lower priority tasks halt
  * while the higher priority tasks executed.
  *
  *
  * The three tasks are:
  * 	- Main thread blinks the red LED at some frequency.
  * 	- The systick toggles the blue LED at another frequency.
  * 	- The push button toggles the green and orange LEDs every time its pressed.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Register_Setup(void);
void Task_Manager(void);

void Checkoff_Two(void);


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {

  // Reset of all peripherals and initialize the Flash interface and the Systick.
  HAL_Init();

  // Configure the system clock
  SystemClock_Config();

  // Initialize the peripheral registers for the LEDs and push-button.
  Register_Setup();

  // Set an external wire (the push button) to trigger an interrupt. Also assign different priorities for the three different tasks.
  Task_Manager();

  // Main thread
  while (1) {
		HAL_Delay(500);
		GPIOC->ODR ^= (1 << 6); // toggle red
  }
}

/*
 * The process for setting external interrupts is:
 * 	1: set the bit in the EXTI_IMR register that corresponding to the interrupt you want to trigger.
 * 	2: Set the corresponding bit in the EXTI_RTSR register to make the interrupt trigger on the rising edge
 * 		or set it in the EXTI_FTSR to make the interrupt trigger on the falling edge.
 * 	3: Configure the NVIC to select an external interrupt.
 */
void Task_Manager(void){

	// Set the Interrupt Mask Register (IMR) to enable external interrupts (instructions on page 219 in the peripheral manual and example code on page 947)
	EXTI->IMR |= 1;

	// For the same bit position, also set the "Rising Trigger Selection Register" (RTSR) or the "Falling Trigger Selection Register (FTSR) to specify the interrupt activates on the rising or falling edge of the signal.
	EXTI->RTSR |= 1; // Rising edge
	// EXTI->FTSR |= 1; // Falling edge

	// Enable the clock for the SYSCFG peripheral by setting the first bit in the reset register 2 (RCC_APB2RSTR) (peripheral manual page 117).
	RCC->APB2RSTR |= 1;

	// Configure the external interrupt configuration register 1 (SYSCFG1) to clear the bottom three bits of PA in order to 'multiplex' the push-button into EXTI0 (pages 169 and 170).
	SYSCFG->EXTICR[0] &= ~(7); //

	// Then configure the NVIC to select an external interrupt.
	NVIC_EnableIRQ(EXTI0_1_IRQn); // Enable Interrupt on EXTIO_1 (line 81 in the stm32...xb.h file)

	// Set the SysTick (blue LED) to have a higher priority than the push-button (EXTI0).
	NVIC_SetPriority(SysTick_IRQn, 0);
	NVIC_SetPriority(EXTI0_1_IRQn, 1);

	// For the last part of the lab, change the priority of SysTick to be lower than that of the push-button
	NVIC_SetPriority(SysTick_IRQn, 2);

	// Un-comment to switch again.
	//NVIC_SetPriority(EXTI0_1_IRQn, 3);
}

/*
* Handler for the push-button interrupt.
* It will toggle the green (PC8) and orange (PC9) LEDs.
*
*
* The "EXTernal Interrupt and events controller (EXTI) allows us to 'multiplex' non-peripheral
* wires so they cause interrupts. In this case, the push button will cause the interrupt.
*/
void EXTI0_1_IRQHandler(void) {

	// Check-off one
	GPIOC->ODR ^= (1 << 8); // green
	GPIOC->ODR ^= (1 << 9); // orange

	// Clear the pending bit to prevent the same interrupt from being re-triggered.
	EXTI->PR |= 1; // Clears the pending bit so execution can leave the interrupt.
	//EXTI->PR &= ~1; // gets execution stuck in the interrupt.

	// Un-comment this to do check-off two.
	//Checkoff_Two();
}


/*
 * Adds a delay to make the external interrupt take longer.
 * With the delay present, you can see the relative priority rankings of the three tasks
 * because all tasks with less priority than the external one (push-button) will be halted.
 */
void Checkoff_Two(void) {

	// Add a delay. This delay does not use the hardware abstraction layer.
	volatile uint32_t accumulator = 0;
	while(accumulator < 1500000)
		accumulator++;

	// Toggle the LEDs again
	GPIOC->ODR ^= (1 << 8); // green
	GPIOC->ODR ^= (1 << 9); // orange
}


void Register_Setup(void) {

	// Enable the GPIOA (for the push button) and GPIOC (for the LEDs) peripheral clocks.
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

	// For the LEDs, set MODER pins to general purpose output mode (01).

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

	// For the push-button, set the MODER pins to input mode (00).
	GPIOA->MODER &= ~(3); // 00

	// For the LEDs, set the OTYPER register pins to push-pull (0).
	GPIOC->OTYPER &= ~(1 << 9); // Green
	GPIOC->OTYPER &= ~(1 << 6); // Red
	GPIOC->OTYPER &= ~(1 << 7); // Blue
	GPIOC->OTYPER &= ~(1 << 8); // Orange

	// For the LEDs AND the push-button, set the OSPEEDR register to 'low speed' (x0).
	GPIOC->OSPEEDR &= ~(1 << 18); // Green
	GPIOC->OSPEEDR &= ~(1 << 12); // Red
	GPIOC->OSPEEDR &= ~(1 << 14); // Blue
	GPIOC->OSPEEDR &= ~(1 << 16); // Orange
	GPIOA->OSPEEDR &= ~(1); // Push-button

	// For the LEDs, set the PUPDR register to 'No pull-up, pull-down' (00).
	GPIOC->PUPDR &= ~(3 << 18); // Green
	GPIOC->PUPDR &= ~(3 << 12); // Red
	GPIOC->PUPDR &= ~(3 << 14); // Blue
	GPIOC->PUPDR &= ~(3 << 16); // Orange

	// For the push button, set the PUPDR register to 'pull-down' (10).
	GPIOA->PUPDR |= 2; // 1
	GPIOA->PUPDR &= ~(1); // 0

	// Set initial (high or low) values for the LEDs.
	GPIOC->ODR |= (1 << 9); // Green
	GPIOC->ODR |= (1 << 6); // Red
	GPIOC->ODR |= (1 << 7); // Blue
	GPIOC->ODR |= (1 << 8); // Orange
}

/* ____________________________________  System setup stuff _____________________________________________________ */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
    Error_Handler();
  }
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
