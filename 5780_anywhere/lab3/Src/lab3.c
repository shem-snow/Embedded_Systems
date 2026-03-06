#include "main.h"
#include "stm32f0xx_hal.h"
#include "My_HAL.h"
#include "System_Setup.h"

void Timer3_Setup(void);
void Timer2_Setup(void);
void LED_Setup(void);
void SystemClock_Config(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {

  // Reset of all peripherals, Initializes the Flash interface and the Systick.
  HAL_Init();
  SystemClock_Config();

  // Enable the peripherals we will use (reference the "reset and clock control" section of the peripheral manual).
  HAL_RCC_CLK_Enable('T', 2); // Timer 2
  HAL_RCC_CLK_Enable('T', 3); // Timer 3
  HAL_RCC_CLK_Enable('C', 0); // GPIOC for LEDs

  LED_Setup();

  // Timer 2 toggles green and orange.
  Timer2_Setup();

  // Timer 3 controls the brightness of red and blue.
  Timer3_Setup();

  // Enable the timer2 interrupt in the NVIC.
  NVIC_EnableIRQ(TIM2_IRQn);
  NVIC_SetPriority(TIM2_IRQn, 1); // A priority is set by default if you don't set one.

  // Main loop
  while (1) {
  }
}

/*
 * Configures and initializes the LEDs.
 * The green and orange LEDs will be toggled by an interrupt so they will be in input mode. Ready to have their value driven.
 * The red and blue LEDs will have their brightness controlled by PWM so put them in alternate function mode.
 */
void LED_Setup(void) {
	
	// pin 6 (red)
	My_GPIO_InitTypeDef init_PC6 = {
		6,
		GPIO_MODE_AF,
		GPIO_Otype_Push_Pull,
		GPIO_SPEED_FREQ_LOW,
		GPIO_PULLDOWN,
		0, // uint32_t InData;
		0, // uint32_t OutData;
		0, // uint32_t PortSetReset;
		0, // uint32_t PortLock;
		AF0,
		0 // uint32_t BitReset;
	};
	My_HAL_GPIO_Init(GPIOC, &init_PC6, 1);

	// pin 7 (blue)
	My_GPIO_InitTypeDef init_PC7 = {
		7,
		GPIO_MODE_AF,
		GPIO_Otype_Push_Pull,
		GPIO_SPEED_FREQ_LOW,
		GPIO_PULLDOWN,
		0, // uint32_t InData;
		0, // uint32_t OutData;
		0, // uint32_t PortSetReset;
		0, // uint32_t PortLock;
		AF0,
		0 // uint32_t BitReset;
	};
	My_HAL_GPIO_Init(GPIOC, &init_PC7, 1);

	// pin 8 (orange)
	My_GPIO_InitTypeDef init_PC8 = {
		8,
		GPIO_MODE_Output,
		GPIO_Otype_Push_Pull,
		GPIO_SPEED_FREQ_LOW,
		GPIO_PULLDOWN,
		0, // uint32_t InData;
		0, // uint32_t OutData;
		0, // uint32_t PortSetReset;
		0, // uint32_t PortLock;
		0,
		0 // uint32_t BitReset;
	};
	My_HAL_GPIO_Init(GPIOC, &init_PC8, 0);

	// pin 9 (green)
	My_GPIO_InitTypeDef init_PC9 = {
		9,
		GPIO_MODE_Output,
		GPIO_Otype_Push_Pull,
		GPIO_SPEED_FREQ_LOW,
		GPIO_PULLDOWN,
		0, // uint32_t InData;
		0, // uint32_t OutData;
		0, // uint32_t PortSetReset;
		0, // uint32_t PortLock;
		0,
		0 // uint32_t BitReset;
	};
	My_HAL_GPIO_Init(GPIOC, &init_PC9, 0);

	// Initialize green high and orange low
	HAL_GPIO_WritePin(GPIOC, GREEN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, ORANGE, GPIO_PIN_RESET);
}

/*
 * Timer 2 triggers an interrupt that toggles the orange and green LEDs at a frequency of 4 Hz.
 */
void Timer2_Setup(void) {

	// Set the PSC and ARR for timer 2 such that the clock frequencies is 4 Hz
	TIM2->PSC = 7999;
	TIM2->ARR = 250;

	// Configure timer2 to generate an interrupt on the Update EVent (UEV). I.E. Enable the update interrupt.
	// The Direct-memory-access Interrupt Event Register (DIER) enables direct memory access for a given timer.
	TIM2->DIER |= 1;

	// Start timer 2 but only after all of its settings are configured.
	TIM2->CR1 |= 1; // TIM_CR1_ARPE | TIM_CR1_CEN; 
}

/*
 * There are three capture/compare registers:
 * 	1. CCRx - Contains the data (its use depends on the mode).
 * 	2. CCMR - Selects the mode. Can be a variety but the three main are:
 * 		i. input capture: helps measure precise timing.
 * 		ii. output compare: modifies the output of a GPIO when a timer's count matches the data register.
 * 		iii. PWM: a special form of output mode that's useful for implementing PWM.
 * 	3. CCER - Enables/Disables the capture/compare channels.
 */
void Timer3_Setup(void) {

	// ___________________ Derive the time base ___________________

	// Set the PSC and ARR for timer 3 such that the clock frequencies is 800 Hz
	// One of the exercises says to make the data register for both channels 20% of the ARR.
	TIM3->PSC = 499;
	TIM3->ARR = 20;

	// Set CCRX to determine the duty cycle.

	// Channel 1, which controls the RED LED, uses PWM mode 2 (111).
	// Therefore setting the CCR1 to 0 makes it brightest, 20 is dimmest, and 21+ is off.
	TIM3->CCR1 = 19; // This has to be down-counting?? Yes because timers count down not up.

	// And channel 2, Which controls the BLUE LED, uses PWM mode 1 (110).
	// Therefore 0 is off and increasing up to 20 increases brightness. Having a duty cycle > 100% doesn't make it any brighter.
	TIM3->CCR2 = 1; // Definitely down-counting. It is a timer after all.


	// ___________________ Configure the Mode register to PWM mode. ___________________

	// Timer 3 has different 'channels' that can each be selected as the alternate function for different LEDs. This is how we will control the LEDs.
	// Channel 1 - PC6 (red)
	// Channel 2 - PC7 (blue)
	// Channel 3 - PC8 (orange)
	// Channel 4 - PC9 (green)
	// CCMR1 manages channels 1 and 2. CCMR2 manages channels 3 and 4.

	// PWM is just a special form of output mode. So set the channel directions to out (00).
	TIM3->CCMR1 &= ~(3); // channel 1
	TIM3->CCMR1 &= ~(3 << 8); // channel 2

	// Set channel 1 to use PWM mode 2 (111). PC6 (RED LED) is only on channel 1.
	TIM3->CCMR1 |= (7<<4);

	// Set channel 2 to use PWM mode 1 (110). PC7 (BLUE LED) is only on channel 2.
	TIM3->CCMR1 |= (3<<13); // 11x
	TIM3->CCMR1 &= ~(1<<12); // xx0

	// Enable the output compare pre-load for both channels (bits 11 and 3). This makes so
	TIM3->CCMR1 |= (1<<11);
	TIM3->CCMR1 |= (1<<3);

	// ___________________ Configure the Capture/Compare Enable Register ___________________

	// Set the output enable bits for channels 1 (bit 0) and 2 (bit 4).
	TIM3->CCER |= 17; // 10001

	// Configure timer2 to generate an interrupt on the Update EVent (UEV). I.E. Enable the update interrupt.
	//TIM3->DIER |= 1; // Not needed. Functions the same either way.

	// Configure timer 3 to start
	TIM3->CR1 |= 1;
}

/*
* Interrupt handler for timer 2. It will toggle the green and orange LEDs.
*/
void TIM2_IRQHandler(void) {

	// Toggle between the green (PC8) and orange (PC9) LEDs
	HAL_GPIO_TogglePin(GPIOC, GREEN);
	HAL_GPIO_TogglePin(GPIOC, ORANGE);

	// Clear the pending flag for the interrupt status register
	TIM2->SR &= ~1; // "Set" (turned to 1) by event. "Cleared" (turned to 0) by software (right here!).
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
