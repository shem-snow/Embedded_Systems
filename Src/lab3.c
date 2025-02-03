/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This lab introduced the concepts of timers and Pulse Width Modulation (PWM).
  *
  * Since each of the LEDs on the STM32 board can be connected to a timer as an alternate function,
  * their brightness can be controlled.
  *
  ******************************************************************************
  */

#include <stm32f0xx_hal.h>
#include <main.h>
#include <assert.h>


// Local prototypes
void SystemClock_Config(void);
void LED_Setup(void);
void Timer2_Setup(void);
void Timer3_Setup(void);
void Play_Around_with_Numbers(void);


/**
  * @brief
  */
int lab3_main(void) {

  // Reset of all peripherals, Initializes the Flash interface and the Systick.
  HAL_Init();

  // Enable the peripherals we will use (reference the "reset and clock control" section of the peripheral manual).
  HAL_RCC_CLK_Enable('T', 2);// Timer 2
  HAL_RCC_CLK_Enable('T', 3); // Timer 3
  HAL_RCC_CLK_Enable('C', 0); // LEDs

  LED_Setup();

  // Timer 2 toggles green and orange.
  Timer2_Setup();

  // Timer 3 controls the brightness of red and blue.
  Timer3_Setup();

  // Configure the system clock.
  SystemClock_Config();

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

	// The orange (PC8) and green (PC9) LEDs will be in input mode (01).
	GPIO_InitTypeDef init_PC8 = {GPIO_PIN_8,
                                MODE_OUTPUT,
                                GPIO_NOPULL,
                                GPIO_SPEED_FREQ_LOW,
                                GPIO_MODE_AF_PP};
    GPIO_InitTypeDef init_PC9 = {GPIO_PIN_9,
                                MODE_OUTPUT,
                                GPIO_NOPULL,
                                GPIO_SPEED_FREQ_LOW,
                                GPIO_MODE_AF_PP};
	HAL_GPIO_Init(GPIOC, &init_PC8); // orange
	HAL_GPIO_Init(GPIOC, &init_PC9); // green

	// Initialize green high and orange low
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
	HAL_GPIO_DeInit(GPIOC, GPIO_PIN_9);

	// The red (PC6) and blue (PC7) LEDs will be in Alternate function mode (10).
	GPIO_InitTypeDef init_PC6 = {GPIO_PIN_6,
                                MODE_OUTPUT,
                                GPIO_NOPULL,
                                GPIO_SPEED_FREQ_LOW,
                                GPIO_MODE_AF_PP};
    GPIO_InitTypeDef init_PC7 = {GPIO_PIN_7,
                                MODE_OUTPUT,
                                GPIO_NOPULL,
                                GPIO_SPEED_FREQ_LOW,
                                GPIO_MODE_AF_PP};
	HAL_GPIO_Init(GPIOC, &init_PC6); // orange
	HAL_GPIO_Init(GPIOC, &init_PC7); // green

	// Configure the alternate function register for port C (LEDs)
	HAL_ALTERNATE_PIN_Init(GPIOC, &init_PC7, 0); // Passing in either struct will do.
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
	// TIM3->DIER |= 1;

	// Start timer 2 but only after all of its settings are configured.
	TIM2->CR1 |= 1;
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
	TIM3->PSC = 499;
	TIM3->ARR = 20;

	// The duty cycle can be changed in two ways:
	//   1. change the frequency (adjust the ratio of PSC and ARR)
	//   2. change the data register (CRRx) to change the logic value at a different time.
	// As the duty cycle changes, the LED's brightness also changes.

	// ___________________ Configuring the Data registers ___________________

	// One of the exercises says to make the data register for both channels 20% of the ARR. This makes both lights bright with no toggle.
	TIM3->CCR1 = 4;
	TIM3->CCR2 = 4;


	// In PWM mode 1, increasing CCRX should dim the lights
	// PWM should be the opposite.

	// Select/comment the following values to observe the red LED which is in PWM mode 2 (keep CCR2 as 4).
	//TIM3->CCR1 = 25; // off
	//TIM3->CCR1 = 20; // dim
	//TIM3->CCR1 = 12; // bright
	//TIM3->CCR1 = 4; // even brighter.

	// Do the same for the blue LED which is in PWM mode 1 (keep CCR1 as 4).
	//TIM3->CCR2 = 0.999999; // off for anything less than 1
	//TIM3->CCR2 = 1; // dim
	//TIM3->CCR2 = 5; // bright.

	// The chosen frequency does not provide a lot of room to for controlling brightness.
	// Let's pick a different one.
	Play_Around_with_Numbers();


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

	// Set channel 1 to use PWM mode 2 (111).
	TIM3->CCMR1 |= (7<<4);

	// Set channel 2 to use PWM mode 1 (110)
	TIM3->CCMR1 |= (3<<13); // 11x
	TIM3->CCMR1 &= ~(1<<12); // xx0

	// Enable the output compare pre-load for both channels (bits 11 and 3).
	TIM3->CCMR1 |= (1<<11);
	TIM3->CCMR1 |= (1<<3);

	// ___________________ Configure the Capture/Compare Enable Register ___________________

	// Set the output enable bits for channels 1 (bit 0) and 2 (bit 4).
	TIM3->CCER |= 17; // 10001

	// Configure timer 3 to start
	TIM3->CR1 |= 1;
}

/*
 * Tests a variety of PSC, ARR, and CCRx values to control the brightness and toggling of LEDs.
 * Both CCR1 and CCR2 have an effect on both LEDs. It can be challenging to get the brightness you want.
 */
void Play_Around_with_Numbers(void) {

	// New frequency
	TIM2->PSC = 7999;
	TIM2->ARR = 250;




	// Denote the red and blue lights as states. D for dim. T for toggle, 0 for off. 1 for on.

	// Red, blue

	// 00

	// 0d

	// 01
	TIM3->CCR2 = 155;
	TIM3->CCR1 = 31;

	// d0

	// dd

	// d1
	//TIM3->CCR2 = 499;
	//TIM3->CCR1 = 20;

	// 10

	// 1d
	//TIM3->CCR2 = 1;
	//TIM3->CCR1 = 1;

	// 11
	//TIM3->CCR2 = 25;
	//TIM3->CCR2 = 5;
}

/*
* Interrupt handler for timer 2. It will toggle the green and orange LEDs.
*/
void TIM2_IRQHandler(void) {

	// Toggle between the green (PC8) and orange (PC9) LEDs
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);

	// Clear the pending flag for the interrupt status register
	TIM2->SR ^= 1;
}
