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
  * 	- The SysTick toggles the blue LED at another frequency.
  * 	- The EXTI selects an external interrupt (push-button) which toggles the green and orange LEDs.
  *
  ******************************************************************************
  */
#include <stm32f0xx_hal.h>
#include <main.h>
#include <assert.h>

// Local prototypes
void Register_Setup(void);
void Checkoff_2_1(void);
void Checkoff_2_2(void);

// Main
int lab2_main(void) {

    // Reset of all peripherals and initialize the Flash interface and the Systick.
    HAL_Init();
    // Configure the system clock
    SystemClock_Config(); 

    // Initialize the peripheral registers for the LEDs, push-button, and external interrupt handler. 
    Register_Setup();

    // Select which external interrupt the EXTI will 'multiplex' into the NVIC.
    Reset_Interrupt('A'); // PinA is the push-button.

    // Set the priorities of each task how you want them.
    NVIC_SetPriority(EXTI0_1_IRQn, 3); // Button toggles orange and green.
    NVIC_SetPriority(SysTick_IRQn, 1); // SysTick toggles blue.

    // Main toggles the red LED every 500 ms.
    while (1) {
        HAL_Delay(500);
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
    }
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
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8); // green
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9); // orange
}

void Checkoff_2_2(void) {

    // Toggle the LEDs once
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8); // green
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9); // orange

	// Insert a delay (without using the hardware abstraction layer) so we can see the lower priority tasks stop.
	volatile uint32_t accumulator = 0;
	while(accumulator < 9990000)
		accumulator++;

	// Toggle the LEDs again and watch the lower priority tasks resume.
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8); // green
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9); // orange
}

void Register_Setup(void) {

    // Enable the clocks for each peripheral of interest..
    HAL_RCC_GPIOX_CLK_Enable('C'); // GPIOC for LEDs
    HAL_RCC_GPIOX_CLK_Enable('A'); // GPIOA for the push-button
    HAL_RCC_GPIOX_CLK_Enable('S'); // SYSCFG for selecting external interrupts.

    // Set up a configuration struct to pass to the initialization function
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
    GPIO_InitTypeDef init_PA0 = {GPIO_PIN_0,
                                GPIO_MODE_INPUT,
                                GPIO_PULLDOWN,
                                GPIO_SPEED_FREQ_LOW};
    // Initialize the LEDs
    HAL_GPIO_Init(GPIOC, &init_PC6);
    HAL_GPIO_Init(GPIOC, &init_PC7);
    HAL_GPIO_Init(GPIOC, &init_PC8);
    HAL_GPIO_Init(GPIOC, &init_PC9);
    HAL_GPIO_Init(GPIOA, &init_PA0);

    // assert(EXTI -> IMR  == 0);
    // assert(EXTI -> FTSR == 0);
    // assert(EXTI -> RTSR == 0);
}