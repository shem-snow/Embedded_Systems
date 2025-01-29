#include "main.h"
// #include "assert.h"
#include <stm32f0xx_hal.h> // Ensure HAL is included

void Checkoff2(void);
void Checkoff3(void);

int lab1_main(void) {

    HAL_Init(); // Reset of all peripherals, init the Flash and Systick
    SystemClock_Config(); //Configure the system clock

    // Enable the peripherals for LEDs (GPIOC) and the push button (GPIOA)
    HAL_RCC_GPIOX_CLK_Enable('C');
    HAL_RCC_GPIOX_CLK_Enable('A');

    // Set up a configuration struct to pass to the initialization function for LEDs
    GPIO_InitTypeDef initStr = {GPIO_PIN_6 | GPIO_PIN_7, GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_LOW, GPIO_NOPULL};

    // Test assert to verify you're setting the right values.
    // assert_param(GPIO->MODER == 0x123456); // TODO: Make so asserts actually do something useful.
    
    // Initialize the red and blue LEDs
    HAL_GPIO_Init(GPIOC, &initStr); 

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET); // Start PC6 (red) high

    // Do the lab
    //Checkoff2();
    Checkoff3();

    return 0;
}


void Checkoff2() {
    while (1) {
        HAL_Delay(200); // Delay 200ms
        // Toggle the output state of both PC6 and PC7
        HAL_GPIO_TogglePin(GPIOC, 6);
        HAL_GPIO_TogglePin(GPIOC, 7);
    }
}

/*
    Toggles the output state of both PC6 (blue) and PC7 (red) every time the user button is pressed.
*/ 
void Checkoff3() {
	
    // Create the debouncer to keep track of the button's state.
	uint32_t debouncer = 0;
    // Create a mask to match the two LEDs in the Output Data Register (ODR).
	uint32_t mask = (3 << 6); // 00..11..00

    // Run
	while (1) {

		// Shift the debouncer left at every loop iteration
		debouncer = debouncer << 1;

		// If the user button is pressed, set the LSB to 1.
		if(GPIOA->IDR & 1)
			debouncer |= 1;
        // Otherwise the LSB remains zero because the register was just shifted.

		// Do nothing when the button is 'steady high' or 'steady low'.
		if( (debouncer == 0xFFFFFFFF) || (debouncer == 0) )
			continue;

		// When transitioning from 'steady low' to 'steady high', toggle the LEDs.
		else if(debouncer == 0x1) {
			GPIOC->ODR ^= mask; // xx..~(xx)..xx
		}
	}
}
