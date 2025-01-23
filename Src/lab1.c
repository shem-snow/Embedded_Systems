#include "main.h"
#include "assert.h"
#include <stm32f0xx_hal.h> // Ensure HAL is included

void Checkoff2(void);
void Checkoff3(void);

int lab1_main(void) {

    HAL_Init(); // Reset of all peripherals, init the Flash and Systick
    SystemClock_Config(); //Configure the system clock

    // Enable the GPIOC peripheral (for the LEDs)
    HAL_RCC_GPIOC_CLK_Enable();

    // Enable the GPIOA peripheral (for the push button)
    HAL_RCC_GPIOA_CLK_Enable();

    // Set up a configuration struct to pass to the initialization function
    GPIO_InitTypeDef initStr = {GPIO_PIN_6 | GPIO_PIN_7, GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_LOW, GPIO_NOPULL};
    
    // Initialize the red and blue LEDs
    HAL_GPIO_Init(GPIOC, &initStr); 
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET); // Start PC6 (red) high

    // Test assert because the instructions said to.
    assert_param(GPIO->MODER == 0x123456); // TODO: This will only freeze the program when it evaluates to false. For some reason the "assert_failed" method does not trigger.

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


void Checkoff3() {
	// Toggle the output state of both PC6 (blue) and PC7 (red) every time the user button is pressed.
	uint32_t debouncer = 0;
	uint32_t mask = (3 << 6); // 00..11..00
	while (1) {

		// Shift the debouncer left at every loop iteration
		debouncer = debouncer << 1;

		// If the user button is pressed, set the LSB to 1.
		if(GPIOA->IDR & 1)
			debouncer |= 1;

		// Do nothing when the button is 'steady high' or 'steady low'.
		if( (debouncer == 0xFFFFFFFF) || (debouncer == 0) )
			continue;

		// When transitioning from 'steady low' to 'steady high', toggle the LEDs.
		else if(debouncer == 0x1) {
			GPIOC->ODR ^= mask; // xx..~(xx)..xx
		}
	}
}
