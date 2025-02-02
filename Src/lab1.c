#include <stm32f0xx_hal.h>
#include <main.h>
#include <assert.h>

int lab1_main(void) {
    HAL_Init(); // Reset of all peripherals, init the Flash and Systick
    SystemClock_Config(); // Configure the system clock

    /* This example uses HAL library calls to control
    the GPIOC peripheral. */
    
    HAL_RCC_GPIOX_CLK_Enable('A'); // Enable the GPIOC clock in the RCC
    HAL_RCC_GPIOX_CLK_Enable('C'); // Enable the GPIOA clock in the RCC
    
    // Set up a configuration struct to pass to the initialization function
    GPIO_InitTypeDef initStrC6 = {GPIO_PIN_6,
                                MODE_OUTPUT,
                                GPIO_NOPULL,
                                GPIO_SPEED_FREQ_LOW,
                                GPIO_MODE_AF_PP};
    GPIO_InitTypeDef initStrC7 = {GPIO_PIN_7,
                                MODE_OUTPUT,
                                GPIO_NOPULL,
                                GPIO_SPEED_FREQ_LOW,
                                GPIO_MODE_AF_PP};
    GPIO_InitTypeDef initStrA0 = {GPIO_PIN_0,
                                GPIO_MODE_INPUT,
                                GPIO_PULLDOWN,
                                GPIO_SPEED_FREQ_LOW};

    HAL_GPIO_Init(GPIOC, &initStrC6); // Initialize pins PC6
    HAL_GPIO_Init(GPIOC, &initStrC7); // Initialize pins PC7
    HAL_GPIO_Init(GPIOA, &initStrA0); // Initialize pins PA0
    // assert(GPIOC->MODER == 0x00050000); 
    // assert(GPIOC->OTYPER == 0x00000300); 
    // assert(GPIOC->OSPEEDR == 0x00050000); 
    // assert(GPIOC->PUPDR == 0x00050000);

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET); // Start PC6 high
    // assert(GPIOC->ODR == 0x00000100);



    // Checkoff_1_2();
    Checkoff_1_3();

    return 1;
}


void Checkoff_1_2() {
    while (1) {
        HAL_Delay(200); // Delay 200ms
        // Toggle the output state of both PC6 and PC7
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
    }
}

/*
    Toggles the output state of both PC6 (blue) and PC7 (red) every time the user button is pressed.
*/ 
void Checkoff_1_3() {
	
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

/*
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)) { // If input signal is set/high
            debouncerCounter |= 0x01; // Set lowest bit of bit-vector
        }
        if (debouncerCounter == 0x7FFFFFFF) {
            // Toggle the output state of both PC6 and PC7
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_7);
        }
    }
*/