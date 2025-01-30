/*
	Files that will be useful to understanding the repository are:
		- stm32f072xb.h: Defines data types 
			for example, GPIO_TypeDef is a struct with all the registers in a GPIO peripheral (on line 382).
	
	If you're ever unsure of a data type or what values to assign it, you can most likely find that information in the files:
			./Drivers/STM32F0xx_HAL_Driver/<"Inc" for header files or "Src" for source files>/stm32f0xx_hal_xxxx.<c or h>
			where "xxxx" is whatever peripheral uses the data type you're wondering about.
		A common one is GPIO_PinState which can be GPIO_PIN_RESET or GPIO_PIN_SET.
	
	A common function parameter used here is "GPIO_InitTypeDef *" which is a pointer to a struct intended to hold items which are
	just integers that encode each value/setting for peripheral registers. The intention is that you decide all the settings you
	want to configure a pin with. Put those settings in said struct. then pass the struct into the function which configures the pin.

*/
#include <stdint.h>
#include <stm32f0xx_hal.h>
#include <stm32f0xx_hal_gpio.h>

void HAL_RCC_GPIOX_CLK_Enable(char X){
	switch (X) {
		case 'A':
			RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    		break;
  		case 'B':
			RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    		break;
		case 'C':
			RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    		break;
		case 'D':
			RCC->AHBENR |= RCC_AHBENR_GPIODEN;
    		break;
		case 'E':
			RCC->AHBENR |= RCC_AHBENR_GPIOEEN;
    		break;
		case 'F':
			RCC->AHBENR |= RCC_AHBENR_GPIOFEN;
    		break;
  		default:
	}
}

/*
	Be aware of the assumption that bits are cleared prior to being initialized. We This funciton only ORs to make more bits equal to 1. 
*/
void HAL_GPIO_Init(char X, GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init) {

	switch (X) {
		case 'A': // The Push button
			// These are the settings that make the push button function. GPIO_InitTypeDef PushButton_Settings = {pinnumber_0, GPIO_MODE_INPUT , GPIO_SPEED_FREQ_LOW, GPIO_PULLDOWN};
			// These settings will be passes through the parameter.
			GPIOA->MODER &= (GPIO_Init.Mode << (2 * PushButton_Settings.Pin) );
			GPIOA->OSPEEDR &= (GPIO_Init.Speed << (2 * PushButton_Settings.Pin) );
			GPIOA->PUPDR &= (GPIO_Init.Pull << (2 * PushButton_Settings.Pin) );
    		break;
		case 'B':
			// TODO:
    		break;
		case 'C': // LEDs
			// TODO:
    		break;
		case 'D':
			// TODO:
    		break;
		case 'E':
			// TODO:
    		break;
		case 'F':
			// TODO:
    		break;
  		default:
	}
	
	/* _____________________________ LEDs ________________________________________

		Mappings:
			uint32_t RED = 1 << 6;
			uint32_t BLUE = 1 << 7;
			uint32_t ORANGE = 1 << 8;
			uint32_t GREEN = 1 << 9;
	*/

	// 1: Set the MODER pins PC6 (blue) and PC7 (red) to general purpose output mode (01).

	// Step one: clear the existing bits
	uint32_t mask = 15; // 1111
	mask = mask << (2*6); // 0000..1111..0000
	GPIOC->MODER &= ~mask; // xxxx..0000..xxxx

	// Step two: set the bits to their new value.
	mask = 5; // 0101
	mask = mask << (2 * 6); // 0000..0101..0000
	GPIOC->MODER |= mask; // xxxx..0101..xxxx

	// 2: Set the LED OTYPER pins to push-pull output type (single-bit 0).
	mask = 3 << 6; // 00..11..00
	GPIOC->OTYPER &= ~mask; // xx..00..xx

	// 3: Set the OSPEEDR register to 'low speed' (x0).
	mask = 5 << (2*6); // 00..0101..00
	GPIOC->OSPEEDR &= ~mask; // xx..x0x0..xx

	// 4: Set the PUPDR register to 'No pull-up, pull-down' (00).
	mask = 15 << (2*6); // 00..1111..00
	GPIOC->PUPDR &= ~mask; // xx..0000..xx
}

/*
	Clears (sets to zero) the bits corresponding to "Pin_Number" in all the registers we use to setup that peripheral.
*/
void HAL_GPIO_DeInit(char X, uint32_t Pin_Number) {

	switch (X) {
		case 'A':
			GPIOA->MODER &= ~(3 << (2 * Pin_Number)); // Two bit register.
			GPIOA->OSPEEDR &= ~(3 << (2 * Pin_Number)); // Two bit register.
			GPIOA->PUPDR &= ~(3 << (2 * Pin_Number)); // Two bit register.
    		break;
  		case 'B':
			GPIOB->MODER &= ~(3 << (2 * Pin_Number)); // Two bit register.
			GPIOB->OSPEEDR &= ~(3 << (2 * Pin_Number)); // Two bit register.
			GPIOB->PUPDR &= ~(3 << (2 * Pin_Number)); // Two bit register.
			// TODO:
    		break;
		case 'C': // Currently 
			GPIOC->MODER &= ~(3 << (2 * Pin_Number)); // Two bit register.
			GPIOC->OTYPER &= ~(1 << Pin_Number); // One bit register.
			GPIOC->OSPEEDR &= ~(3 << (2 * Pin_Number)); // Two bit register.
			GPIOC->PUPDR &= ~(3 << (2 * Pin_Number)); // Two bit register.
			GPIOC->ODR &= ~(1 << Pin_Number); // One bit register.
    		break;
		case 'D':
			// TODO: Implement this if you ever want to set registers in GPIOD to zero.
    		break;
		case 'E':
			// TODO: Implement this if you ever want to set registers in GPIOE to zero.
    		break;
		case 'F':
			// TODO: Implement this if you ever want to set registers in GPIOF to zero.
    		break;
  		default:
	}
}

/*
	Specify a Pin number and Peripheral. This function will return the current state of its Output Data Register.
*/
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) { 
    return -1; // TODO: I don't really see a use for this. Maybe I won't implement it. Maybe I will use it for assert statements. 
}

void HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState) {

// TODO: make this function a switch statement

    // 5: Initialize PC6 (blue) high (1) and PC7 (red) to low (0) => set it to 10.

	// Step one: clear xx..00..xx
	uint32_t mask = 3 << 6; // 00..11..00
	GPIOC->ODR &= ~mask; // 11..00..11

	// Step two: set to 10
	mask = 2 << 6; // 00..10..00
	GPIOC->ODR |= mask; // xx..10..xx
}

void HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
    uint32_t mask = (1 << GPIO_Pin); // 00..1..00
	GPIOC->ODR ^= mask; // xx..~(x)..xx
}
