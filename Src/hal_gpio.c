#include <stdint.h>
#include <stm32f0xx_hal.h>
#include <stm32f0xx_hal_gpio.h>

void HAL_RCC_GPIOC_CLK_Enable(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // on lines 449 (RCC_TypeDef struct) and 7869 of the device's header file.
}

void HAL_RCC_GPIOA_CLK_Enable(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // on line 7863
}

void HAL_GPIO_Init(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init) {

	// _____________________________ Push Button ________________________________________

	// 1: Set the MODER pin to input mode (00).
	GPIOA->MODER &= ~(3); // 111100

	// 2: Set the OSPEEDR register to low speed (x0).
	GPIOA->OSPEEDR &= ~1;

	// 3: Set the PUPDR register to pull-down (10).
	GPIOA->PUPDR &= ~3; // xx..00
	GPIOA->PUPDR |= 2; // xx..10
	
	
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
void HAL_GPIO_DeInit(GPIO_TypeDef  *GPIOx, uint32_t GPIO_Pin)
{
}
*/

/*
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    return -1;
}
*/

void HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState) {
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
