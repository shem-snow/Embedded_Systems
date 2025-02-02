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

	Definitions because I keep forgetting:

		LED Mappings:
		uint32_t RED = 1 << 6;
		uint32_t BLUE = 1 << 7;
		uint32_t ORANGE = 1 << 8;
		uint32_t GREEN = 1 << 9;

		typedef struct {
		uint32_t Pin;
		uint32_t Mode;
		uint32_t Pull;
		uint32_t Speed;
		uint32_t Alternate; 
		} GPIO_InitTypeDef;


		typedef struct {
		__IO uint32_t MODER;
		__IO uint32_t OTYPER;
		__IO uint32_t OSPEEDR;
		__IO uint32_t PUPDR;
		__IO uint32_t IDR;
		__IO uint32_t ODR;
		__IO uint32_t BSRR;
		__IO uint32_t LCKR;
		__IO uint32_t AFR[2];
		__IO uint32_t BRR;
		} GPIO_TypeDef;

		{
  		GPIO_PIN_RESET = 0U,
  		GPIO_PIN_SET
		}GPIO_PinState;

	Even though it would be better to create masks then call these funcitons, the class still decided to edit only one pin at a time.

*/
#include <stdint.h>
#include <stm32f0xx_hal.h>
#include <stm32f0xx_hal_gpio.h>

void HAL_RCC_GPIOX_CLK_Enable(char GPIOx) {
	switch (GPIOx) {
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
Enable the SYSCFG clock.
*/
void HAL_RCC_SYSCFG_CLK_Enable()
{
    RCC -> APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
    RCC -> APB2RSTR |= RCC_APB2ENR_SYSCFGCOMPEN;
}

/*
Initialize the GPIO peripheral's pin according to the type def.
*/
void HAL_GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef *GPIO_Init)
{
    // Find the pin number according to the struct's pin.
    uint16_t pinNumber;
    for (pinNumber = 0; pinNumber < 16; pinNumber++) {
        if (GPIO_Init -> Pin == (1U << pinNumber)) {
            break;
        }
    }

    // Reset GPIOx location.
    GPIOx -> MODER   &= ~(3 << (2 * pinNumber));
    GPIOx -> OTYPER  &= ~(1 << (pinNumber));
    GPIOx -> OSPEEDR &= ~(3 << (2 * pinNumber));
    GPIOx -> PUPDR   &= ~(3 << (2 * pinNumber));
    
    // Active the pin according to how the struct is set up.
    GPIOx -> MODER   |= (GPIO_Init -> Mode) << (2 * pinNumber);
    GPIOx -> OTYPER  |= (GPIO_Init -> Alternate) << (pinNumber);
    GPIOx -> OSPEEDR |= (GPIO_Init -> Speed) << (2 * pinNumber);
    GPIOx -> PUPDR   |= (GPIO_Init -> Pull) << (2 * pinNumber);
}

/*
De-Initialize the pin specified.
*/
void HAL_GPIO_DeInit(GPIO_TypeDef* GPIOx, uint32_t GPIO_Pin)
{
    // Find the pin number according to the struct's pin.
    uint16_t pinNumber;
    for (pinNumber = 0; pinNumber < 16; pinNumber++) {
        if (GPIO_Pin == (1U << pinNumber)) {
            break;
        }
    }

    // Clear the bits at GPIOx location.
    GPIOx -> MODER   &= ~(3 << (2 * pinNumber));
    GPIOx -> OTYPER  &= ~(1 << (pinNumber));
    GPIOx -> OSPEEDR &= ~(3 << (2 * pinNumber));
    GPIOx -> PUPDR   &= ~(3 << (2 * pinNumber));
}

/*
	Returns 1 or 0 to indicate if the pin specified by the parameters is active.
	Return 1 if it is otherwise return 0.
*/
GPIO_PinState GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) { 
    return (GPIOx -> IDR) &= GPIO_Pin; 
}

/*
Turn on or off the pin that is specified.
*/
void HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
    // Write the pins to be active or deactive.
    GPIOx -> ODR |= (PinState ? ~0 : 0) & GPIO_Pin;
} // TODO: you can select which GPIO peripheral to use.

/*
	Toggle the pin that is specified.
*/
void HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    // Flip the value of the current pin.
    GPIOx -> ODR ^= GPIO_Pin;
}

/*
Setup the EXTI to connect to pin 0 as an interupt.
*/
void EXTI_Pin0_Init()
{
    EXTI -> IMR  |= EXTI_IMR_MR0; // Make pin 0 the trigger
    EXTI -> FTSR &= ~EXTI_FTSR_TR0; // Disable falling edge trigger
    EXTI -> RTSR |= EXTI_RTSR_TR0; // Enable rising edge trigger
}

/*
Setup the EXTI1 to connect PA0 to an interupt.
*/
void EXTI1_To_A0_Init()
{
    SYSCFG -> EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0; // Clear the bits for EXTI0
    SYSCFG -> EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA; // Set PA0 to route to EXTI0
}