/*
	There are two strategies to write code for our board. First is to look in the datasheet and directly edit register values which 
	are memory-mapped to peripheral controls.
	
	Second is to look in this file:
		./Drivers/STM32F0xx_HAL_Driver/<"Inc" for header files or "Src" for source files>/stm32f0xx_hal_xxxx.<c or h>
			where "xxxx" is whatever peripheral uses the data type you're wondering about.
		and focus on the peripheral registers you want to edit.ADC1_2_EXTERNALTRIG_T1_CC4

	The file:
		- stm32f072xb.h: Defines data types 
		contains all the pre-defined values that you might set to the registers in the device header file first mentioned.
	
	
	An additional thing to note: A common function parameter used here is "GPIO_InitTypeDef *" which is a pointer to a struct intended to hole
	items which are just integers that encode each value/setting for peripheral registers. The intention is that you decide all the settings.
	you want to configure a pin with. Put those settings in said struct. then pass the struct into the function which configures the pin.

	Definitions I keep forgetting so I left them here for easy reference:

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

*/
#include <stdint.h>
#include <stm32f0xx_hal.h>
#include <stm32f0xx_hal_gpio.h>

// Local prototypes that will not be used by other C files.
int Get_GPIO_Pin_Number(GPIO_InitTypeDef *GPIO_Init);

/*
	This would have been more readable if I passed in a string instead of the two parameters... Eh.

	For now:
	@param char GPIOx - Pass in a character identifier. GPIOs are A-F.
	@param uint32_t number - Second identifier.
*/
void HAL_RCC_CLK_Enable(char GPIOx, uint32_t number) {
	switch (GPIOx) {
		case 'A': // GPIOA
			RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    		break;
  		case 'B': // GPIOB
			RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    		break;
		case 'C': // GPIOC
			RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    		break;
		case 'D': // GPIOD
			RCC->AHBENR |= RCC_AHBENR_GPIODEN;
    		break;
		case 'E': // GPIOE
			RCC->AHBENR |= RCC_AHBENR_GPIOEEN;
    		break;
		case 'F': // GPIOF
			RCC->AHBENR |= RCC_AHBENR_GPIOFEN;
    		break;
		case 'I': // Interrupts
			// TODO: At some point when I want to use the NVIC for more interrupts than just the button, I'll need to create this.
			// For now, I'll just enable the clocks manually as I need to.
			break;
		case 'S': // SYSCFG
			RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
			RCC->APB2RSTR |= RCC_APB2ENR_SYSCFGCOMPEN;
    		break;
		case 'T': // Timers
			if(number == 1)
				; // TODO: Timer 1
			else if(number == 2)
				RCC->APB1ENR |=  RCC_APB1ENR_TIM2EN; // Timer 2
			else if(number == 3) 
				RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // Timer 3
			break;
			// TODO: else??
  		default:
	}
}

/*
Initialize the GPIO peripheral's pin according to the type def.
*/
void HAL_GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef *GPIO_Init) {
    // Find the pin number according to the struct's pin.
	uint16_t pinNumber = Get_GPIO_Pin_Number(GPIO_Init);
    

    // Reset GPIOx location.
    GPIOx->MODER   &= ~(3 << (2 * pinNumber));
    GPIOx->OTYPER  &= ~(1 << (pinNumber));
    GPIOx->OSPEEDR &= ~(3 << (2 * pinNumber));
    GPIOx->PUPDR   &= ~(3 << (2 * pinNumber));
	GPIOx->AFR[0]  &= 0;
	GPIOx->AFR[1]  &= 0;
    
    // Active the pin according to how the struct is set up.
    GPIOx->MODER   |= (GPIO_Init->Mode) << (2 * pinNumber);
    GPIOx->OTYPER  |= (0 << pinNumber); // The pre-defined struct doesn't define push-pull (0) or open-drain (1) modes. But there is an option to specify them (lines 116 and 117 in the HAL). I default to push-pull here.
	GPIOx->OSPEEDR |= (GPIO_Init->Speed) << (2 * pinNumber);
    GPIOx->PUPDR   |= (GPIO_Init->Pull) << (2 * pinNumber);
}

/*
	Configures a specified Pin to the specified Alternate Function mode.
	The "AFR_high" parameter works as a Boolean to identify if the high or low Alternate Function register will be edited.
*/
void HAL_ALTERNATE_PIN_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef *GPIO_Init, uint8_t AFR_high) {
	// Assign the alternate function.
	if(AFR_high)
		GPIOx->AFR[1]  |= (GPIO_Init->Alternate);
	else {
		GPIOx->AFR[0]  |= (GPIO_Init->Alternate);
	}
		
}

int Get_GPIO_Pin_Number(GPIO_InitTypeDef *GPIO_Init){
	uint16_t pinNumber;
	for(pinNumber = 0; pinNumber < 16; pinNumber++) {
        if (GPIO_Init->Pin == (1U << pinNumber)) {
            return pinNumber;
        }
    }

	return -1;
}

/*
	Set bits to zero in the specified pin.
*/
void HAL_GPIO_DeInit(GPIO_TypeDef* GPIOx, uint32_t GPIO_Pin) {
    // Find the pin number according to the struct's pin.
    uint16_t pinNumber;
    for(pinNumber = 0; pinNumber < 16; pinNumber++) {
        if (GPIO_Pin == (1U << pinNumber)) {
            break;
        }
    }

    // Clear the bits at GPIOx location.
    GPIOx->MODER   &= ~(3 << (2 * pinNumber));
    GPIOx->OTYPER  &= ~(1 << (pinNumber));
    GPIOx->OSPEEDR &= ~(3 << (2 * pinNumber));
    GPIOx->PUPDR   &= ~(3 << (2 * pinNumber));
}

/*
	Returns 1 or 0 to indicate if the pin specified by the parameters is active.
	Return 1 if it is otherwise return 0.
*/
GPIO_PinState GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) { 
    return (GPIOx->IDR) &= GPIO_Pin; 
}

/*
Turn on or off the pin that is specified.
*/
void HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState) {
    // Write the pins to be active or deactive.
    GPIOx->ODR |= (PinState ? ~0 : 0) & GPIO_Pin;
} 

/*
	Toggle the pin that is specified.
*/
void HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
    // Flip the value of the current pin.
    GPIOx->ODR ^= GPIO_Pin;
}


/*
 * The process for setting external interrupts is:
 * 	1: set the bit in the EXTI_IMR register that corresponding to the interrupt you want to trigger.
 * 	2: Set the corresponding bit in the EXTI_RTSR register to make the interrupt trigger on the rising edge
 * 		or set it in the EXTI_FTSR to make the interrupt trigger on the falling edge.
 * 	3: Configure the NVIC to select the specific external interrupt.
 */
void Reset_Interrupt(char pin) {
	switch(pin) {
		case 'A':
			// Set the Interrupt Mask Register (IMR) to enable external interrupts (instructions on page 219 in the peripheral manual and example code on page 947)
			EXTI->IMR  |= EXTI_IMR_MR0; // |= 1

			// For the same bit position, also set the "Rising Trigger Selection Register" (RTSR) or the "Falling Trigger Selection Register (FTSR) to specify the interrupt activates on the rising or falling edge of the signal.
			EXTI -> RTSR |= EXTI_RTSR_TR0; // Rising edge
			EXTI -> FTSR &= ~EXTI_FTSR_TR0; // Falling edge

			// Enabling the clock for the SYSCFG peripheral (done in an earlier step) is not enough to trigger an interrupt. We must also reset the peripheral with the "APB peripheral reset register 2" (RCC_APB2RSTR) (peripheral manual page 117).
			RCC->APB2RSTR |= RCC_APB2RSTR_SYSCFGRST;

			// Configure the bottom three bits in the external interrupt configuration register 1 (SYSCFG1) to 'multiplex' which pin should trigger interrupts on EXTI0 (pages 169 and 170).
			SYSCFG -> EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0; // Clear the bits to select the push-button.
			SYSCFG -> EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA; // Set PA0 to route to EXTI0

			// assert(EXTI -> IMR  == 1);
			// assert(EXTI -> FTSR == 0);
			// assert(EXTI -> RTSR == 1);
			// assert(SYSCFG -> EXTICR[0] == 0);

			// Configure the NVIC to select the external interrupt.
			NVIC_EnableIRQ(EXTI0_1_IRQn); // Enable Interrupt on EXTIO_1 (line 81 in the stm32...xb.h file)
			break;
		case 'B':
			// TODO:
			break;
		case 'C':
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
}