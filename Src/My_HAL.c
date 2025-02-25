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
#include "My_HAL.h"


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
		case 'I': // Interrupts and I2C
			if(number == 1)
				; // TODO:
			else if(number == 2)
				RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
			else
				; // TODO:
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
		case 'U': // USART3
			// TODO: I could use the second (integer) parameter to specify USART other than 3
			RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
			// TODO: else??
  		default:
	}
}

/*
Initialize the GPIO peripheral's pin according to the type def.
*/
void My_HAL_GPIO_Init(GPIO_TypeDef* GPIOx, My_GPIO_InitTypeDef *GPIO_Init) {
	
    // Reset GPIOx location.
    GPIOx->MODER   &= ~(3 << (2 * (GPIO_Init->PinNumber)));
    GPIOx->OTYPER  &= ~(1 << (1 * (GPIO_Init->PinNumber)));
    GPIOx->OSPEEDR &= ~(3 << (2 * (GPIO_Init->PinNumber)));
    GPIOx->PUPDR   &= ~(3 << (2 * (GPIO_Init->PinNumber)));
	GPIOx->AFR[0]  &= 0;
	GPIOx->AFR[1]  &= 0;
    
    // Activate the pin according to how the struct is set up.
    GPIOx->MODER   |= (GPIO_Init->Mode) << (2 * (GPIO_Init->PinNumber));
    GPIOx->OTYPER  |= (GPIO_Init->Otype) << (1 * (GPIO_Init->PinNumber));
	GPIOx->OSPEEDR |= (GPIO_Init->Speed) << (2 * (GPIO_Init->PinNumber));
    GPIOx->PUPDR   |= (GPIO_Init->Pull) << (2 * (GPIO_Init->PinNumber));
}
/*
	Labs 1 and 2 still use the provided version so I'll leave it here for them.
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
*/
void HAL_ALTERNATE_PIN_Init(GPIO_TypeDef* GPIOx, My_GPIO_InitTypeDef *GPIO_Init) {
	int index = (GPIO_Init->PinNumber > 7)? 1 : 0;

	GPIOx->AFR[index] &= ~(15 << (4*(GPIO_Init->PinNumber))); // Clear the current bits
	GPIOx->AFR[index] |= ((GPIO_Init->AlternateFunction) << (4*(GPIO_Init->PinNumber))); // Set the new bits
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
	State is either GPIO_PIN_SET (1) or GPIO_PIN_RESET (0)
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
void Reset_Interrupt(char peripheral) {
	switch(peripheral) {
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


/*
* Initializes LEDs (PC6-9) turned off.
*/
void Init_LEDs(void) {
	
	// Initialize Port C: LEDs and pins
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	
	// Set the MODER to output mode (01)
	GPIOC->MODER &= ~(1<< (2*6 +1) );
	GPIOC->MODER &= ~(1<< (2*7 +1) );
	GPIOC->MODER &= ~(1<< (2*8 +1) );
	GPIOC->MODER &= ~(1<< (2*9 +1) );
	
	GPIOC->MODER |= 1 << 2*6;
	GPIOC->MODER |= 1 << 2*7;
	GPIOC->MODER |= 1 << 2*8;
	GPIOC->MODER |= 1 << 2*9;
	
	// Set output type register to push-pull (0)
	GPIOC->OTYPER &= ~(RED);
	GPIOC->OTYPER &= ~(BLUE);
	GPIOC->OTYPER &= ~(ORANGE);
	GPIOC->OTYPER &= ~(GREEN);
	
	// Set output speed register to low speed (x0)
	GPIOC->OSPEEDR &= ~(1 << 2*6);
	GPIOC->OSPEEDR &= ~(1 << 2*7);
	GPIOC->OSPEEDR &= ~(1 << 2*8);
	GPIOC->OSPEEDR &= ~(1 << 2*9);
	
	// Set the pins to no pull-up, pull-down (00)
	GPIOC->PUPDR &= ~(3<<2*6);
	GPIOC->PUPDR &= ~(3<<2*7);
	GPIOC->PUPDR &= ~(3<<2*8);
	GPIOC->PUPDR &= ~(3<<2*9);
	
	// Initialize each light to be off
	GPIOC->ODR &= ~RED;
	GPIOC->ODR &= ~BLUE;
	GPIOC->ODR &= ~ORANGE;
	GPIOC->ODR &= ~GREEN;
}

/*
	Follows the I2C protocol to enable the gyroscope.
	That's done by transmitting two values:
		1. The address of the "CTRL_REG1" register which controls how the gyroscope works.
		2. The settings we want configured so the gyroscope operates the way we want. 
*/
void Init_Gyroscope(void) {
	
	// Set the parameters for the current transaction (in CR2)
	I2C2->CR2 |= (0x69 << 1); // Slave address for the gyroscope is 0x69. Shift by 1 because the 8th and 0th bits are don't cares.
	I2C2->CR2 |= (2  << 16); // NBYTES = 2
	I2C2->CR2 &= ~(1 << 10); // Write direction
	I2C2->CR2 |= (1 << 13); // Start generation
	
	// Wait until either TXIS (1<<1) or NACKF (1<<4) flags are set
	while(!( (I2C2->ISR) & ( (I2C_ISR_TXIS) | (I2C_ISR_NACKF) ) )) {}

	// Then write the address of the "CTRL_REG1" register into TXDR
	I2C2->TXDR = 0x20;
	
	// Indicate an error by setting the RED LED high.
	if (I2C2->ISR & I2C_ISR_NACKF)
		HAL_GPIO_WritePin(GPIOC, RED, GPIO_PIN_SET); 
		
		
	// Wait until either TXIS (1<<1) or NACKF (1<<4) flags are set
	while( ! ( (I2C2->ISR & I2C_ISR_TXIS) | (I2C2->ISR & I2C_ISR_NACKF) ) ) {}
	
	// Now write the intended value of the control reg 1 into TXDR.
	// The third bit specifies "normal or sleep mode" when set but "power down mode" when not set.
	// The 0th, 1st, and 2nd bits enable the x, y, and z axis respectively.
	I2C2->TXDR = ( (1<<3) | (1<<1) | (1<<0) ); // 0x0B;

	// Indicate an error by setting the RED LED high.
	if (I2C2->ISR & I2C_ISR_NACKF)
		HAL_GPIO_WritePin(GPIOC, RED, GPIO_PIN_SET);

	// Wait for the transmission to complete.
	while(!( (I2C2->ISR) & ( (I2C_ISR_TC) | (I2C_ISR_NACKF)) )) {} // Transfer Complete is the 6th bit.
	
	// Indicate an error by setting the RED LED high.
	if (I2C2->ISR & I2C_ISR_NACKF)
		HAL_GPIO_WritePin(GPIOC, RED, GPIO_PIN_SET);
	
	// Release the I2C BUS.
	// I2C2->CR2 |= I2C_CR2_STOP; // 1<<14
}

/*
	Our gyroscope uses 8-bit registers to represent 16-bit data for orientation on an axis.
	Because of this, we must mask the LSB and MSB together.ADC1_2_EXTERNALTRIG_T1_CC4

	Important addresses are:
		- x_LSB: 0x28
		- x_MSB: 0x29
		- y_LSB: 0x2a
		- y_MSB: 0x2b
		- z_LSB: 0x2c
		- z_MSB: 0x2d

	For each value we want, we must operate in two steps.
	The first is to specify the address of the register we're reading and the second is to read its actual data.
	
	This tells us the physical orientation of the gyroscope in the axis' direction (depending on parameter "c").

	@param c: 
		The way I decided to specify which bit to read is by relating upper and lower case to MSB or LSB respectively.
*/
int8_t Read_Gyroscope_Output(char c) {

	// Figure out what axis is being read.
	int address;
	switch (c) {
	case 'x':
		address = 0x28;
		break;
	case 'X':
		address = 0x29;
		break;
	case 'y':
		address = 0x2a;
		break;
	case 'Y':
		address = 0x2b;
		break;
	// TODO: implement z if you come back here.
	default:
		return -1; // Sentinal value for an invalid input.
	}

	// ________________________________________ Step one: write the address of the register you want to read from to the TXDR ________________________________________

	// Clear the number of bytes (bits 23:16) and slave address (bits 9:0) fields of CR2
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));

	// Set the parameters for the current transaction (in CR2 => CTRL_REG1). First step is to write the address of what you want to read from.
	I2C2->CR2 |= (0xD2 << I2C_CR2_SADD_Pos); // Slave address for the gyroscope.
	I2C2->CR2 |= (0x1 << I2C_CR2_NBYTES_Pos); // bit 16
	I2C2->CR2 &= ~(I2C_CR2_RD_WRN_Msk); // Write direction (1<<10)
	I2C2->CR2 |= (I2C_CR2_START_Msk); // Start generation (1<<13)

	// Wait until either TXIS (1<<1) or NACKF (1<<4) flags are set
	while( ! ( (I2C2->ISR & I2C_ISR_TXIS) | (I2C2->ISR & I2C_ISR_NACKF) ) ) {
		
	}

	// Write the address of which axis we want to read from.
	if ((I2C2->ISR & I2C_ISR_TXIS))
		I2C2->TXDR = address;

	// If the slave did not respond, indicate that by turning the orange LED on.
	if ((I2C2->ISR & I2C_ISR_NACKF))
		HAL_GPIO_WritePin(GPIOC, ORANGE, GPIO_PIN_SET);
	
	// Wait for the transmission to complete.
	while(!( (I2C2->ISR) & ( (I2C_ISR_TC) | (I2C_ISR_NACKF)) )) { // Transfer Complete is the 6th bit.
		
	} 
	
	// Indicate an error by setting the RED LED high.
	if (I2C2->ISR & I2C_ISR_NACKF)
		HAL_GPIO_WritePin(GPIOC, RED, GPIO_PIN_SET);
	

	// ________________________________________ Step two: read from the register and return the value________________________________________
	
	// Set the parameters for the current transaction (in CR2 => CTRL_REG1) to READ
	I2C2->CR2 |= (0xD2 << I2C_CR2_SADD_Pos);   // Slave address = 0x69. 0xD2 is 0x69 << 1
	I2C2->CR2 |= (0x1  << I2C_CR2_NBYTES_Pos); // bit 16
	I2C2->CR2 |= (I2C_CR2_RD_WRN_Msk); // Read direction (1<<10)
	I2C2->CR2 |= (I2C_CR2_START_Msk); // Start generation (1<<13)

	// Wait until either RXNE (1<<2) or NACKF (1<<4) flags are set
	while( ! ( (I2C2->ISR & I2C_ISR_RXNE) | (I2C2->ISR & I2C_ISR_NACKF) ) ) {
		
	}

	// If the slave did not respond, indicate that on the ORANGE LED.
	if ((I2C2->ISR & I2C_ISR_NACKF))
		HAL_GPIO_WritePin(GPIOC, ORANGE, GPIO_PIN_SET);

	// Then save the value you read.
	int8_t ret = I2C2->RXDR;

	// Wait for the transmission to complete.
	while(!( (I2C2->ISR) & ( (I2C_ISR_TC) | (I2C_ISR_NACKF)) )) { // Transfer Complete is the 6th bit.
		
	} 
	
	// Indicate an error by setting the RED LED high.
	if (I2C2->ISR & I2C_ISR_NACKF)
		HAL_GPIO_WritePin(GPIOC, RED, GPIO_PIN_SET);

	return ret;
}
