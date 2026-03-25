/*
	There are two strategies to write code for our board:
	
	1. First is to look in the datasheet and directly edit register values which are memory-mapped to peripheral controls.
	
	2. Second is to look in this file:
		./Drivers/STM32F0xx_HAL_Driver/<"Inc" for header files or "Src" for source files>/stm32f0xx_hal_xxxx.<c or h>
			where "xxxx" is whatever peripheral uses the data type you're wondering about.
		and focus on the peripheral registers you want to edit.ADC1_2_EXTERNALTRIG_T1_CC4

	The file:
		- stm32f072xb.h (which is located at 6780_ShemSnow/Drivers/CMSIS/Device/ST/STM32F0xx/Include/stm32f072xb.h)
		contains all the pre-defined values that you might set to the registers in the device header file first mentioned.
	
	
	An additional thing to note: A common function parameter used here is "GPIO_InitTypeDef *" which is a pointer to a struct intended to hold
	items which are just integers that encode each value/setting for peripheral registers. The intention is that you decide all the settings.
	you want to configure a pin with. Put those settings in said struct. then pass the struct into the function which configures the pin.

	Some definitions in the predefined HAL that might be good to know when helping students:
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
			if(number == 0)
				RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
			else
				RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // non-zero number => ADC
    		break;
  		case 'B': // GPIOB
			RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    		break;
		case 'C': // GPIOC
			RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    		break;
		case 'D': // GPIOD
			if(number == 0)
				RCC->AHBENR |= RCC_AHBENR_GPIODEN;
			else
				RCC->APB1ENR |= RCC_APB1ENR_DACEN; // non-zero number => DAC
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
			RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN; // This one enables SYSCFG's clock.
			RCC->APB2RSTR |= RCC_APB2ENR_SYSCFGCOMPEN; // This one resets SYSCFG.
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
			if(number == 3) {
				RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
			}
			else
				// TODO: else??
  		default:
			;
	}
}

/*
	Initialize the GPIO peripheral's pin according to the type def.
*/
void My_HAL_GPIO_Init(GPIO_TypeDef* GPIOx, My_GPIO_InitTypeDef *GPIO_Init, int8_t alt_func) {
	
	if(GPIO_Init->Mode < 4) {
		GPIOx->MODER   &= ~(3 << (2 * (GPIO_Init->PinNumber)));
		GPIOx->MODER   |= (GPIO_Init->Mode) << (2 * (GPIO_Init->PinNumber));
	}

	if(GPIO_Init->Otype < 2) {
		GPIOx->OTYPER  &= ~(1 << (1 * (GPIO_Init->PinNumber)));
		GPIOx->OTYPER  |= (GPIO_Init->Otype) << (1 * (GPIO_Init->PinNumber));
	}

	if(GPIO_Init->Speed < 4) {
		GPIOx->OSPEEDR &= ~(3 << (2 * (GPIO_Init->PinNumber)));
		GPIOx->OSPEEDR |= (GPIO_Init->Speed) << (2 * (GPIO_Init->PinNumber));
	}

	if(GPIO_Init->Pull < 4) {
		GPIOx->PUPDR   &= ~(3 << (2 * (GPIO_Init->PinNumber)));
		GPIOx->PUPDR   |= (GPIO_Init->Pull) << (2 * (GPIO_Init->PinNumber));
	}
	
	// Use the alternative function parameter to determine.
	if(alt_func == 0) {
		GPIOx->AFR[0]  &= 0;
		GPIOx->AFR[1]  &= 0;


	}
	else if(alt_func == 1)
		HAL_ALTERNATE_PIN_Init(GPIOx, GPIO_Init);
    
}

/*
	Configures a specified Pin to the Alternate Function mode specified in the initalized GPIO struct.
*/
void HAL_ALTERNATE_PIN_Init(GPIO_TypeDef* GPIOx, My_GPIO_InitTypeDef *GPIO_Init) {
	int index = (GPIO_Init->PinNumber > 7)? 1 : 0; // pins 8-15 are the higher ones (1) and pins 0-7 are the lower ones (0).

	GPIOx->AFR[index] &= ~(15 << (4*(GPIO_Init->PinNumber))); // Clear the current bits
	GPIOx->AFR[index] |= ((GPIO_Init->AlternateFunction) << (4*(GPIO_Init->PinNumber))); // Set the new bits
}

/*
	TODO: I think it would be better to make a function for Mask_Pins
*/
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
void HAL_GPIO_UpdatePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin_Mask, GPIO_PinState PinState) {
    // Write the pins to be active or deactive. It's okay to assign rather than mask here because we are assigning a mask (cleared_position | new_state_mask).
	GPIOx->ODR = (GPIOx->ODR & ~GPIO_Pin_Mask) | (PinState ? GPIO_Pin_Mask : 0); 
}

/*
	Toggle the pin that is specified.
*/
void My_HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) { // TODO: HAL does not make obvious when pin number is literal pin number or when it's a 1 shifted into pin position.
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
			;
	}
}

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
	GPIOC->ODR &= ~(RED);
	GPIOC->ODR &= ~(BLUE);
	GPIOC->ODR &= ~(ORANGE);
	GPIOC->ODR &= ~(GREEN);
}

void Init_Button(void) {
	My_GPIO_InitTypeDef init_PA0 =  {
      0, // uint32_t PinNumber,
      GPIO_MODE_Input, // uint32_t Mode,
      GPIO_Otype_Push_Pull, // uint32_t Otype,
      GPIO_SPEED_FREQ_LOW, // uint32_t Speed,
      GPIO_Pull_down, // uint32_t Pull,
      0, // uint32_t InData,
      0, // uint32_t OutData,
      0, // uint32_t PortSetReset,
      0, // uint32_t PortLock,
      0, // uint32_t AlternateFunction,
      0 // uint32_t PortBitReset,
    };
    My_HAL_GPIO_Init(GPIOA, &init_PA0, 0);
}


void Init_USART(int number) {
	
	// Feed the clock into the USART3 peripheral
    HAL_RCC_CLK_Enable('U', number);

	// In this lab we only want to set the mode and alternate function registers. The others are at default values.
	// Set the discovery board pins (connected to USART3) to alternate function mode (10)
    My_GPIO_InitTypeDef init_PC4 = { // Receiver (relative to the USARD device. Not the discovery board)
		4,
		GPIO_MODE_AF,
		69,
		69,
		69,
		0, // uint32_t InData;
		0, // uint32_t OutData;
		0, // uint32_t PortSetReset;
		0, // uint32_t PortLock;
		AF1,
		0 // uint32_t BitReset;
	};
    My_HAL_GPIO_Init(GPIOC, &init_PC4, 1);

    My_GPIO_InitTypeDef init_PC5 = { // Transmitter (relative to the USARD device. Not the discovery board)
		5,
		GPIO_MODE_AF,
		69,
		69,
		69,
		0, // uint32_t InData;
		0, // uint32_t OutData;
		0, // uint32_t PortSetReset;
		0, // uint32_t PortLock;
		AF1,
		0 // uint32_t BitReset;
	};
    My_HAL_GPIO_Init(GPIOC, &init_PC5, 1);
	
    // Set the Baud rate for communcation to be 115,200 bits/second. The system clock is 8 MHz.
	USART3->BRR = HAL_RCC_GetHCLKFreq() / 115200;
	
	// Enable USART in the control register.
	USART3->CR1 |= 1 << 2; // bit 2 enables the receiver
	USART3->CR1 |= 1 << 3; // bit 3 enables the transmitter.
	USART3->CR1 |= 1 << 0; // bit zero is the general enable bit.
	
	
	// Enable the "receive register not empty interrrupt" and the "transmit register empty interrupt".
	// USART3->CR1 |= 1 << 5;
	// USART3->CR1 |= 1 << 7;
		
	// Enable the interrupt in the NVIC and set its priority.
	// NVIC_EnableIRQ(USART3_4_IRQn);
	// NVIC_SetPriority(USART3_4_IRQn, 2);
}

/*
* Does nothing while the transmit data register is empty. When it's not empty, transmit the character.
*/
void Transmit_Char(char c) {
	while( !(USART3->ISR & (1<<7) ) ) {}
	USART3->TDR = c;
}

/*
* Loops over each character in the "str" array and transmits the character there. 
*/
void Transmit_String(char* str) {
	for(int i = 0; str[i] != '\0'; i++)
		Transmit_Char(str[i]);
}

void Init_I2C2(void) {

	// PB15 in input mode and open-drain output type.
	GPIOB->MODER &= ~(3 << 2*15);
	GPIOB->OTYPER |= (1 << 15);
	
	// PB11 AF1 is I2C2_SDA.
	// Open-drain.
	GPIOB->MODER |= (2 << 2*11); // GPIOB->MODER |= GPIO_MODER_MODER11_1;
	GPIOB->OTYPER |= (1 << 11); // GPIOB->OTYPER |= GPIO_OTYPER_OT_11;
	GPIOB->AFR[1] |= (1 << 4*3); // GPIOB->AFR[1] |= (0x1 << GPIO_AFRH_AFSEL11_Pos);

	// PB13 AF5 is I2C2_CLK.
	// Open-drain.
	GPIOB->MODER |= (2 << 2*13); // GPIOB->MODER |=GPIO_MODER_MODER13_1;
	GPIOB->OTYPER |= (1 << 13); // GPIOB->OTYPER |= GPIO_OTYPER_OT_13;
	GPIOB->AFR[1] |= (5 << 20); // GPIOB->AFR[1] |= (0x5 << GPIO_AFRH_AFSEL13_Pos);

	// Pin PC0 is connected to the Gyroscope's mode select line between SPI and I2C.
	// Output mode and push-pull output type.
	// Initialize high.
	GPIOC->MODER &= ~(2 << 0);
	GPIOC->MODER |= (1 <<0); // GPIOB->MODER |= GPIO_MODER_MODER0_0;
	GPIOC->OTYPER &= ~(1 << 0); // GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_0);
	GPIOC->ODR |= (1 << 0); // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);

	// Setting the 0'th bit high selects I2C. Setting it low selects SPI. 0 means no action (use default which is SPI). 1 means do the other thing.
	// GPIOC->BSRR = (1 << 0);
	GPIOC->ODR |= 1;
	
	// PB14 controls the slave address when in I2C mode.
	// Output mode and push-pull output type.
	// Initialize high.
	GPIOB->MODER &= ~(2 << 2*14);
	GPIOB->MODER |= (1 << 2*14); // GPIOB->MODER |= GPIO_MODER_MODER14_0;
	GPIOB->OTYPER &= ~(1 << 14); // GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_14);
	GPIOB->ODR |= (1 << 14); // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	
	// Configure the I2C2 peripheral to operate at 100 kHz (standard-mode).
	I2C2->TIMINGR |= 1 << 28; // Prescaler = 1  // I2C2->TIMINGR |= (0x1 << I2C_TIMINGR_PRESC_Pos);
	I2C2->TIMINGR |= 0x13 << 0; // SCLL = 0x13  // I2C2->TIMINGR |= (0x13 << I2C_TIMINGR_SCLL_Pos);
	I2C2->TIMINGR |= 0xF << 8; // SCLH = 0xF	// I2C2->TIMINGR |= (0xF << I2C_TIMINGR_SCLH_Pos);
	I2C2->TIMINGR |= 0x2 << 16; // SCADEL = 0x2 // I2C2->TIMINGR |= (0x2 << I2C_TIMINGR_SDADEL_Pos);
	I2C2->TIMINGR |= 0x4 << 20; // SCLDEL = 0x4 // I2C2->TIMINGR |= (0x4 << I2C_TIMINGR_SCLDEL_Pos);

	// Enable the I2C peripheral in CR1
	I2C2->CR1 |= I2C_CR1_PE; // 1;
}


int8_t I2C_Read(uint32_t slave_address, uint32_t payload_address){

	int8_t payload = 0;
	
	// Clear the number of bits (8 bits) and slave address (first 10 bits).
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));

    // Configure the peripheral to write to the slave.
    I2C2->CR2 |= (slave_address << 1); // Slave (gyroscope) address is 0x69. Shift 1 because the 0th and 9th bits are don't cares.
    I2C2->CR2 |= (1 << 16);
    I2C2->CR2 &= ~(1 << 10); // 0 for write, 1 for read (relative to the master device).
    I2C2->CR2 |= I2C_CR2_START; // Start is the 13th bit.

    // Not-Acknowledge status (which means transmission failed) is bit 4. Transmit interrupt status (which means transmission succeeded) is bit 1.
    while(!( (I2C2->ISR) & ( (I2C_ISR_TXIS) | (I2C_ISR_NACKF) ) )) {} 
    // Then write the address of the register we want to communicate with into the TXDR.
    
	if( (I2C2->ISR) & (I2C_ISR_NACKF))
		Error_loop(RED, 1000);
	else
		I2C2->TXDR = payload_address;
    
	// Wait for the transmission to complete.
	while (!(I2C2->ISR & I2C_ISR_TC)) {}

	// configure the peripheral to read.
	I2C2->CR2 &= ~((0xFF << 16) | (0x3FF << 0)); // clear
	I2C2->CR2 |= slave_address << 1; // Shift 1 because the 0th and 9th bits are don't cares.
	I2C2->CR2 |= 1 << 16;
	I2C2->CR2 |= 1 << 10;
	I2C2->CR2 |= 1 << 13; // I2C_CR2_START_Msk and I2C_CR2_START are both equivalent.

	while(!( (I2C2->ISR) & ( (I2C_ISR_RXNE) | (I2C_ISR_NACKF)) )) {}

	if( (I2C2->ISR) & (I2C_ISR_NACKF))
		Error_loop(RED, 1000);
	else
		while (!(I2C2->ISR & I2C_ISR_TC)) {}

	payload = I2C2->RXDR;
	Transmit_String("Read "); Transmit_String(int_to_str(payload)); Transmit_String(" from address "); Transmit_String(int_to_str(payload_address)); Transmit_Char('\n');
	
	return payload;
}

void I2C_Write(uint8_t slave_address, int8_t payload_address, int8_t payload) {
	// Clear the number of bits (8 bits) and slave address (first 10 bits).
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));

    // Configure parameters for the current transaction (CR2).
    I2C2->CR2 |= (slave_address << 1); // Slave (gyroscope) address is 0x69. Shift 1 because the 0th and 9th bits are don't cares.
    I2C2->CR2 |= (2 << 16);
    I2C2->CR2 &= ~(1 << 10); // 0 for write, 1 for read (relative to the master device).
    I2C2->CR2 |= I2C_CR2_START; // Start is the 13th bit.

    // Not-Acknowledge status (which means transmission failed) is bit 4. Transmit interrupt status (which means transmission succeeded) is bit 1.
    while(!( (I2C2->ISR) & ( (I2C_ISR_TXIS) | (I2C_ISR_NACKF) ) )) {} 
    // Then write the address of the register we want to communicate with into the TXDR.
    
	if( (I2C2->ISR) & (I2C_ISR_NACKF) )
		Error_loop(RED, 1000);
	else 
		I2C2->TXDR = payload_address;

	// Wait for communication to be established (master calls out to slave).
	while(!( (I2C2->ISR) & ( (I2C_ISR_TXIS) | (I2C_ISR_NACKF) ) )) {}

	// Then write the address of the register we want to communicate with into the TXDR.
	if( (I2C2->ISR) & (I2C_ISR_NACKF) )
		Error_loop(RED, 100);
	else 
		I2C2->TXDR = payload; 

	// Wait for the transmission to complete.
	while(!( (I2C2->ISR) & (I2C_ISR_TC) )) {} // Transfer Complete is the 6th bit.
}

/*
	TODO: num_bytes doesn't really do what it's supposed to.
	In the meantime, we can only do one byte at a time:
	 	- for reads, set num_bytes = 1
		- for writes, set num_bytes = 2

	Transacts with the gyroscope over I2C happen in two step:
    1. Master (M0) writes the address of the register we want to read from or write to into the TXDR.
    2a. In the case of reads, the slave responds with the value at the address.
    2b. In the case of writes, the master writes again with the new value we want to write at that address.
*/
int8_t gyro_transaction(char direction, uint8_t slave_address, int8_t payload_address, int8_t payload) {		

	// Clear the number of bits (8 bits) and slave address (first 10 bits).
	I2C2->CR2 &= ~( (0xFF << 16) | (0x3FF << 0) );

    // Configure parameters for the current transaction (CR2).
    I2C2->CR2 |= (slave_address << 1); // Slave (gyroscope) address is 0x69. Shift 1 because the 0th and 9th bits are don't cares.
    I2C2->CR2 |= ( (direction=='r'? 1 : 2) << 16);
    I2C2->CR2 &= ~(1 << 10); // 0 for write, 1 for read (relative to the master device).
    I2C2->CR2 |= I2C_CR2_START; // Start is the 13th bit.

    // Not-Acknowledge status (which means transmission failed) is bit 4. Transmit interrupt status (which means transmission succeeded) is bit 1.
    while(!( (I2C2->ISR) & ( (I2C_ISR_TXIS) | (I2C_ISR_NACKF) ) )) {} 
    // Then write the address of the register we want to communicate with into the TXDR.
    
	if( (I2C2->ISR) & (I2C_ISR_TXIS) )
		I2C2->TXDR = payload_address;
	else
		Error_loop(RED, 1000);
		// HAL_GPIO_UpdatePin(GPIOC, RED, GPIO_PIN_SET);
		// Transmit_String("NACK after write-TXIS.\n");
    
    switch(direction) {
        case 'r':

			// Wait for the transmission to complete.
			while(!( (I2C2->ISR) & ( (I2C_ISR_TC) | (I2C_ISR_NACKF)) )) {} // Transfer Complete is the 6th bit.

			if( (I2C2->ISR) & (I2C_ISR_NACKF) )
				Error_loop(ORANGE, 1000);
				// HAL_GPIO_UpdatePin(GPIOC, ORANGE, GPIO_PIN_SET);
				// Transmit_String("NACK after write-TC\n");

            // configure the peripheral to read.
            I2C2->CR2 |= 0x69 << 1; // 0x69 is the gyrocope's address. Shift 1 because the 0th and 9th bits are don't cares.
            I2C2->CR2 |= 1 << 16;
            I2C2->CR2 |= 1 << 10;
            I2C2->CR2 |= 1 << 13; // I2C_CR2_START_Msk and I2C_CR2_START are both equivalent.

            while(!((I2C2->ISR) & ((I2C_ISR_RXNE) | (I2C_ISR_NACKF)))) {}

            // Enter an error loop on failure
            int8_t read_value;
			if ((I2C2->ISR) & (I2C_ISR_RXNE))
				read_value = I2C2->RXDR;
			else
				Error_loop(GREEN, 1000);

            // Wait for the transmission to complete.
            while(!( (I2C2->ISR) & ( (I2C_ISR_TC) | (I2C_ISR_NACKF)) )) {} // Transfer Complete is the 6th bit.

            // Enter an error loop on failure
            if( (I2C2->ISR) & (I2C_ISR_NACKF) )
                Error_loop(BLUE, 700);

			return read_value;

            break;
		
		case 'w':

			// Wait for communication to be established (master calls out to slave).
    		while(!( (I2C2->ISR) & ( (I2C_ISR_TXIS) | (I2C_ISR_NACKF) ) )) {}

    		// Then write the address of the register we want to communicate with into the TXDR.
    		if( (I2C2->ISR) & (I2C_ISR_NACKF) )
        		Error_loop(RED, 100);
    		else 
				I2C2->TXDR = payload; 

            // Wait for the transmission to complete.
            while(!( (I2C2->ISR) & ( (I2C_ISR_TC) | (I2C_ISR_NACKF)) )) {} // Transfer Complete is the 6th bit.

            // Return all ones to indicate success or enter an error loop on failure.
            if( (I2C2->ISR) & (I2C_ISR_NACKF) )
                Error_loop(RED, 100);
            else
				return 0b11111111;
			break;

		default:
			Transmit_String("Invalid direction parameter. Must be 'r' or 'w'.");
    }
    
  return -1; // Just to satisfy the compiler. Execution should never reach here.
}

/*
	Gets you stuck in an infinite loop that toggles an LED periodically to indicate errors.
*/
void Error_loop(int LED_color, int frequency) {
	while(1) {
		HAL_Delay(frequency);
		GPIOC->ODR ^= LED_color;
	}
}

char* int_to_str(int num) {
    // 'static' keeps this memory alive for the life of the program
    // 12 bytes is enough for -2147483648 and a null terminator
    static char buffer[12]; 
    int i = 10;
    int is_negative = 0;

    buffer[11] = '\0'; // Null terminator at the very end

    if (num == 0) {
        buffer[i--] = '0';
    } else {
        if (num < 0) {
            is_negative = 1;
            num = -num;
        }
        while (num > 0 && i > 0) {
            buffer[i--] = (num % 10) + '0';
            num /= 10;
        }
        if (is_negative) buffer[i--] = '-';
    }

    // Return the pointer to where the string actually starts
    return &buffer[i + 1];
}

void Init_ADC(GPIO_TypeDef* GPIOx, uint16_t pin_number) {
	
	GPIOx->MODER |= 3 << pin_number; // Analog mode (11)
	GPIOx->PUPDR &= ~(3 << pin_number); // No pull-up, pull down (00)
	ADC1->CHSELR |= (1 << 10 ); // Configure the pin for ADC conversion on channel 10 (therefore the 10th bit position)
	
	// 8-bit resolution (10)
	ADC1->CFGR1 |= (2 << 3);
	ADC1->CFGR1 &= ~(1 << 3);
	
	// Continuous conversion mode
	ADC1->CFGR1 |= (1 << 13);
	
	// Trigger source: Hardware triggers disabled (software-triggered only).
	ADC1->CFGR1 &= ~(3 << 10);
}

void Calibrate_and_start_ADC(void) {
	
	// ___________________Calibrate (reference appendix A.7.1)___________________
	// Calibration is initialted when ADEN = 1. So initialize it to zero/disable it.
	if( (ADC1->CR & ADC_CR_ADEN) !=0)
		ADC1->CR |= 1 << 1;
	// Wait for the action to complete.
	while ( (ADC1->CR & ADC_CR_ADEN) != 0) {
	}
	// Clear the DMA bit to disable DMA.
	ADC1->CFGR1 &= ~(1);
	// Trigger the calibration in the control register.
	ADC1->CR |= (1 << 31);
	// Wait for the action to complete.
	while ( (ADC1->CR & ADC_CR_ADCAL) != 0) {
	}
	
	// ___________________ Enable Sequence code (reference appendix A.7.1)___________________
	// Make sure the ISR knows ADC is ready.
	if( (ADC1->ISR & ADC_ISR_ADRDY) != 0)
		ADC1->ISR |= ADC_ISR_ADRDY;
	ADC1->CR |= ADC_CR_ADEN; // 1 << 0
	// Wait for the action to complete
	while ( (ADC1->ISR & ADC_ISR_ADRDY) == 0 ) {
	}
	
	// _____________________ Start _____________________
	ADC1->CR |= (1 << 2);
}

void Init_DAC(GPIO_TypeDef* GPIOx, uint16_t pin_number) {
	// Configure PA4
	GPIOx->MODER |= 3 << (2*pin_number); // Analog mode (11)
	
	// Actually initialize the DAC
	DAC1->CR |= DAC_CR_TSEL1; // Software-triggered (111). (7 << 19) for channel 2. (7<<3) for channel 1.
	DAC1->CR |= DAC_CR_EN1; // Enable the DAC Channel 1 (1<<0)
}
