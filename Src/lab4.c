/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include <stm32f0xx_hal.h>
#include <stm32f0xx_hal_gpio.h>
// #include <assert.h>
#include "main.h"
#include "My_HAL.h"


/* Local prototypes -----------------------------------------------*/

void Checkoff_4_1(void);
void Checkoff_4_2(void);

// Separations of concern.
void Init_USART3(void);

// Global Variables
volatile char received_byte;
//volatile uint8_t message_received_flag;

char LED_ID;
char action_ID;


/**
  * @brief  The application entry point.
  * @retval int
  */
int lab4_main(void) {
	// Initializations and Instantiations
	HAL_Init();
	SystemClock_Config();
	//message_received_flag = 0;
	received_byte = '&'; // I initialized it to some junk that would never be processed.
	
	// Enable all the peripherals we're using.
	Init_LEDs();
	Init_USART3();
	
	//Checkoff_4_1();
	Checkoff_4_2();
	
	return 0;
}


void Checkoff_4_1(void) {
	//Test your transmission methods
	HAL_Delay(1000);
	Transmit_String("Jeffery Epstein didn't kill himself.\r\n");
	while (1) {
		//Do nothing while the RECEIVE data register is empty, otherwise handle the (received) data inside it.
		if( (USART3->ISR & (1<<5) ) ) {
			Process_TDR_Part_I( USART3->RDR & (0xFF) ); // Bottom 8 bits is the character.
		}
	  }
}

/*
	You put in two characters to cause an action.
		First character selects one of the LEDs ('r' 'g' 'b' 'o')
        Second character selects the action to perform on that LED
			'0' - off
            '1' - on
            '2' - toggle
*/
void Checkoff_4_2(void) {
	while(1) {
		// Display a prompt to the user to get two characters.
		Transmit_String("\r\nWhat would you have me do?");
		
		// Receive the first character
		while( !(USART3->ISR & (1<<5) )) {
		}
		received_byte = USART3->RDR & 0xFF;
		Transmit_String("\r\nFirst character is "); Transmit_Char(received_byte);
		
		// If it's an accepted letter, save it and receive the second character.
		if ( received_byte == 'r' || received_byte == 'g' || received_byte == 'b' || received_byte == 'o') {
			LED_ID = received_byte;
			
			// Recieve the second character
			while( !(USART3->ISR & (1<<5) )) {
			}
			received_byte = USART3->RDR & 0xFF;
			Transmit_String("\r\nSecond character is "); Transmit_Char(received_byte);
			
			// If it's a number then perform the corresponding action.
			if(received_byte >= '0' && received_byte <= '2') {
				action_ID = received_byte;
				Process_TDR_Part_II(LED_ID, action_ID);
				continue;
			}
		}
		// Otherwise the input was invalid. Broadcast an error message and return to the beginning state.
		Transmit_String("\r\nThat's is not a valid command, Try again."); // Error message for invalid character
  }
}

// ________________________________________________ Helper Functions ____________________________________________


void Init_USART3(void) {
	
	// Feed the clock into the USART3 peripheral
    HAL_RCC_CLK_Enable('U', 3);

	// In this lab we only want to set the mode and alternate function registers. The others are at default values.
	// Set the discovery board pins (connected to USART3) to alternate function mode (10)
	GPIOC->MODER |= 2 << (2*4); // pin 4 is the USART3 receiver in alternate function mode 1
	GPIOC->MODER &= ~(1<< (2*4) );
	
	GPIOC->MODER |= 2 << (2*5); // pin 5 is the USART3 transmitter in alternate function mode 1
	GPIOC->MODER &= ~(1<< (2*5) );
	

	// TODO: If I can find out what the default values are then I'll use the provided HAL. One idea is to not reset bits in MY_HAL functions. That would require constant use of the De-init funciton.
    My_GPIO_InitTypeDef init_PC4 = { // Receiver
		4,
		GPIO_MODE_AF,
		GPIO_Otype_Open_Drain,
		GPIO_SPEED_FREQ_HIGH,
		GPIO_Pull_up,
		0, // uint32_t InData;
		0, // uint32_t OutData;
		0, // uint32_t PortSetReset;
		0, // uint32_t PortLock;
		AF1,
		0 // uint32_t BitReset;
	};
    //My_HAL_GPIO_Init(GPIOC, &init_PC4);
	HAL_ALTERNATE_PIN_Init(GPIOC, &init_PC4);

    My_GPIO_InitTypeDef init_PC5 = { // Transmitter
		5,
		GPIO_MODE_AF,
		GPIO_Otype_Push_Pull,
		GPIO_SPEED_FREQ_LOW,
		GPIO_Pull_none,
		0, // uint32_t InData;
		0, // uint32_t OutData;
		0, // uint32_t PortSetReset;
		0, // uint32_t PortLock;
		AF1,
		0 // uint32_t BitReset;
	};
    //My_HAL_GPIO_Init(GPIOC, &init_PC5);
	HAL_ALTERNATE_PIN_Init(GPIOC, &init_PC5);
	
    // Set the Baud rate for communcation to be 115,200 bits/second. The system clock is 8 MHz.
	USART3->BRR = HAL_RCC_GetHCLKFreq() / 115200;
	
	// Enable USART in the control register.
	USART3->CR1 |= 12; // ..1100, bit 2 enables the receiver and bit 3 enables the transmitter.
	USART3->CR1 |= 1; // bit zero is the general enable bit.
	
	
	// Enable the "receive register not empty interrrupt".
	USART3->CR1 |= 1<<5;
		
	// Enable the inturrupt in the NVIC and set its priority.
	NVIC_EnableIRQ(USART3_4_IRQn);
	NVIC_SetPriority(USART3_4_IRQn, 2);
}

/*
* Does nothing if the RECEIVE data register is empty, otherwise this method saves the data in the Receive data register (RDR)
*  to a global variable and sets a flag indicating that action.
*/
void USART3_4_IRQHandler(void) {
	if( (USART3->ISR & (1<<5) ) ) {
		received_byte = USART3->RDR | 0xFF;
		// message_received_flag = 1;
	}
}

/*
* Does nothing while the transmit data register is empty.
* When it's not empty, it transmits the character.
*/
void Transmit_Char(char c) {
	
	while( !(USART3->ISR & (1<<7) ) ) {
		//HAL_Delay(100);
	}
	USART3->TDR = c;
}

/*
* Loops over each character in the "str" array and transmits the character there. 
*/
void Transmit_String(char* str) {
	 for(int i = 0; str[i] != '\0'; i++) {
		 //HAL_Delay(50);
		 Transmit_Char(str[i]);
	 }
}

/*
* Uses a switch statement to detect if the user typed in a letter corresponding to one of the four LEDs (r, g, b, o).
* If the user's input was one of those then the corresponding LED is toggled. Otherwise an error message is displayed.
*/
void Process_TDR_Part_I(char c) {
	
	switch(c) {
		case '\0':
			break;
		case 'r':
			GPIOC->ODR ^= RED;
			break;
		case 'g':
			GPIOC->ODR ^= GREEN;
			break;
		case 'b':
			GPIOC->ODR ^= BLUE;
			break;
		case 'o':
			GPIOC->ODR ^= ORANGE;
			break;
		default:
			Transmit_String("You're only allowed to type one of the 4 colors. Try again nerd.");
	}
	
}

/*
* Decodes the input message as an action.
* The first element is a letter that indicates which LED to change.
* The second element is the number and indicates the action to take on it.
* If the input is not in the set of predefined commands then an error message is displayed.
*/
void Process_TDR_Part_II(char LED_ID, char action_ID) {
	
	int ODR_Value;
	switch(LED_ID) {
		case 'r':
			ODR_Value = RED;
			Transmit_String("\nRed LED ");
			break;
		case 'g':
			ODR_Value = GREEN;
		Transmit_String("\nGreen LED ");
			break;
		case 'b':
			ODR_Value = BLUE;
			Transmit_String("\nBlue LED ");
			break;
		case 'o':
			ODR_Value = ORANGE;
			Transmit_String("\nOrange LED ");
			break;
		default:
			Transmit_String("\nWrong color homie. ");
			return;
	}

	switch(action_ID) {
		case '0':
			Transmit_String("off. ");
			GPIOC->ODR &= ~(ODR_Value);
			break;
		case '1':
			Transmit_String("on. ");
			GPIOC->ODR |= ODR_Value;
			break;
		case '2':
			Transmit_String("toggle. ");
			GPIOC->ODR ^= ODR_Value;
			break;
		default:
			Transmit_String("Wrong number. ");
	}
}