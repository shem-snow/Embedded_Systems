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
#include "main.h"

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Transmit_Char(char c);
void Transmit_String(char* str);
void Process_TDR_Part_I(char c);
void Process_TDR_Part_II(char LED, char action_ID);
void USART3_4_IRQHandler(void);

// Separations of concern.
void Init_USART3(void);
void Init_LEDs(void);

// Global Variables
volatile char received_byte;
volatile uint8_t message_received_flag;


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	// Initializations and Instantiations
  HAL_Init();
  SystemClock_Config();
	message_received_flag = 0;
	received_byte = '&';
	
	// Enable all the peripherals we're using.
	Init_LEDs();
	Init_USART3();
	
	// Display a prompt to the user to get two characters.
	Transmit_String("What would you have me do?");
	
	// Instantiate a null-terminated array to hold incoming messages.
	char LED_ID;
	char action_ID;
  while (1) {
		
		// Test your transmission methods
		// Transmit_String("Jeffery Epstein didn't kill himself.");
		
		// Part I check-off
		
		// Do nothing while the RECEIVE data register is empty, otherwise handle the (received) data inside it.
		//if( (USART3->ISR & (1<<5) ) ) {
		//	Process_TDR_Part_I( USART3->RDR & (0xFF) ); // Bottom 8 bits is the character.
		//}
		
		
		// Part II check-off
		
		// Receive the first character
		while( !(USART3->ISR & (1<<5) )) {
		}
		received_byte = USART3->RDR & 0xFF;
		Transmit_String("First character is "); Transmit_Char(received_byte);
		
		// If it's a letter, save it and receive the second character.
		if ( received_byte >= 'a' && received_byte <= 'z' ) {
			LED_ID = received_byte;
			
			// Recieve the second character
			while( !(USART3->ISR & (1<<5) )) {
			}
			received_byte = USART3->RDR & 0xFF;
			Transmit_String("Second character is "); Transmit_Char(received_byte);
			
			// If it's a number then perform the corresponding action.
			if(received_byte >= '0' && received_byte <= '2') {
				Transmit_Char('\n');
				action_ID = received_byte;
				Process_TDR_Part_II(LED_ID, action_ID);
				continue;
			}
		}
		// Otherwise the input was invalid. Broadcast an error message and return the the beginning state.
			Transmit_String("That's is not a valid command, Try again."); // Error message for invalid character
  }
}

// __________________________________________________________________ Helper methods _______________________________________________

void Init_USART3(void) {
	
	// Feed the clock into the USART3 peripheral
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	
	// Set the discovery board pins (connected to USART3) to alternate function mode (10)
	GPIOC->MODER |= 2 << (2*4); // pin 4
	GPIOC->MODER &= ~(1<< (2*4) );
	
	GPIOC->MODER |= 2 << (2*5); // pin 5
	GPIOC->MODER &= ~(1<< (2*5) );
	
	// Multiplex to alternate function mode 1 ([3:0] = 0001)
	GPIOC->AFR[0] &= ~(14 << 4*4); // pin 4
	GPIOC->AFR[0] |= (1 << 4*4);
	
	GPIOC->AFR[0] &= ~(14 << 4*5); // pin 5
	GPIOC->AFR[0] |= (1 << 4*5);
	
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
		message_received_flag = 1;
	}
}

/*
* Initializes LEDs PC6-9
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
	GPIOC->OTYPER &= ~(1<<6);
	GPIOC->OTYPER &= ~(1<<7);
	GPIOC->OTYPER &= ~(1<<8);
	GPIOC->OTYPER &= ~(1<<9);
	
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
	
	// Initialize each light to be on
	GPIOC->ODR |= 1<<6;
	GPIOC->ODR |= 1<<7;
	GPIOC->ODR |= 1<<8;
	GPIOC->ODR |= 1<<9;
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
			GPIOC->ODR ^= 1 << 6;
			break;
		case 'g':
			GPIOC->ODR ^= 1 << 9;
			break;
		case 'b':
			GPIOC->ODR ^= 1 << 7;
			break;
		case 'o':
			GPIOC->ODR ^= 1 << 8;
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
			ODR_Value = 1 << 6;
			Transmit_String("Red LED ");
			break;
		case 'g':
			ODR_Value = 1 << 9;
		Transmit_String("Green LED ");
			break;
		case 'b':
			ODR_Value = 1 << 7;
			Transmit_String("Blue LED ");
			break;
		case 'o':
			ODR_Value = 1 << 8;
			Transmit_String("Orange LED ");
			break;
		default:
			Transmit_String("Wrong color homie. ");
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

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
  __disable_irq();
  while (1) {
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line) {
}
#endif
