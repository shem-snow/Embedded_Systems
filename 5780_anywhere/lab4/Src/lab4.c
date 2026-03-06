#include "main.h"
#include "stm32f0xx_hal.h"
#include "My_HAL.h"
#include "System_Setup.h"

/* Local prototypes -----------------------------------------------*/
void Checkoff_4_1(void);
void Process_TDR_Part_I(char c);

void Checkoff_4_2(void);
void Process_TDR_Part_II(char LED, char action_ID);

// Global Variables
volatile char received_byte;
char LED_ID;
char action_ID;


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {

	// Initializations and Instantiations
	HAL_Init();
	SystemClock_Config();
	received_byte = '&'; // I initialized it to some junk that would never be processed.
	
	// Enable all the peripherals we're using.
	Init_LEDs();
	Init_USART(3);
	
	// Checkoff_4_1();
	Checkoff_4_2();
	
	return 0;
}


// void USART3_4_IRQHandler(void) {

// 	if( (USART3->ISR & (1<<5) ) )
// 		received_byte = USART3->RDR | 0xFF;
// 	else if(USART3->ISR & (1<<7) )
// 		USART3->TDR = char_to_send;

// }

void Checkoff_4_1(void) {
	// Test your transmission methods
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
		while( !(USART3->ISR & (1<<5) )) {}
    	received_byte = USART3->RDR & 0xFF;

		Transmit_String("\r\nFirst character is "); Transmit_Char(received_byte);
		
		// If it's an accepted letter, save it and receive the second character.
		if ( received_byte == 'r' || received_byte == 'g' || received_byte == 'b' || received_byte == 'o') {
			LED_ID = received_byte;
			Transmit_String("\r\nEnter 0 for off, 1 for on, or 2 for toggle. ");
			// Recieve the second character
			while( !(USART3->ISR & (1<<5) )) {}
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
