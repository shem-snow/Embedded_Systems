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
	* Shem Snow
  ******************************************************************************
  */

#include "main.h"
#include "stm32f072xb.h"


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Init_LEDs(void);
void Init_I2C2(void);
void Init_Gyroscope(void);

void Checkoff_One(void);
void Checkoff_Two(void);

void Read_Two_Bytes(int y_axis);

// ___________________________Constants________________________________________
int GREEN = (1 << 9);
int ORANGE = (1 << 8);
int BLUE = (1 << 7);
int RED = (1 << 6);

int SDA = 11; // PB11 TODO:
int SCL = 13; // PB13 TODO:
int output = 14; // PB14 TODO:

uint8_t x_1;
uint8_t x_2;
int16_t x_total;
int16_t x_position = 0;

uint8_t y_1;
uint8_t y_2;
int16_t y_total;
int16_t y_position = 0;

// _____________________________Program start______________________________________
int main(void)
{

  // Reset all peripherals, the Flash interface, and the Systick.
  HAL_Init();
	SystemClock_Config();
	
	// Feed the clock into each peripherals we will be using to enable them.
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // GPIOC (bit 19)
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // GPIOB (bit 18)
	RCC->APB1ENR |= (1 << 22); // I2C (bit 22)
	
	// Set up the pins to be used for I2C
	Init_I2C2();
	Init_LEDs();
	
	
	Checkoff_One();
	
	//Init_Gyroscope();
	//Checkoff_Two();
	
}

// _________________Helper Methods________________________________________________________________________________________________________________________________

void Init_LEDs(void) {
	
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
	
	// Initialize each light to be off
	GPIOC->BSRR = GPIO_BSRR_BR_6;
	GPIOC->BSRR = GPIO_BSRR_BR_7;
	GPIOC->BSRR = GPIO_BSRR_BR_8;
	GPIOC->BSRR = GPIO_BSRR_BR_9;
}
void Init_I2C2(void) {
	
	// Set pins PB11 and PB13 on the discovery board to alternate function mode
	GPIOB->MODER |= (2 << 2*11); // PB11
	GPIOB->OTYPER |= (1 << 11); // Open Drain
	GPIOB->AFR[1] |= (1 << 12); // AF1 is I2C2_SDA
	GPIOB->MODER |= GPIO_MODER_MODER11_1;
	GPIOB->OTYPER |= GPIO_OTYPER_OT_11;
	GPIOB->AFR[1] |= (0x1 << GPIO_AFRH_AFSEL11_Pos);
	
	GPIOB->MODER |= (2 << 2*13); // PB13
	GPIOB->OTYPER |= (1 << 13); // Open Drain
	GPIOB->AFR[1] |= (5 << 20); // AF5 is I2C2_SCL
	GPIOB->MODER |=GPIO_MODER_MODER13_1;
	GPIOB->OTYPER |= GPIO_OTYPER_OT_13;
	GPIOB->AFR[1] |= (0x5 << GPIO_AFRH_AFSEL13_Pos);
	
	// Set pins PB14 and PC0 into output mode
	GPIOB->MODER &= ~(2 << 2*14);
	GPIOB->MODER |= (1 << 2*14);
	GPIOB->OTYPER &= ~(1 << 14); // Push-pull
	GPIOB->ODR |= (1 << 14); // Initialize to high
	GPIOB->MODER |= GPIO_MODER_MODER14_0;
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_14);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	
	GPIOC->MODER &= ~(2 << 0);
	GPIOC->MODER |= (1 <<0);
	GPIOC->OTYPER &= ~(1 << 0); // Push-pull
	GPIOC->ODR |= (1 << 0); // Initialize to high
	GPIOB->MODER |= GPIO_MODER_MODER0_0;
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_0);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
	
	// Set the parameteres in the TIMINGR to use 100 kHz standard-mode I2C
	I2C2->TIMINGR |= 1 << 28; // Prescaler = 1
	I2C2->TIMINGR |= 0x13 << 0; // SCLL = 0x13
	I2C2->TIMINGR |= 0xF << 8; // SCLH = 0xF
	I2C2->TIMINGR |= 0x2 << 16; // SCADEL = 0x2
	I2C2->TIMINGR |= 0x4 << 20; // SCLDEL = 0x4
	I2C2->TIMINGR |= (0x1 << I2C_TIMINGR_PRESC_Pos);
	I2C2->TIMINGR |= (0x13 << I2C_TIMINGR_SCLL_Pos);
	I2C2->TIMINGR |= (0xF << I2C_TIMINGR_SCLH_Pos);
	I2C2->TIMINGR |= (0x2 << I2C_TIMINGR_SDADEL_Pos);
	I2C2->TIMINGR |= (0x4 << I2C_TIMINGR_SCLDEL_Pos);
	
	// Enable the I2C peripheral in CR1
	I2C2->CR1 |= I2C_CR1_PE; // 1;
	
	// Clear Nbits and SADD
	I2C2->CR2 &= ( ~(0x7F << 16) & ~(0x3FF << 0) );
}


void Checkoff_One(void) {

	// Set the parameters for the current transaction (in CR2)
	I2C2->CR2 |= (0x69 << 1); // slave address
	I2C2->CR2 |= (1 << 16); // number of bytes to transmit
	I2C2->CR2 &= ~(1 << 10); // Write direction
	I2C2->CR2 |= (1 << 13); // Start generation
	

	// Wait until either TXIS or NACKF flags are set
	while( ! ( (I2C2->ISR & (1<<1) ) | (I2C2->ISR & (1<<4) ) ) ) {
		// Turn on Red to indicate execution is stuck here.
		GPIOC->ODR |= RED;
		HAL_Delay(100);
	}
	
	// Toggle green to indicate success.
	GPIOC->ODR ^= GREEN;
	HAL_Delay(100);
	GPIOC->ODR ^= GREEN;
	HAL_Delay(100);
	
	// If the slave did not respond, turn orange on to indicate failure
	if(I2C2->ISR & 1<<4)
		GPIOC->ODR |= ORANGE;
	
	else {
		// Write the address of the register into TXDR (from the gyroscopt datasheet).
		I2C2->TXDR |= 0xF;
				
		// Wait until the TC flag is set
		while( !( I2C2->ISR & (1<<6) )) {
			// Toggle Red to indicate execution is stuck here.
			GPIOC->ODR ^= RED;
			HAL_Delay(100);
		}
		
		// Toggle blue to indicate success.
		GPIOC->ODR ^= BLUE;
		HAL_Delay(100);
		GPIOC->ODR ^= BLUE;
		HAL_Delay(100);
		
		// __________________________________read ___________________________________________________
		
		// Reload the CR2 register with the same parameters as before but set the direction to read.
		I2C2->CR2 |= 0x69 << 1; // slave address
		I2C2->CR2 |= 1 << 16; // number of bytes to transmit
		I2C2->CR2 |= 1 << 10; // Read direction
		I2C2->CR2 |= 1 << 13; // Start generation
					
		// Wait until either the RXNE or NACKF flags are set.
		while(1) {
			if ((I2C2->ISR & (1<<2)) | (I2C2->ISR & (1<<4)))
				break;
			// Toggle red to indicate exeution is stuck
			GPIOC->ODR ^= RED;
			HAL_Delay(100);
		}
		
		// Toggle orange to indicate success
		GPIOC->ODR ^= ORANGE;
		HAL_Delay(100);
		GPIOC->ODR ^= ORANGE;
		
		// Turn orange on if the slave did not acknowledge
		if( !(I2C2->ISR & (1<<2) ) )
			GPIOC->ODR |= ORANGE;
		
		else {
					
			// Wait until the TC flag is set
			while(!(I2C2->ISR & (1<<6) ) ) {
				// Toggle red to indicate exeution is stuck
				GPIOC->ODR ^= RED;
				HAL_Delay(100);
			}
			
		// Toggle green to indicate success
		GPIOC->ODR ^= GREEN;
		HAL_Delay(100);
		GPIOC->ODR ^= GREEN;
			
			// Check the contents of the RXDR register to see if it matches 0xD3 and indicate it by maintaining Green on.
			if(I2C2->RXDR == 0xD3)
				GPIOC->ODR |= GREEN;
					
			// if it fails turn maintain orange on.
			else
				GPIOC->ODR |= ORANGE;
		}
	}

	// Set the STOP bit in CR2 to release the I2C2 BUS
	I2C2->CR2 |= (1<<14);
	
	// maintain blue on to indicate success.
	GPIOC->ODR |= BLUE;
}


void Checkoff_Two(void) {
	
	// Update the x and y positions two bytes at a time.
	while(1) {
		
		// X
		Read_Two_Bytes(0);
		
		// Y
		Read_Two_Bytes(1);
		
		// If the x position is negative, turn on the orange LED and turn off the green LED
		if (x_position < 0) {
			GPIOC->ODR |= ORANGE;
			GPIOC->ODR &= ~GREEN;
		}
		// Otherwise the x position is positive. Turn off orange and turn on green.
		else {
			GPIOC->ODR &= ~ORANGE;
			GPIOC->ODR |= GREEN;
		}
	
		// If the y position is negative, turn blue on and red off
		if (y_position < 0) {
			GPIOC->ODR |= BLUE;
			GPIOC->ODR &= ~RED;
		}
		// Otherwise the y position is positive. Turn off blue and turn on red.
		else {
			GPIOC->ODR &= ~BLUE;
			GPIOC->ODR |= RED;
		}
	}
	
}
void Init_Gyroscope(void) {
	
	// Set the parameters for the current transaction (in CR2)
	I2C2->CR2 |= (0x69 << 1); // Slave address = 0x69 << 1 = 0xD2 = 1101 0010
	I2C2->CR2 |= (2  << 16); // NBYTES = 1
	I2C2->CR2 &= ~(1 << 10); // Write direction
	I2C2->CR2 |= (1 << 13); // Start generation
	
	// Wait until either TXIS or NACKF flags are set
	while(1) {
		if (I2C2->ISR & I2C_ISR_TXIS) {
			// Write the address of the "CTRL_REG1" register into TXDR
			I2C2->TXDR = 0x20;
			break;
		}
		
		if (I2C2->ISR & I2C_ISR_NACKF)
			;
	}
	
	// wait until either TXIS or NACKF flags are set
	while( ! ( (I2C2->ISR & (1<<1) ) | (I2C2->ISR & (1<<4) ) ) ) {
		// Turn on Red to indicate execution is stuck here.
		GPIOC->ODR |= RED;
		HAL_Delay(100);
	}
	
	
	// wait until either TXIS or NACKF flags are set
	while(1) {
		if ( (I2C2->ISR & I2C_ISR_TXIS) | (I2C2->ISR & I2C_ISR_NACKF) ){
			// Write the address of the "normal or sleep mode" into TXDR
			I2C2->TXDR = 0x0B;
			break;
		}
	}
	
	// Wait for TC flag is set
	while(1) {
		if (I2C2->ISR & I2C_ISR_TC)
			break;
	}
}

/*
* If the y_axis flag is 0 then we read the x-axis.
* Otherwise we read the y-axis.
*/
void Read_Two_Bytes(int y_axis) {
	
	//____________________________________First Byte____________________________________________________
	
		// Clear the NBYTES SADD fields of CR2
		I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
		
		// Set the parameters for the current transaction (in CR2 => CTRL_REG1) to WRITE
		I2C2->CR2 |= (0xD2 << I2C_CR2_SADD_Pos); // Slave address = 0x69 << 1 = 0xD2 = 1101 0010
		I2C2->CR2 |= (0x1  << I2C_CR2_NBYTES_Pos); // NBYTES = 1
		I2C2->CR2 &= ~(I2C_CR2_RD_WRN_Msk); // Write direction
		I2C2->CR2 |= (I2C_CR2_START_Msk); // Start generation
	
	
		// Wait until TXIS or NACKF flags are set
		while(1) {
			if ((I2C2->ISR & I2C_ISR_TXIS)) {
				I2C2->TXDR = (y_axis)? 0x2A : 0x28;
				break;
			}
		
			// If the slave did not respond, turn orange on to indicate failure
			if ((I2C2->ISR & I2C_ISR_NACKF)) {
				GPIOC->ODR |= ORANGE;
				continue;
			}
		}
	
		// Wait until the TC flag is set
		// int count = 0;
		while(1) {
			if (I2C2->ISR & I2C_ISR_TC) {
				break;
			}
			
			// Toggle red to indicate execution is stuck
			// count += 1;
			//GPIOC->ODR ^= RED;
		}
	
		// Set the parameters for the current transaction (in CR2 => CTRL_REG1) to READ
		I2C2->CR2 |= (0xD2 << I2C_CR2_SADD_Pos);   // Slave address = 0x69 << 1 = 0xD2 = 1101 0010
		I2C2->CR2 |= (0x1  << I2C_CR2_NBYTES_Pos); // NBYTES = 1
		I2C2->CR2 |= (I2C_CR2_RD_WRN_Msk);         // Read Direction
		I2C2->CR2 |= (I2C_CR2_START_Msk);          // Generate Start
	
		// Wait until RXNE or NACKF flags are set
		while(1) {
			// Then write the lower 8 bits
			if ((I2C2->ISR & I2C_ISR_RXNE)) {
				if (y_axis) {
					y_1 = I2C2->RXDR;
				}
				else {
					x_1 = I2C2->RXDR;
				}
				break;
			}
		
			// If the slave did not respond, turn orange on to indicate failure
			if ((I2C2->ISR & I2C_ISR_NACKF)) {
				GPIOC->ODR |= ORANGE;
				continue;
			}
		}
	
		// Wait for the TC flag to set
		while(1) {
			if (I2C2->ISR & I2C_ISR_TC) {
				break;
			}
			
			// Toggle red to indicate execution is stuck
			//GPIOC->ODR ^= RED;
		}
	
	
		//____________________________________Second Byte____________________________________________________
		
		// Clear the NBYTES SADD fields of CR2
		I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
	
		// Set the parameters for the current transaction (in CR2 => CTRL_REG1) to WRITE
		I2C2->CR2 |= (0xD2 << I2C_CR2_SADD_Pos); // Slave address = 0x69 << 1 = 0xD2 = 1101 0010
		I2C2->CR2 |= (0x1  << I2C_CR2_NBYTES_Pos); // NBYTES = 1
		I2C2->CR2 &= ~(I2C_CR2_RD_WRN_Msk); // Write direction
		I2C2->CR2 |= (I2C_CR2_START_Msk); // Start generation
	
		// Wait until TXIS or NACKF flags are set
		while(1) {
			if ((I2C2->ISR & I2C_ISR_TXIS)) {
				I2C2->TXDR = (y_axis)? 0x2B : 0x29; // OUT_Y_H or OUT_X_H
				break;
			}
		
			// If the slave did not respond, turn orange on to indicate failure
			if ((I2C2->ISR & I2C_ISR_NACKF)) {
				GPIOC->ODR |= ORANGE;
				continue;
			}
		}
	
		// Wait until the TC flag is set
		while(1) {
			if (I2C2->ISR & I2C_ISR_TC)
				break;
			
			// Toggle red to indicate execution is stuck
			//GPIOC->ODR ^= RED;
		}
	
		// Set the parameters for the current transaction (in CR2 => CTRL_REG1) to READ
		I2C2->CR2 |= (0xD2 << I2C_CR2_SADD_Pos); // Slave address = 0x69 << 1 = 0xD2 = 1101 0010
		I2C2->CR2 |= (0x1  << I2C_CR2_NBYTES_Pos); // NBYTES = 1
		I2C2->CR2 |= (I2C_CR2_RD_WRN_Msk); // Read Direction
		I2C2->CR2 |= (I2C_CR2_START_Msk); // Generate Start
	
		// Wait until RXNE or NACKF flags are set
		while(1) {
			// Then write the upper 8 bits
			if ((I2C2->ISR & I2C_ISR_RXNE)) {
				if(y_axis)
					y_2 = I2C2->RXDR;
				else
					x_2 = I2C2->RXDR;
				break;
			}
		
			// If the slave did not respond, turn orange on to indicate failure
			if ((I2C2->ISR & I2C_ISR_NACKF)) {
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
				continue;
			}
		}
	
		// Wait for the TC flag to set
		while(1) {
			if (I2C2->ISR & I2C_ISR_TC)
				break;
			
			// Toggle red to indicate execution is stuck
			//GPIOC->ODR ^= RED;
		}
	
		// Store both bytes into the total
		if(y_axis) {
			y_total = (y_2 << 8) | (y_1 << 0);
			y_position += y_total;
		}
		else {
			x_total = (x_2 << 8) | (x_1 << 0);
			x_position += x_total;
		}
	
		// Delay 100 ms so only one read occurs during that time.
		HAL_Delay(100);
}

// _________________System________________________________________________________________________________________________________________________________
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
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
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
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
