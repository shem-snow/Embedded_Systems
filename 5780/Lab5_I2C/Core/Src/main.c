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

void Checkoff_One(void);
void Checkoff_Two(void);

void Read_Gyroscope(int numbytes, int *addr, int read_write, int reg_addr);

// ___________________________Constants________________________________________
int GREEN = (1 << 9);
int ORANGE = (1 << 8);
int BLUE = (1 << 7);
int RED = (1 << 6);

int SDA = 11; // PB11
int SCL = 13; // PB13
int output = 14; // PB14

// _____________________________Program start______________________________________
int main(void)
{

  /* Reset all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
	SystemClock_Config();
	
	// Feed the clock into each peripherals we will be using to enable them
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // GPIOC (bit 19)
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // GPIOB (bit 18)
	RCC->APB1ENR |= (1 << 22); // I2C (bit 22)
	
	// Set up the pins to be used for I2C
	Init_I2C2();
	Init_LEDs();
	
	
	Checkoff_One();
	
	//Checkoff_Two();
	
}

// _________________Helper Methods________________________________________________________________________________________________________________________________

/*
* Initializes LEDs PC6-9
*/
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

/*
* Initializes I2C
*/
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
	

	// wait until either TXIS or NACKF flags are set
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
	
	// Clear CR2 bits for the fields: NBYTES, SADD, and RD_WRN
	I2C2->CR2 &= ~( (0xFF << 16) | (0x3FF) | (1<<10) );
	
	// Configure CR2 for the current operation.
	I2C2->CR2 |= (0xD2 << I2C_CR2_SADD_Pos);   // Set the L3GD20 slave address = 0x69 = 0110 1001; left shift: 0xD2 = 1101 0010
	I2C2->CR2 |= (0x2  << I2C_CR2_NBYTES_Pos); // Set the number of bytes to transmit = 2
	I2C2->CR2 &= ~(I2C_CR2_RD_WRN_Msk);        // Set the RD_WRN to write operation
	I2C2->CR2 |= (I2C_CR2_START_Msk);          // Set START bit
	
	// Wait until TXIS or NACKF flags are set (1)
	while(1) {
		if (I2C2->ISR & I2C_ISR_TXIS) {
			// Write the address of the "CTRL_REG1" register into TXDR
			I2C2->TXDR = 0x20;
			break;
		}
		
		if (I2C2->ISR & I2C_ISR_NACKF)
			;
	}
	
	// Wait again until TXIS or NACKF flags are set (2)
	while(1) {
		if (I2C2->ISR & I2C_ISR_TXIS) {
			// Write the address of the "normal or sleep mode", enable x-axis and y-axis
			I2C2->TXDR = 0x0B;
			break;
		}
		
		if (I2C2->ISR & I2C_ISR_NACKF)
			;
	}
	
	// Wait for TC flag is set
	while(1) {
		if (I2C2->ISR & I2C_ISR_TC)
			break;
	}
	
	// Set the STOP bit in CR2 to release the bus
	//I2C2->CR2 |= (I2C_CR2_STOP);
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
