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

#include "main.h"
#include "stm32f072xb.h"


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Init_LEDs(void);
void Init_I2C2(void);

void Checkoff_One(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
	SystemClock_Config();
	__HAL_RCC_GPIOC_CLK_ENABLE(); // TODO: uncomment the other ones
	__HAL_RCC_GPIOB_CLK_ENABLE(); // TODO:
	__HAL_RCC_I2C2_CLK_ENABLE(); // TODO:
  
	
	// Set up the peripherals
	Init_I2C2();
	Init_LEDs();
	
	
	Checkoff_One();
	
	// Checkoff_Two();
	
}

// _________________Helper Methods________________________________________________________________________________________________________________________________

/*
* Initializes LEDs PC6-9
*/
void Init_LEDs(void) {
	
	// Initialize Port C: LEDs and pins
	//RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // bit 19
	
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
* Initializes I2C
*/
void Init_I2C2(void) {

	// Enable the GPIOB peripheral because PB10 and PB11 are compatable with I2C
	//RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // bit 18
	
	// Feed the clock into the I2C peripheral
	//RCC->APB1ENR |= (1 << 22);
	
	// Set pins PB11 and PB13 on the discovery board to alternate function mode
	//GPIOB->MODER |= (2 << 2*11); // PB11
	//GPIOB->OTYPER |= (1 << 11); // Open Drain
	//GPIOB->AFR[1] |= (1 << 12); // AF1 is I2C2_SDA
	GPIOB->MODER |= GPIO_MODER_MODER11_1;
	GPIOB->OTYPER |= GPIO_OTYPER_OT_11;
	GPIOB->AFR[1] |= (0x1 << GPIO_AFRH_AFSEL11_Pos);
	
	//GPIOB->MODER |= (2 << 2*13); // PB13
	//GPIOB->OTYPER |= (1 << 13); // Open Drain
	//GPIOB->AFR[1] |= (5 << 20); // AF5 is I2C2_SLC
	GPIOB->MODER |=GPIO_MODER_MODER13_1;
	GPIOB->OTYPER |= GPIO_OTYPER_OT_13;
	GPIOB->AFR[1] |= (0x5 << GPIO_AFRH_AFSEL13_Pos);
	
	// Set pins PB14 and PC0 into output mode
	//GPIOB->MODER &= ~(2 << 2*14);
	//GPIOB->MODER |= (1 << 2*14);
	//GPIOB->OTYPER &= ~(1 << 14); // Push-pull
	//GPIOB->ODR |= (1 << 14); // Initialize to high
	GPIOB->MODER |= GPIO_MODER_MODER14_0;
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_14);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	
	//GPIOC->MODER &= ~(2 << 0);
	//GPIOC->MODER |= (1 <<0);
	//GPIOC->OTYPER &= ~(1 << 0); // Push-pull
	//GPIOC->ODR |= (1 << 0); // Initialize to high
	GPIOB->MODER |= GPIO_MODER_MODER0_0;
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_0);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
	
	// Set the parameteres in the TIMINGR to use 100 kHz standard-mode I2C
	//I2C2->TIMINGR |= 1 << 28; // Prescaler = 1
	//I2C2->TIMINGR |= 0x13 << 0; // SCLL = 0x13
	//I2C2->TIMINGR |= 0xF << 8; // SCLH = 0xF
	//I2C2->TIMINGR |= 0x2 << 16; // SCADEL = 0x2
	//I2C2->TIMINGR |= 0x4 << 20; // SCLDEL = 0x4
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
	//I2C2->CR2 |= (0x69 << 1); // slave address
	//I2C2->CR2 |= (1 << 16); // number of bytes to transmit
	//I2C2->CR2 &= ~(1 << 10); // Write direction
	//I2C2->CR2 |= (1 << 13); // Start generation
	
	I2C2->CR2 |= (0xD2 << I2C_CR2_SADD_Pos);
	I2C2->CR2 |= (0x1 << I2C_CR2_NBYTES_Pos);
	I2C2->CR2 &= ~(I2C_CR2_RD_WRN_Msk);
	I2C2->CR2 |= I2C_CR2_START_Msk;
	
	// wait until either TXIS or NACKF flags are set
	//while( !(I2C2->ISR & (1<<1) ) && !(I2C2->ISR & (1<<4) ) ) {
		//HAL_Delay(100);
		//GPIOC->ODR ^= (1<<7);
	//}
	
	while(1) {
		//if( (I2C2->ISR & (1<<1) ) | (I2C2->ISR & (1<<4)) )
		if((I2C2->ISR & I2C_ISR_TXIS) | (I2C2->ISR & I2C_ISR_NACKF))
			break;
		
		// TODO: Toggle an LED for testing.
		HAL_Delay(100);
		GPIOC->ODR ^= (1<<8);
	}
	
	// TODO: Toggle an LED for testing.
	HAL_Delay(100);
	GPIOC->ODR ^= (1<<7);

		
	if(I2C2->ISR & 1<<4)
		;// The slave did not respond
		
	else {
		// Write the address of the register into TXDR
		I2C2->CR2 |= 0xD3;
				
		// Wait until the TC flag is set
		while(!(I2C2->ISR & (1<<6) ) ) {}
				
		// Reload the CR2 register with the same parameters as before but set the direction to read.
		I2C2->CR2 |= 0x69 << 1; // slave address
		I2C2->CR2 |= 1 << 16; // number of bytes to transmit
		I2C2->CR2 |= 1 << 10; // Read direction
		I2C2->CR2 |= 1 << 13; // Start generation
					
		// Wait until either the RXNE or NACKF flags are set.
		while( !( (I2C2->ISR & (1<<2) ) && (I2C2->ISR & (1<<4) ) ) ) {}
				
		if( !(I2C2->ISR & (1<<2) ) )
			; // The slave tdid not acknowledge
		else {
					
			// Wait until the TC flag is set
			while(!(I2C2->ISR & (1<<6) ) ) {}
					
			// Check the contents of the RXDR register to see if it matches 0xD3 and indicate it by turning off LED 6
			if(I2C2->RXDR == 0xD3)
				GPIOC->ODR &= ~(1 << 6);
					
			// if it fails turn off LED8
			else
				GPIOC->ODR &= ~(1 << 8);
		}
	}

	// Set the STOP bit in CR2 to release the I2C2 BUS
	I2C2->CR2 |= (1<<14);

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
