#include "main.h"
#include "stm32f0xx_hal.h"

#include <assert.h>
#include "My_HAL.h"
#include "System_Setup.h"

void SystemClock_Config(void);

// Local Prototypes
void Checkoff_1_2(void);
void Checkoff_1_3(void);

int main(void) {
    HAL_Init(); // Reset of all peripherals, init the Flash and Systick
    SystemClock_Config(); // Configure the system clock
    
    HAL_RCC_CLK_Enable('A', 0); // Enable the GPIOC clock in the RCC
    HAL_RCC_CLK_Enable('C', 0); // Enable the GPIOA clock in the RCC
    
    // Set up configuration structs to pass into the initialization functions for GPIO pins.
    My_GPIO_InitTypeDef initStrC6 = {
      6, // uint32_t PinNumber;
      MODE_OUTPUT, // uint32_t Mode;
      GPIO_Otype_Push_Pull, // uint32_t Otype; 
      GPIO_SPEED_FREQ_LOW, // uint32_t Speed; 
      0, // uint32_t Pull;
      0, // uint32_t InData;
      0, // uint32_t OutData;
      0, // uint32_t PortSetReset;
      0, // uint32_t PortLock;
      0, // uint32_t AlternateFunction;
      0, // uint32_t PortBitReset;
    };

    My_GPIO_InitTypeDef initStrC7 = {
      7, // uint32_t PinNumber;
      MODE_OUTPUT, // uint32_t Mode;
      GPIO_Otype_Push_Pull, // uint32_t Otype; 
      GPIO_SPEED_FREQ_LOW, // uint32_t Speed; 
      0, // uint32_t Pull;
      0, // uint32_t InData;
      0, // uint32_t OutData;
      0, // uint32_t PortSetReset;
      0, // uint32_t PortLock;
      0, // uint32_t AlternateFunction;
      0, // uint32_t PortBitReset;
    };

    My_GPIO_InitTypeDef initStrA0 = {
      0, // uint32_t PinNumber;
      MODE_INPUT, // uint32_t Mode;
      GPIO_Pull_down, // uint32_t Otype; 
      GPIO_SPEED_FREQ_LOW, // uint32_t Speed; 
      0, // uint32_t Pull;
      0, // uint32_t InData;
      0, // uint32_t OutData;
      0, // uint32_t PortSetReset;
      0, // uint32_t PortLock;
      0, // uint32_t AlternateFunction;
      0, // uint32_t PortBitReset;
    };

    // Initialize the GPIO pins by passing in the init struct.
    My_HAL_GPIO_Init(GPIOC, &initStrC6, 0); // Initialize pin PC6
    My_HAL_GPIO_Init(GPIOC, &initStrC7, 0); // Initialize pin PC7
    My_HAL_GPIO_Init(GPIOA, &initStrA0, 0); // Initialize pin PA0

    // Start PC6 high
    HAL_GPIO_UpdatePin(GPIOC, (1 << 6), GPIO_PIN_SET); //GPIOC->ODR |= (GPIO_PIN_SET? ~0 : 0) & GPIO_PIN_6;
    
    // Checkoff_1_2();
    Checkoff_1_3();

    return 1;
}

/*
  Toggle the output state of both PC6 and PC7
*/
void Checkoff_1_2() {
    while (1) {
        HAL_Delay(200); // Delay 200ms
        GPIOC->ODR ^= 1 << 6;
        GPIOC->ODR ^= 1 << 7;
    }
}

/*
    Toggles the output state of both PC6 (blue) and PC7 (red) every time the user button is pressed.

        The current way edits registers directly. This is how you would do the loop with the HAL:

        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)) { // If input signal is set/high
            debouncerCounter |= 0x01; // Set lowest bit of bit-vector
        }
        if (debouncerCounter == 0x7FFFFFFF) {
            // Toggle the output state of both PC6 and PC7
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_7);
        }
*/ 
void Checkoff_1_3() {
	
    // Create the debouncer to keep track of the button's state.
	uint32_t debouncer = 0;
    // Create a mask to match the two LEDs in the Output Data Register (ODR).
	uint32_t mask = (3 << 6); // 00..11..00

    // Run
	while (1) {

		// Shift the debouncer left at every loop iteration
		debouncer = debouncer << 1;

		// If the user button is pressed, set the LSB to 1.
		if(GPIOA->IDR & 1)
			debouncer |= 1;
        // Otherwise the LSB remains zero because the register was just shifted.

		// Do nothing when the button is 'steady high' or 'steady low'.
		if( (debouncer == 0xFFFFFFFF) || (debouncer == 0) )
			continue;

		// When transitioning from 'steady low' to 'steady high', toggle the LEDs.
		else if(debouncer == 0x1) {
      My_HAL_GPIO_TogglePin(GPIOC, mask); // GPIOC->ODR ^= mask; // xx..~(xx)..xx
		}
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
void Error_Handler(void)
{
  /* User can add their own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add their own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */
