#include "main.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_it.h"
#include "My_Hal.h"

/******************************************************************************/
/*           Cortex-M0 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
   while (1)
  {
  }
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  while (1)
  {
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void) {
  
  HAL_IncTick();

  // This accumulating variable can be either local-static or global-volatile. Both will work.
  static int SysTick_accumulator;

  // Toggle the blue LED every 200 ms
  if(SysTick_accumulator >= 200) {
    My_HAL_GPIO_TogglePin(GPIOC, BLUE);
	  SysTick_accumulator = 0;
  }
  else
	  SysTick_accumulator += 1;
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

