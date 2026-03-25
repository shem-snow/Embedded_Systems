#include <stdint.h>
#include <stm32f0xx_hal.h>
#include <stm32f0xx_hal_gpio.h>

/*
    The HAL we were provided is so inconvenient to look through for certain features because the intended use doesn't match
    what the data sheet says to do.

    I made this header file to define what to do in such cases because it's more work to look up values in the provided HAL
    than it is just to define our own logic.
*/
#define RED  1<<6
#define BLUE 1<<7
#define ORANGE 1<<8
#define GREEN 1<<9

#define AF0 0x0
#define AF1 0x1
#define AF2 0x2
#define AF3 0x3
#define AF4 0x4
#define AF5 0x5
#define AF6 0x6
#define AF7 0x7

#define GPIO_MODE_Input 0x0
#define GPIO_MODE_Output 0x1
#define GPIO_MODE_AF 0x2
#define GPIO_MODE_Analog 0x3
#define GPIO_Otype_Push_Pull 0x0
#define GPIO_Otype_Open_Drain 0x1
#define GPIO_Pull_none 0x0
#define GPIO_Pull_up 0x1
#define GPIO_Pull_down 0x2

/** 
  * @brief  This struct contains the registers you would use to configure any GPIO pin.
  *         Not all items will be used, but all uses will use these items.
  */
typedef struct {
  uint32_t PinNumber;

  uint32_t Mode;

  uint32_t Otype;

  uint32_t Speed;

  uint32_t Pull;

  uint32_t InData;

  uint32_t OutData;

  uint32_t PortSetReset;

  uint32_t PortLock;

  uint32_t AlternateFunction;

  uint32_t PortBitReset;

} My_GPIO_InitTypeDef;


/*
 *
 * ________________________________________ Function prototypes: ________________________________________
 *
 */

// ________________________________________ Clock and interrupt _________________________________________
void HAL_RCC_CLK_Enable(char GPIOx, uint32_t number); // Clock gating for peripherals
void Reset_Interrupt(char pin);

// ________________________________________ GPIOs _________________________________________
void Init_LEDs(void);
void Init_Button(void);
void My_HAL_GPIO_Init(GPIO_TypeDef* GPIOx, My_GPIO_InitTypeDef *GPIO_Init, int8_t alt_func);
void My_HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void HAL_GPIO_UpdatePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);

void HAL_ALTERNATE_PIN_Init(GPIO_TypeDef* GPIOx, My_GPIO_InitTypeDef *GPIO_Init);


// ________________________________________ UART _________________________________________
void Init_USART(int number);
void Transmit_Char(char c);
void Transmit_String(char* str);

// ________________________________________ I2C _________________________________________
void Init_I2C2(void);
int8_t gyro_transaction(char direction, uint8_t slave_address, int8_t payload_address, int8_t payload);

void Error_loop(int LED_color, int frequency);
char* int_to_str(int num);

void I2C_Write(uint8_t slave_address, int8_t payload_address, int8_t payload);
int8_t I2C_Read(uint32_t slave_address, uint32_t payload_address);

// ________________________________________ ADC _________________________________________
void Init_ADC(GPIO_TypeDef* GPIOx, uint16_t pin_number);
void Calibrate_and_start_ADC(void);
void Init_DAC(GPIO_TypeDef* GPIOx, uint16_t pin_number);

// ________________________________________ PID _________________________________________
void Lab7_Systick_Callback(void);