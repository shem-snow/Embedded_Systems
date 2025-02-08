/*
    The HAL we were provided is so inconvenient to look through for certain features because the intended use doesn't match
    what the data sheet says to do.

    I made this header file to define what to do in such cases because it's more work to look up values in the provided HAL
    than it is just to define our own logic.
*/
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
  * @brief  This struct contains the registers you would use for GPIO pins.
  */
typedef struct {
  uint32_t PinNumber;

  uint32_t Mode;

  uint32_t Otype;

  uint32_t Speed;

  uint32_t Pull;

  uint32_t InData;

  uint32_t OutData;

  uint32_t LCKR;

  uint32_t PortLock;

  uint32_t AlternateFunction;

  uint32_t BitReset;

} My_GPIO_InitTypeDef;

void HAL_RCC_CLK_Enable(char GPIOx, uint32_t number);
void Reset_Interrupt(char pin);
void HAL_ALTERNATE_PIN_Init(GPIO_TypeDef* GPIOx, My_GPIO_InitTypeDef *GPIO_Init, uint8_t AFR_high);
void My_HAL_GPIO_Init(GPIO_TypeDef* GPIOx, My_GPIO_InitTypeDef *GPIO_Init);