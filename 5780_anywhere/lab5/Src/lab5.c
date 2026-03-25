#include "stm32f0xx_hal.h"
#include "My_HAL.h"
#include "System_Setup.h"

// Local Prototypes
void Checkoff_5_1(void);
void Checkoff_5_1_and_a_half(void);
void Checkoff_5_2(void);

void Read_Gyroscope(int num_bytes, uint8_t slave_address, int8_t payload_address);
void LED_logic();

// Global Variables
static int16_t x_total = 0;
static int16_t y_total = 0;
static int LED_threshold = 350;

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
  // Setup
  HAL_Init();
  SystemClock_Config();

  // Ungate clocks
  HAL_RCC_CLK_Enable('B', 0); // GPIOB
  HAL_RCC_CLK_Enable('C', 0); // GPIOC
  HAL_RCC_CLK_Enable('I', 2); // I2C2

  // Initialize peripherals
  Init_LEDs();
  Init_I2C2();

  // For debugging
  Init_USART(3);
  
//   Checkoff_5_1();
//   Checkoff_5_1_and_a_half();
  Checkoff_5_2();
	return -1;
}

/*
    Read from a few Gyroscope registers to see if I2C is working.
*/
void Checkoff_5_1(void) {

    int mismatch = 0;
    // if(gyro_transaction(0xF, 1, 'r', -1) != 0xD3) // 0xF is the address of the "WHO_AM_I" register which should always hold the value 0xD3.
    if(I2C_Read(0x69, 0xF) != (int8_t) 0xD3)
        mismatch += 1;
    if(gyro_transaction(0x30, 1, 'r', -1) != 0) // 0xF is the address of the "WHO_AM_I" register which should always hold the value 0xD3.
        mismatch += 1;
    if(gyro_transaction(0x38, 1, 'r', -1) != 0) // 0xF is the address of the "WHO_AM_I" register which should always hold the value 0xD3.
        mismatch += 1;
    
    // Indicate success by turning the GREEN LED on and falure with the RED one.
    if(mismatch)
        Error_loop(RED, 500);
    else
        HAL_GPIO_UpdatePin(GPIOC, GREEN, GPIO_PIN_SET);
}

/*
 *  see if you can write to a register. 
 */
void Checkoff_5_1_and_a_half(void) {

    // default value is 0b111, but sometimes it contains the value written the last time the board was flashed.
    // gyro_transaction(0x20, 2, 'w', 0b111);
    // gyro_transaction(0x20, 1, 'r', -1);
    // gyro_transaction(0x20, 2, 'w', 0b01000101);
    // HAL_Delay(1000);

    // if (gyro_transaction(0x20, 1, 'r', -1) != 69)
    //     Error_loop(ORANGE, 500);
    // else {
    //     Transmit_String("LETS GOOOO! \n");
    //     gyro_transaction(0x20, 2, 'w', 0b111); // put it back to its default value.
    //     HAL_GPIO_UpdatePin(GPIOC, GREEN, GPIO_PIN_SET);
    //     gyro_transaction(0x20, 1, 'r', -1);
    // }

    I2C_Write(0x69, 0x20, 0b111);
    I2C_Read(0x69, 0x20);
    I2C_Write(0x69, 0x20, 0b01000101);
    HAL_Delay(1000);
    if (I2C_Read(0x69, 0x20) != 69)
        Error_loop(ORANGE, 500);
    else {
        Transmit_String("LETS GOOOO! \n");
        I2C_Write(0x69, 0x20, 0b111); // put it back to its default value.
        HAL_GPIO_UpdatePin(GPIOC, GREEN, GPIO_PIN_SET);
        I2C_Read(0x69, 0x20);
    }
}

void Checkoff_5_2(void) {

    // Initialize the gyroscope. 0x20 is the address of CTRL_REG1. 0x0B sets normal (bit 3) mode and enables x (bit 0) and y (bit 1) axis (z axis would be bit 2).
    gyro_transaction('w', 0x69, 0x20, 0x0B); // or equivalently, I2C_Write(0x69, 0x20, 0x0B);

    // Transmit_String(gyro_transaction(0x20, 1, 'r', -1) == 0x0B ? "Successfully set up the gyroscope in normal mode with x and y axes enabled! \n" : "Failed to set up the gyroscope. \n");
    // Transmit_String(int_to_str(gyro_transaction(0x20, 1, 'r', -1))); Transmit_Char('\n');

    Transmit_String(I2C_Read(0x69, 0x20) == 0x0B ? "Successfully set up the gyroscope in normal mode with x and y axes enabled! \n" : "Failed to set up the gyroscope. \n");
    
    // Insert a delay in between reading and processing so orientation is read only once in an observable period.
    while(1) {
        HAL_Delay(100);
        Read_Gyroscope(4, 0x69, 0x28);
        LED_logic();
    }
}

void Read_Gyroscope(int num_bytes, uint8_t slave_address, int8_t payload_address) {

    I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0)); //Clear the bit fields for NBYTES and SADD
    I2C2->CR2 |= ((1 << 16) | (slave_address << 1)); //NBYTES = 1 to transmit 1 byte, 0x69 for SADD since this is address of gyroscope.
    I2C2->CR2 &= ~(1 << 10); //Set RD_WRN to 0 to indicate a write.
    I2C2->CR2 |= (1 << 13); //Set the start signal START in CR2 which is bit 13.

    //Again we wait for either NACKF or TXIS to be set and set the red LED if NACKF was set.
    while(!( (I2C2->ISR) & ( (I2C_ISR_TXIS) | (I2C_ISR_NACKF) ) )) {} 
  
    if( (I2C2->ISR) & (I2C_ISR_NACKF))
		Error_loop(RED, 1000);
    else
		I2C2->TXDR = payload_address | (1 << 7); // 1<<7 is the auto-increment bit
 
    while (!(I2C2->ISR & I2C_ISR_TC)) {}

    I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0)); //Clear the bit fields for NBYTES and SADD
    I2C2->CR2 |= ((num_bytes << 16) | (slave_address << 1));
    I2C2->CR2 |= (1 << 10); //Set RD_WRN to 1 to indicate a read.
    I2C2->CR2 |= (1 << 13); //Set the start signal START in CR2 which is bit 13.

    uint8_t data_array[4];
    for(int i = 0; i < num_bytes; i++){ 
        while(!( (I2C2->ISR) & ( (I2C_ISR_RXNE) | (I2C_ISR_NACKF)) )) {}

        if( (I2C2->ISR) & (I2C_ISR_NACKF) )
		    Error_loop(RED, 1000);
        else
            data_array[i] = I2C2->RXDR;
    }

    while (!(I2C2->ISR & I2C_ISR_TC)) {}

    //Now that we have stored the read X and Y L and H values, we can assemble the full 16 bit registers
    x_total += (int16_t)((data_array[1] << 8) | data_array[0]);
    y_total += (int16_t)((data_array[3] << 8) | data_array[2]);

}

void LED_logic() {
    
    if (x_total > LED_threshold) {
      HAL_GPIO_UpdatePin(GPIOC, GREEN, GPIO_PIN_SET);
      HAL_GPIO_UpdatePin(GPIOC, ORANGE, GPIO_PIN_RESET);
    }
    else if (x_total < -LED_threshold) {
      HAL_GPIO_UpdatePin(GPIOC, ORANGE, GPIO_PIN_SET);
      HAL_GPIO_UpdatePin(GPIOC, GREEN, GPIO_PIN_RESET);
    }

    if (y_total > LED_threshold){
      HAL_GPIO_UpdatePin(GPIOC, RED, GPIO_PIN_SET);
      HAL_GPIO_UpdatePin(GPIOC, BLUE, GPIO_PIN_RESET);
    }else if (y_total < -LED_threshold){
      HAL_GPIO_UpdatePin(GPIOC, BLUE, GPIO_PIN_SET);
      HAL_GPIO_UpdatePin(GPIOC, RED, GPIO_PIN_RESET);
    }
}
