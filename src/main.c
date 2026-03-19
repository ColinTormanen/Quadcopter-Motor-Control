#include "bidshot.h"
#include "dshot.h"
#include "motor_control.h"
#include "spi.h"
#include "stm32f411xe.h"
#include "system.h"
#include <stdint.h>

#define MotorStartDelay 20000000

uint16_t ThrottleCommand[4]; 


void BusyWait(uint32_t delay) 
{
    for (volatile uint32_t i = 0; i < delay; i++);
}

void InitSpiMotorThrottle()
{
    DMA1_Stream3->M0AR  = (uint32_t) ThrottleCommand; // Set the memory location to the buffer
    DMA1_Stream3->NDTR = 4; // Buffer size to transfer
    __NVIC_EnableIRQ(DMA1_Stream3_IRQn);
    DMA1_Stream3->CR |= 1; // Enable the DMA stream
    SPI2->CR1 |= SPI_CR1_SPE; // Enable the SPI peripheral

    
}

/*  Startup sequence:
    1. Send 0 throttle for ~1 second to arm the ESC 
    2. Send any commands to program the ESC if needed, in this case, enable extended telemetry, set direction
    3. Send throttle value of 48 to start motors
    4. Motors are now spinning
*/
void MotorStartupSequence()
{
    StartMotors();

    BusyWait(MotorStartDelay);

    // Construct the command sequence to program the ESC, most commands need to be repeated 6 times for the ESC to accept them
    uint16_t cwCommandValues[10] = {
        7, // Set spin direction
        9,  // 9: Disable bidirectional spinning
        14, // 14: Disable extended telemetry
        12, // 12: Saves settings to eeprom
        48  // Last command should help transition the motors consistently into spinning
    }; 

    uint16_t ccwCommandValues[10] = {
        8, // Set spin direction
        9,  // 9: Disable bidirectional spinning
        14, // 14: Disable extended telemetry
        12, // 12: Saves settings to eeprom
        48  // Last command should help transition the motors consistently into spinning
    }; 
    uint8_t repeat[10] = {6, 6, 6, 6, 100}; 
    ConstructCommandSequence(motor1, cwCommandValues, repeat, 5);
    ConstructCommandSequence(motor2, ccwCommandValues, repeat, 5);
    ConstructCommandSequence(motor3, cwCommandValues, repeat, 5);
    ConstructCommandSequence(motor4, ccwCommandValues, repeat, 5);

    // Set minimum throttle to start the motors
    SetMotorThrottle(motor1, 48);
    SetMotorThrottle(motor2, 48);
    SetMotorThrottle(motor3, 48);
    SetMotorThrottle(motor4, 48);
}

int main() 
{
    SystemClock_Config_100MHz_HSE();

    LedInit();
    LedOff();

    SPI_Init();
    InitSpiMotorThrottle(); // Enable SPI and DMA for receiving throttle commands

    while (1)
    {
        while (!(GPIOC->IDR & (1 << 15))); // Wait for on signal

        InitMotors();
        InitBidshot();

        MotorStartupSequence(); // Initialize motors

        LedOn();
        GPIOC->ODR |= (1 << 14); // Set C14 high to indicate we're ready for commands

        while (GPIOC->IDR & (1 << 15)); // Wait for off signal
        GPIOC->ODR &= ~(1 << 14); // Set C14 low to indicate we're no longer accepting commands

        StopMotors(); // Stop motors

        LedOff();
        // DMA1_Stream3->CR &= ~(1 << 0); // Disable the DMA stream
        // while (DMA1_Stream3->CR & 1) {
        // }
        // SPI2->CR1 &= ~(SPI_CR1_SPE); // Disable the SPI peripheral
    }

    return 0;
}

// SPI transfer complete interrupt handler
// This function is used to update the motor throttle values after receiving new data over SPI
// The Throttle Commands are sent in 16 bit frames:
//      Bits 15-11 are used as a header to indicate the motor
//      Bits 10-0 are used as the throttle value
void DMA1_Stream3_IRQHandler()
{
    if (DMA1->LISR & (1 << 27)) // If transfer complete interrupt
    {
        DMA1->LIFCR |= (1 << 27); // Clear the interrupt

        for (int i = 0; i < 4; i++) {

            switch (ThrottleCommand[i] & 0xF800)
            {
                case 0x800: // Motor 1 command
                    SetMotorThrottle(motor1, ThrottleCommand[i] & 0x07FF);
                    break;
                case 0x1000: // Motor 2 command
                    SetMotorThrottle(motor2, ThrottleCommand[i] & 0x07FF);
                    break;
                case 0x1800: // Motor 3 command
                    SetMotorThrottle(motor3, ThrottleCommand[i] & 0x07FF);
                    break;
                case 0x2000: // Motor 4 command
                    SetMotorThrottle(motor4, ThrottleCommand[i] & 0x07FF);
                    break;
                default:
                    // Invalid command, ignore
                    break;
            }
        }

        for (int i = 0; i < 4; i++) {
            ThrottleCommand[i] = 0; // Clear the command after processing
        }

        // Reset the DMA stream for the next transfer
        DMA1_Stream3->M0AR  = (uint32_t) ThrottleCommand; // Set the memory location to the buffer
        DMA1_Stream3->NDTR = 4; // Buffer size to transfer
        while (DMA1_Stream3->CR & 1) {
        }
        DMA1_Stream3->CR |= 1; // Enable the DMA stream
    }
}