#include "bidshot.h"
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

void InitSpi()
{
    SPI_Init();
    DMA1_Stream3->M0AR  = (uint32_t) ThrottleCommand; // Set the memory location to the buffer
    DMA1_Stream3->NDTR = 4; // Buffer size to transfer
    __NVIC_EnableIRQ(DMA1_Stream3_IRQn);
    DMA1_Stream3->CR |= 1; // Enable the DMA stream
    SPI2->CR1 |= 1; // Enable the SPI peripheral
}

int main() 
{
    SystemClock_Config_100MHz_HSE();

    LedInit();
    LedOff();
    InitSpi();
    InitMotors();
    InitBidshot();
    StartMotors();

    BusyWait(MotorStartDelay);

    LedOn();

    

    while(1)
    {

        SetMotorThrottle(motor1, 48);
        BusyWait(100000);
        
    }

    return 0;
}

void DMA1_Stream3_IRQHandler()
{
    if (DMA1->HISR & (1 << 27)) // If transfer complete interrupt
    {
        DMA1->HIFCR |= (1 << 27); // Clear the interrupt
        SetMotorThrottle(motor1, ThrottleCommand[0]);
        SetMotorThrottle(motor2, ThrottleCommand[1]);
        SetMotorThrottle(motor3, ThrottleCommand[2]);
        SetMotorThrottle(motor4, ThrottleCommand[3]);
        for (int i = 0; i < 4; i++) {
            ThrottleCommand[i] = 0;
        }

        // Reset the DMA stream for the next transfer
        DMA1_Stream3->M0AR  = (uint32_t) ThrottleCommand; // Set the memory location to the buffer
        DMA1_Stream3->NDTR = 4; // Buffer size to transfer
        while (DMA1_Stream3->CR & 1) {
        }
        DMA1_Stream3->CR |= 1; // Enable the DMA stream
    }
}