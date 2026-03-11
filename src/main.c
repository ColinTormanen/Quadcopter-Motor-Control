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
    DMA1_Stream3->NDTR = 1; // Buffer size to transfer
    __NVIC_EnableIRQ(DMA1_Stream3_IRQn);
    DMA1_Stream3->CR |= 1; // Enable the DMA stream
    SPI2->CR1 |= SPI_CR1_SPE; // Enable the SPI peripheral
}

int main() 
{
    SystemClock_Config_100MHz_HSE();

    LedInit();
    LedOff();
    InitMotors();
    InitBidshot();
    StartMotors();

    BusyWait(MotorStartDelay);

    LedOn();

    
    SetMotorThrottle(motor1, 48);
    BusyWait(100000);

    // InitSpi();
    while(1)
    {
        
        // for (int i = 48; i <= 100; i++)
        // {
        //     SetMotorThrottle(motor1, i);
        //     SetMotorThrottle(motor2, i);
        //     SetMotorThrottle(motor3, i);
        //     SetMotorThrottle(motor4, i);
        //     for(volatile int i = 0; i < 5000000; i++); // delay
        //     ToggleLed();
        // }

        // BusyWait(100000);
        // for (int i = 48; i <= 200; i++)
        // {
        //     SetMotorThrottle(motor1, i);
        //     SetMotorThrottle(motor2, i);
        //     SetMotorThrottle(motor3, i);
        //     SetMotorThrottle(motor4, i);
        //     for(volatile int i = 0; i < 5000000; i++); // delay
        //     ToggleLed();
        // }
        
    }

    return 0;
}

void DMA1_Stream3_IRQHandler()
{
    if (DMA1->LISR & (1 << 27)) // If transfer complete interrupt
    {
        DMA1->LIFCR |= (1 << 27); // Clear the interrupt
        if (ThrottleCommand[0] > 50) {
            SetMotorThrottle(motor1, ThrottleCommand[0]);
        }
            // SetMotorThrottle(motor2, ThrottleCommand[1]);
        // SetMotorThrottle(motor3, ThrottleCommand[2]);
        // SetMotorThrottle(motor4, ThrottleCommand[3]);
        for (int i = 0; i < 1; i++) {
            ThrottleCommand[i] = 0;
        }

        LedOff();

        // Reset the DMA stream for the next transfer
        DMA1_Stream3->M0AR  = (uint32_t) ThrottleCommand; // Set the memory location to the buffer
        DMA1_Stream3->NDTR = 1; // Buffer size to transfer
        while (DMA1_Stream3->CR & 1) {
        }
        DMA1_Stream3->CR |= 1; // Enable the DMA stream
    }
}