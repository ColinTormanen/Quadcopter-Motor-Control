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

/*  Startup sequence:
    1. Send any commands to program the ESC if needed, in this case, enable extended telemetry
    2. Send 0 throttle for ~1 second to arm the ESC
    3. Send throttle value of 48 to start motors
    4. Motors are now armed
*/
void MotorStartupSequence()
{
    StartMotors();

    BusyWait(MotorStartDelay);

    SetMotorThrottle(motor1, 48);
    SetMotorThrottle(motor2, 48);
    SetMotorThrottle(motor3, 48);
    SetMotorThrottle(motor4, 48);

    BusyWait(MotorStartDelay);
}

int main() 
{
    SystemClock_Config_100MHz_HSE();

    LedInit();
    LedOff();
    InitMotors();
    InitBidshot();

    LedOn();

    // InitSpi();
    while(1)
    {
        uint16_t rpmLog[1000];
        for (int i = 0; i < 1000; i++) {
            rpmLog[i] = 0;
        }
        int index = 0;
        
        for (int i = 50; i <= 1000; i+=50)
        {
            SetMotorThrottle(motor1, i);
            SetMotorThrottle(motor2, i);
            SetMotorThrottle(motor3, i);
            SetMotorThrottle(motor4, i);
            for(volatile int i = 0; i < 5000000; i++); // delay
            ToggleLed();
            rpmLog[index] = motor1->RPM;
            index++;
        }

        BusyWait(100000);
        for (int i = 1000; i >= 50; i-=50)
        {
            SetMotorThrottle(motor1, i);
            SetMotorThrottle(motor2, i);
            SetMotorThrottle(motor3, i);
            SetMotorThrottle(motor4, i);
            for(volatile int i = 0; i < 5000000; i++); // delay
            ToggleLed();
            rpmLog[index] = motor1->RPM;
            index++;
        }
        index = 0;
        

        // Motor throttle value is controlled via SPI
        // Main loop just checks Emergency Stop GPIO pin and cuts throttle if it's triggered
        // if (GPIOA->IDR & 1) // If the emergency stop button is pressed
        // {
        //     StopMotors();
        // }
    }

    return 0;
}

// SPI transfer complete interrupt handler
// This funciton is used to update the motor throttle values after receiving new data over SPI
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