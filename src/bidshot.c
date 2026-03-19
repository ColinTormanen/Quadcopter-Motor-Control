#include "bidshot.h"
#include "dshot.h"
#include "motor_control.h"
#include <stdint.h>
#include <stm32f411xe.h>

#define Motor1Pin 0b10000
#define Motor2Pin 0b100000
#define Motor3Pin 0b1
#define Motor4Pin 0b10
#define pollTimerWidth 128

#define TelemetrySize 20 // Technically, there are 21 bits in the telemetry frame, but the first bit is always 0, and it missed by the polling

int motor_counter = 0;
int motor_data = 0;
uint32_t motor_data_buffer[TelemetrySize]; // Buffer to hold the GPIO telemetry data
uint8_t current_motor = Motor1Pin;

const uint8_t gcr_table[] = { // Nibble decoding table
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x9, 0xA,
    0xB, 0x0, 0xD, 0xE, 0xF, 0x0, 0x0, 0x2, 0x3, 0x0, 0x5,
    0x6, 0x7, 0x0, 0x0, 0x8, 0x1, 0x0, 0x4, 0xC, 0x0,
};

void InitBidshot() {
    for (int i = 0; i < TelemetrySize; i++) {
        motor_data_buffer[i] = 0;
    }   

    // Invert the PWM to indicate Bi-Directional DSHOT mode to the ESCs
    TIM3->CCMR1 |= (7<<4) | (7<<12); // Set to PWM mode 2 for CCR1, CCR2
    TIM3->CCMR2 |= (7<<4) | (7<<12); // Set to PWM mode 2 for CCR3, CCR4

    // Enable the half transfer interrupts to read the telemetry data
    DMA1_Stream4->CR |= (1<<3); 
    DMA1_Stream5->CR |= (1<<3); 
    DMA1_Stream7->CR |= (1<<3); 
    DMA1_Stream2->CR |= (1<<3);

    // ==================== Configure Telemetry reading Timers ==================== //
    // Motor 1 -> B4, EXTI4_IRQn
    // Motor 2 -> B5, EXTI9_5_IRQn
    // Motor 3 -> B0, EXTI0_IRQn
    // Motor 4 -> B1, EXTI1_IRQn

    RCC->APB2ENR |= 1 << RCC_APB2ENR_TIM1EN_Pos; // Enable clock for TIM1
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // Enable clock for SYSCFG
    RCC->AHB1ENR |= 1 << RCC_AHB1ENR_DMA2EN_Pos; // Enable clock for DMA2
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Enable clock for GPIOA

    GPIOA->MODER |= (2 << 16); // Set PA8 to alternate function mode for TIM1 CH1
    GPIOA->AFR[1] |= (1 << 0); // Set alternate function to AF01, TIM1
    GPIOA->OSPEEDR |= (3 << 16); // High output speed on PA8

    TIM1->CR1 = 0; // Reset the clock
    TIM1->CCR1 = pollTimerWidth; // Set poll duration
    TIM1->ARR = pollTimerWidth+1;
    TIM1->DIER |= (1<<9); // Enable DMA request on CCR1 match
    TIM1->EGR |= (1<<1); // Generate DMA request on CCR1 match
    TIM1->CR2 &= ~(1<<3); // DMA request on CC match

    TIM1->CCMR1 |= (3<<4); // Set to toggle mode for CCR1
    TIM1->CCER |=  TIM_CCER_CC1E; // Enable output on CCR1
    TIM1->BDTR |= TIM_BDTR_MOE; // Main output enable for advanced timers
    

    // GPIO interrupt configuration for telemetry
    SYSCFG->EXTICR[0] &= ~(0xFF); // Clear EXTI0, 1
    SYSCFG->EXTICR[0] |= (1<<0) | (1<<4); // Set EXTI0, 1 to port B
    SYSCFG->EXTICR[1] &= ~(0xFF); // Clear EXTI4, 5
    SYSCFG->EXTICR[1] |= (1<<0) | (1<<4); // Set EXTI4, 5 to port B
    EXTI->IMR |= (1<<0) | (1<<1) | (1<<4) | (1<<5); // Unmask EXTI0, 1, 4, 5
    EXTI->FTSR |= (1<<0) | (1<<1) | (1<<4) | (1<<5); // Falling edge trigger

    // ==================== Configure DMA for Telemetry reading ==================== //
    
    DMA2_Stream6->CR = 0; // Reset the stream

    while(DMA2_Stream6->CR & 1) {} // Wait for the stream to stop
    DMA2_Stream6->CR |= (0<<25); // Set to channel 0
    DMA2_Stream6->CR |= (1<<16); // High priority
    DMA2_Stream6->CR |= (0<<6); // Periph-to-mem direction
    DMA2_Stream6->CR &= ~(1<<9); // Peripheral address is fixed
    DMA2_Stream6->CR |= (1<<10); // Memory incremented
    DMA2_Stream6->CR |= (2<<11); // Peripheral data size 32 bits
    DMA2_Stream6->CR |= (2<<13); // Memory data size 32 bits
    DMA2_Stream6->CR |= (1<<4); // Transfer complete interrupt enabled

    DMA2_Stream6->NDTR = TelemetrySize; // Buffer size to transfer
    DMA2_Stream6->PAR = (uint32_t) &GPIOB->IDR; // Set the peripheral memory address to the GPIO input data register
    DMA2_Stream6->M0AR = (uint32_t) motor_data_buffer; // Set the memory location to the buffer

    // DMA2_Stream6->CR |= 1; // Enable the DMA stream
    // TIM1->CR1 |= 1; // Start the timer
    
    __NVIC_SetPriority(EXTI0_IRQn, 5);
    __NVIC_SetPriority(EXTI1_IRQn, 5);
    __NVIC_SetPriority(EXTI4_IRQn, 5);
    __NVIC_SetPriority(EXTI9_5_IRQn, 5);

    __NVIC_SetPriority(DMA1_Stream4_IRQn, 15);
    __NVIC_SetPriority(DMA1_Stream5_IRQn, 15);
    __NVIC_SetPriority(DMA1_Stream7_IRQn, 15);
    __NVIC_SetPriority(DMA1_Stream2_IRQn, 15);
    __NVIC_SetPriority(DMA1_Stream3_IRQn, 20);
    __NVIC_SetPriority(DMA2_Stream6_IRQn, 20);

    __NVIC_EnableIRQ(DMA2_Stream6_IRQn);
}

int Decode(int data, dshotMotor *motor) {
    int gcr = (data ^ (data >> 1));
    int telemetry = 0;

    for (int i = 0; i < 4; i++) {
        uint8_t nibble = (gcr >> (i * 5)) & 0x1F;
        telemetry = gcr_table[nibble] << (i * 4) | telemetry;
    }
    int crc = telemetry & 0xF;
    telemetry = telemetry >> 4 & 0xFFF;
    uint16_t calc_crc = (~(telemetry ^ (telemetry >> 4) ^ (telemetry >> 8))) & 0xF;
    if (crc != calc_crc) {
        return 0;
    }

    if (telemetry & (1 << 8)) {
        uint16_t data = telemetry & 0x1FF;
        uint8_t exp = (telemetry >> 9) & 0x7;
        uint16_t period = (data << exp) * 7; // Period of one rotation in microseconds, 14 poles, so multiply by 7 to get the period of one electrical rotation
        motor->RPM = (60000000 / period); // Convert period to RPM
        
    } else {
        uint16_t data = telemetry & 0xFF;
        uint8_t type = (telemetry >> 8) & 0xF;
        switch (type) {
        case 0x02:
            motor->temperature = data;
            break;
        case 0x04:
            motor->voltage = data;
            break;
        case 0x06:
            motor->current = data;
            break;
        }
    }
    return 1;
}

// GPIO interrupt handlers for telemetry reading
// Motor 1
void EXTI4_IRQHandler() {
    if (EXTI->PR & (1 << 4) && !(GPIOB->IDR & (1 << 4))) {
        current_motor = Motor1Pin;
        DMA2_Stream6->CR |= 1; // Start the DMA transfer
        TIM1->CR1 |= 1; // Start the timer
        EXTI->PR |= (1 << 4); // Clear the interrupt
        __NVIC_DisableIRQ(EXTI4_IRQn);
        __NVIC_EnableIRQ(DMA2_Stream6_IRQn);
        
        motor_data = 0;
    }
    else
    {
        EXTI->PR |= (1 << 4); // Clear the interrupt
    }
}

// Motor 2
void EXTI9_5_IRQHandler() {
    if (EXTI->PR & (1 << 5) && !(GPIOB->IDR & (1 << 5))) {
        current_motor = Motor2Pin;
        DMA2_Stream6->CR |= 1; // Start the DMA transfer
        TIM1->CR1 |= 1; // Start the timer
        EXTI->PR |= (1 << 5); // Clear the interrupt
        __NVIC_DisableIRQ(EXTI9_5_IRQn);
        __NVIC_EnableIRQ(DMA2_Stream6_IRQn);
        
        motor_data = 0;
    }
    else
    {
        EXTI->PR |= (1 << 5); // Clear the interrupt
    }
}

// Motor 3
void EXTI0_IRQHandler() {
    if (EXTI->PR & (1 << 0) && !(GPIOB->IDR & (1 << 0))) {
        current_motor = Motor3Pin;
        DMA2_Stream6->CR |= 1; // Start the DMA transfer
        TIM1->CR1 |= 1; // Start the timer
        EXTI->PR |= (1 << 0); // Clear the interrupt
        __NVIC_DisableIRQ(EXTI0_IRQn);
        __NVIC_EnableIRQ(DMA2_Stream6_IRQn);
        
        motor_data = 0;
    }
    else
    {
        EXTI->PR |= (1 << 0); // Clear the interrupt
    }
}
// Motor 4
void EXTI1_IRQHandler() {
    if (EXTI->PR & (1 << 1) && !(GPIOB->IDR & (1 << 1))) {
        current_motor = Motor4Pin;
        DMA2_Stream6->CR |= 1; // Start the DMA transfer
        TIM1->CR1 |= 1; // Start the timer
        EXTI->PR |= (1 << 1); // Clear the interrupt
        __NVIC_DisableIRQ(EXTI1_IRQn);
        __NVIC_EnableIRQ(DMA2_Stream6_IRQn);
        
        motor_data = 0;
    }
    else
    {
        EXTI->PR |= (1 << 1); // Clear the interrupt
    }
}

// Motor telemetry DMA interrupt handler
// This is triggered after all telemetry bits have been read
void DMA2_Stream6_IRQHandler()
{
    if (DMA2->HISR & (1 << 21)) // If transfer complete interrupt
    {
        DMA2->HIFCR |= (1 << 21); // Clear the interrupt
        TIM1->CR1 &= ~1; // Stop the timer
        TIM1->CNT = 0; // Reset the timer counter

        __NVIC_DisableIRQ(DMA2_Stream6_IRQn);

        for (int i = 0; i < TelemetrySize; i++) {
            if (motor_data_buffer[i] & current_motor) {
                motor_data = (motor_data << 1) | 1; // Shift in the new bit
            } else {
                motor_data = (motor_data << 1); // Shift in a 0 bit
            }
            motor_data_buffer[i] = 0;
        }

        dshotMotor* currentMotor = motor1;

        switch (current_motor) {
            case Motor1Pin:
                currentMotor = motor1;
                break;
            case Motor2Pin:
                currentMotor = motor2;
                break;
            case Motor3Pin:
                currentMotor = motor3;
                break;
            case Motor4Pin:
                currentMotor = motor4;
                break;
            default:
                currentMotor = motor1; // Default to motor1 if no match
        }

        // Process the telemetry data in motor_data_buffer
        Decode(motor_data, currentMotor);
        currentMotor->interruptCounter++;

        // Advance to the next motor for sequential processing
        next_motor = (next_motor + 1) % 4;

        while (DMA2_Stream6->CR & 1) {
        }
        DMA2_Stream6->NDTR = TelemetrySize;
        DMA2_Stream6->M0AR = (uint32_t)motor_data_buffer;
    }
}
