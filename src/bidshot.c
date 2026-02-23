#include "bidshot.h"
#include "dshot.h"
#include <stm32f411xe.h>

#define TelemetrySize 20 // Technically, there are 21 bits in the telemetry frame, but the first bit is always 0, and it missed by the polling
#define Motor1Pin 4
#define Motor2Pin 5
#define Motor3Pin 0
#define Motor4Pin 1

int motor_counter = 0;
int motor_data = 0;
int motor_data_buffer[TelemetrySize]; // Buffer to hold the GPIO telemetry data
int current_motor = Motor1Pin;

const uint8_t gcr_table[] = { // Nibble decoding table
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x9, 0xA,
    0xB, 0x0, 0xD, 0xE, 0xF, 0x0, 0x0, 0x2, 0x3, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x1, 0x0, 0x4, 0xC, 0x0,
};

void InitBidshot() {

    // ==================== Configure Telemetry reading Timers ==================== //
    // Motor 1 -> B4, EXTI4_IRQn
    // Motor 2 -> B5, EXTI9_5_IRQn
    // Motor 3 -> B0, EXTI0_IRQn
    // Motor 4 -> B1, EXTI1_IRQn

    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; // Enable clock for TIM4
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // Enable clock for SYSCFG

    TIM4->CR1 = 0; // Reset the clock
    TIM4->CCR1 = pollTimerWidth; // Set poll duration
    // TIM4->DIER |= 2; // Enable CC1 interrupt
    TIM4->ARR = pollTimerWidth;
    TIM4->DIER |= (1<<9); // Enable DMA request on CCR1 match
    TIM4->EGR |= (1<<1); // Generate DMA request on CCR1 match

    // GPIO interrupt configuration for telemetry
    SYSCFG->EXTICR[0] &= ~(0xFF); // Clear EXTI0, 1
    SYSCFG->EXTICR[0] |= (1<<0) | (1<<4); // Set EXTI0, 1 to port B
    SYSCFG->EXTICR[1] &= ~(0xFF); // Clear EXTI4, 5
    SYSCFG->EXTICR[1] |= (1<<0) | (1<<4); // Set EXTI4, 5 to port B
    EXTI->IMR |= (1<<0) | (1<<1) | (1<<4) | (1<<5); // Unmask EXTI0, 1, 4, 5
    EXTI->FTSR |= (1<<0) | (1<<1) | (1<<4) | (1<<5); // Falling edge trigger

    // ==================== Configure DMA for Telemetry reading ==================== //
    
    DMA2_Stream0->CR = 0; // Reset the stream

    while(DMA1_Stream4->CR & 1) {} // Wait for the steam to stop
    DMA1_Stream0->CR |= (2<<25); // Set to channel 2
    DMA1_Stream0->CR |= (2<<16); // High priority
    DMA1_Stream0->CR |= (0<<6); // Mem-to-periph direction
    DMA1_Stream0->CR &= ~(1<<9); // Peripheral address is fixed
    DMA1_Stream0->CR |= (1<<10); // Memory incremented
    DMA1_Stream0->CR |= (2<<11); // Peripheral data size 32 bits
    DMA1_Stream0->CR |= (2<<13); // Memory data size 32 bits
    DMA1_Stream0->CR |= (1<<4); // Transfer complete interrupt enabled

    DMA1_Stream0->NDTR = TelemetrySize; // Buffer size to transfer
    DMA1_Stream0->PAR = (uint32_t) &GPIOB->IDR; // Set the peripheral memory address to the GPIO input data register
    DMA1_Stream0->M0AR = (uint32_t) motor_data_buffer; // Set the memory location to the buffer

    // Testing, used to output when the timer is triggered
    GPIOB->MODER |= (1<<14); // Set pin to output
    GPIOB->OSPEEDR  |= 3<<14;
    GPIOB->ODR &= ~(1<<7);

    __NVIC_SetPriority(EXTI0_IRQn, 5);
    __NVIC_SetPriority(EXTI1_IRQn, 5);
    __NVIC_SetPriority(EXTI4_IRQn, 5);
    __NVIC_SetPriority(EXTI9_5_IRQn, 5);

    __NVIC_SetPriority(TIM4_IRQn, 10);

    __NVIC_SetPriority(DMA1_Stream4_IRQn, 15);
    __NVIC_SetPriority(DMA1_Stream5_IRQn, 15);
    __NVIC_SetPriority(DMA1_Stream7_IRQn, 15);
    __NVIC_SetPriority(DMA1_Stream2_IRQn, 15);

}

void Decode(int data, dshotMotor *motor) {
    int gcr = (data ^ (data >> 1));
    int telemetry = 0;

    for (int i = 0; i < 4; i++) {
        uint8_t nibble = (gcr >> (i * 5)) & 0x1F;
        telemetry = gcr_table[nibble] << (i * 4) | telemetry;
    }
    int crc = telemetry & 0xF;
    telemetry = telemetry >> 4 & 0xFFF;
    uint16_t calc_crc =
        ~(telemetry ^ (telemetry >> 4) ^ (telemetry >> 8)) & 0xF;
    if (crc != calc_crc) {
        return;
    }

    if (telemetry & (1 << 8)) {
        uint16_t data = telemetry & 0xFF;
        uint8_t exp = (telemetry >> 9) & 0x7;
        motor->eRPM = data << exp;
    } else {
        uint16_t data = telemetry & 0xFF;
        uint8_t type = (telemetry >> 8) & 0xF;
        switch (type) {
        case 0x02:
            motor->temperature = data;
            break;
        case 0x04:
            motor->voltage = data * 25;
            break;
        case 0x06:
            motor->current = data;
            break;
        }
    }
}

// GPIO interrupt handlers for telemetry reading
// Motor 1
void EXTI4_IRQHandler() {
    if (EXTI->PR & (1 << 4) && !(GPIOB->IDR & (1 << 4))) {
        EXTI->PR |= (1 << 4); // Clear the interrupt
        __NVIC_DisableIRQ(EXTI4_IRQn);
        __NVIC_EnableIRQ(TIM4_IRQn);
        motor_data = 0;
        TIM4->CNT = 0;  // Restart the timer
        TIM4->CR1 |= 1; // Start the timer
    }
}

// Motor 2
void EXTI9_5_IRQHandler() {
    if (EXTI->PR & (1 << 5) && !(GPIOB->IDR & (1 << 5))) {
        EXTI->PR |= (1 << 5); // Clear the interrupt
        __NVIC_DisableIRQ(EXTI9_5_IRQn);
        __NVIC_EnableIRQ(TIM5_IRQn);
        motor_data = 0;
        TIM5->CNT = 0;  // Restart the timer
        TIM5->CR1 |= 1; // Start the timer
    }
}

// Motor 3
void EXTI0_IRQHandler() {
    if (EXTI->PR & (1 << 0) && !(GPIOB->IDR & (1 << 0))) {
        EXTI->PR |= (1 << 0); // Clear the interrupt
        __NVIC_DisableIRQ(EXTI0_IRQn);
        __NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
        motor_data = 0;
        TIM9->CNT = 0;  // Restart the timer
        TIM9->CR1 |= 1; // Start the timer
    }
}
// Motor 4
void EXTI1_IRQHandler() {
    if (EXTI->PR & (1 << 1) && !(GPIOB->IDR & (1 << 1))) {
        EXTI->PR |= (1 << 1); // Clear the interrupt
        __NVIC_DisableIRQ(EXTI1_IRQn);
        __NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
        motor_data = 0;
        TIM10->CNT = 0;  // Restart the timer
        TIM10->CR1 |= 1; // Start the timer
    }
}
// Motor 1 timer interrupt handler
void TIM4_IRQHandler() {
    if (TIM4->SR & (1 << 1)) {
        TIM4->SR &= ~(1 << 1); // Clear the interrupt
        motor_counter++;
        motor_data = (motor_data << 1) | ((GPIOB->IDR & (1 << 4)) >> 4);
        if (motor_counter > 19) {
            TIM4->CR1 &= ~1; // Stop the timer
            motor_counter = 0;
            Decode(motor_data, motor1);
            __NVIC_DisableIRQ(TIM4_IRQn);
        }
    }
}

void DMA1_Stream0_IRQHandler()
{
    if (DMA1->LISR & (1 << 5)) // If transfer complete interrupt
    {
        DMA1->LIFCR |= (1 << 5); // Clear the interrupt

        for (int i = 0; i < TelemetrySize; i++) {
            motor_data = (motor_data << 1) | ((motor_data_buffer[i] & current_motor)); // Shift in the new bit
            motor_data_buffer[i] = 0;
        }


        dshotMotor* currentMotor;

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
        for (int i = 0; i < TelemetrySize; i++) {
            Decode(motor_data, currentMotor);
        }

        while (DMA2_Stream0->CR & 1) {
        }
        DMA2_Stream0->NDTR = TelemetrySize;
        DMA2_Stream0->M0AR = (uint32_t)motor_data_buffer;

        GPIOB->ODR ^= (1 << 7); // Toggle pin for testing
    }
}
