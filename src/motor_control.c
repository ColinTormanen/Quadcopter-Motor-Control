#include "motor_control.h"
#include "dshot.h"
#include <stdint.h>
#include <stm32f411xe.h>

// NOTE:
// This implementation has a prototype Bi-Directional DSHOT telemetry decoder
// To Enable telemetry reading, uncomment the EXTI interrupts in the DMA interrupt handlers

// Provide single definitions for the motor objects and pointers declared extern
// in the header.
static dshotMotor motor1_obj;
static dshotMotor motor2_obj;
static dshotMotor motor3_obj;
static dshotMotor motor4_obj;

dshotMotor *motor1 = &motor1_obj;
dshotMotor *motor2 = &motor2_obj;
dshotMotor *motor3 = &motor3_obj;
dshotMotor *motor4 = &motor4_obj;

void InitMotors() {
    InitiMotor(motor1);
    InitiMotor(motor2);
    InitiMotor(motor3);
    InitiMotor(motor4);

    InitDshot(motor1, motor2, motor3, motor4);
}

void StartMotors() {
    __NVIC_EnableIRQ(DMA1_Stream4_IRQn);
    __NVIC_EnableIRQ(DMA1_Stream5_IRQn);
    __NVIC_EnableIRQ(DMA1_Stream7_IRQn);
    __NVIC_EnableIRQ(DMA1_Stream2_IRQn);
    DMA1_Stream4->CR |= 1;
    DMA1_Stream5->CR |= 1;
    DMA1_Stream7->CR |= 1;
    DMA1_Stream2->CR |= 1;
    TIM3->EGR |= (1 << 6); // Initialize an update
    TIM3->CR1 |= 1;        // Start the timer
}

void StopMotors() {
    DMA1_Stream4->CR &= ~1;
    DMA1_Stream5->CR &= ~1;
    DMA1_Stream7->CR &= ~1;
    DMA1_Stream2->CR &= ~1;
    __NVIC_DisableIRQ(DMA1_Stream4_IRQn);
    __NVIC_DisableIRQ(DMA1_Stream5_IRQn);
    __NVIC_DisableIRQ(DMA1_Stream7_IRQn);
    __NVIC_DisableIRQ(DMA1_Stream2_IRQn);
    while (DMA1_Stream4->CR & 1 && DMA1_Stream5->CR & 1 &&
           DMA1_Stream7->CR & 1 && DMA1_Stream2->CR & 1) {
    }
    TIM3->CR1 &= ~1; // Stop the timer
}

void SetMotorThrottle(dshotMotor *motor, uint16_t throttle) {

    // Cap throttle to valid range
    if(throttle > 2048)
        throttle = 2048;
    ConstructDshotFrame(motor, throttle);
}

// This function can be used to keep the motors running in the background
void IdleMotors() {
    DMA1_Stream4->CR |= (1<<8); // Enable circular mode
    NVIC_DisableIRQ(DMA1_Stream4_IRQn);
    DMA1_Stream5->CR |= (1<<8); // Enable circular mode 
    NVIC_DisableIRQ(DMA1_Stream5_IRQn);
    DMA1_Stream7->CR |= (1<<8); // Enable circular mode 
    NVIC_DisableIRQ(DMA1_Stream7_IRQn);
    DMA1_Stream2->CR |= (1<<8); // Enable circular mode 
    NVIC_DisableIRQ(DMA1_Stream2_IRQn);
}

// This function is used to restart the normal motor control after idling
void StopIdleMotors() {
    DMA1_Stream4->CR &= ~(1<<8); // Disable circular mode
    NVIC_EnableIRQ(DMA1_Stream4_IRQn);
    DMA1_Stream5->CR &= ~(1<<8); // Disable circular mode 
    NVIC_EnableIRQ(DMA1_Stream5_IRQn);
    DMA1_Stream7->CR &= ~(1<<8); // Disable circular mode 
    NVIC_EnableIRQ(DMA1_Stream7_IRQn);
    DMA1_Stream2->CR &= ~(1<<8); // Disable circular mode 
    NVIC_EnableIRQ(DMA1_Stream2_IRQn);
}

// Motor 1 dma interrupt handler
void DMA1_Stream4_IRQHandler() {
    if (DMA1->HISR & (1 << 5)) // If transfer complete interrupt
    {
        DMA1->HIFCR |= (1 << 5); // Clear the interrupt

        if (motor1->updateBuffer) {
            for (int i = 0; i < dmaTransferSize; i++) {
                motor1->dshotBuffer[i] = motor1->dshotBuffer2[i];
            }
            motor1->updateBuffer = false;
        }
        while (DMA1_Stream4->CR & 1) {
        }
        DMA1_Stream4->NDTR = dmaTransferSize;
        DMA1_Stream4->M0AR = (uint32_t)motor1->dshotBuffer;

        GPIOB->MODER |= (2 << 8);
        DMA1_Stream4->CR |= 1;
    } else if (DMA1->HISR & (1 << 4)) // If half transfer interrupt
    {
        DMA1->HIFCR |= (1 << 4); // Clear the interrupt

        GPIOB->MODER &= ~(3 << 8); // Set pin to input
        EXTI->PR |= (1 << 4);      // Clear the interrupt
        // __NVIC_EnableIRQ(EXTI4_IRQn);
    }
}

// Motor 2 dma interrupt handler
void DMA1_Stream5_IRQHandler() {
    if (DMA1->HISR & (1 << 11)) // If transfer complete interrupt
    {
        DMA1->HIFCR |= (1 << 11); // Clear the interrupt

        if (motor2->updateBuffer) {
            for (int i = 0; i < dmaTransferSize; i++) {
                motor2->dshotBuffer[i] = motor2->dshotBuffer2[i];
            }
            motor2->updateBuffer = false;
        }
        while (DMA1_Stream5->CR & 1) {
        }
        DMA1_Stream5->NDTR = dmaTransferSize;
        DMA1_Stream5->M0AR = (uint32_t)motor2->dshotBuffer;

        GPIOB->MODER |= (2 << 10);
        DMA1_Stream5->CR |= 1;
    } else if (DMA1->HISR & (1 << 10)) // If half transfer interrupt
    {
        DMA1->HIFCR |= (1 << 10); // Clear the interrupt

        GPIOB->MODER &= ~(3 << 10); // Set pin to input
        EXTI->PR |= (1 << 5);       // Clear the interrupt
        // __NVIC_EnableIRQ(EXTI9_5_IRQn);
    }
}

// Motor 3 dma interrupt handler
void DMA1_Stream7_IRQHandler() {
    if (DMA1->HISR & (1 << 27)) // If transfer complete interrupt
    {
        DMA1->HIFCR |= (1 << 27); // Clear the interrupt

        if (motor3->updateBuffer) {
            for (int i = 0; i < dmaTransferSize; i++) {
                motor3->dshotBuffer[i] = motor3->dshotBuffer2[i];
            }
            motor3->updateBuffer = false;
        }
        while (DMA1_Stream7->CR & 1) {
        }
        DMA1_Stream7->NDTR = dmaTransferSize;
        DMA1_Stream7->M0AR = (uint32_t)motor3->dshotBuffer;

        GPIOB->MODER |= (2 << 0);
        DMA1_Stream7->CR |= 1;
    } else if (DMA1->HISR & (1 << 26)) // If half transfer interrupt
    {
        DMA1->HIFCR |= (1 << 26); // Clear the interrupt

        GPIOB->MODER &= ~(3 << 0); // Set pin to input
        EXTI->PR |= (1 << 0);      // Clear the interrupt
        // __NVIC_EnableIRQ(EXTI0_IRQn);
    }
}

// Motor 4 dma interrupt handler
void DMA1_Stream2_IRQHandler() {
    // If transfer complete interrupt
    if (DMA1->LISR & (1 << 21)) {
        DMA1->LIFCR |= (1 << 21); // Clear the interrupt

        if (motor4->updateBuffer) {
            for (int i = 0; i < dmaTransferSize; i++) {
                motor4->dshotBuffer[i] = motor4->dshotBuffer2[i];
            }
            motor4->updateBuffer = false;
        }
        while (DMA1_Stream2->CR & 1) {
        }
        DMA1_Stream2->NDTR = dmaTransferSize;
        DMA1_Stream2->M0AR = (uint32_t)motor4->dshotBuffer;

        GPIOB->MODER |= (2 << 2);
        DMA1_Stream2->CR |= 1;
    } else if (DMA1->LISR & (1 << 20)) // If half transfer interrupt
    {
        DMA1->LIFCR |= (1 << 20); // Clear the interrupt

        GPIOB->MODER &= ~(3 << 2); // Set pin to input
        EXTI->PR |= (1 << 1);      // Clear the interrupt
        // __NVIC_EnableIRQ(EXTI1_IRQn);
    }
}

