#include "motor_control.h"
#include "dshot.h"
#include <stdint.h>
#include <stm32f411xe.h>

// NOTE:
// This implementation has a prototype Bi-Directional DSHOT telemetry decoder

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
    if(throttle > 2047)
        throttle = 2047;
    ConstructThrottle(motor, throttle);
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

        // If we're in command mode, load the next command sequence into the buffer
        if (motor1->CommandMode && motor1->numCommands > 0)
        {
            for (int i = 0; i < dmaTransferSize; i++) {
                motor1->dshotBuffer[i] = motor1->commands[motor1->commandIndex].command[i];
            }
            motor1->commands[motor1->commandIndex].repeat--;

            // If the command has been repeated enough times, move to the next command
            if (motor1->commands[motor1->commandIndex].repeat == 0) {
                motor1->commandIndex++;
                if (motor1->commandIndex >= motor1->numCommands) {
                    motor1->CommandMode = false;
                    motor1->commandIndex = 0;
                    motor1->numCommands = 0;
                }
            }
        }
        else if (motor1->updateBuffer) { // If not command mode, check if we need to update the throttle
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
        __NVIC_DisableIRQ(EXTI4_IRQn);
    } else if (DMA1->HISR & (1 << 4)) // If half transfer interrupt
    {
        DMA1->HIFCR |= (1 << 4); // Clear the interrupt

        GPIOB->MODER &= ~(3 << 8); // Set pin to input
        EXTI->PR |= (1 << 4);      // Clear the interrupt
        __NVIC_EnableIRQ(EXTI4_IRQn);
    }
}

// Motor 2 dma interrupt handler
void DMA1_Stream5_IRQHandler() {
    if (DMA1->HISR & (1 << 11)) // If transfer complete interrupt
    {
        DMA1->HIFCR |= (1 << 11); // Clear the interrupt

        if (motor2->CommandMode && motor2->numCommands > 0)
        {
            for (int i = 0; i < dmaTransferSize; i++) {
                motor2->dshotBuffer[i] = motor2->commands[motor2->commandIndex].command[i];
            }
            motor2->commands[motor2->commandIndex].repeat--;

            if (motor2->commands[motor2->commandIndex].repeat == 0) {
                motor2->commandIndex++;
                if (motor2->commandIndex >= motor2->numCommands) {
                    motor2->CommandMode = false;
                    motor2->commandIndex = 0;
                    motor2->numCommands = 0;
                }
            }
        }
        else if (motor2->updateBuffer) {
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
        __NVIC_DisableIRQ(EXTI9_5_IRQn);
    } else if (DMA1->HISR & (1 << 10)) // If half transfer interrupt
    {
        DMA1->HIFCR |= (1 << 10); // Clear the interrupt

        GPIOB->MODER &= ~(3 << 10); // Set pin to input
        EXTI->PR |= (1 << 5);       // Clear the interrupt
        __NVIC_EnableIRQ(EXTI9_5_IRQn);
    }
}

// Motor 3 dma interrupt handler
void DMA1_Stream7_IRQHandler() {
    if (DMA1->HISR & (1 << 27)) // If transfer complete interrupt
    {
        DMA1->HIFCR |= (1 << 27); // Clear the interrupt

        if (motor3->CommandMode && motor3->numCommands > 0)
        {
            for (int i = 0; i < dmaTransferSize; i++) {
                motor3->dshotBuffer[i] = motor3->commands[motor3->commandIndex].command[i];
            }
            motor3->commands[motor3->commandIndex].repeat--;

            if (motor3->commands[motor3->commandIndex].repeat == 0) {
                motor3->commandIndex++;
                if (motor3->commandIndex >= motor3->numCommands) {
                    motor3->CommandMode = false;
                    motor3->commandIndex = 0;
                    motor3->numCommands = 0;
                }
            }
        }
        else if (motor3->updateBuffer) {
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
        __NVIC_DisableIRQ(EXTI0_IRQn);
    } else if (DMA1->HISR & (1 << 26)) // If half transfer interrupt
    {
        DMA1->HIFCR |= (1 << 26); // Clear the interrupt

        GPIOB->MODER &= ~(3 << 0); // Set pin to input
        EXTI->PR |= (1 << 0);      // Clear the interrupt
        __NVIC_EnableIRQ(EXTI0_IRQn);
    }
}

// Motor 4 dma interrupt handler
void DMA1_Stream2_IRQHandler() {
    // If transfer complete interrupt
    if (DMA1->LISR & (1 << 21)) {
        DMA1->LIFCR |= (1 << 21); // Clear the interrupt

        if (motor4->CommandMode && motor4->numCommands > 0)
        {
            for (int i = 0; i < dmaTransferSize; i++) {
                motor4->dshotBuffer[i] = motor4->commands[motor4->commandIndex].command[i];
            }
            motor4->commands[motor4->commandIndex].repeat--;

            if (motor4->commands[motor4->commandIndex].repeat == 0) {
                motor4->commandIndex++;
                if (motor4->commandIndex >= motor4->numCommands) {
                    motor4->CommandMode = false;
                    motor4->commandIndex = 0;
                    motor4->numCommands = 0;
                }
            }
        }
        else if (motor4->updateBuffer) {
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
        __NVIC_DisableIRQ(EXTI1_IRQn);
    } else if (DMA1->LISR & (1 << 20)) // If half transfer interrupt
    {
        DMA1->LIFCR |= (1 << 20); // Clear the interrupt

        GPIOB->MODER &= ~(3 << 2); // Set pin to input
        EXTI->PR |= (1 << 1);      // Clear the interrupt
        __NVIC_EnableIRQ(EXTI1_IRQn);
    }
}

