#ifndef BIDSHOT_H
#define BIDSHOT_H

#include "motor_control.h"

void InitBidshot();

void Decode(int data, dshotMotor *motor);

void EXTI4_IRQHandler();

void EXTI9_5_IRQHandler();

void EXTI0_IRQHandler();

void EXTI1_IRQHandler();

void TIM4_IRQHandler();

void DMA1_Stream0_IRQHandler();

#endif