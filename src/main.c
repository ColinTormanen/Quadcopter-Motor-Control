#include "motor_control.h"
#include "system.h"

#define MotorStartDelay 1000000

void BusyWait(uint32_t delay) 
{
    for (volatile uint32_t i = 0; i < delay; i++);
}

int main() 
{
    SystemClock_Config_100MHz_HSE();

    LedInit();
    InitMotors();
    StartMotors();

    BusyWait(MotorStartDelay);

    LedOn();

    

    while(1)
    {
        for(volatile int i = 0; i < 100000000; i++); // delay
        SetMotorThrottle(motor1, 50);
        for(volatile int i = 0; i < 10000000; i++); // delay
        SetMotorThrottle(motor2, 50);
        for(volatile int i = 0; i < 10000000; i++); // delay
        SetMotorThrottle(motor3, 50);
        for(volatile int i = 0; i < 10000000; i++); // delay
        SetMotorThrottle(motor4, 50);
        for(volatile int i = 0; i < 10000000; i++); // delay
    }

    return 0;
}