#include "motor_control.h"
#include "system.h"

#define MotorStartDelay 50000000

void BusyWait(uint32_t delay) 
{
    for (volatile uint32_t i = 0; i < delay; i++);
}

int main() 
{
    SystemClock_Config_100MHz_HSE();

    LedInit();
    LedOff();
    InitMotors();
    StartMotors();

    BusyWait(MotorStartDelay);

    LedOn();

    

    while(1)
    {
        for (int i = 48; i <= 100; i++)
        {
            SetMotorThrottle(motor1, i);
            SetMotorThrottle(motor2, i);
            SetMotorThrottle(motor3, i);
            SetMotorThrottle(motor4, i);
            for(volatile int i = 0; i < 5000000; i++); // delay
            ToggleLed();
        }

        BusyWait(100000);
        for (int i = 48; i <= 500; i++)
        {
            SetMotorThrottle(motor1, i);
            SetMotorThrottle(motor2, i);
            SetMotorThrottle(motor3, i);
            SetMotorThrottle(motor4, i);
            for(volatile int i = 0; i < 5000000; i++); // delay
            ToggleLed();
        }
        
    }

    return 0;
}