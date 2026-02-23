#ifndef SYSTEM_H
#define SYSTEM_H

// Initialize onboard LED (PC13)
void LedInit(void);

// Toggle LED state
void ToggleLed(void);

// Turn LED on
void LedOn(void);

// Turn LED off
void LedOff(void);

void SystemClock_Config_100MHz_HSE(void);

#endif // SYSTEM_H
