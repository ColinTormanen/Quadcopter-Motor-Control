#ifndef DSHOT_H
#define DSHOT_H

// Configured for DShot600
#define dshotWidth 42
#define dshotHigh 30
#define dshotLow 15

// DShot is 16 bits. The rest are used for inter-frame gap 
#define dmaTransferSize 52
#define true 1
#define false 0

#include <stdint.h>

typedef struct {
    uint16_t command[dmaTransferSize];
    uint16_t repeat;
} dshotCommand;

typedef struct {
    uint8_t updateBuffer;
    uint16_t dshotBuffer[dmaTransferSize];
    uint16_t dshotBuffer2[dmaTransferSize];
    uint16_t throttle;
    uint16_t temperature;
    uint16_t voltage;
    uint16_t current;
    uint16_t RPM;
    uint8_t CommandMode;
    dshotCommand commands[10]; // Should never need more than 10 commands
    uint8_t numCommands; // Number of commands in the buffer
    uint8_t commandIndex; // Index of the current command being executed
    uint32_t interruptCounter;
} dshotMotor;

void ConstructCommandSequence(dshotMotor* motor, uint16_t* commandValues, uint16_t* repeat, uint8_t numCommands);

void ConstructThrottle(dshotMotor* motor, uint16_t throttle);

void InitDshot(dshotMotor* motor1, dshotMotor* motor2, dshotMotor* motor3, dshotMotor* motor4);

void InitiMotor(dshotMotor* motor);

#endif