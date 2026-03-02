#ifndef SPI_H
#define SPI_H

#include <stdint.h>

// SPI is entirely interrupt/dma driven, so we only init
void SPI_Init(void);

#endif
