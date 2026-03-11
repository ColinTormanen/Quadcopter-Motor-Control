#include "spi.h"
#include "stm32f411xe.h"

void SPI_Init(void) {
    // Enable clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // GPIO B
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;  // SPI2

    // Configure pins B13(SCK), B15(MOSI) as alternate function
    GPIOB->MODER &= ~((3 << (13 * 2)) | (3 << (15 * 2)));
    GPIOB->MODER |= (2 << (13 * 2)) | (2 << (15 * 2)); // Alternate function

    // Set alternate function to AF5 for SPI2
    GPIOB->AFR[1] &= ~((0xF << ((13 - 8) * 4)) | (0xF << ((15 - 8) * 4)));
    GPIOB->AFR[1] |= (5 << ((13 - 8) * 4)) | (5 << ((15 - 8) * 4)); // AF5 for SPI2

    // Set high speed
    GPIOB->OSPEEDR |= (3 << (13 * 2)) | (3 << (15 * 2)); // High speed


    // Configure SPI2
	SPI2->CR1 = 0;
	SPI2->CR1 |= (1<< 11); // 16 bits data frame
	SPI2->CR2 |= (1 << 0); // Enable RX DMA Request
	// SPI2->CR1 |= SPI_CR1_SSM; // Software slave management
    // SPI2->CR1 |= SPI_CR1_SSI; // Internal slave select
	// SPI2->CR1 |= (1 << 10); // RX only

	// Configure DMA for SPI2 RX
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
	DMA1_Stream3->CR = 0; // Reset the stream
	while (DMA1_Stream3->CR & 1); // Wait for the stream to be ready
	DMA1_Stream3->CR |= (0 << 25); // Channel 0 for SPI2_RX
	DMA1_Stream3->CR |= (2 << 16); // High priority
	DMA1_Stream3->CR |= (0 << 6); // Peripheral to memory
	DMA1_Stream3->CR &= ~(1 << 9); // Peripheral address is fixed
	DMA1_Stream3->CR |= (1 << 10); // Memory incremented
	DMA1_Stream3->CR |= (1 << 11); // Peripheral data size
	DMA1_Stream3->CR |= (1 << 13); // Memory data size
	DMA1_Stream3->CR |= (1 << 4); // Transfer complete interrupt enabled

	DMA1_Stream3->NDTR = 0; // Will be set when starting a transfer
	DMA1_Stream3->PAR = (uint32_t) &SPI2->DR;
	DMA1_Stream3->M0AR = 0; // Will be set when starting a transfer

	// DMA1_Stream3->CR |= 1; // Enable the DMA stream
	// SPI2->CR1 |= 1; // Enable the SPI peripheral
}
