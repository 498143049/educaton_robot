#include "DMACircularBuffer.h"

uint8_t DMACircularBuffer_Buffer[DMACircularBuffer_Size+3];

void DMACircularBuffer_Start(hdma_usart1_rx *hdma) {
    HAL_UART_Receive_DMA(hdma);
}

