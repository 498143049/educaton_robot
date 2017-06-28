/**	
 * |----------------------------------------------------------------------
 * | Copyright (C) Tilen Majerle, 2015
 * | 
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |  
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * | 
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * |----------------------------------------------------------------------
 */
#include "esp8266_ll.h"

extern UART_HandleTypeDef huart1;
uint8_t ESP8266_LL_USARTInit(uint32_t baudrate) {
	/* Init USART */
    huart1.Instance = USART1;
    huart1.Init.BaudRate = baudrate;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	/* Return 0 = Successful */
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        return 1;
    } else {
        return 0;
    }
}

uint8_t ESP8266_LL_USARTSend(uint8_t* data, uint16_t count) {
	/* Send data via USART */
	/* Return 0 = Successful */
//    uint8_t binary = 0;
//    for (uint32_t i = 0; i < count; i++) {
//        if (data[i] > 127) {
//            binary = 1;
//            break;
//        }
//    }
//    if (!binary) {
//        SEGGER_RTT_WriteString(0, ">");
//        SEGGER_RTT_Write(0, data, count);
//    } else {
//        SEGGER_RTT_WriteString(0, ">[BINARY]");
//    }
	if (HAL_UART_Transmit(&huart1, data, count, 1000) == HAL_OK) {
        return 0;
    } else {
        SEGGER_RTT_WriteString(0, "UART FAIL!!!\r\n");
        while (1);
        return 1;
    }
}

/* USART receive interrupt handler */
void TM_USART1_ReceiveHandler(uint8_t ch) {
	/* Send received character to ESP stack */
	ESP8266_DataReceived(&ch, 1);
}




