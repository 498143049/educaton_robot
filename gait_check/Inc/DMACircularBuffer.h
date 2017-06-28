#ifndef __DMAFIFO_H
#define __DMAFIFO_H

#include <stdbool.h>
#include "stm32f1xx_hal.h"

// 缓冲区大小
#define DMAFIFO_Size  6000
extern UART_HandleTypeDef *DMAFIFO_phuart;
// 缓冲区读指针
extern uint8_t *DMAFIFO_rp;
// 缓冲区
extern uint8_t DMAFIFO_Buffer[DMAFIFO_Size+4];

void DMAFIFO_Init(UART_HandleTypeDef phuart);
// 启动环形DMA接收
void DMAFIFO_Start();
// 缓冲区头指针
#define DMAFIFO_sp    DMAFIFO_Buffer
// 缓冲区尾指针
#define DMAFIFO_ep      (DMAFIFO_Buffer + DMAFIFO_Size)
// 缓冲区写指针
#define DMAFIFO_wp \
        (DMAFIFO_sp \
         + (DMAFIFO_Size - DMAFIFO_phuart->hdmarx->Instance->CNDTR))
// 获取可读字节数
uint32_t DMAFIFO_AvailableBytes() {
    uint8_t *wp = DMAFIFO_wp;
    return ((wp >= DMAFIFO_rp) 
            ? wp - DMAFIFO_rp 
            : wp + DMAFIFO_Size - DMAFIFO_rp);
}
// 获取可连续读的字节数
uint32_t DMAFIFO_ContinuouslyBytes() {
    uint8_t *wp = DMAFIFO_wp;
    return ((wp >= DMAFIFO_rp) 
            ? wp - DMAFIFO_rp 
            : DMAFIFO_ep - DMAFIFO_rp);
}
// 同步缓冲区后的四字节与头部的四字节
#define DMAFIFO_Sync4Bytes() \
        (*(uint32_t *)DMAFIFO_ep = *(uint32_t *)DMAFIFO_sp)
// 查找指定字符, 返回匹配指定字符指针, 不存在返回NULL
uint8_t *DMAFIFO_FindChar(char ch) {
    uint8_t *p = DMAFIFO_rp;
    while (p != DMAFIFO_wp) {
        if (*p == ch) {
            return p;
        }
        p++;
        if (p == DMAFIFO_ep) {
            p = DMAFIFO_sp;
        }
    }
    return NULL;
}
// 寻找新行起始的"+IPD"或非空行"\r\n"或"新行起始的"> ", 返回第一个匹配字符串开始位置, 不存在返回-1
int32_t DMAFIFO_ReadSpecial() {
    uint16_t CRLF_VAL = *(uint16_t *)"\r\n";
    uint16_t DATA_VAL = *(uint16_t *)"> ";
    uint8_t *p = DMAFIFO_rp;
    while (1) {
        uint8_t *wp = DMAFIFO_wp;
        uint32_t bytes;
        if (wp >= p) { 
            bytes = wp - p;
        } else {
            bytes = wp + DMAFIFO_Size - p;
            DMAFIFO_Sync4Bytes();
        }
        if (bytes >= 4 && *(uint32_t *)p == *(uint32_t *)"+IPD") {
            // 找到"+IPD"
        } else if (bytes >= 2 && *(uint16_t *)p == *(uint16_t *)"> ") {
            // 找到"> "
        } else {
            
        }
    }
}
// 将读指针向前推进n个字节
bool DMAFIFO_ReadOut(uint32_t n);
// 清空缓冲区
void DMAFIFO_Clear();

#endif
