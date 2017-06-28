#ifndef __DMAFIFO_H
#define __DMAFIFO_H

#include <stdbool.h>
#include "stm32f1xx_hal.h"

// ��������С
#define DMAFIFO_Size  6000
extern UART_HandleTypeDef *DMAFIFO_phuart;
// ��������ָ��
extern uint8_t *DMAFIFO_rp;
// ������
extern uint8_t DMAFIFO_Buffer[DMAFIFO_Size+4];

void DMAFIFO_Init(UART_HandleTypeDef phuart);
// ��������DMA����
void DMAFIFO_Start();
// ������ͷָ��
#define DMAFIFO_sp    DMAFIFO_Buffer
// ������βָ��
#define DMAFIFO_ep      (DMAFIFO_Buffer + DMAFIFO_Size)
// ������дָ��
#define DMAFIFO_wp \
        (DMAFIFO_sp \
         + (DMAFIFO_Size - DMAFIFO_phuart->hdmarx->Instance->CNDTR))
// ��ȡ�ɶ��ֽ���
uint32_t DMAFIFO_AvailableBytes() {
    uint8_t *wp = DMAFIFO_wp;
    return ((wp >= DMAFIFO_rp) 
            ? wp - DMAFIFO_rp 
            : wp + DMAFIFO_Size - DMAFIFO_rp);
}
// ��ȡ�����������ֽ���
uint32_t DMAFIFO_ContinuouslyBytes() {
    uint8_t *wp = DMAFIFO_wp;
    return ((wp >= DMAFIFO_rp) 
            ? wp - DMAFIFO_rp 
            : DMAFIFO_ep - DMAFIFO_rp);
}
// ͬ��������������ֽ���ͷ�������ֽ�
#define DMAFIFO_Sync4Bytes() \
        (*(uint32_t *)DMAFIFO_ep = *(uint32_t *)DMAFIFO_sp)
// ����ָ���ַ�, ����ƥ��ָ���ַ�ָ��, �����ڷ���NULL
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
// Ѱ��������ʼ��"+IPD"��ǿ���"\r\n"��"������ʼ��"> ", ���ص�һ��ƥ���ַ�����ʼλ��, �����ڷ���-1
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
            // �ҵ�"+IPD"
        } else if (bytes >= 2 && *(uint16_t *)p == *(uint16_t *)"> ") {
            // �ҵ�"> "
        } else {
            
        }
    }
}
// ����ָ����ǰ�ƽ�n���ֽ�
bool DMAFIFO_ReadOut(uint32_t n);
// ��ջ�����
void DMAFIFO_Clear();

#endif
