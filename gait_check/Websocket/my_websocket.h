#ifndef __MY_WEBSOCKET
#define __MY_WEBSOCKET

#include "stdint.h"
#include "esp8266.h"
#include "websocket.h"

void MY_WSServer_Init(ESP8266_t *esp8266);

// ���յ�TCP����
void MY_WSServer_OnTCPDataReceived(ESP8266_Connection_t *conn, const uint8_t *pdata, uint32_t length);

// TCP���ӽ���
void MY_WSServer_OnTCPConnected(ESP8266_Connection_t *conn);

// TCP���ӶϿ�
void MY_WSServer_OnTCPDisconnected(ESP8266_Connection_t *conn);

// ����������
void MY_WSServer_OnMain(void);

// ���յ�����֡ʱ�ص�
void MY_WSServer_CallBack_OnFrameReceived(enum wsFrameType frameType, uint8_t *PayLoad, uint32_t PayLoadLength);

// WS���ӶϿ�ʱ�ص�
void MS_WSServer_Callback_OnWSClosed(void);

// ��������֡
int32_t MY_WSServer_SendFrame(enum wsFrameType frameType, uint8_t *PayLoad, uint32_t PayLoadLength);
#endif
