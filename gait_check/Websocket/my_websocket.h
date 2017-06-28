#ifndef __MY_WEBSOCKET
#define __MY_WEBSOCKET

#include "stdint.h"
#include "esp8266.h"
#include "websocket.h"

void MY_WSServer_Init(ESP8266_t *esp8266);

// 接收到TCP数据
void MY_WSServer_OnTCPDataReceived(ESP8266_Connection_t *conn, const uint8_t *pdata, uint32_t length);

// TCP连接建立
void MY_WSServer_OnTCPConnected(ESP8266_Connection_t *conn);

// TCP连接断开
void MY_WSServer_OnTCPDisconnected(ESP8266_Connection_t *conn);

// 主函数调用
void MY_WSServer_OnMain(void);

// 接收到数据帧时回调
void MY_WSServer_CallBack_OnFrameReceived(enum wsFrameType frameType, uint8_t *PayLoad, uint32_t PayLoadLength);

// WS连接断开时回调
void MS_WSServer_Callback_OnWSClosed(void);

// 发送数据帧
int32_t MY_WSServer_SendFrame(enum wsFrameType frameType, uint8_t *PayLoad, uint32_t PayLoadLength);
#endif
