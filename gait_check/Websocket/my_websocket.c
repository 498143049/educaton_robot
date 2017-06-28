#include "string.h"
#include "stdbool.h"
#include "stm32f1xx_hal.h"
#include "SEGGER_RTT.h"
#include "my_websocket.h"
#include "Motor.h"
#include "Control.h"

#define HANDSHAKE_TIMEOUT   2000
#define CLOSE_TIMEOUT       2000
#define MAX_FRAME_SIZE      4096

static uint8_t FrameBuffer[MAX_FRAME_SIZE+1];
static uint32_t FrameBufferLength;
static uint32_t LastTime;
static enum wsState State;
static struct handshake hs;
static ESP8266_Connection_t *TCPConnection;
static ESP8266_t *ESP8266;
static bool ParseFrame;

extern UART_HandleTypeDef huart4;
// 关闭连接并初始化变量
static void AbortAndLog(char *errinfo) {
    if (errinfo) {
        SEGGER_RTT_printf(0, "WS: Error! %s\r\n", errinfo);
    } else {
        SEGGER_RTT_WriteString(0, "WS: Error!\r\n");
    }
    if (State != WS_STATE_CLOSED) {
        State = WS_STATE_CLOSING;
        ESP8266_CloseConnection_Blocking(ESP8266, TCPConnection);
        LastTime = HAL_GetTick();
        State = WS_STATE_CLOSED;
    }
    TCPConnection = 0;
    FrameBufferLength = 0;
    ParseFrame = false;
    MS_WSServer_Callback_OnWSClosed();
}

void MY_WSServer_Init(ESP8266_t *esp8266) {
    // 变量初始化
    ESP8266 = esp8266;
    TCPConnection = 0;
    FrameBufferLength = 0;
    LastTime = (uint32_t)-1;
    State = WS_STATE_CLOSED;
    ParseFrame = false;
    // ESP设为80端口监听
    while (ESP8266_ServerEnable(ESP8266, 80) != ESP_OK) ;
    // ESP连接2秒无数据自动断开
    while (ESP8266_SetServerTimeout(ESP8266, 2) != ESP_OK) ;
}

void MY_WSServer_OnTCPDataReceived(ESP8266_Connection_t *conn, const uint8_t *pdata, uint32_t length) {
    if (conn != TCPConnection) {
        // 接收到其他连接的数据
        SEGGER_RTT_WriteString(0, "WS: Recved other conn data\r\n");
    } else if (FrameBufferLength + length <= MAX_FRAME_SIZE) {
        // 接收到的数据存入缓冲区
        memcpy(FrameBuffer + FrameBufferLength, pdata, length);
        FrameBufferLength += length;
        FrameBuffer[FrameBufferLength] = '\0';
        ParseFrame = true;
    } else {
        // 发生错误, 帧缓冲区溢出
        AbortAndLog("buffer full");
    }
}

void MY_WSServer_OnTCPConnected(ESP8266_Connection_t *conn) {
    // 有新的TCP连接
    if (TCPConnection == NULL) {
        // 还未建立连接
        TCPConnection = conn;
        LastTime = HAL_GetTick();
        State = WS_STATE_OPENING;
        SEGGER_RTT_WriteString(0, "WS: Connected\r\n");
    } else {
        // 连接已建立, 拒绝新的连接
        ESP8266_CloseConnection_Blocking(ESP8266, conn);
        SEGGER_RTT_WriteString(0, "WS: Warning! Only one conn is allowed. New conn is closed\r\n");
    }
}

void MY_WSServer_OnTCPDisconnected(ESP8266_Connection_t *conn) {
    // 已断开TCP连接
    if (conn == TCPConnection) {
        if (State == WS_STATE_CLOSING) {
            // 正常断开
            TCPConnection = 0;
            ParseFrame = false;
            FrameBufferLength = 0;
            SEGGER_RTT_WriteString(0, "WS: Disconnected\r\n");
            LastTime = HAL_GetTick();
            State = WS_STATE_CLOSED;
            MS_WSServer_Callback_OnWSClosed();
        } else if (State == WS_STATE_CLOSED) {
            // 重复断开
            AbortAndLog("repeat disconnect");
        } else {
            // 异常断开
            AbortAndLog("remote disconnected unexpectly");
        }
    } else {
        SEGGER_RTT_WriteString(0, "WS: Other conn disconnected\r\n");
    }
}

void MY_WSServer_OnMain(void) {
    if (State == WS_STATE_OPENING) {
        // 正在建立连接
        if (HAL_GetTick() - LastTime > HANDSHAKE_TIMEOUT) {
            // 发生错误, 握手超时
            AbortAndLog("handshake timeout");
        } else if (ParseFrame) {
            // 尝试解析握手包
            enum wsFrameType type = wsParseHandshake(FrameBuffer, FrameBufferLength, &hs);
            switch (type) {
            case WS_INCOMPLETE_FRAME:
                // 握手包不完整, 继续等待
                break;
            case WS_OPENING_FRAME:
                // 接收到握手包
                // 发送握手回复
                SEGGER_RTT_WriteString(0, "WS: Hanshake received\r\n");
                FrameBufferLength = MAX_FRAME_SIZE;
                wsGetHandshakeAnswer(&hs, FrameBuffer, &FrameBufferLength);
                ESP8266_Result_t result;
                result = ESP8266_RequestSendData_Blocking(ESP8266, TCPConnection, (char *)FrameBuffer, FrameBufferLength);
                if (result == ESP_BUSY) {
                    // ESP8266繁忙, 继续等待
                } else if (result == ESP_OK && TCPConnection != NULL) {
                    // 发送成功
                    ParseFrame = false;
                    LastTime = HAL_GetTick();
                    State = WS_STATE_NORMAL;
                    SEGGER_RTT_WriteString(0, "WS: Hanshake success\r\n");
                    // 重置接收缓冲
                    FrameBufferLength = 0;
                    FrameBuffer[FrameBufferLength] = '\0';
                } else {
                    // 发生错误, 无法发送握手包
                    AbortAndLog("can't send handshake");
                }
                break;
            case WS_ERROR_FRAME:
            default:
                // 发生错误, 接收到错误握手包
                AbortAndLog("bad handshake");
                break;
            }
        }
    } else if (State == WS_STATE_NORMAL) { 
        // 连接已建立
        while (ParseFrame && FrameBufferLength) {
            // 有TCP数据未处理
            // 检查是否接收到一帧
            uint8_t *data;
            size_t dataLength;
            size_t readedLength;
            enum wsFrameType type = wsParseInputFrame(FrameBuffer, FrameBufferLength, &data, &dataLength, &readedLength);
            switch (type) {
            case WS_INCOMPLETE_FRAME:
                // 数据帧不完整, 等待新数据
                ParseFrame = false;
                break;
            case WS_TEXT_FRAME:
                // 接收到文本帧
            case WS_BINARY_FRAME:
                // 接收到数据帧
                MY_WSServer_CallBack_OnFrameReceived(type, data, dataLength);
                // 从缓冲区中取出已读取的这一帧数据
                if (FrameBufferLength > readedLength) {
                    memmove(FrameBuffer, FrameBuffer + readedLength, FrameBufferLength - readedLength);
                    FrameBufferLength -= readedLength;
                } else {
                    FrameBufferLength = 0;
                    ParseFrame = false;
                }
                break;
            case WS_CLOSING_FRAME:
                // 接收到关闭帧, 回复关闭帧
                SEGGER_RTT_WriteString(0, "WS: close frame recved\r\n");
                State = WS_STATE_CLOSING;
                FrameBufferLength = MAX_FRAME_SIZE;
                wsMakeFrame(NULL, 0, FrameBuffer, &FrameBufferLength, WS_CLOSING_FRAME);
                ESP8266_Result_t result;
                 while (true) {
                    result = ESP8266_RequestSendData_Blocking(ESP8266, TCPConnection, (char *)FrameBuffer, FrameBufferLength);
                    if (result == ESP_BUSY) {
                        // ESP8266繁忙, 继续等待
                    } else if (result == ESP_OK) {
                        // 发送成功
                        LastTime = HAL_GetTick();
                        if (State != WS_STATE_CLOSED) {
                            ESP8266_CloseConnection_Blocking(ESP8266, TCPConnection);
                            State = WS_STATE_CLOSED;
                            MS_WSServer_Callback_OnWSClosed();
                        }
                        TCPConnection = 0;
                        FrameBufferLength = 0;
                        ParseFrame = false;
                        SEGGER_RTT_WriteString(0, "WS: close reply success\r\n");
                        break;
                    } else {
                        // 发生错误, 无法发送关闭帧
                        AbortAndLog("send close reply");
                        break;
                    }
                }
                break;
            case WS_ERROR_FRAME:
                // 发生错误, 立即断开连接
                AbortAndLog("frame error detected");
                break;
            default:
                // 发生错误, 立即断开连接
                AbortAndLog("bad frame detected");
                break;
            }
        }
    } else if (State == WS_STATE_CLOSING) {
        // 正在关闭连接
        if (HAL_GetTick() - LastTime > CLOSE_TIMEOUT) {
            // 发生错误, 关闭超时, 立即断开连接
            AbortAndLog("wait close frame timeout");
        } else if (ParseFrame) {
            // 检查是否接收到一帧
            uint8_t *data;
            size_t dataLength;
            size_t readedLength;
            enum wsFrameType type = wsParseInputFrame(FrameBuffer, FrameBufferLength, &data, &dataLength, &readedLength);
            switch (type) {
            case WS_INCOMPLETE_FRAME:
                // 帧不完整, 等待新数据
                ParseFrame = false;
                break;
            case WS_TEXT_FRAME:
                // 接收到文本帧
            case WS_BINARY_FRAME:
                // 接收到数据帧
                MY_WSServer_CallBack_OnFrameReceived(type, data, dataLength);
                break;
            case WS_CLOSING_FRAME:
                // 接收到关闭帧, 断开连接
                ParseFrame = false;
                LastTime = HAL_GetTick();
                if (State != WS_STATE_CLOSED) {
                    ESP8266_CloseConnection_Blocking(ESP8266, TCPConnection);
                    State = WS_STATE_CLOSED;
                    MS_WSServer_Callback_OnWSClosed();
                }
                TCPConnection = 0;
                FrameBufferLength = 0;
                break;
            case WS_ERROR_FRAME:
            default:
                // 发生错误, 立即断开连接
                AbortAndLog("bad frame detected");
                break;
            }
        }
    }
}

static bool needReplyCMD = true;
static char ReplyCMD[100];
//机械臂的运动 
const uint8_t* robot_code[9]={ (const uint8_t *)"\xaa\xaa\x04\x49\x01\x00\x00\xb6",
                              (const uint8_t *)"\xaa\xaa\x04\x49\x01\x00\x01\xb5",
                              (const uint8_t *)"\xaa\xaa\x04\x49\x01\x00\x02\xb4",
                              (const uint8_t *)"\xaa\xaa\x04\x49\x01\x00\x03\xb3",
                              (const uint8_t *)"\xaa\xaa\x04\x49\x01\x00\x04\xb2",
                              (const uint8_t *)"\xaa\xaa\x04\x49\x01\x00\x05\xb1",
                              (const uint8_t *)"\xaa\xaa\x04\x49\x01\x00\x06\xb0",
                              (const uint8_t *)"\xaa\xaa\x04\x49\x01\x00\x07\xaf",
                              (const uint8_t *) "\xaa\xaa\x04\x49\x01\x00\x08\xae"};
//爪子的状态
const uint8_t* claw_code[2]={ (const uint8_t *)"\xaa\xaa\x04\x3f\x01\x01\x01\xbe",
                              (const uint8_t *)"\xaa\xaa\x04\x3f\x01\x01\x00\xbf"};

__weak void MY_WSServer_CallBack_OnFrameReceived(enum wsFrameType frameType, uint8_t *PayLoad, uint32_t PayLoadLength) {
    // 接收到一帧
    if (frameType == WS_TEXT_FRAME) {
        SEGGER_RTT_printf(0, "WS: Text frame received. \"%s\"\r\n", PayLoad);
        float speed, speeddiff;
        uint8_t robot,claw;
        if (sscanf((char *)PayLoad, "set %f %f %c %c ", &speed, &speeddiff,&robot,&claw) == 4) {
            Control_DesiredSpeed = speed;
            Control_DesiredDiff = speeddiff;
            //进行数据发送
            HAL_UART_Transmit(&huart4, ( uint8_t *)robot_code[0], 8, 1000);     //停止运动
            HAL_UART_Transmit(&huart4, ( uint8_t *)robot_code[robot-'0'], 8, 1000);
            HAL_UART_Transmit(&huart4, ( uint8_t *)claw_code[claw-'0'], 8, 1000);
            sprintf(ReplyCMD, "{\"speed\":%.3f,\"speeddiff\":%.3f,\"robot\":%c,\"claw\":%c}", speed, speeddiff,robot,claw);
            needReplyCMD = true;
            SEGGER_RTT_printf(0, "CMD: speed=%f speeddiff=%f robot=%c claw=%c\r\n", speed, speeddiff,robot,claw);
        } else if (strcmp((char *)PayLoad, "reset") == 0) {
            SEGGER_RTT_printf(0, "CMD: reset\r\n");
            NVIC_SystemReset();
        }
    } else if (frameType == WS_BINARY_FRAME) {
        SEGGER_RTT_printf(0, "WS: Binary frame received. len = %d\r\n", PayLoadLength);
    }
}

__weak void MS_WSServer_Callback_OnWSClosed(void) {
    // WS连接已断开
//    Control_DesiredSpeed = 0;
//    Control_DesiredDiff = 0;
//    Motor_SetDutyLeft(0);
//    Motor_SetDutyRight(0);
}

#define SEND_FRAME_SIZE 4096
static uint8_t SendFrame[SEND_FRAME_SIZE];
static uint32_t SendFrameLength;

int32_t MY_WSServer_SendFrame(enum wsFrameType frameType, uint8_t *PayLoad, uint32_t PayLoadLength) {
    if (State != WS_STATE_NORMAL) {
        return -2;
    }
    
    if (ESP8266->ActiveCommand != 0) {
        return -3;
    }
    int32_t ret = 0;
    if (needReplyCMD) {
        SendFrameLength = SEND_FRAME_SIZE;
        wsMakeFrame((uint8_t *)ReplyCMD, strlen(ReplyCMD), SendFrame, &SendFrameLength, WS_TEXT_FRAME);
        TCPConnection->Flags.F.Blocking = 1;                             /* Set as blocking */
        TCPConnection->BlockingData = (char *)SendFrame;                      /* Set pointer to blocking data */
        TCPConnection->BlockingDataLength = SendFrameLength;                      /* Set blocking data length */
        if (ESP8266_RequestSendData(ESP8266, TCPConnection) == ESP_OK) {
            needReplyCMD = false;
        } else {
            needReplyCMD = true;
        }
        return -4;
    } else {
        SendFrameLength = SEND_FRAME_SIZE;
        wsMakeFrame(PayLoad, PayLoadLength, SendFrame, &SendFrameLength, frameType);
        TCPConnection->Flags.F.Blocking = 1;                             /* Set as blocking */
        TCPConnection->BlockingData = (char *)SendFrame;                      /* Set pointer to blocking data */
        TCPConnection->BlockingDataLength = SendFrameLength;                      /* Set blocking data length */
        if (ESP8266_RequestSendData(ESP8266, TCPConnection) == ESP_OK) {
            return 0;
        } else {
            return -1;
        }
    }
}

