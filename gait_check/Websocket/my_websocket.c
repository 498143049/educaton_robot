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
// �ر����Ӳ���ʼ������
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
    // ������ʼ��
    ESP8266 = esp8266;
    TCPConnection = 0;
    FrameBufferLength = 0;
    LastTime = (uint32_t)-1;
    State = WS_STATE_CLOSED;
    ParseFrame = false;
    // ESP��Ϊ80�˿ڼ���
    while (ESP8266_ServerEnable(ESP8266, 80) != ESP_OK) ;
    // ESP����2���������Զ��Ͽ�
    while (ESP8266_SetServerTimeout(ESP8266, 2) != ESP_OK) ;
}

void MY_WSServer_OnTCPDataReceived(ESP8266_Connection_t *conn, const uint8_t *pdata, uint32_t length) {
    if (conn != TCPConnection) {
        // ���յ��������ӵ�����
        SEGGER_RTT_WriteString(0, "WS: Recved other conn data\r\n");
    } else if (FrameBufferLength + length <= MAX_FRAME_SIZE) {
        // ���յ������ݴ��뻺����
        memcpy(FrameBuffer + FrameBufferLength, pdata, length);
        FrameBufferLength += length;
        FrameBuffer[FrameBufferLength] = '\0';
        ParseFrame = true;
    } else {
        // ��������, ֡���������
        AbortAndLog("buffer full");
    }
}

void MY_WSServer_OnTCPConnected(ESP8266_Connection_t *conn) {
    // ���µ�TCP����
    if (TCPConnection == NULL) {
        // ��δ��������
        TCPConnection = conn;
        LastTime = HAL_GetTick();
        State = WS_STATE_OPENING;
        SEGGER_RTT_WriteString(0, "WS: Connected\r\n");
    } else {
        // �����ѽ���, �ܾ��µ�����
        ESP8266_CloseConnection_Blocking(ESP8266, conn);
        SEGGER_RTT_WriteString(0, "WS: Warning! Only one conn is allowed. New conn is closed\r\n");
    }
}

void MY_WSServer_OnTCPDisconnected(ESP8266_Connection_t *conn) {
    // �ѶϿ�TCP����
    if (conn == TCPConnection) {
        if (State == WS_STATE_CLOSING) {
            // �����Ͽ�
            TCPConnection = 0;
            ParseFrame = false;
            FrameBufferLength = 0;
            SEGGER_RTT_WriteString(0, "WS: Disconnected\r\n");
            LastTime = HAL_GetTick();
            State = WS_STATE_CLOSED;
            MS_WSServer_Callback_OnWSClosed();
        } else if (State == WS_STATE_CLOSED) {
            // �ظ��Ͽ�
            AbortAndLog("repeat disconnect");
        } else {
            // �쳣�Ͽ�
            AbortAndLog("remote disconnected unexpectly");
        }
    } else {
        SEGGER_RTT_WriteString(0, "WS: Other conn disconnected\r\n");
    }
}

void MY_WSServer_OnMain(void) {
    if (State == WS_STATE_OPENING) {
        // ���ڽ�������
        if (HAL_GetTick() - LastTime > HANDSHAKE_TIMEOUT) {
            // ��������, ���ֳ�ʱ
            AbortAndLog("handshake timeout");
        } else if (ParseFrame) {
            // ���Խ������ְ�
            enum wsFrameType type = wsParseHandshake(FrameBuffer, FrameBufferLength, &hs);
            switch (type) {
            case WS_INCOMPLETE_FRAME:
                // ���ְ�������, �����ȴ�
                break;
            case WS_OPENING_FRAME:
                // ���յ����ְ�
                // �������ֻظ�
                SEGGER_RTT_WriteString(0, "WS: Hanshake received\r\n");
                FrameBufferLength = MAX_FRAME_SIZE;
                wsGetHandshakeAnswer(&hs, FrameBuffer, &FrameBufferLength);
                ESP8266_Result_t result;
                result = ESP8266_RequestSendData_Blocking(ESP8266, TCPConnection, (char *)FrameBuffer, FrameBufferLength);
                if (result == ESP_BUSY) {
                    // ESP8266��æ, �����ȴ�
                } else if (result == ESP_OK && TCPConnection != NULL) {
                    // ���ͳɹ�
                    ParseFrame = false;
                    LastTime = HAL_GetTick();
                    State = WS_STATE_NORMAL;
                    SEGGER_RTT_WriteString(0, "WS: Hanshake success\r\n");
                    // ���ý��ջ���
                    FrameBufferLength = 0;
                    FrameBuffer[FrameBufferLength] = '\0';
                } else {
                    // ��������, �޷��������ְ�
                    AbortAndLog("can't send handshake");
                }
                break;
            case WS_ERROR_FRAME:
            default:
                // ��������, ���յ��������ְ�
                AbortAndLog("bad handshake");
                break;
            }
        }
    } else if (State == WS_STATE_NORMAL) { 
        // �����ѽ���
        while (ParseFrame && FrameBufferLength) {
            // ��TCP����δ����
            // ����Ƿ���յ�һ֡
            uint8_t *data;
            size_t dataLength;
            size_t readedLength;
            enum wsFrameType type = wsParseInputFrame(FrameBuffer, FrameBufferLength, &data, &dataLength, &readedLength);
            switch (type) {
            case WS_INCOMPLETE_FRAME:
                // ����֡������, �ȴ�������
                ParseFrame = false;
                break;
            case WS_TEXT_FRAME:
                // ���յ��ı�֡
            case WS_BINARY_FRAME:
                // ���յ�����֡
                MY_WSServer_CallBack_OnFrameReceived(type, data, dataLength);
                // �ӻ�������ȡ���Ѷ�ȡ����һ֡����
                if (FrameBufferLength > readedLength) {
                    memmove(FrameBuffer, FrameBuffer + readedLength, FrameBufferLength - readedLength);
                    FrameBufferLength -= readedLength;
                } else {
                    FrameBufferLength = 0;
                    ParseFrame = false;
                }
                break;
            case WS_CLOSING_FRAME:
                // ���յ��ر�֡, �ظ��ر�֡
                SEGGER_RTT_WriteString(0, "WS: close frame recved\r\n");
                State = WS_STATE_CLOSING;
                FrameBufferLength = MAX_FRAME_SIZE;
                wsMakeFrame(NULL, 0, FrameBuffer, &FrameBufferLength, WS_CLOSING_FRAME);
                ESP8266_Result_t result;
                 while (true) {
                    result = ESP8266_RequestSendData_Blocking(ESP8266, TCPConnection, (char *)FrameBuffer, FrameBufferLength);
                    if (result == ESP_BUSY) {
                        // ESP8266��æ, �����ȴ�
                    } else if (result == ESP_OK) {
                        // ���ͳɹ�
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
                        // ��������, �޷����͹ر�֡
                        AbortAndLog("send close reply");
                        break;
                    }
                }
                break;
            case WS_ERROR_FRAME:
                // ��������, �����Ͽ�����
                AbortAndLog("frame error detected");
                break;
            default:
                // ��������, �����Ͽ�����
                AbortAndLog("bad frame detected");
                break;
            }
        }
    } else if (State == WS_STATE_CLOSING) {
        // ���ڹر�����
        if (HAL_GetTick() - LastTime > CLOSE_TIMEOUT) {
            // ��������, �رճ�ʱ, �����Ͽ�����
            AbortAndLog("wait close frame timeout");
        } else if (ParseFrame) {
            // ����Ƿ���յ�һ֡
            uint8_t *data;
            size_t dataLength;
            size_t readedLength;
            enum wsFrameType type = wsParseInputFrame(FrameBuffer, FrameBufferLength, &data, &dataLength, &readedLength);
            switch (type) {
            case WS_INCOMPLETE_FRAME:
                // ֡������, �ȴ�������
                ParseFrame = false;
                break;
            case WS_TEXT_FRAME:
                // ���յ��ı�֡
            case WS_BINARY_FRAME:
                // ���յ�����֡
                MY_WSServer_CallBack_OnFrameReceived(type, data, dataLength);
                break;
            case WS_CLOSING_FRAME:
                // ���յ��ر�֡, �Ͽ�����
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
                // ��������, �����Ͽ�����
                AbortAndLog("bad frame detected");
                break;
            }
        }
    }
}

static bool needReplyCMD = true;
static char ReplyCMD[100];
//��е�۵��˶� 
const uint8_t* robot_code[9]={ (const uint8_t *)"\xaa\xaa\x04\x49\x01\x00\x00\xb6",
                              (const uint8_t *)"\xaa\xaa\x04\x49\x01\x00\x01\xb5",
                              (const uint8_t *)"\xaa\xaa\x04\x49\x01\x00\x02\xb4",
                              (const uint8_t *)"\xaa\xaa\x04\x49\x01\x00\x03\xb3",
                              (const uint8_t *)"\xaa\xaa\x04\x49\x01\x00\x04\xb2",
                              (const uint8_t *)"\xaa\xaa\x04\x49\x01\x00\x05\xb1",
                              (const uint8_t *)"\xaa\xaa\x04\x49\x01\x00\x06\xb0",
                              (const uint8_t *)"\xaa\xaa\x04\x49\x01\x00\x07\xaf",
                              (const uint8_t *) "\xaa\xaa\x04\x49\x01\x00\x08\xae"};
//צ�ӵ�״̬
const uint8_t* claw_code[2]={ (const uint8_t *)"\xaa\xaa\x04\x3f\x01\x01\x01\xbe",
                              (const uint8_t *)"\xaa\xaa\x04\x3f\x01\x01\x00\xbf"};

__weak void MY_WSServer_CallBack_OnFrameReceived(enum wsFrameType frameType, uint8_t *PayLoad, uint32_t PayLoadLength) {
    // ���յ�һ֡
    if (frameType == WS_TEXT_FRAME) {
        SEGGER_RTT_printf(0, "WS: Text frame received. \"%s\"\r\n", PayLoad);
        float speed, speeddiff;
        uint8_t robot,claw;
        if (sscanf((char *)PayLoad, "set %f %f %c %c ", &speed, &speeddiff,&robot,&claw) == 4) {
            Control_DesiredSpeed = speed;
            Control_DesiredDiff = speeddiff;
            //�������ݷ���
            HAL_UART_Transmit(&huart4, ( uint8_t *)robot_code[0], 8, 1000);     //ֹͣ�˶�
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
    // WS�����ѶϿ�
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

