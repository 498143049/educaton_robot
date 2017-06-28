#include "SEGGER_RTT.h"
#include "my_websocket.h"
#include "my_esp8266.h"

static volatile bool esp8266_got_ip = false;

void ESP8266_Callback_WifiGotIP(ESP8266_t* ESP8266) {
    esp8266_got_ip = true;
}

//void ESP8266_Callback_ClientConnectionDataReceived(ESP8266_t* ESP8266, ESP8266_Connection_t* Connection, char* Buffer) {
//    SEGGER_RTT_WriteString(0, Buffer);
//}

/**********************************/
/* Callbacks for Websocket Server */
/**********************************/
void ESP8266_Callback_ServerConnectionActive(ESP8266_t* ESP8266, ESP8266_Connection_t* Connection) {
    MY_WSServer_OnTCPConnected(Connection);
}
void ESP8266_Callback_ServerConnectionDataReceived(ESP8266_t* ESP8266, ESP8266_Connection_t* Connection, char* Buffer) {
    MY_WSServer_OnTCPDataReceived(Connection, (uint8_t *)Buffer, Connection->DataSize);
}
void ESP8266_Callback_ServerConnectionClosed(ESP8266_t* ESP8266, ESP8266_Connection_t* Connection) {
    MY_WSServer_OnTCPDisconnected(Connection);
}

void MY_ESP8266_Init(ESP8266_t* ESP8266, const char *ssid, const char *pwd, uint32_t baudrate) {
    // ��ʼ����ESP8266������
    SEGGER_RTT_WriteString(0, "ESP8266 init...");
    while (ESP8266_Init(ESP8266, 115200) != ESP_OK) {
        SEGGER_RTT_WriteString(0, "ESP8266 init fail");
    }
    // ����ΪSTAģʽ, �����Զ�����WiFi
    while (ESP8266_SetMode(ESP8266, ESP8266_Mode_STA) != ESP_OK) ;
    while (ESP8266_SetAutoConnect(ESP8266, ESP8266_AutoConnect_Off) != ESP_OK) ;
    // �ȴ����ӳɹ�����ȡ��ip
    while (!esp8266_got_ip) {
        ESP8266_WifiConnect(ESP8266, ssid, pwd);
        ESP8266_WaitReady(ESP8266);
    }
    // �л������ٲ�����
    while (ESP8266_SetUART(ESP8266, baudrate) != ESP_OK) ;
    // ��ȡ����ӡESP8266��IP��ַ
    while (ESP8266_GetSTAIPBlocking(ESP8266) != ESP_OK) ;
    SEGGER_RTT_printf(0, "OK(%d.%d.%d.%d)\r\n", ESP8266->STAIP[0], ESP8266->STAIP[1], ESP8266->STAIP[2], ESP8266->STAIP[3]);
}
