#ifndef __MY_ESP8266
#define __MY_ESP8266

#include "string.h"
#include "stdbool.h"
#include "stm32f1xx_hal.h"
#include "esp8266.h"

void MY_ESP8266_Init(ESP8266_t* ESP8266, const char *ssid, const char *pwd, uint32_t baudrate);

#endif
