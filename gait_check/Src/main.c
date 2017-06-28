/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "SEGGER_RTT.h"
#include "string.h"
#include "bmx055.h"
#include "esp8266.h"
#include "Motor.h"
#include "Encoder.h"
#include "control.h"
#include "my_esp8266.h"
#include "my_websocket.h"
#include "bmx055.h"
#include "attitude.h"
#include "dr.h"
#include "main.h"

#include "signal.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

DMA_HandleTypeDef hdma_memtomem_dma1_channel1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

uint32_t byteswritten;                /* File write counts */
uint32_t bytesread;                   /* File read counts */
char wtext[1024*10]; /* File write buffer */
uint8_t rtext[100];                     /* File read buffers */
char filename[] = "STM.txt";

bmx055_t bmx055;
ESP8266_t ESP8266;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_UART4_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void MY_I2C_BusyWorkround(void) {
    GPIO_InitTypeDef GPIO_InitStruct;
    if (__HAL_I2C_GET_FLAG(&hi2c2, I2C_FLAG_BUSY) != RESET) {
        // 1. Disable the I2C peripheral by clearing the PE bit in I2Cx_CR1 register
        __HAL_I2C_DISABLE(&hi2c2);
        // 2. Configure the SCL and SDA I/Os as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
        HAL_GPIO_WritePin(IMU_SCL_GPIO_Port, IMU_SCL_Pin|IMU_SDA_Pin, GPIO_PIN_SET);
        GPIO_InitStruct.Pin = IMU_SCL_Pin|IMU_SDA_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(IMU_SCL_GPIO_Port, &GPIO_InitStruct);
        HAL_Delay(1);
        // 3. Check SCL and SDA High level in GPIOx_IDR.
        if (HAL_GPIO_ReadPin(IMU_SCL_GPIO_Port, IMU_SCL_Pin) == GPIO_PIN_RESET
            || HAL_GPIO_ReadPin(IMU_SDA_GPIO_Port, IMU_SDA_Pin) == GPIO_PIN_RESET) {
            Error_Handler();
        }
        // 4. Configure the SDA I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
        HAL_GPIO_WritePin(IMU_SDA_GPIO_Port, IMU_SDA_Pin, GPIO_PIN_RESET);
        HAL_Delay(1);
        // 5. Check SDA Low level in GPIOx_IDR.
        if (HAL_GPIO_ReadPin(IMU_SDA_GPIO_Port, IMU_SDA_Pin) != GPIO_PIN_RESET) {
            Error_Handler();
        }
        // 6. Configure the SCL I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
        HAL_GPIO_WritePin(IMU_SCL_GPIO_Port, IMU_SCL_Pin, GPIO_PIN_RESET);
        HAL_Delay(1);
        // 7. Check SCL Low level in GPIOx_IDR.
        if (HAL_GPIO_ReadPin(IMU_SCL_GPIO_Port, IMU_SCL_Pin) != GPIO_PIN_RESET) {
            Error_Handler();
        }
        // 8. Configure the SCL I/O as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
        HAL_GPIO_WritePin(IMU_SCL_GPIO_Port, IMU_SCL_Pin, GPIO_PIN_SET);
        HAL_Delay(1);
        // 9. Check SCL High level in GPIOx_IDR.
        if (HAL_GPIO_ReadPin(IMU_SCL_GPIO_Port, IMU_SCL_Pin) != GPIO_PIN_SET) {
            Error_Handler();
        }
        // 10. Configure the SDA I/O as General Purpose Output Open-Drain , High level (Write 1 to GPIOx_ODR).
        HAL_GPIO_WritePin(IMU_SDA_GPIO_Port, IMU_SDA_Pin, GPIO_PIN_SET);
        HAL_Delay(1);
        // 11. Check SDA High level in GPIOx_IDR.
        if (HAL_GPIO_ReadPin(IMU_SDA_GPIO_Port, IMU_SDA_Pin) != GPIO_PIN_SET) {
            Error_Handler();
        }
        // 12. Configure the SCL and SDA I/Os as Alternate function Open-Drain.
        GPIO_InitStruct.Pin = IMU_SCL_Pin|IMU_SDA_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(IMU_SCL_GPIO_Port, &GPIO_InitStruct);
        // 13. Set SWRST bit in I2Cx_CR1 register.
        SET_BIT(hi2c2.Instance->CR1, I2C_CR1_SWRST);
        // 14. Clear SWRST bit in I2Cx_CR1 register.
        CLEAR_BIT(hi2c2.Instance->CR1, I2C_CR1_SWRST);
        // 15. Enable the I2C peripheral by setting the PE bit in I2Cx_CR1 register
        __HAL_I2C_ENABLE(&hi2c2);
        SEGGER_RTT_printf(0, "I2C BUSY WORKAROUND Finish\r\n");
    } else {
        SEGGER_RTT_printf(0, "I2C BUSY WORKAROUND not run\r\n");
    }
}

// 基于Systick获取从开机到现在的精确时间, 单位: 秒
double MY_GetClock(void) {
#include "core_cm3.h"
    static int32_t last_val = 0;
    static uint32_t last_ticks = 0;
    static double last_ret = 0;

    uint32_t ticks = HAL_GetTick();
    int32_t val = SysTick->VAL;
    double ret;
    
    if (last_ticks == ticks && val > last_val) {
        ticks++;
    } else if (last_ticks > ticks) {
        ticks++;
    }
    ret = ticks * 0.001 + (SysTick->LOAD - val) / 72000000.0;
    last_val = val;
    last_ticks = ticks;
    last_ret = ret;
    
    return ret;
}

static char databuf[2000];
static char *pdatabuf = databuf;
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
    SEGGER_RTT_printf(0, "\r\n\r\n==============================System Start==============================\r\n");
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_UART4_Init();

  /* USER CODE BEGIN 2 */
    // 点亮红色LED
    HAL_GPIO_WritePin(GPIO_GLED_GPIO_Port, GPIO_GLED_Pin, GPIO_PIN_RESET);
    // 启动电机PWM输出
    Motor_Init(&htim2, &htim2);
    
    SEGGER_RTT_WriteString(0, "BMX055 init...");
    while (bmx055_init(&bmx055) != 0);
    SEGGER_RTT_WriteString(0, "OK\r\n");
    
    MY_ESP8266_Init(&ESP8266, "HC5761", "shuixiang", 115200);
    MY_WSServer_Init(&ESP8266);
    Encoder_Init(&htim3, &htim4);
    Control_Init();
    Attitude_Init();
    DR_Init();
    
//    SEGGER_RTT_WriteString(0, "Connecting to PC...");
//    ESP8266_Connection_t* conn;
//    if (ESP8266_StartClientConnectionTCP_Blocking(&ESP8266, &conn, "default", "192.168.123.186", 23, NULL) == ESP_OK) {
//        SEGGER_RTT_WriteString(0, "OK\r\n");
//    } else {
//        SEGGER_RTT_WriteString(0, "FAIL\r\n");
//    }
    
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    char text[100];
    bool needSendTextFrame = false;
    bool output_data = false;
    uint32_t per1000ms = HAL_GetTick();
    uint32_t per250ms = HAL_GetTick();
    uint32_t per100ms = HAL_GetTick();
    uint32_t per50ms = HAL_GetTick();
    uint32_t per10ms = HAL_GetTick();
    uint32_t lastSent = HAL_GetTick();
    uint32_t countMain = 0;
    uint32_t countSend = 0;
    uint32_t countFrame = 0;
    double clock = 0;
    double lastclock = 0;
    while (1) {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
        ESP8266_Update(&ESP8266);
        MY_WSServer_OnMain();
        if (HAL_GetTick() >= per1000ms) {
            per1000ms += 1000;
            sprintf(text, "Sent: %dHz Frame: %dHz Main: %dHz\r\n", countSend, countFrame, countMain);
            SEGGER_RTT_WriteString(0, text);
            if (MY_WSServer_SendFrame(WS_TEXT_FRAME, (uint8_t*)text, strlen(text)) == 0) {
                needSendTextFrame = false;
            } else {
                needSendTextFrame = true;
            }
            countMain = 0;
            countSend = 0;
            countFrame = 0;
        }
        if (HAL_GetTick() >= per250ms) {
            per250ms += 250;
            HAL_GPIO_TogglePin(GPIO_GLED_GPIO_Port, GPIO_GLED_Pin);
//            char text[100];
//            sprintf(text, "L=%.2f(%.2f) R=%.2f(%.2f)\r\n", Encoder_SpeedLeft, Encoder_SumLeft,Encoder_SpeedRight,  Encoder_SumRight);
//            SEGGER_RTT_WriteString(0, text);
//            MY_WSServer_SendFrame(WS_TEXT_FRAME, (uint8_t*)text, strlen(text));
        }
        if (HAL_GetTick() >= per100ms) {
            per100ms += 100;
            output_data = true;
        }
        if (HAL_GetTick() >= per50ms) {
            per50ms += 50;
            BMX055_ReadMag(&bmx055);
//            sprintf(text, "acc: %.3f %.3f %.3f, gyro: %.1f %.1f %.1f, mag: %.0f %.0f %.0f\r\n", 
//                    bmx055.ax, bmx055.ay, bmx055.az, 
//                    bmx055.gx, bmx055.gy, bmx055.gz, 
//                    bmx055.mx, bmx055.my, bmx055.mz);
//            SEGGER_RTT_WriteString(0, text);
        }
        if (HAL_GetTick() >= per10ms) {
            per10ms += 10;
            Encoder_Measure();
            Control_Main();
            DR_Update();
            BMX055_ReadAccGyro(&bmx055);
            Attitude_UpdateGyro();
            Attitude_UpdateAcc();
        }
        if (output_data && pdatabuf - databuf + 40 * 4 <= 2000) {
            output_data = false;
            countSend++;
            int32_t check_sum = 0;
            float tmp;
#define pushfloat(dat)    {tmp = (float)(dat); *(float*)pdatabuf = tmp; pdatabuf += 4; check_sum ^= *((int32_t*)&tmp);}
            pushfloat(MY_GetClock());
            // 角速度, 1-3
            pushfloat(bmx055.gx);
            pushfloat(bmx055.gy);
            pushfloat(bmx055.gz);
            // 重力, 4-6
            pushfloat(bmx055.ax);
            pushfloat(bmx055.ay);
            pushfloat(bmx055.az);
            // 磁力, 7-9
            pushfloat(bmx055.mx);
            pushfloat(bmx055.my);
            pushfloat(bmx055.mz);
            // 轮速, 10-12
            pushfloat(Encoder_SpeedLeft);
            pushfloat(Encoder_SpeedRight);
            pushfloat(0);
            // 欧拉角, 13-15
            pushfloat(Attitude_EulerAngle.Roll / PI * 180);
            pushfloat(Attitude_EulerAngle.Pitch / PI * 180);
            pushfloat(Attitude_EulerAngle.Yaw / PI * 180);
            // 电机占空比, 34-35
            pushfloat(Motor_GetDutyLeft());
            pushfloat(Motor_GetDutyRight());
            // 根据左右轮积分得到的航向角(°), 36
            pushfloat(0);
//            pushfloat(DR_EncoderYaw);
            // 航位推算数据
            // 芯片净加速度(载体坐标系), 22-24
            pushfloat(0);
            pushfloat(0);
            pushfloat(0);
//            pushfloat(DR_AccChip.X);
//            pushfloat(DR_AccChip.Y);
//            pushfloat(DR_AccChip.Z);
            // 转换到轴中点的加速度(载体坐标系), 31-33
            pushfloat(0);
            pushfloat(0);
            pushfloat(0);
//            pushfloat(DR_AccCar.X);
//            pushfloat(DR_AccCar.Y);
//            pushfloat(DR_AccCar.Z);
            // 方法一估计速度, 19-21
            pushfloat(0);
            pushfloat(0);
            pushfloat(0);
//            pushfloat(DR_SpeedCar.X);
//            pushfloat(DR_SpeedCar.Y);
//            pushfloat(DR_SpeedCar.Z);
            // 方法一估计位置, 25-27
            pushfloat(0);
            pushfloat(0);
            pushfloat(0);
//            pushfloat(DR_Position.X);
//            pushfloat(-DR_Position.Y);
//            pushfloat(DR_Position.Z);
            // 编码器+航向角的推算位置, 16-18
//            pushfloat(0);
//            pushfloat(0);
//            pushfloat(0);
            pushfloat(DR_EncoderYawPosition.X);
            pushfloat(DR_EncoderYawPosition.Y);
            pushfloat(DR_EncoderYawPosition.Z);
            // 编码器的推算位置, 28-30
            pushfloat(0);
            pushfloat(0);
            pushfloat(0);
//            pushfloat(DR_EncoderPosition.X);
//            pushfloat(-DR_EncoderPosition.Y);
//            pushfloat(DR_EncoderPosition.Z);
            // 运动状态
            pushfloat(0);
            pushfloat(0);
//            pushfloat(DR_StandStill ? 1.0f : 0.0f);
//            pushfloat(DR_Tilt ? 1.0f : 0.0f);
//            pushfloat(DR_Skid ? 1.0f : 0.0f);
            *(int32_t*)pdatabuf = check_sum; pdatabuf += 4;
        }
        if (needSendTextFrame) {
            if (MY_WSServer_SendFrame(WS_TEXT_FRAME, (uint8_t*)text, strlen(text)) == 0) {
                needSendTextFrame = false;
            } else {
                needSendTextFrame = true;
            }
        } else if (pdatabuf > databuf) {
            if (MY_WSServer_SendFrame(WS_BINARY_FRAME, (uint8_t *)databuf, pdatabuf - databuf) == 0) {
                countFrame++;
                pdatabuf = databuf;
                lastSent = HAL_GetTick() + 10;
            }
        }
        countMain++;
        // 检查定时器
//        clock = MY_GetClock();
//        SEGGER_RTT_printf(0, "%d\r\n", (int32_t)((clock - lastclock) * 1000000));
//        lastclock = clock;
        // 检查电池电压
//        HAL_GPIO_WritePin(BAT_VGND_GPIO_Port, BAT_VGND_Pin, GPIO_PIN_RESET);
//        if (HAL_ADC_Start(&hadc1) == HAL_OK) {
//            if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
//                float voltage = HAL_ADC_GetValue(&hadc1) / 65536.0f * 3.3 * 2.02 - 0.01;
//                char tmp[20];
//                sprintf(tmp, "battery voltage: %.3f\r\n", voltage);
//                SEGGER_RTT_WriteString(0, tmp);
//            }
//            HAL_ADC_Stop(&hadc1);
//        }
//        HAL_GPIO_WritePin(BAT_VGND_GPIO_Port, BAT_VGND_Pin, GPIO_PIN_SET);
    }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_LEFT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* I2C2 init function */
static void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 7200;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* UART4 init function */
static void MX_UART4_Init(void)
{

  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}

/** 
  * Enable DMA controller clock
  * Configure DMA for memory to memory transfers
  *   hdma_memtomem_dma1_channel1
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* Configure DMA request hdma_memtomem_dma1_channel1 on DMA1_Channel1 */
  hdma_memtomem_dma1_channel1.Instance = DMA1_Channel1;
  hdma_memtomem_dma1_channel1.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_memtomem_dma1_channel1.Init.PeriphInc = DMA_PINC_ENABLE;
  hdma_memtomem_dma1_channel1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_memtomem_dma1_channel1.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_memtomem_dma1_channel1.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_memtomem_dma1_channel1.Init.Mode = DMA_NORMAL;
  hdma_memtomem_dma1_channel1.Init.Priority = DMA_PRIORITY_LOW;
  if (HAL_DMA_Init(&hdma_memtomem_dma1_channel1) != HAL_OK)
  {
    Error_Handler();
  }
        
  

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_GLED_Pin|SERIAL_CTL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIO_RLED_GPIO_Port, GPIO_RLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BAT_VGND_GPIO_Port, BAT_VGND_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : GPIO_GLED_Pin GPIO_RLED_Pin SERIAL_CTL_Pin */
  GPIO_InitStruct.Pin = GPIO_GLED_Pin|GPIO_RLED_Pin|SERIAL_CTL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BAT_STDBY_Pin BAT_CHRG_Pin GPIO_SELECT1_Pin GPIO_SELECT2_Pin */
  GPIO_InitStruct.Pin = BAT_STDBY_Pin|BAT_CHRG_Pin|GPIO_SELECT1_Pin|GPIO_SELECT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BAT_VGND_Pin */
  GPIO_InitStruct.Pin = BAT_VGND_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BAT_VGND_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_ENTER_Pin IMU_DRDY_Pin IMU_INT_Pin */
  GPIO_InitStruct.Pin = GPIO_ENTER_Pin|IMU_DRDY_Pin|IMU_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: SEGGER_RTT_printf(0, "Wrong parameters value: file %s on line %d\r\n", file, line) */
    SEGGER_RTT_printf(0, "Wrong parameters value: file %s on line %d\r\n", file, line);
    Error_Handler();
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
