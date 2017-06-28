/**
  ******************************************************************************
  * File Name          : mxconstants.h
  * Description        : This file contains the common defines of the application
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MXCONSTANT_H
#define __MXCONSTANT_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define MOTOR1_A_Pin GPIO_PIN_0
#define MOTOR1_A_GPIO_Port GPIOA
#define MOTOR1_B_Pin GPIO_PIN_1
#define MOTOR1_B_GPIO_Port GPIOA
#define MOTOR2_A_Pin GPIO_PIN_2
#define MOTOR2_A_GPIO_Port GPIOA
#define MOTOR2_B_Pin GPIO_PIN_3
#define MOTOR2_B_GPIO_Port GPIOA
#define GPIO_GLED_Pin GPIO_PIN_4
#define GPIO_GLED_GPIO_Port GPIOA
#define GPIO_RLED_Pin GPIO_PIN_5
#define GPIO_RLED_GPIO_Port GPIOA
#define BAT_STDBY_Pin GPIO_PIN_6
#define BAT_STDBY_GPIO_Port GPIOA
#define BAT_CHRG_Pin GPIO_PIN_7
#define BAT_CHRG_GPIO_Port GPIOA
#define BAT_VCHECK_Pin GPIO_PIN_0
#define BAT_VCHECK_GPIO_Port GPIOB
#define BAT_VGND_Pin GPIO_PIN_1
#define BAT_VGND_GPIO_Port GPIOB
#define GPIO_ENTER_Pin GPIO_PIN_2
#define GPIO_ENTER_GPIO_Port GPIOB
#define IMU_SCL_Pin GPIO_PIN_10
#define IMU_SCL_GPIO_Port GPIOB
#define IMU_SDA_Pin GPIO_PIN_11
#define IMU_SDA_GPIO_Port GPIOB
#define IMU_DRDY_Pin GPIO_PIN_14
#define IMU_DRDY_GPIO_Port GPIOB
#define IMU_INT_Pin GPIO_PIN_15
#define IMU_INT_GPIO_Port GPIOB
#define SERIAL_CTL_Pin GPIO_PIN_8
#define SERIAL_CTL_GPIO_Port GPIOA
#define SERIAL_TX_Pin GPIO_PIN_9
#define SERIAL_TX_GPIO_Port GPIOA
#define SERIAL_RX_Pin GPIO_PIN_10
#define SERIAL_RX_GPIO_Port GPIOA
#define GPIO_SELECT1_Pin GPIO_PIN_11
#define GPIO_SELECT1_GPIO_Port GPIOA
#define GPIO_SELECT2_Pin GPIO_PIN_12
#define GPIO_SELECT2_GPIO_Port GPIOA
#define ENCODER1_A_Pin GPIO_PIN_4
#define ENCODER1_A_GPIO_Port GPIOB
#define ENCODER1_B_Pin GPIO_PIN_5
#define ENCODER1_B_GPIO_Port GPIOB
#define ENCODER2_A_Pin GPIO_PIN_6
#define ENCODER2_A_GPIO_Port GPIOB
#define ENCODER2_B_Pin GPIO_PIN_7
#define ENCODER2_B_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MXCONSTANT_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
