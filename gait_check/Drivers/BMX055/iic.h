/*******************************************************************************
 * @name    : I2C接口驱动头文件
 * @author  : 布谷鸟
 * @web     : WWW.UCORTEX.COM
 * @version : V1.0
 * @date    : 2014-04-03
 * @MDK     : KEIL MDK4.72a & KEL MDK 5.10
 * @brief   : 实现了GPIO模拟I2C接口，带I2C波特率设置
 * ---------------------------------------------------------------------------- 
 * @copyright
 *
 * UCORTEX版权所有，违者必究！例程源码仅供大家参考，旨在交流学习和节省开发时间，
 * 对于因为参考本文件内容，导致您的产品直接或间接受到破坏，或涉及到法律问题，作
 * 者不承担任何责任。对于使用过程中发现的问题，如果您能在WWW.UCORTEX.COM网站反
 * 馈给我们，我们会非常感谢，并将对问题及时研究并作出改善。例程的版本更新，将不
 * 做特别通知，请您自行到WWW.UCORTEX.COM下载最新版本，谢谢。
 * 对于以上声明，UCORTEX保留进一步解释的权利！
 * ----------------------------------------------------------------------------
 * @description
 * 该驱动程序使用通用GPIO实现I2C通信，并且带I2C速度设置(1Kbps~400Kbps)，在不同
 * I2C从设备对速度要求不一样的时候，可以在访问前，设置与之匹配的速度，访问结束
 * 后，再恢复原来的波特率。
 *-----------------------------------------------------------------------------
 * @history
 * ----------------------------------------------------------------------------
 * 更改时间：2014-04-06    更改人：布谷鸟
 * 版本记录：V1.0
 * 更改内容：新建
 * ----------------------------------------------------------------------------
 *
 ******************************************************************************/
#ifndef __IIC_H
#define __IIC_H

#include "stm32f1xx_hal.h"
#include "bitband.h"

#define I2C_SPEED_1K        5000    //默认速度，根据处理器速度设置，这里处理器速度是72MHz

//I2C端口定义
#define I2C_SCL    GPIOout(GPIOB, 10)    //SCL
#define I2C_SDA    GPIOout(GPIOB, 11)    //SDA     
#define READ_SDA   GPIOin( GPIOB, 11)    //输入SDA

//设置PB11输入输出
#define SDA_IN()  {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=8<<12;}
#define SDA_OUT() {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=3<<12;}

typedef enum
{
    I2C_SUCCESS = 0,
    I2C_TIMEOUT,
    I2C_ERROR,
}I2C_StatusTypeDef;

extern uint32_t i2c_speed;    //I2C访问速度 = I2C_SPEED_1K / i2c_speed

/* ---------------------------依照I2C协议编写的时序函数------------------------------*/
void I2C_Init(void);                //初始化I2C的IO口                 
void I2C_Start(void);                //发送I2C开始信号
void I2C_Stop(void);                //发送I2C停止信号
uint8_t I2C_Wait_ACK(void);    //I2C等待ACK信号
void I2C_ACK(void);                    //I2C发送ACK信号
void I2C_NACK(void);                //I2C不发送ACK信号
void I2C_Send_Byte(uint8_t data);        //I2C发送一个字节
uint8_t I2C_Read_Byte(uint8_t ack);    //I2C读取一个字节
uint16_t I2C_SetSpeed(uint16_t speed);//设置I2C速度(1Kbps~400Kbps,speed单位，Kbps)

/* ---------------------------以下部分是封装好的I2C读写函数--------------------------- */

//具体到某一个器件，请仔细阅读器件规格书关于I2C部分的说明，因为某些器件在I2C的读写操作会
//有一些差异，下面的代码我们在绝大多数的I2C器件中，都是验证OK的！
I2C_StatusTypeDef I2C_WriteOneByte(uint8_t DevAddr, uint8_t DataAddr, uint8_t Data);//向I2C从设备写入一个字节
I2C_StatusTypeDef I2C_WriteBurst(uint8_t DevAddr, uint8_t DataAddr, uint8_t* pData, uint32_t Num);//向I2C从设备连续写入Num个字节
I2C_StatusTypeDef I2C_ReadOneByte(uint8_t DevAddr, uint8_t DataAddr, uint8_t* Data);//从I2C从设备读取一个字节
I2C_StatusTypeDef I2C_ReadBurst(uint8_t DevAddr, uint8_t DataAddr, uint8_t* pData, uint32_t Num);//从I2C设备连续读取Num个字节
I2C_StatusTypeDef I2C_WriteBit(uint8_t DevAddr, uint8_t DataAddr, uint8_t Bitx, uint8_t BitSet);

#endif

/********************* (C) COPYRIGHT 2014 WWW.UCORTEX.COM **********END OF FILE**********/

