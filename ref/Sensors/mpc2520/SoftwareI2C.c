/*******************************************************************************
* Copyright (C), 2000-2016,  Maier Technology Co., Ltd.
* 文件名: SoftwareI2C.c
* 作  者: Jason.Li
* 版  本:
* 日  期: 2016-1-8     //完成日期
* 说  明: 基于MCU IO控制的I2C程序
* 修订历史:       
*    1. 时间: 2016-1-8
*       修订者: Jason.Li
*       修订内容: 创建
*    2.
* 其它:           //
*******************************************************************************/

#ifndef SOFTWARE_I2C_C
#define SOFTWARE_I2C_C

#include "SoftwareI2C.h"
#include "stm32f37x.h"

// #define PIN_SCL P0_4
// #define PIN_SDA P0_3
// #define SCL_OUTPUT() P0DIR|=(1<<4)
// #define SDA_INPUT() P0DIR&=~(1<<3)
// #define SDA_OUTPUT() P0DIR|=(1<<3)

//===========================================================================
//配置I2C硬件接口定义
#define TWI_SCL_PIN       		GPIO_Pin_6
#define TWI_SCL_GPIO_PORT 		GPIOB
#define TWI_SCL_GPIO_CLK  		RCC_AHBPeriph_GPIOB
#define TWI_SCl_SOURCE     		GPIO_PinSource6


#define TWI_SDA_PIN       		GPIO_Pin_7
#define TWI_SDA_GPIO_PORT 		GPIOB
#define TWI_SDA_GPIO_CLK  		RCC_AHBPeriph_GPIOB
#define TWI_SDA_SOURCE    		GPIO_PinSource7


static __inline void TWI_SCL_0(void)  	{TWI_SCL_GPIO_PORT->BRR = TWI_SCL_PIN;}
static __inline void TWI_SCL_1(void)  	{TWI_SCL_GPIO_PORT->BSRR = TWI_SCL_PIN;}
static __inline void TWI_SDA_0(void)  	{TWI_SDA_GPIO_PORT->BRR = TWI_SDA_PIN;}
static __inline void TWI_SDA_1(void)  	{TWI_SDA_GPIO_PORT->BSRR = TWI_SDA_PIN;}

static __inline u8 TWI_SDA_STATE(void) 	{return (TWI_SDA_GPIO_PORT->IDR & TWI_SDA_PIN) ? 1 : 0;}
//======================================================================================

//=================================================================
//设置GPIO为输入
//
//=================================================================
void Input_pin_sda(void)
{
		GPIO_InitTypeDef  GPIO_InitStructure;
		
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;	
//		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	
//		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Pin = TWI_SDA_PIN;
		GPIO_Init(GPIOB, &GPIO_InitStructure);	
}

//==================================================================
//设置输出SDA
//
//===================================================================
void out_pin_sda(void)
{
		GPIO_InitTypeDef  GPIO_InitStructure;
		
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Pin = TWI_SDA_PIN;
		GPIO_Init(GPIOB, &GPIO_InitStructure);	
}

//==================================================================
//设置输出SCL
//
//===================================================================
void out_pin_scl(void)
{
		GPIO_InitTypeDef  GPIO_InitStructure;
		
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Pin = TWI_SCL_PIN;
		GPIO_Init(GPIOB, &GPIO_InitStructure);	
}

#define DelayTick 0

void SoftI2C_delay(uint8 i)
{
    while(i--);
}

void SoftI2C_stop()
{
    //PIN_SCL = 1;
		TWI_SCL_1();
    SoftI2C_delay(DelayTick);
    //PIN_SDA = 1;
		TWI_SDA_1();
    SoftI2C_delay(DelayTick);
}
void SoftI2C_start()
{
    //SCL_OUTPUT();
		out_pin_scl();
    //SDA_OUTPUT();
		out_pin_sda();
    //PIN_SDA = 1;
		TWI_SDA_1();
    SoftI2C_delay(DelayTick);
    //PIN_SCL = 1;
		TWI_SCL_1();
    //PIN_SDA = 0;
		TWI_SDA_0();
    SoftI2C_delay(DelayTick);
    //PIN_SCL = 0;
		TWI_SCL_0();
}
void SoftI2C_write_byte(uint8 value)
{
    uint8 i;
    uint8 result = 0;
    i = 8;
    while (i)
    {
        if (value & 0x80)
            //PIN_SDA = 1;
						TWI_SDA_1();
        else
            //PIN_SDA = 0;
						TWI_SDA_0();
        SoftI2C_delay(DelayTick);
        //PIN_SCL = 1;
				TWI_SCL_1();
        i--;
        value <<= 1;
        SoftI2C_delay(DelayTick);
        //PIN_SCL = 0;
				TWI_SCL_0();
        SoftI2C_delay(DelayTick);
    }
    //SDA_INPUT();        //input
		Input_pin_sda();
    SoftI2C_delay(DelayTick);

    //PIN_SCL = 1;
		TWI_SCL_1();
    SoftI2C_delay(DelayTick);
    //if (TWI_SDA_STATE)//PIN_SDA
		if(GPIO_ReadInputDataBit(GPIOB, TWI_SDA_PIN))
    {
        result = 1;
    }
    //PIN_SCL = 0;
		TWI_SCL_0();
    //PIN_SDA = 0;
		TWI_SDA_0();
    //SDA_OUTPUT();   //output
		out_pin_sda();
}
uint8 SoftI2C_read_byte(uint8 ack)
{
    uint8 i;
    uint8 result = 0;
    i = 8;
    //SDA_INPUT();        //input
		Input_pin_sda();
    while (i)
    {
        result <<= 1;
        //PIN_SCL = 1;
				TWI_SCL_1();
        i--;
        SoftI2C_delay(DelayTick);
        //if (TWI_SDA_STATE)//PIN_SDA
				if(GPIO_ReadInputDataBit(GPIOB, TWI_SDA_PIN))
        {
            result |= 0x01;
        }
        //PIN_SCL = 0;
				TWI_SCL_0();

        SoftI2C_delay(DelayTick);
    }
    //SDA_OUTPUT();   //output
		out_pin_sda();
    if (ack)
        //PIN_SDA = 1;
				TWI_SDA_1();
    else
        //PIN_SDA = 0;
				TWI_SDA_0();
    SoftI2C_delay(DelayTick);
    //PIN_SCL = 1;
		TWI_SCL_1();
    SoftI2C_delay(DelayTick);
    //PIN_SCL = 0;
		TWI_SCL_0();
    //PIN_SDA = 0;
		TWI_SDA_0();
    SoftI2C_delay(3);
    return result;
}

#endif

