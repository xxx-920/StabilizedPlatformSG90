#ifndef _BSP_IIC_H
#define _BSP_IIC_H

# include "main.h"

//IO口操作宏定义
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum))  

# define IIC_SCL_PIN        GPIO_Pin_10
# define IIC_SCL_PORT       GPIOB
# define IIC_SCL_CLK        RCC_APB2Periph_GPIOB

# define IIC_SDA_PIN        GPIO_Pin_11
# define IIC_SDA_PORT       GPIOB
# define IIC_SDA_CLK        RCC_APB2Periph_GPIOB

# define IIC_W_SCL(x)       GPIO_WriteBit(IIC_SCL_PORT, IIC_SCL_PIN, (BitAction)(x))
# define IIC_W_SDA(x)       GPIO_WriteBit(IIC_SDA_PORT, IIC_SDA_PIN, (BitAction)(x))

# define IIC_R_SCL          GPIO_ReadInputDataBit(IIC_SCL_PORT, IIC_SCL_PIN)
# define IIC_R_SDA          GPIO_ReadInputDataBit(IIC_SDA_PORT, IIC_SDA_PIN)

//IIC所有操作函数
void IIC_Init(void);                //初始化IIC的IO口				 
void IIC_Start(void);				//发送IIC开始信号
void IIC_Stop(void);	  			//发送IIC停止信号
void IIC_Send_Byte(uint8_t Byte);			//IIC发送一个字节
uint8_t IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
uint8_t IIC_Wait_Ack(void); 				//IIC等待ACK信号
void IIC_Ack(void);					//IIC发送ACK信号
void IIC_NAck(void);				//IIC不发送ACK信号

unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr);
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data);
uint8_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data);
uint8_t IICwriteBits(uint8_t dev,uint8_t reg,uint8_t bitStart,uint8_t length,uint8_t data);
uint8_t IICwriteBit(uint8_t dev,uint8_t reg,uint8_t bitNum,uint8_t data);
uint8_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data);

uint8_t i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
uint8_t i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);

#endif

//------------------End of File----------------------------
