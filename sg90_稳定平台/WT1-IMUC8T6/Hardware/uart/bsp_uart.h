# ifndef _BSP_UART_H
# define _BSP_UART_H

# include "stm32f10x.h"
# include "stdio.h"

// 注释将禁止外设串口
# define PCI_USART_USER

/****************************调试串口****************************/
#define  DEBUG_USART                        USART1
#define  DEBUG_USART_CLK                    RCC_APB2Periph_USART1
#define  DEBUG_USART_APBxClkCmd             RCC_APB2PeriphClockCmd
#define  DEBUG_USART_BAUDRATE               115200

// 外设串口 GPIO 引脚宏定义
#define  DEBUG_USART_GPIO_CLK               RCC_APB2Periph_GPIOA
#define  DEBUG_USART_GPIO_APBxClkCmd        RCC_APB2PeriphClockCmd
  
#define  DEBUG_USART_TX_GPIO_PIN            GPIO_Pin_9
#define  DEBUG_USART_TX_GPIO_PORT           GPIOA   

#define  DEBUG_USART_RX_GPIO_PIN            GPIO_Pin_10
#define  DEBUG_USART_RX_GPIO_PORT           GPIOA

#define  DEBUG_USART_IRQ                    USART1_IRQn

/****************************外设串口****************************/
# if defined PCI_USART_USER

#define  PCI_USART                          USART2
#define  PCI_USART_CLK                      RCC_APB1Periph_USART2
#define  PCI_USART_APBxClkCmd               RCC_APB1PeriphClockCmd
#define  PCI_USART_BAUDRATE                 115200

// 外设串口 GPIO 引脚宏定义
#define  PCI_USART_GPIO_CLK                 RCC_APB2Periph_GPIOA
#define  PCI_USART_GPIO_APBxClkCmd          RCC_APB2PeriphClockCmd
  
#define  PCI_USART_TX_GPIO_PIN              GPIO_Pin_2  
#define  PCI_USART_TX_GPIO_PORT             GPIOA   

#define  PCI_USART_RX_GPIO_PIN              GPIO_Pin_3
#define  PCI_USART_RX_GPIO_PORT             GPIOA

#define  PCI_USART_IRQ                      USART2_IRQn
#define  PCI_USART_IRQHandler               USART2_IRQHandler

# endif

typedef struct{

    float Roll;
    float Pitch;
    float Yaw;

}WT1_IMU_ANGLE;

/*调试串口配置*/
void MyUSART_Init(void);

/*调试串口函数*/
int fputc(int ch, FILE *f);

/*外设串口函数*/
# if defined PCI_USART_USER

void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array, uint8_t Length);
void Serial_SendString(char *String);
uint32_t Serial_Pow(uint32_t x, uint32_t y);
void Serial_SendNumber(uint32_t Number, uint8_t Lenght);
uint8_t USARTx_GetRxFlag(void);

# endif

# endif
