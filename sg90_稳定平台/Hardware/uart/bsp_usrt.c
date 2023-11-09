# include "./uart/bsp_uart.h"

/*调试串口配置*/
void DEBUG_UART_Config(void){

    GPIO_InitTypeDef    GPIO_InitStructure;
    USART_InitTypeDef    USART_InitStructure;

    /*第一步：初始化GPIO*/
    /*打开GPIO 端口的时钟*/
    DEBUG_USART_GPIO_APBxClkCmd(DEBUG_USART_GPIO_CLK, ENABLE);

    /*将UART Tx 的GPIO 配置为推挽复用模式*/
    GPIO_InitStructure.GPIO_Pin = DEBUG_USART_TX_GPIO_PIN;
    /*复用模式 推挽输出*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init( DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStructure);

    /*将UART Rx 的GPIO 配置为浮空输入模式*/
    GPIO_InitStructure.GPIO_Pin = DEBUG_USART_RX_GPIO_PIN;
    /*浮空输入模式*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init( DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStructure);


    /*第二步：配置串口的初始化结构体*/
    /*打开串口外设时钟*/
    DEBUG_USART_GPIO_APBxClkCmd(DEBUG_USART_CLK, ENABLE);
    
    // 配置串口的工作参数
	// 配置波特率
	USART_InitStructure.USART_BaudRate = DEBUG_USART_BAUDRATE;
	// 配置 针数据字长
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	// 配置停止位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	// 配置校验位
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	// 配置硬件流控制
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	// 配置工作模式，收发一起
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	// 完成串口的初始化配置
	USART_Init(DEBUG_USARTx, &USART_InitStructure);

    /*第三步：使能串口*/
    USART_Cmd(DEBUG_USARTx, ENABLE);	    
}

/*重定向c 库函数printf 到串口，重定向后可使用printf*/
int fputc(int ch, FILE *f){

    /*发送一个字节数据到串口*/
    USART_SendData(DEBUG_USARTx, (uint8_t) ch);

    /*等待发送完毕*/
    while(USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_TXE) == RESET);

    return (ch);
}

