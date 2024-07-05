# include "./uart/bsp_uart.h"
# include "string.h"
# include "math.h"

WT1_IMU_ANGLE WT1_IMU_AngleData;

uint8_t Angle_Arr[6];

uint8_t Serial_RxFlag;
/**
 *  @brief  调试串口 GPIO 初始化
 *  @param  无
 *  @retval 无
 */
static void DEBUG_USART_GPIO_Config(void){

    DEBUG_USART_GPIO_APBxClkCmd(DEBUG_USART_GPIO_CLK, ENABLE);

    GPIO_InitTypeDef    GPIO_InitStructure;

    /*调试串口GPIO初始化*/
    GPIO_InitStructure.GPIO_Pin = DEBUG_USART_TX_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = DEBUG_USART_RX_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStructure);
}

void DEBUg_USART_Config(void){

    DEBUG_USART_GPIO_Config();

    DEBUG_USART_APBxClkCmd(DEBUG_USART_CLK, ENABLE);
    
    USART_InitTypeDef   USART_InitStructure;
    USART_InitStructure.USART_BaudRate = DEBUG_USART_BAUDRATE;                          //配置波特率
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;     //硬件流控制选择
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;                     //串口模式
    USART_InitStructure.USART_Parity = USART_Parity_No;                                 //选择是否使用奇偶校验位
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                              //停止位长度
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                         //配置字长
    USART_Init(DEBUG_USART, &USART_InitStructure);

    /*使能串口*/
    USART_Cmd(DEBUG_USART, ENABLE);
}

/*重定向c 库函数printf 到串口，重定向后可使用printf*/
int fputc(int ch, FILE *f){

    /*发送一个字节数据到串口*/
    USART_SendData(DEBUG_USART, (uint8_t) ch);

    /*等待发送完毕*/
    while(USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TXE) == RESET);

    return (ch);
}

# if defined PCI_USART_USER

/**
 *  @brief  外设串口 GPIO 初始化
 *  @param  无
 *  @retval 无
 */
static void PCI_USART_GPIO_Config(void){

    PCI_USART_GPIO_APBxClkCmd(PCI_USART_GPIO_CLK, ENABLE);

    GPIO_InitTypeDef    GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = PCI_USART_TX_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(PCI_USART_TX_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = PCI_USART_RX_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(PCI_USART_RX_GPIO_PORT, &GPIO_InitStructure);
}

static void PCI_USART_NVIC_Config(void){

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    NVIC_InitTypeDef    NVIC_InitStructure;

    /*外设串口中断初始化*/
    NVIC_InitStructure.NVIC_IRQChannel = PCI_USART_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_Init(&NVIC_InitStructure);
}

void PCI_USART_Config(void){

    PCI_USART_GPIO_Config();

    PCI_USART_APBxClkCmd(PCI_USART_CLK, ENABLE);

    USART_InitTypeDef   USART_InitStructure;
    USART_InitStructure.USART_BaudRate = PCI_USART_BAUDRATE;                            //配置波特率
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;     //硬件流控制选择
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;                     //串口模式
    USART_InitStructure.USART_Parity = USART_Parity_No;                                 //选择是否使用奇偶校验位
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                              //停止位长度
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                         //配置字长
    USART_Init(PCI_USART, &USART_InitStructure);

    PCI_USART_NVIC_Config();

    //串口接收中断需要用的
    USART_ITConfig(PCI_USART, USART_IT_RXNE, ENABLE);

    USART_Cmd(PCI_USART, ENABLE);   
}

# endif

/**
 *  @brief  串口初始化
 *  @param  无
 *  @retval 无
 */
void MyUSART_Init(void){
    
    # if defined PCI_USART_USER
        PCI_USART_Config();
    # endif

    DEBUg_USART_Config();
}

 # if defined PCI_USART_USER

/**
 *  @brief  发送 1 字节数据
 *  @param  无
 *  @retval 无
 */
void Serial_SendByte(uint8_t Byte){
    
    USART_SendData(PCI_USART, Byte);
    while(USART_GetFlagStatus(PCI_USART, USART_FLAG_TXE) == RESET);
}

/**
 *  @brief  发送数组
 *  @param  无
 *  @retval 无
 */
void Serial_SendArray(uint8_t *Array, uint8_t Length){

    uint8_t i;
    for (i = 0; i < Length; i ++){
        Serial_SendByte(Array[i]);
    }
}

/**
 *  @brief  发送字符串
 *  @param  无
 *  @retval 无
 */
void Serial_SendString(char *String){

    uint8_t i;
    for (i = 0; String[i] != '\0'; i ++){
        Serial_SendByte(String[i]);
    }
}

/**
 *  @brief  发送次方函数
 *  @param  无
 *  @retval 无
 */
uint32_t Serial_Pow(uint32_t x, uint32_t y){

    uint32_t Result = 1;
    while(y --){
        Result *= x;
    }

    return Result;
}

/**
 *  @brief  发送一个数字
 *  @param  无
 *  @retval 无
 */
void Serial_SendNumber(uint32_t Number, uint8_t Lenght){

    uint8_t i;
    for(i = 0; i < Lenght; i ++){

        Serial_SendByte(Number/Serial_Pow(10, Lenght-i-1) % 10 + '0');
    }
}

/**
 *  @brief  获取串口接收中断标志位
 *  @param  无
 *  @retval 无
 */
uint8_t USARTx_GetRxFlag(void){

    if (Serial_RxFlag == 1){

        WT1_IMU_AngleData.Roll  = (int16_t)((Angle_Arr[1] << 8) | Angle_Arr[0]) / 32768.0f *180.0f;
        WT1_IMU_AngleData.Pitch = (int16_t)((Angle_Arr[3] << 8) | Angle_Arr[2]) / 32768.0f *180.0f;
        WT1_IMU_AngleData.Yaw   = (int16_t)((Angle_Arr[5] << 8) | Angle_Arr[4]) / 32768.0f *180.0f;
        memset(Angle_Arr, 6, 0x00);

        Serial_RxFlag = 0;
        return 1;
    }

    return 0;
}

/**
 *  @brief  外设串口中断函数
 *  @param  无
 *  @retval 无
 */
void PCI_USART_IRQHandler(void){

    static uint8_t RxState = 0;
    static uint8_t pRxPacket = 0;

    if(USART_GetITStatus(PCI_USART, USART_IT_RXNE) == SET){

        uint8_t RxData = USART_ReceiveData(PCI_USART);

        if (RxState == 0){
            if (RxData == 0x55){
                RxState = 1;
            }
            else RxState = 0;
        }
        else if (RxState == 1){
            if (RxData == 0x53){
                RxState = 2;
            }
            else RxState = 0;
        }
        else if (RxState == 2){
           
           Angle_Arr[pRxPacket] = RxData;
           pRxPacket ++;
           if (pRxPacket >= 6) {

            pRxPacket = 0;
            RxState = 0;
            Serial_RxFlag = 1;
           }
        }
        USART_ClearITPendingBit(PCI_USART, USART_IT_RXNE);
    }
}

# endif
