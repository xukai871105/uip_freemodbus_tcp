/* 引用头文件 *****************************************************************/
#include "stm32f10x.h"
#include "bsp_spi1.h"
/* 外部数据类型 ***************************************************************/
/* 外部常数宏 *****************************************************************/
/* 外部动作宏 *****************************************************************/
/* 外部变量 *******************************************************************/
/* 外部函数声明 ***************************************************************/

/*
********************************************************************************
* 函 数 名: BSP_ConfigSPI1
* 功能说明: SPI1相关配置操作，包括SPI1模式，SPI1通信速度和相关IO口
* 参    数：无
* 返 回 值: 无
* 使用说明：在BSP初始化中调用
* 调用方法：BSP_ConfigSPI1();
********************************************************************************
*/
void BSP_ConfigSPI1(void)
{
    /* GPIO结构体 */
    GPIO_InitTypeDef  GPIO_InitStructure; 
    /* SPI结构体 */
    SPI_InitTypeDef SPI_InitStructure; 
    
    /* 使能APB2上相关时钟 */
    /* 使能SPI时钟，使能GPIOA时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1 |\
        RCC_APB2Periph_GPIOA ,ENABLE );
    
    /* SPI1 SCK@GPIOA.5 SPI1 MOSI@GPIOA.7 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5  |  GPIO_Pin_7; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
    /* 复用推挽输出 */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure); 
    
    /* SPI1 MISO@GPIOA.6 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
    /* 浮动输入 */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure); 
    
    
    /* 双线双向全双工 */
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; 
	/* 主机模式 */
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master; 
    /* 8位帧结构 */
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b; 
    /* 时钟空闲时为低 */
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;        
    /* 第1个上升沿捕获数据 */
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;      
    /* MSS 端口软件控制 */
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;         
    /* SPI时钟 72Mhz / 8 = 9M */ 
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; 
    /* 数据传输高位在前 */
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; 
    
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    /* 初始化SPI1 */
    SPI_Init(SPI1, &SPI_InitStructure);
    
	/* 把使能SPI口的SS输出功能 GPIOA.4 */
	SPI_SSOutputCmd(SPI1,ENABLE);
    /* 使能SPI1 */
    SPI_Cmd(SPI1, ENABLE); 
}

/*
********************************************************************************
* 函 数 名: BSP_SPI1SendByte
* 功能说明: SPI1发送字节数据
* 参    数：uint8_t byte  发送字节
* 返 回 值: uint8_t       返回字节
* 使用说明：根据SPI1通信原理，发送字节必有返回字节
* 调用方法：BSP_SPI1SendByte(value);
********************************************************************************
*/
uint8_t BSP_SPI1SendByte(uint8_t byte)
{
    /* 等待发送缓冲寄存器为空 */
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
    /* 发送数据 */
    SPI_I2S_SendData(SPI1, byte);		
    
    /* 等待接收缓冲寄存器为非空 */
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
    
    return SPI_I2S_ReceiveData(SPI1);
    
}

/*
********************************************************************************
* 函 数 名: BSP_SPI1ReceiveByte
* 功能说明: SPI1接收字节数据
* 参    数：无
* 返 回 值: uint8_t       返回字节
* 使用说明：根据SPI1通信原理，发送字节必有返回字节，该函数和发送函数相同
* 调用方法：value = BBSP_SPI1ReceiveByte();
********************************************************************************
*/
uint8_t BSP_SPI1ReceiveByte(void)
{
    /* 等待发送缓冲寄存器为空 */
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
    /* 发送数据,通过发送0xFF,获得返回数据 */
    SPI_I2S_SendData(SPI1, 0xFF);		
    /* 等待接收缓冲寄存器为非空 */
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
    /* 返回从SPI通信中接收到的数据 */
    return SPI_I2S_ReceiveData(SPI1);
}
/***************************************************************END OF FILE****/
