/* 引用头文件 *****************************************************************/
#include "bsp_usart.h"
#include <stdio.h>
/* 私有数据类型 ***************************************************************/
/* 私有常数宏 *****************************************************************/
#define DEBUG_USART1  (1)     /* 使用USART1 进行调试 */
#define DEBUG_USART2  (0)     /* 使用USART2 进行调试 */
#define USART1_ENISR  (0)     /* 使能USART1的发送和接收中断 */
#define USART2_ENISR  (0)     /* 使能USART2的发送和接收中断 */
/* 私有动作宏 *****************************************************************/
/* 私有变量 *******************************************************************/
/* 私有函数声明 ***************************************************************/
/* 私有函数  ******************************************************************/

/*
********************************************************************************
* 函 数 名: BSP_ConfigUSART1
* 功能说明: 初始化USART1
* 参    数：无
* 返 回 值: 无
* 使用说明：系统上电初始化时调用！
* 调用方法：BSP_ConfigUSART1();
********************************************************************************
*/
void BSP_ConfigUSART1(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;

  /* 第1步：打开GPIO和USART时钟 */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);

  /* 第2步：将USART1 Tx@PA9的GPIO配置为推挽复用模式 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* 第3步：将USART1 Rx@PA10的GPIO配置为浮空输入模式 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* 第4步：配置USART1参数
      波特率   = 115200
      数据长度 = 8
      停止位   = 1
      校验位   = No
      禁止硬件流控(即禁止RTS和CTS)
      使能接收和发送
  */
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &USART_InitStructure);

  /* 第5步：使能 USART1， 配置完毕 */
  USART_Cmd(USART1, ENABLE);

  /* 清除发送完成标志，Transmission Complete flag */
  USART_ClearFlag(USART1, USART_FLAG_TC);
  
}

/*
********************************************************************************
* 函 数 名: BSP_ConfigUSART2
* 功能说明: 初始化USART2
* 参    数：无
* 返 回 值: 无
* 使用说明：系统上电初始化时调用！
* 调用方法：BSP_ConfigUSART2();
********************************************************************************
*/
void BSP_ConfigUSART2(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;

  /* 第1步：打开GPIOA和USART2时钟 */
  /* 使能GPIOA时钟 */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA ,ENABLE);
  /* 使能USART2时钟 */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
  
  /* 第2步：将USART2 Tx@PA2的GPIO配置为推挽复用模式 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* 第3步：将USART2 Rx@PA3的GPIO配置为浮空输入模式 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /* 第4步：配置USART2参数
      波特率   = 9600
      数据长度 = 8
      停止位   = 1
      校验位   = No
      禁止硬件流控(即禁止RTS和CTS)
      使能接收和发送
  */
  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART2, &USART_InitStructure);
  
  /* 第5步：使能 USART2， 配置完毕 */
  USART_Cmd(USART2, ENABLE);
  
  /* 清除发送完成标志，Transmission Complete flag */
  USART_ClearFlag(USART2, USART_FLAG_TC);
}

/*
********************************************************************************
* 函 数 名: fputc
* 功能说明: 系统函数，发送数据至串口
* 参    数：无
* 返 回 值: 无
* 使用说明：根据宏定义选择 DEBUG_USART1 选择串口1调试
                           DEBUG_USART2 选择串口2调试
            如果都没有定义为1，则不使用fputc功能。
* 调用方法：通过使用printf函数间接调用
********************************************************************************
*/
#if DEBUG_USART1 || DEBUG_USART2
int fputc(int ch, FILE *f)
{

#if (DEBUG_USART1 == 1) && (DEBUG_USART2 == 1)
#warning 不能同时设定USART1和USART2，请修改DEBUG_USARTx定义
#endif
  
#if (DEBUG_USART1 == 1) && (DEBUG_USART2 == 0)
  /* 写一个字节到USART1 */
  USART_SendData(USART1, (uint8_t) ch);
  /* 等待发送结束 */
  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
  {}
  return ch;
#endif
  
#if (DEBUG_USART1 == 0) && (DEBUG_USART2 == 1)
  /* 写一个字节到USART2 */
  USART_SendData(USART2, (uint8_t) ch);
  /* 等待发送结束 */
  while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
  {}
  return ch;
#endif
  
}
#endif
/*
********************************************************************************
* 函 数 名: fgetc
* 功能说明: 系统函数，通过串口接收数据
* 参    数：无
* 返 回 值: 无
* 使用说明：根据宏定义选择 DEBUG_USART1 选择串口1调试
                           DEBUG_USART2 选择串口2调试
* 调用方法：通过使用scanf函数间接调用
********************************************************************************
*/
#if DEBUG_USART1 || DEBUG_USART2
int fgetc(FILE *f)
{
#if (DEBUG_USART1 == 1) && (DEBUG_USART2 == 0)
  /* 等待串口1输入数据 */
  while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);
  return (int)USART_ReceiveData(USART1);
#endif
 
#if (DEBUG_USART1 == 0) && (DEBUG_USART2 == 1)
  /* 等待串口2输入数据 */
  while (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET);
  return (int)USART_ReceiveData(USART2);  
#endif 
}
#endif

