#include "stm32f10x.h"
#include <stdio.h>

#include "uip.h"
#include "uip_arp.h"

#include "mb.h"
#include "mbutils.h"

#include "tapdev.h"
#include "enc28j60.h"	

#include "bsp_spi1.h"
#include "bsp_usart.h"
#include "timer.h"

#define LED1_ON()   GPIO_SetBits(GPIOB,GPIO_Pin_5)
#define LED1_OFF()  GPIO_ResetBits(GPIOB,GPIO_Pin_5)

#define LED2_ON()   GPIO_SetBits(GPIOD,GPIO_Pin_6)
#define LED2_OFF()  GPIO_ResetBits(GPIOD,GPIO_Pin_6)

#define LED3_ON()   GPIO_SetBits(GPIOD,GPIO_Pin_3)
#define LED3_OFF()  GPIO_ResetBits(GPIOD,GPIO_Pin_3)

#define REG_INPUT_START       0x0000                // 输入寄存器起始地址
#define REG_INPUT_NREGS       16                    // 输入寄存器数量

#define REG_HOLDING_START     0x0000                // 保持寄存器起始地址
#define REG_HOLDING_NREGS     16                    // 保持寄存器数量

#define REG_COILS_START       0x0000                // 线圈起始地址
#define REG_COILS_SIZE        24                    // 线圈数量

#define REG_DISCRETE_START    0x0000                // 开关寄存器起始地址
#define REG_DISCRETE_SIZE     16                    // 开关寄存器数量

// 输入寄存器内容
uint16_t usRegInputBuf[REG_INPUT_NREGS] = {0x1000,0x1001,0x1002,0x1003,0x1004,0x1005,0x1006,0x1007};
// 寄存器起始地址
uint16_t usRegInputStart = REG_INPUT_START;
// 保持寄存器内容
uint16_t usRegHoldingBuf[REG_HOLDING_NREGS] = {0x147b,0x3f8e,0x147b,0x400e,0x1eb8,0x4055,0x147b,0x408e};
// 保持寄存器起始地址
uint16_t usRegHoldingStart = REG_HOLDING_START;
// 线圈状态
uint8_t ucRegCoilsBuf[REG_COILS_SIZE / 8] = {0x0F,0x02,0x03};
// 开关状态
uint8_t ucRegDiscreteBuf[REG_DISCRETE_SIZE / 8] = {0x01,0x02};

#define BUF ((struct uip_eth_hdr *)&uip_buf[0])	

void GPIO_Config(void);
void led_poll(void);

int main(void)
{
    timer_typedef periodic_timer, arp_timer;
    uip_ipaddr_t ipaddr;
    
    /* 设定查询定时器 ARP定时器 */
    timer_set(&periodic_timer, CLOCK_SECOND / 2);
    timer_set(&arp_timer, CLOCK_SECOND * 10);
    
    /* IO口初始化 主要是为了避免SPI总线上的其他设备 */
	GPIO_Config();                     
    
	/* 配置systic作为1ms中断 */
    timer_config(); 
    /* 初始化SPI1 */
    BSP_ConfigSPI1();
	
    /* ENC28J60初始化 */
	tapdev_init();                     		 
	/* UIP协议栈初始化 */
	uip_init();		
    
    /* 设置IP地址 */
	uip_ipaddr(ipaddr, 192,168,1,15);	
	uip_sethostaddr(ipaddr);
    /* 设置默认路由器IP地址 */
	uip_ipaddr(ipaddr, 192,168,1,1);		 
	uip_setdraddr(ipaddr);
    /* 设置网络掩码 */
	uip_ipaddr(ipaddr, 255,255,255,0);		 
	uip_setnetmask(ipaddr);	
    
    eMBTCPInit(MB_TCP_PORT_USE_DEFAULT);      // 侦听默认端口 502
    eMBEnable();	
    
    BSP_ConfigUSART1();
    printf("\r\nuip start!\r\n");
    printf("ipaddr:192.168.1.15\r\n");
    
	while (1)
	{	
        eMBPoll();
        led_poll();
        
        /* 从网络设备读取一个IP包,返回数据长度 */
        uip_len = tapdev_read();
        /* 收到数据	*/
		if(uip_len > 0)			    
		{
			/* 处理IP数据包 */
			if(BUF->type == htons(UIP_ETHTYPE_IP))
			{
				uip_arp_ipin();
				uip_input();
                
				if (uip_len > 0)
				{
					uip_arp_out();
					tapdev_send();
				}
			}
			/* 处理ARP报文 */
			else if (BUF->type == htons(UIP_ETHTYPE_ARP))
			{
				uip_arp_arpin();
				if (uip_len > 0)
				{
					tapdev_send();
				}
			}
		}
        
        /* 0.5秒定时器超时 */
        if(timer_expired(&periodic_timer))			
        {
            timer_reset(&periodic_timer);
            
            /* 测试使用 */
            GPIOB->ODR ^= GPIO_Pin_5;
            /* 处理TCP连接, UIP_CONNS缺省是10个 */
            for(uint8_t i = 0; i < UIP_CONNS; i++)
            {
                /* 处理TCP通信事件 */
                uip_periodic(i);		
                if(uip_len > 0)
                {
                    uip_arp_out();
                    tapdev_send();
                }
            }
            
#if UIP_UDP
            /* 轮流处理每个UDP连接, UIP_UDP_CONNS缺省是10个 */
            for(uint8_t i = 0; i < UIP_UDP_CONNS; i++)
            {
                uip_udp_periodic(i);	/*处理UDP通信事件 */
                /* 如果上面的函数调用导致数据应该被发送出去，全局变量uip_len设定值> 0 */
                if(uip_len > 0)
                {
                    uip_arp_out();
                    tapdev_send();
                }
            }
#endif /* UIP_UDP */
            
            /* 定期ARP处理 */
            if (timer_expired(&arp_timer))
            {
                timer_reset(&arp_timer);
                uip_arp_timer();
            }
        }
	}
}

/****************************************************************************
* 名    称：void GPIO_Configuration(void)
* 功    能：通用IO口配置
* 入口参数：无
* 出口参数：无
* 说    明：
* 调用方法：
****************************************************************************/  
void GPIO_Config(void)
{
    
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_USART1 | 
                            RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                            RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
                            RCC_APB2Periph_GPIOE, ENABLE);
    
    // LED1控制
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;				     
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);					 
    
    // LED2, LED3控制
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_3;		 
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    //SST25VF016B SPI片选
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;					 
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    // PB12---VS1003 SPI片选（V2.1) PB7---触摸屏芯片XPT2046 SPI 片选
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_7;		 
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // 禁止SPI1总线上的其他设备 非常重要
    GPIO_SetBits(GPIOB, GPIO_Pin_7);    // 触摸屏芯片XPT2046 SPI 片选禁止  
    GPIO_SetBits(GPIOB, GPIO_Pin_12);   // VS1003 SPI片选（V2.1)禁止 
    GPIO_SetBits(GPIOC, GPIO_Pin_4);    // SST25VF016B SPI片选禁止  
    
    //ENC28J60接收完成中断引脚 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;	         	 	
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;     //内部上拉输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);		 
}


void uip_log(char *m)
{
    
}

eMBErrorCode
eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;
    
    //查询是否在寄存器范围内
    //为了避免警告，修改为有符号整数
    if( ( (int16_t) usAddress >= REG_INPUT_START ) \
        && ( usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS ) )
    {
        iRegIndex = ( int )( usAddress - usRegInputStart );
        while( usNRegs > 0 )
        {
            *pucRegBuffer++ = ( unsigned char )( usRegInputBuf[iRegIndex] >> 8 );
            *pucRegBuffer++ = ( unsigned char )( usRegInputBuf[iRegIndex] & 0xFF );
            iRegIndex++;
            usNRegs--;
        }
    }
    else
    {
        //返回错误状态，无寄存器  
        eStatus = MB_ENOREG;
    }
    
    return eStatus;
}

eMBErrorCode
eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs,
                eMBRegisterMode eMode )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;
    
    if( ( (int16_t)usAddress >= REG_HOLDING_START ) \
        && ( usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS ) )
    {
        iRegIndex = ( int )( usAddress - usRegHoldingStart );
        switch ( eMode )
        {
        case MB_REG_READ:            
            while( usNRegs > 0 )
            {
                *pucRegBuffer++ = ( unsigned char )( usRegHoldingBuf[iRegIndex] >> 8 );
                *pucRegBuffer++ = ( unsigned char )( usRegHoldingBuf[iRegIndex] & 0xFF );
                iRegIndex++;
                usNRegs--;
            }
            break;
            
        case MB_REG_WRITE:
            while( usNRegs > 0 )
            {
                usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
                usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
                iRegIndex++;
                usNRegs--;
            }
            break;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    
    return eStatus;
}


eMBErrorCode
eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils,
              eMBRegisterMode eMode )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    short           iNCoils = ( short )usNCoils;
    unsigned short  usBitOffset;
    
    if( ( (int16_t)usAddress >= REG_COILS_START ) &&
       ( usAddress + usNCoils <= REG_COILS_START + REG_COILS_SIZE ) )
    {
        usBitOffset = ( unsigned short )( usAddress - REG_COILS_START );
        switch ( eMode )
        {

        case MB_REG_READ:
            while( iNCoils > 0 )
            {
                *pucRegBuffer++ = xMBUtilGetBits( ucRegCoilsBuf, usBitOffset,
                                                 ( unsigned char )( iNCoils >
                                                                   8 ? 8 :
                                                                        iNCoils ) );
                iNCoils -= 8;
                usBitOffset += 8;
            }
            break;
            
            /* Update current register values. */
        case MB_REG_WRITE:
            while( iNCoils > 0 )
            {
                xMBUtilSetBits( ucRegCoilsBuf, usBitOffset,
                               ( unsigned char )( iNCoils > 8 ? 8 : iNCoils ),
                               *pucRegBuffer++ );
                iNCoils -= 8;
                usBitOffset += 8;
            }
            break;
        }
        
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}

eMBErrorCode
eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
    //错误状态
    eMBErrorCode    eStatus = MB_ENOERR;
    short           iNDiscrete = ( short )usNDiscrete;
    unsigned short  usBitOffset;
    
    /* Check if we have registers mapped at this block. */
    if( ( (int16_t)usAddress >= REG_DISCRETE_START ) &&
       ( usAddress + usNDiscrete <= REG_DISCRETE_START + REG_DISCRETE_SIZE ) )
    {
        usBitOffset = ( unsigned short )( usAddress - REG_DISCRETE_START );
        
        while( iNDiscrete > 0 )
        {
            *pucRegBuffer++ = xMBUtilGetBits( ucRegDiscreteBuf, usBitOffset,
                                             ( unsigned char)( iNDiscrete > 
                                                              8 ? 8 : 
                                                                   iNDiscrete ) );
            iNDiscrete -= 8;
            usBitOffset += 8;
        }
        
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}

void led_poll(void)
{
    uint8_t led_state = ucRegCoilsBuf[0];
    
    led_state & 0x01 ? LED1_ON():LED1_OFF();
    led_state & 0x02 ? LED2_ON():LED2_OFF();
    led_state & 0x04 ? LED3_ON():LED3_OFF();
}






