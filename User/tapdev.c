#include "tapdev.h"
#include "uip.h"
#include "uip_arp.h"

#include "enc28j60.h"

struct uip_eth_addr uip_mac;
static unsigned char ethernet_mac[6] = {0x04,0x02,0x35,0x00,0x00,0x01};  //MAC地址       
/*---------------------------------------------------------------------------*/
/* 配置网卡硬件，设置IP地址 */
void tapdev_init(void)
{ 
    unsigned char i;     
    
    /*初始化 enc28j60*/
    enc28j60_init(ethernet_mac);        
    
    for (i = 0; i < 6; i++)
    {
        uip_mac.addr[i] = ethernet_mac[i];
    }
    
    /* 设定mac地址*/
    uip_setethaddr(uip_mac);
    
}

/****************************************************************************
* 名    称：uint16_t tapdev_read(void)
* 功    能：                                                                         
* 入口参数：读取一包数据
* 出口参数: 如果一个数据包收到返回数据包长度，以字节为单位，否则为零。
* 说    明：
* 调用方法：
****************************************************************************/ 
uint16_t tapdev_read(void)
{   
    return  enc28j60_packet_receive(uip_buf, 1500);
}
/****************************************************************************
* 名    称：void tapdev_send(void)
* 功    能：                                                                         
* 入口参数：发送一包数据
* 出口参数: 
* 说    明：
* 调用方法：
****************************************************************************/ 
void tapdev_send(void)
{
    enc28j60_packet_send(uip_buf,uip_len);
}

