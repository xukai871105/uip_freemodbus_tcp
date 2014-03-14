
#ifndef __UIP_CONF_H
#define __UIP_CONF_H

#include <inttypes.h>

typedef uint8_t                   u8_t;
typedef uint16_t                  u16_t;	
typedef unsigned short            uip_stats_t;

/* 最大TCP连接数 */
#define UIP_CONF_MAX_CONNECTIONS  10
/* 最大端口监听数 */
#define UIP_CONF_MAX_LISTENPORTS  10	  
/* uIP 缓存大小*/
#define UIP_CONF_BUFFER_SIZE      1500
/* CPU字节顺序 */ 
#define UIP_CONF_BYTE_ORDER       UIP_LITTLE_ENDIAN
/* 日志开关	 */			
#define UIP_CONF_LOGGING          1
/* UDP支持开关*/
#define UIP_CONF_UDP              0
/* UDP校验和开关 */
#define UIP_CONF_UDP_CHECKSUMS    1
/* uIP统计开关 */
#define UIP_CONF_STATISTICS       1

#include "uip_modbus.h"
#endif
