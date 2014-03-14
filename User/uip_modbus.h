#ifndef __UIP_MODBUS_H
#define __UIP_MODBUS_H

#include "uipopt.h"
#include "psock.h"

/* 应用程序状态 */
typedef int uip_tcp_appstate_t;

/* 定义应用程序 并指定UIP_APPCALL宏定义 */
void uip_modbus_appcall(void);
#ifndef UIP_APPCALL
#define UIP_APPCALL uip_modbus_appcall
#endif

void uip_modbus_init(void);
void uip_modbus_appcall(void);

#endif