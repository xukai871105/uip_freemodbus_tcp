/*
* FreeModbus Libary: lwIP Port
* Copyright (C) 2006 Christian Walter <wolti@sil.at>
*
* This library is free software; you can redistribute it and/or
* modify it under the terms of the GNU Lesser General Public
* License as published by the Free Software Foundation; either
* version 2.1 of the License, or (at your option) any later version.
*
* This library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License along with this library; if not, write to the Free Software
* Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*
* File: $Id: porttcp.c,v 1.1 2006/08/30 23:18:07 wolti Exp $
*/

/* ----------------------- System includes ----------------------------------*/
#include <stdio.h>
#include <string.h>
#include "port.h"

/* ----------------------- uIP includes ------------------------------------*/
#include "uip_modbus.h"
#include "uip.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- MBAP Header --------------------------------------*/
#define MB_TCP_UID          6
#define MB_TCP_LEN          4
#define MB_TCP_FUNC         7

/* ----------------------- Defines  -----------------------------------------*/
#define MB_TCP_DEFAULT_PORT  502          /* TCP listening port. */
#define MB_TCP_BUF_SIZE     ( 256 + 7 )   /* Must hold a complete Modbus TCP frame. */

/* ----------------------- Prototypes ---------------------------------------*/
static UCHAR    ucTCPRequestFrame[MB_TCP_BUF_SIZE];
static USHORT   ucTCPRequestLen;

static UCHAR    ucTCPResponseFrame[MB_TCP_BUF_SIZE];
static USHORT   ucTCPResponseLen;

BOOL   bFrameSent = FALSE;
// BOOL   is_frame_send = FALSE;

/* ----------------------- Static variables ---------------------------------*/

/* ----------------------- Static functions ---------------------------------*/


/* ----------------------- Begin implementation -----------------------------*/
BOOL
xMBTCPPortInit( USHORT usTCPPort )
{
    BOOL bOkay = FALSE;

    USHORT usPort;
    if( usTCPPort == 0 )
    {
        usPort = MB_TCP_DEFAULT_PORT;
    }
    else
    {
        usPort = ( USHORT ) usTCPPort;
    }

    // 侦听端口 502端口
    uip_listen(HTONS( usPort));
    
    bOkay = TRUE;
    return bOkay;
}


void
vMBTCPPortClose(  )
{
    
}

void
vMBTCPPortDisable( void )
{
    
}

BOOL
xMBTCPPortGetRequest( UCHAR ** ppucMBTCPFrame, USHORT * usTCPLength )
{
    *ppucMBTCPFrame = &ucTCPRequestFrame[0];
    *usTCPLength = ucTCPRequestLen;
    
    /* Reset the buffer. */
    ucTCPRequestLen = 0;
    return TRUE;
}

BOOL
xMBTCPPortSendResponse( const UCHAR * pucMBTCPFrame, USHORT usTCPLength )
{
    memcpy( ucTCPResponseFrame , pucMBTCPFrame , usTCPLength);
    ucTCPResponseLen = usTCPLength;
    
    bFrameSent = TRUE;      // 通过uip_poll发送数据
    return bFrameSent;
}


void uip_modbus_appcall(void)
{
    if( uip_connected() )
    {
        printf("connected!\r\n");
    }
    
    if( uip_closed() )
    {
        printf("closed\r\n");
    }
    
    if( uip_newdata() )
    {
        printf("request!\r\n");
        // 获得modbus请求
        memcpy( ucTCPRequestFrame , uip_appdata, uip_len );
        ucTCPRequestLen = uip_len;
        // 向 modbus poll发送消息
        ( void )xMBPortEventPost( EV_FRAME_RECEIVED );
    }
    
    if( uip_poll() )
    {
        if( bFrameSent )
        {
            bFrameSent = FALSE;
            // uIP发送Modbus应答数据包
            uip_send( ucTCPResponseFrame , ucTCPResponseLen );
        }
    }
}