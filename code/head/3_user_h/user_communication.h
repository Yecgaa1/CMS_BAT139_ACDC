
/*****************************************************************************
Copyright (C) 2021 China Micro Semiconductor Limited Company. All Rights Reserved.
@Description: 
@Author     : 
@Version    : 0.0.1  
@History    : 
@Attention  £º
*****************************************************************************/

#ifndef  __USER_COMMUNICATION_H
#define  __USER_COMMUNICATION_H

#include "bat32g139.h"
#include "sys_struct.h"

void User_UART_View(void);
void INV_DataSend(void);
void COM_Uartx_HeartCheck(UARTx_Var_t *uart_Info);
uint16_t crc16_modbus(uint8_t *data, uint16_t length, uint16_t CRC_Init);

void Function_TxSendDebug(int32_t data1,int32_t data2,int32_t data3,int32_t data4,int32_t data5);
#endif

