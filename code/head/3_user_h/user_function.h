
/*****************************************************************************
Copyright (C) 2021 China Micro Semiconductor Limited Company. All Rights Reserved.
@Description: 
@Author     : 
@Version    : 0.0.1  
@History    : 
@Attention  £º
*****************************************************************************/

#ifndef  __USER_FUNCTION_H
#define  __USER_FUNCTION_H

#include "BAT32G139.h"
void COM_UPS_Deal(void);
void User_LED_Deal(void);
void User_Key_Deal(void);
void COM_CloseDrive(void);
void COM_Function(void);
void User_DelayTime_us(uint32_t delay);
void User_DelayTime_ms(uint32_t delay);
int32_t User_Divider(int32_t dividend, int32_t divisor);

#endif

