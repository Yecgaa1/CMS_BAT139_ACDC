
/*****************************************************************************
Copyright (C) 2021 China Micro Semiconductor Limited Company. All Rights Reserved.
@Description: 
@Author     : 
@Version    : 0.0.1  
@History    : 
@Attention  £º
*****************************************************************************/


#ifndef  __SYS_HARDWARE_INIT_H
#define  __SYS_HARDWARE_INIT_H


/****************************************************************************/
/*	include files
*****************************************************************************/
#include "BAT32G139.h"
#include "user_sample.h"
/****************************************************************************/



void Sys_HardConfigInit(void);
void Sys_Variableinit(void);
void PFC_Run_VariableInit(void);
void INV_Run_VariableInit(void);
void INV_PWM_Enable(void);
void PFC_PWM_Enable(void);
void COM_PWM_Disable(void);

#endif

