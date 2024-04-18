/*****************************************************************************
Copyright (C) 2021 China Micro Semiconductor Limited Company. All Rights Reserved.
@Description: 
@Author     : 
@Version    : 0.0.1  
@History    : 
@Attention  ：
*****************************************************************************/

#ifndef __PFC_CTRL_H
#define __PFC_CTRL_H

#include "stdint.h"
#include "sys_define_config.h"              

#define     PFC_PWM_DUTY_DN_LIMIT                           (23*1)
#define     PFC_PWM_DUTY_UP_LIMIT                           (PFC_PWM_PERIOD-PFC_PWM_DUTY_DN_LIMIT) //限制占空比

typedef struct { 
                int32_t     AD_VBus;
                int32_t     AD_VolPeak;
                int32_t     AD_InducCurPeak;
                int32_t     AD_LoadCurPeak; 
                int32_t     AC_Vol_Freq;
                int32_t     AC_Vol_Freq_Fir;
                int32_t     RMS_Vol;    
                int32_t     RMS_LoadCur;
                int32_t     RMS_InductorCur;
                int32_t     active_Power;
                int32_t     PWM_Period;
                int32_t     PWM_Duty;
                int32_t     PWM_DutyB;
                int32_t     PWM_Duty_Up; //占空比上限值
                int32_t     PWM_Duty_Dn; //占空比下限值 
//                int32_t     periodDot_Cnt;
                int32_t     periodDot_Val;
//                int32_t     dot_Reciprocal;//点数分辨率
                int32_t     vBus_Ref_SS;//电压环软启动变量
                int16_t     AC_VolBase;  
                int32_t     feedforward_Duty;//前馈占空比
                int32_t     DCDC_CHG_Out;//DCDC传输的控制电压值
                 int8_t     DCDC_CHG_State;//DCDC侧充电状态：0表示需要充电，1表示不需要充电    
                int16_t     Zero_DutySS;//PFC过零缓占空比启动
                
                int16_t     PFC_FreqOK_Cnt;// 
                int16_t     vBUS_Ref_Max;
                int16_t     vBUS_Ref_Min;                
			   }PFC_Ctrl_Var_t;  

void PFC_VBus_RefCalc(void);
void PFC_Ctrl(void);         

extern  PFC_Ctrl_Var_t    PFC_Ctrl_Info;       

typedef struct { 
    uint16_t    u16SS_Step_Val;        
    uint8_t     u8SS_Shift_Val;
    uint32_t    u32SS_Set_Val;  
    uint32_t    u32SS_vBus_Hold;  
}PFC_Ref_Var_t; 
extern  PFC_Ref_Var_t    PFC_Ref_Info;   



#endif 
/*-------------------------------------------------------------------------------------
 *  No more.
 *------------------------------------------------------------------------------------*/
