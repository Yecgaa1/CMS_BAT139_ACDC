/*****************************************************************************
Copyright (C) 2021 China Micro Semiconductor Limited Company. All Rights Reserved.
@Description: 
@Author     : 
@Version    : 0.0.1  
@History    : 
@Attention  ：
*****************************************************************************/

#ifndef __INV_CTRL_H
#define __INV_CTRL_H

#include "stdint.h"

#define     SPWMDUTY_DN_LIMIT                           (1*1)
#define     SPWMDUTY_UP_LIMIT                           (INV_PWM_PERIOD-SPWMDUTY_DN_LIMIT) //限制占空比

//限流环参数
#define     INV_CUR_STEP_VAL                            (25)//每次步进值
#define     INV_CUR_ACTIVE_POWER_VAL                    (2200)//限流环补偿有功功率阈值设定(w)
#define     INV_CUR_UP_LIMIT                            ((int32_t)(25.0*4096/COM_CUR_INDUC_BASE)*3>>1) //3800W/220V*1.414 = 24.4A
#define     INV_CUR_DN_LIMIT                            (-INV_CUR_UP_LIMIT)
#define     INV_CUR_UP_LIMIT_INIT                       (INV_CUR_UP_LIMIT*9>>4)
#define     INV_CUR_DN_LIMIT_INIT                       (INV_CUR_DN_LIMIT*9>>4)
//----------------------------------------------------------------------------------------------
typedef struct
{
        int16_t     test_Cnt;
    
        int32_t     DCIM_Val;    
        int32_t     DCIM_Val_Fir;
        int32_t     DCIM_Sum;

        int32_t     vBus;
        int32_t     PWM_Freq_Init_TempFir;
        int32_t     PWM_Freq_Init_Temp;
        int32_t     PWM_Freq_Init;
        int32_t     PWM_Period_Init;
        int32_t     V_ACIN_Freq_Init;  
        int16_t     PWM_Period_43Hz;
        int16_t     PWM_Period_67Hz;
        int32_t     PWM_Period;
        int32_t     PWM_Duty;
        int32_t     PWM_DutyA;
        int32_t     PWM_DutyB;    
        int32_t     PWM_Duty_Up;
        int32_t     PWM_Duty_Dn;    
        int32_t     AC_Vol_RMS; 
        int32_t     AC_Vol_Peak;
        int16_t     AC_Vol_Freq;
        int32_t     AC_Vol_AMP_Target_Ref;//软启动参考值
        int32_t     AC_Vol_AMP_Target;//幅值
        int16_t     SS_Step_Value;        
        int8_t      SS_Shift_Value;       
        int32_t     SS_AMP_Target_Hold;             
        int32_t     curLoad_RMS;
        int32_t     curLoad_Peak; 
        int32_t     curInduc_Peak;
        int16_t     I_ACOUT_P_SCP_Val;
        int16_t     V_ACOUT_P_SCP_Val;
        
        int32_t     curInduc_RMS;
        uint8_t     short_Start_Flag;//0：表示短路功能启用  ;1：表示短路功能关闭
        int16_t     short_Start_Cnt; //启用短路计数值
        int16_t     short_Start_Time_Val; //设定计数次数，大于此计数值强制开启短路功能
        int16_t     short_Vol_Peak_Cnt; //用于启机短路判断计数
        int16_t     short_Vol_Peak_Time_Val; //用于启机短路判断计数阈值
        int32_t     active_Power;//有功功率    
        int32_t     apparent_Power;//视在功率
        int32_t     reavtive_Power;//无功功率  
        uint16_t    periodDot_Cnt;
        uint32_t    periodDot_Val;  
        uint16_t    periodDot_Val_Half;          
        int32_t     virtual_Res_Coeff;//虚拟阻抗系数
        int32_t     virtual_Res_Coeff_Min;//虚拟阻抗系数最小值
        int8_t      mode_Operate;//开环or闭环
        int8_t      mode_PID_Loop;//PID环路模式：单环or双环
        int8_t      mode_Repeat_Ctrl;//重复控制使能
        int8_t      mode_Power_Ctrl;//限功率控制使能 
        int8_t      mode_PQ_Droop_Ctrl;//PQ下垂控制使能 
        int8_t      mode_AC_Freq_Select;//50Hz/60Hz
        int8_t      mode_AC_Software_Ctrl;//AC输出软启动控制
       
        int16_t     curLoop_Up;
        int16_t     curLoop_Dn;		
        int16_t     curLoop_UpDef;
        int16_t     curLoop_DnDef;	 
        int16_t     curLoop_UpDef_Init;
        int16_t     curLoop_DnDef_Init; 
        int16_t     curLoop_Step_Val; //每次步进值
        int16_t     curLoop_active_Power_Val; //限流环补偿有功功率阈值设定(w)       
        int16_t     VolErr_Integral_Fact;          
const short int     *SineTab_Ptr;
           void     (*Calc)();          /* Pointer to calculation function */
}INV_Ctrl_Var_t;
void INV_Ctrl(void); 
extern INV_Ctrl_Var_t INV_Ctrl_Info;


void INV_Lock_Phase(void);




#endif
/*-------------------------------------------------------------------------------------
*  No more.
*------------------------------------------------------------------------------------*/
