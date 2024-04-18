/*****************************************************************************
Copyright (C) 2021 China Micro Semiconductor Limited Company. All Rights Reserved.
@Description: 
@Author     : 
@Version    : 0.0.1  
@History    : 
@Attention  ��
*****************************************************************************/

#ifndef __INV_CTRL_H
#define __INV_CTRL_H

#include "stdint.h"

#define     SPWMDUTY_DN_LIMIT                           (1*1)
#define     SPWMDUTY_UP_LIMIT                           (INV_PWM_PERIOD-SPWMDUTY_DN_LIMIT) //����ռ�ձ�

//����������
#define     INV_CUR_STEP_VAL                            (25)//ÿ�β���ֵ
#define     INV_CUR_ACTIVE_POWER_VAL                    (2200)//�����������й�������ֵ�趨(w)
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
        int32_t     AC_Vol_AMP_Target_Ref;//�������ο�ֵ
        int32_t     AC_Vol_AMP_Target;//��ֵ
        int16_t     SS_Step_Value;        
        int8_t      SS_Shift_Value;       
        int32_t     SS_AMP_Target_Hold;             
        int32_t     curLoad_RMS;
        int32_t     curLoad_Peak; 
        int32_t     curInduc_Peak;
        int16_t     I_ACOUT_P_SCP_Val;
        int16_t     V_ACOUT_P_SCP_Val;
        
        int32_t     curInduc_RMS;
        uint8_t     short_Start_Flag;//0����ʾ��·��������  ;1����ʾ��·���ܹر�
        int16_t     short_Start_Cnt; //���ö�·����ֵ
        int16_t     short_Start_Time_Val; //�趨�������������ڴ˼���ֵǿ�ƿ�����·����
        int16_t     short_Vol_Peak_Cnt; //����������·�жϼ���
        int16_t     short_Vol_Peak_Time_Val; //����������·�жϼ�����ֵ
        int32_t     active_Power;//�й�����    
        int32_t     apparent_Power;//���ڹ���
        int32_t     reavtive_Power;//�޹�����  
        uint16_t    periodDot_Cnt;
        uint32_t    periodDot_Val;  
        uint16_t    periodDot_Val_Half;          
        int32_t     virtual_Res_Coeff;//�����迹ϵ��
        int32_t     virtual_Res_Coeff_Min;//�����迹ϵ����Сֵ
        int8_t      mode_Operate;//����or�ջ�
        int8_t      mode_PID_Loop;//PID��·ģʽ������or˫��
        int8_t      mode_Repeat_Ctrl;//�ظ�����ʹ��
        int8_t      mode_Power_Ctrl;//�޹��ʿ���ʹ�� 
        int8_t      mode_PQ_Droop_Ctrl;//PQ�´�����ʹ�� 
        int8_t      mode_AC_Freq_Select;//50Hz/60Hz
        int8_t      mode_AC_Software_Ctrl;//AC�������������
       
        int16_t     curLoop_Up;
        int16_t     curLoop_Dn;		
        int16_t     curLoop_UpDef;
        int16_t     curLoop_DnDef;	 
        int16_t     curLoop_UpDef_Init;
        int16_t     curLoop_DnDef_Init; 
        int16_t     curLoop_Step_Val; //ÿ�β���ֵ
        int16_t     curLoop_active_Power_Val; //�����������й�������ֵ�趨(w)       
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
