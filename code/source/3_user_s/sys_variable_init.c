/*****************************************************************************
Copyright (C) 2021 China Micro Semiconductor Limited Company. All Rights Reserved.
@Description: 
@Author     : 
@Version    : 0.0.1  
@History    : 
@Attention  ��
*****************************************************************************/

/****************************************************************************/
/*----include files---------------------------------------------------------*/
#include "sys_mcu_header.h"
#include "sys_define_param.h"
/***************************************************************************/

/*************************************************
Description: Sys_Variableinit
Input      : 
Return     : 
Others     : 
*************************************************/
extern uint16_t PFC_VBUS_READY_OK_Cnt;
void Sys_Variableinit(void)
{    
    PFC_Ctrl_Info.vBUS_Ref_Max = PFC_VBUS_REF_MAX;
    PFC_Ctrl_Info.vBUS_Ref_Min = PFC_VBUS_REF_MIN;
    
    UART1_Info.RXD_CRC_Init = 0xffff;//��ʼ��CRC�Ĵ�����ʼֵ
    UART1_Info.TXD_CRC_Init = 0xffff;//��ʼ��CRC�Ĵ�����ʼֵ
    COM_Ctr_Info.PWM_Enable             = 0;//������ʹ���ź�    
    COM_Ctr_Info.PFC_AC_Vol_OK_Cnt      = 0;
    COM_Ctr_Info.PFC_AC_Vol_OK_TimeVal  = 1300;//����е�OK
    
    COM_Ctr_Info.PFC_AC_Vol_NOK_Cnt     = 0;
    COM_Ctr_Info.PFC_AC_Vol_NOK_TimeVal = 25;//����е�NOK

    COM_Ctr_Info.NO_Mode_OK_Cnt         = 0; 
    COM_Ctr_Info.NO_Mode_OK_TimeVal     = 2; 
    
    COM_Ctr_Info.PFC_FREQ_Cnt           = 0;
    COM_Ctr_Info.PFC_FREQ_TimeVal       = 50;
    COM_Ctr_Info.PFC_FREQ_State         = 0;//Ƶ�ʼ��OK״̬Ϊ 1
    
    COM_Ctr_Info.INV_PFC_Mode_Select    = 0;//Ĭ��INVģʽ

    INV_Ctrl_Info.DCIM_Sum      = 0;
    INV_Ctrl_Info.DCIM_Val_Fir  = 0;         
    INV_PID_DCIM.out            = 0;    
    INV_PID_DCIM.err_Integral   = 0;   
    INV_Ctrl_Info.PWM_Freq_Init = INV_PWM_FREQ;

    
    INV_Ctrl_Info.curLoop_UpDef         = INV_CUR_UP_LIMIT;
    INV_Ctrl_Info.curLoop_DnDef         = INV_CUR_DN_LIMIT;	 
    INV_Ctrl_Info.curLoop_UpDef_Init    = INV_CUR_UP_LIMIT_INIT;
    INV_Ctrl_Info.curLoop_DnDef_Init    = INV_CUR_DN_LIMIT_INIT;
    INV_Ctrl_Info.curLoop_Step_Val      = INV_CUR_STEP_VAL;    
    INV_Ctrl_Info.curLoop_active_Power_Val  = INV_CUR_ACTIVE_POWER_VAL;
    INV_Ctrl_Info.VolErr_Integral_Fact      = INV_VOL_ERR_INTEGRAL_FACT;  
    
    UPS_Ctr_Info.PWM_Period     = 0;//UPS������µ�����
    UPS_Ctr_Info.delta_CMP_Val  = 0;//�����ѹ�������ѹ������CMP����������ֵ
    UPS_Ctr_Info.delta_P        = 0;//�������������Ĳ�ֵ*50
    UPS_Ctr_Info.delta_T        = 0;//

    UPS_Ctr_Info.lock_Phase_Cnt = 0;
    UPS_Ctr_Info.lock_Phase_Val = 190;//��Ƶ����
    
    UPS_Ctr_Info.lock_Phase_OK      = 0;//����OK
    UPS_Ctr_Info.lock_Phase_OK_Cnt  = 0;//����OK����
    UPS_Ctr_Info.V_ACIN_OK          = 0;//�е����OK��־
    UPS_Ctr_Info.V_ACIN_OK_Cnt      = 0;//�е����OK���� 
    
    UPS_Ctr_Info.V_ACIN_NOK_Cnt_P   = 0;//�������е����NOK����  
    UPS_Ctr_Info.V_ACIN_NOK_Cnt_N   = 0;//�������е����NOK����
    
    UPS_Ctr_Info.V_ACIN_HYS_ADV     = 0;//1���е粨���ͺ�2���е粨�γ�ǰ
    UPS_Ctr_Info.PFC_CMP_Val        = 0;//���������ѹ����ʱ��CMP��������
    
    UPS_Ctr_Info.flag1  = 0; 
    UPS_Ctr_Info.flag2  = 0; 
    UPS_Ctr_Info.flag3  = 0;   


    //--------------------------------------------------------------------------------------
    #if INV_AC_VOL_FREQ_SELECT == INV_AC_VOL_FREQ_50HZ
        INV_Ctrl_Info.SineTab_Ptr           = &Sine_Table_50Hz[0];
        INV_Ctrl_Info.periodDot_Val         = SPWMWAVE_DOT_50Hz;
        INV_Ctrl_Info.periodDot_Val_Half    = SPWMWAVE_DOT_50Hz_2;
    
        INV_Repeat_Info.periodDot_Val           = SPWMWAVE_DOT_50Hz;     
        INV_Ctrl_Info.short_Vol_Peak_Time_Val   = SPWMWAVE_DOT_50Hz_4;//��������    
        RMS_PQ_Info.dot_Reciprocal              = SPWMWAVE_DOT_RECIPROCAL_50Hz; 
    
    #else
        INV_Ctrl_Info.SineTab_Ptr           = &Sine_Table_60Hz[0];
        INV_Ctrl_Info.periodDot_Val         = SPWMWAVE_DOT_60Hz;
        INV_Ctrl_Info.periodDot_Val_Half    = SPWMWAVE_DOT_60Hz_2;
    
        INV_Repeat_Info.periodDot_Val           = SPWMWAVE_DOT_60Hz;     
        INV_Ctrl_Info.short_Vol_Peak_Time_Val   = SPWMWAVE_DOT_60Hz_4;//��������
        RMS_PQ_Info.dot_Reciprocal              = SPWMWAVE_DOT_RECIPROCAL_60Hz;    
    #endif


    //--------------------------------------------------------------------------------------
    #if ( OPERATING_MODE == NORMAL_MODE )			 		
        INV_Ctrl_Info.AC_Vol_AMP_Target_Ref     = INV_AC_VOL_AMP_VAL_NORMAL_REF;
    #else
        INV_Ctrl_Info.AC_Vol_AMP_Target_Ref     = INV_AC_VOL_AMP_VAL_DEBUG_REF; 
    #endif	

    
    INV_Parall_Info.Flag.masterFlag         = 0;		//Ĭ�Ϸ�����������ȷ�ϱ���״̬  
    INV_Parall_Info.Flag.slaveFlag          = 0;		//Ĭ�ϷǴӻ�������ȷ�ϱ���״̬      
    INV_Parall_Info.Flag.syn_Freq_OK        = 0;       
    INV_Parall_Info.Flag.selectStateOK      = 0;
    INV_Parall_Info.slave_Cnt               = 0;
    INV_Parall_Info.master_Cnt              = 0;
    INV_Parall_Info.selectSlave_Cnt         = 0;    
    INV_Parall_Info.judge_Vol_RMS_Val       = INV_JUDGE_VOL_RMS_VAL;
    

    
    RMS_PQ_Info.out_RMS_INV_AC_Vol          = 0;  
    RMS_PQ_Info.out_RMS_PFC_AC_Vol          = 0;      
    RMS_PQ_Info.out_RMS_CurInduc            = 0;
    RMS_PQ_Info.out_RMS_CurLoad             = 0;    
    RMS_PQ_Info.out_P                       = 0;
    RMS_PQ_Info.periodDot_Cnt               = 0;    
    INV_PID_Power.ref                       = INV_POWER_REF;//���ʻ��ο�ֵ��ʼ��

    //׼��̬��ر������
    AD_Correct_V_ACOUT.Flag.bit.ADRef_Correct_Ok    = E_FALSE;
    AD_Correct_V_ACIN.Flag.bit.ADRef_Correct_Ok     = E_FALSE;
    AD_Correct_I_Induc.Flag.bit.ADRef_Correct_Ok    = E_FALSE;
    AD_Correct_I_Load.Flag.bit.ADRef_Correct_Ok     = E_FALSE;
    AD_Correct_Vref.Flag.bit.ADRef_Correct_Ok       = E_FALSE;
    
    AD_Correct_V_ACOUT.Flag.all             = 0;
    AD_Correct_V_ACIN.Flag.all              = 0;
    AD_Correct_I_Induc.Flag.all             = 0;
    AD_Correct_I_Load.Flag.all              = 0;    
    AD_Correct_Vref.Flag.all                = 0;
    
    StartCheck_Flag_Info.all                = 0;
    System_ProtectFlag_Info.all             = 0;
    
    INV_Ctrl_Info.V_ACIN_Freq_Init          = INV_AC_VOL_FREQ_SELECT;
    
    INV_Ctrl_Info.short_Start_Time_Val      = INV_ACOUT_SCP_START_TIME;//(ms)
    INV_Ctrl_Info.short_Start_Flag          = INV_ACOUT_SCP_START_FLAG;//��ʼ̬�����ö�·����
    INV_Ctrl_Info.PWM_Period_Init           = INV_PWM_PERIOD;
    INV_Ctrl_Info.PWM_Period                = INV_PWM_PERIOD;
    INV_Ctrl_Info.mode_Operate              = OPERATING_MODE;
    INV_Ctrl_Info.mode_PID_Loop             = INV_PID_LOOP_MODE;
    INV_Ctrl_Info.mode_Repeat_Ctrl          = INV_REPEAT_CTRL_SELECT;//�ظ�����ʹ��
    INV_Ctrl_Info.mode_Power_Ctrl           = INV_POWER_CTRL_SELECT;//�޹��ʿ���ʹ�� 
    INV_Ctrl_Info.mode_PQ_Droop_Ctrl        = INV_PQ_DROOP_CTRL_SELECT;//PQ�´�����ʹ��     
    INV_Ctrl_Info.mode_AC_Freq_Select       = INV_AC_VOL_FREQ_SELECT;//50Hz/60Hz 
    INV_Ctrl_Info.mode_AC_Software_Ctrl     = INV_SOFTWARE_CTRL_SELECT;//AC�������������
   
    INV_Ctrl_Info.virtual_Res_Coeff         = INV_VIRTUAL_RES;//�����迹  
    INV_Ctrl_Info.virtual_Res_Coeff_Min     = INV_VIRTUAL_RES_MIN;//�����迹 
    
    INV_Ctrl_Info.curLoop_Up                = INV_CUR_UP_LIMIT;
    INV_Ctrl_Info.curLoop_Dn                = INV_CUR_DN_LIMIT;  
    INV_Ctrl_Info.PWM_Duty_Up               = SPWMDUTY_UP_LIMIT;
    INV_Ctrl_Info.PWM_Duty_Dn               = SPWMDUTY_DN_LIMIT;      
    INV_Ctrl_Info.short_Vol_Peak_Cnt        = 0;
    INV_Ctrl_Info.SS_Step_Value             = INV_SOFTWARE_STEP_VALUE;//AC������ʱ����ֵϵ��ÿ�β���ֵ        
    INV_Ctrl_Info.SS_Shift_Value            = INV_SOFTWARE_SHIFT_VALUE;//AC������ʱ����ֵϵ��������λֵ       
    INV_Ctrl_Info.SS_AMP_Target_Hold        = INV_SOFTWARE_HOLD_INIT;//�Ŵ��ķ�ֵϵ����ʼֵ 

    INV_Ctrl_Info.short_Start_Cnt           = 0;

    PFC_Ref_Info.u8SS_Shift_Val     = PFC_VBUS_SHIFT_VAL;  //PFC������ʱ��ref������λֵ
    PFC_Ref_Info.u16SS_Step_Val     = PFC_VBUS_STEP_VAL; //PFC������ʱ��refÿ�β���ֵ 
    PFC_Ref_Info.u32SS_vBus_Hold    = PFC_VBUS_REF_INIT_SS;  //PFC��ѹ���������ο�ֵ���� 
    
    PFC_Ctrl_Info.AC_VolBase        = 255;//�����ѹ��׼��ʼֵ
    PFC_Ctrl_Info.PWM_Period        = PFC_PWM_PERIOD;
    PFC_Ctrl_Info.PWM_Duty          = 0;//ռ�ձ�Ϊ��ʱ�ر���ѹ        
    PFC_Ctrl_Info.vBus_Ref_SS       = PFC_VBUS_REF_INIT_SS;//PFC��ѹ���������ο�ֵ����  
    PFC_Ctrl_Info.PWM_Duty_Up       = PFC_PWM_DUTY_UP_LIMIT;
    PFC_Ctrl_Info.PWM_Duty_Dn       = PFC_PWM_DUTY_DN_LIMIT;
    PFC_PID_Vol.err_Integral        = 0;//�����ۼ�ֵ��ʼ��
    PFC_PID_Cur.err_Integral        = 0;//�����ۼ�ֵ��ʼ��
    PFC_Ctrl_Info.PFC_FreqOK_Cnt    = 0;    

    StartCheck_Flag_Info.bit.auxPower_High_OK   = E_FALSE;
    StartCheck_Flag_Info.bit.auxPower_Low_OK    = E_FALSE;
    StartCheck_Flag_Info.bit.AC_Vol_High_OK     = E_FALSE;
    StartCheck_Flag_Info.bit.AC_Vol_Low_OK      = E_FALSE; 
    StartCheck_Flag_Info.bit.FREQ_High_OK       = E_FALSE;
    StartCheck_Flag_Info.bit.FREQ_Low_OK        = E_FALSE;     
    StartCheck_Flag_Info.bit.vBus_High_OK       = E_FALSE;
    StartCheck_Flag_Info.bit.vBus_Low_OK        = E_FALSE;      
    
    COM_StartCheck_AuxPower_Up.period_Cnt       = 0;
    COM_StartCheck_AuxPower_Down.period_Cnt     = 0;
    PFC_StartCheck_AC_VOL_Up.period_Cnt         = 0;
    PFC_StartCheck_AC_VOL_Down.period_Cnt       = 0;
    
    COM_Ctr_Info.operate_Mode       = OPERATING_MODE;
    
    RMS_PQ_Info.periodDot_Val       = COM_RMS_DOT_VAL;
    RMS_PQ_Info.dot_Reciprocal      = COM_RMS_DOT_RECIPROCAL; 
    
    INV_Ctrl_Info.PWM_Period_43Hz = (COM_MCU_CLK  / (INV_Ctrl_Info.periodDot_Val * PLL_VACOUT_FREQ_MIN*2)) + INV_DEADTIME - 2;   //1/fk* (m+2�Cp)* 2
    INV_Ctrl_Info.PWM_Period_67Hz = (COM_MCU_CLK  / (INV_Ctrl_Info.periodDot_Val * PLL_VACOUT_FREQ_MAX*2)) + INV_DEADTIME - 2;


    PFC_VPID_Info.u32kp_Min         =PFC_VPID_KP_MIN;   //���غ�Kpֵ�����仯����Сֵ 
    PFC_VPID_Info.u32kp_Max         =PFC_VPID_KP_MAX;        //����ʱKpֵ���
    PFC_VPID_Info.u32kp_Hold        = PFC_VPID_KP_HOLD_INIT; //�Ŵ���kp��ʼֵ
    PFC_VPID_Info.u16kp_Step_Val    =PFC_VPID_KP_STEP_VAL; //ÿ�β���ֵ  
    PFC_VPID_Info.u8kp_Shift_Val    =PFC_VPID_KP_SHIFT_VAL;//������λֵ
    PFC_VPID_Info.u16kp_I_Induc_RMS_Val = PFC_I_INDUC_RMS_VAL; //PFC��ѹ��KP�仯ʱ���ж���������е�����Чֵ-��:1A*512   

}


/*************************************************
Description: PFC_Variableinit
Input      : 
Return     : 
Others     : �Ӵ���״̬�л�����̬������ʼ��
*************************************************/
extern int16_t	RepeatOut[600] ;
extern int16_t	RepeatErr[600] ;
extern int16_t  INV_AC_Vol_table[680];
extern int16_t PFC_AC_Vol_table[680];
int16_t i16_variableI = 0;
extern int16_t PFC_StartCount;
void PFC_Run_VariableInit(void)
{     
    COM_Ctr_Info.PWM_Enable         = 0;//������ʹ���ź�    
    UPS_Ctr_Info.PWM_Period         = 0;//UPS������µ�����
    UPS_Ctr_Info.delta_CMP_Val      = 0;//�����ѹ�������ѹ������CMP����������ֵ
    UPS_Ctr_Info.delta_P            = 0;//�������������Ĳ�ֵ*50
    UPS_Ctr_Info.delta_T            = 0;//
    
    UPS_Ctr_Info.lock_Phase_Cnt     = 0;
    
    UPS_Ctr_Info.lock_Phase_OK      = 0;//����OK
    UPS_Ctr_Info.lock_Phase_OK_Cnt  = 0;//����OK����
    UPS_Ctr_Info.V_ACIN_OK          = 0;//�е����OK��־
    UPS_Ctr_Info.V_ACIN_OK_Cnt      = 0;//�е����OK���� 
    UPS_Ctr_Info.V_ACIN_NOK_Cnt_P   = 0;//�������е����NOK����  
    UPS_Ctr_Info.V_ACIN_NOK_Cnt_N   = 0;//�������е����NOK����      
    UPS_Ctr_Info.V_ACIN_HYS_ADV     = 0;//1���е粨���ͺ�2���е粨�γ�ǰ
    UPS_Ctr_Info.PFC_CMP_Val        = 0;//���������ѹ����ʱ��CMP��������
    
    UPS_Ctr_Info.flag1  = 0; 
    UPS_Ctr_Info.flag2  = 0; 
    UPS_Ctr_Info.flag3  = 0;     


    PFC_Ref_Info.u8SS_Shift_Val         = PFC_VBUS_SHIFT_VAL;  //PFC������ʱ��ref������λֵ
    PFC_Ref_Info.u16SS_Step_Val         = PFC_VBUS_STEP_VAL; //PFC������ʱ��refÿ�β���ֵ 
    PFC_Ref_Info.u32SS_vBus_Hold        = PFC_VBUS_REF_INIT_SS;  //PFC��ѹ���������ο�ֵ����

    INV_Ctrl_Info.AC_Vol_AMP_Target     = 0;
    PFC_Ctrl_Info.DCDC_CHG_Out          = 0;
    
    PFC_Ctrl_Info.AC_VolBase            = 255;//�����ѹ��׼��ʼֵ
    PFC_Ctrl_Info.PWM_Period            = PFC_PWM_PERIOD;
    PFC_Ctrl_Info.PWM_Duty              = 0;//ռ�ձ�Ϊ��ʱ�ر���ѹ        
    PFC_Ctrl_Info.vBus_Ref_SS           = PFC_VBUS_REF_INIT_SS;//��ѹ���������ο�ֵ����  
    PFC_Ctrl_Info.PWM_Duty_Up           = PFC_PWM_DUTY_UP_LIMIT;
    PFC_Ctrl_Info.PWM_Duty_Dn           = PFC_PWM_DUTY_DN_LIMIT;
    PFC_PID_Vol.err_Integral            = 0;//�����ۼ�ֵ��ʼ��
    PFC_PID_Cur.err_Integral            = 0;//�����ۼ�ֵ��ʼ��
    PFC_Ctrl_Info.Zero_DutySS           = PFC_Ctrl_Info.PWM_Period*9 >> 7;//PFC���㻺ռ�ձ�����
    PFC_Ctrl_Info.PFC_FreqOK_Cnt        = 0;
    
    //׼��̬��ر������
    StartCheck_Flag_Info.bit.AC_Vol_High_OK             = E_FALSE;
    StartCheck_Flag_Info.bit.AC_Vol_Low_OK              = E_FALSE; 
    StartCheck_Flag_Info.bit.FREQ_High_OK               = E_FALSE;
    StartCheck_Flag_Info.bit.FREQ_Low_OK                = E_FALSE;     
    StartCheck_Flag_Info.bit.vBus_High_OK               = E_FALSE;
    StartCheck_Flag_Info.bit.vBus_Low_OK                = E_FALSE;    
    
    COM_StartCheck_AuxPower_Up.period_Cnt               = 0;
    COM_StartCheck_AuxPower_Down.period_Cnt             = 0;
    PFC_StartCheck_AC_VOL_Up.period_Cnt                 = 0;
    PFC_StartCheck_AC_VOL_Down.period_Cnt               = 0;

    //�����е�ת����л�ʱ�䣬��˷���PFC��ʼ����
    for(i16_variableI= 0;i16_variableI<680;i16_variableI++)
    {
        if(i16_variableI<600)
        {
            RepeatOut[i16_variableI] = 0;
            RepeatErr[i16_variableI] = 0;
        }
        INV_AC_Vol_table[i16_variableI] = 0;
    }
    
    RMS_PQ_Info.out_RMS_INV_AC_Vol          = 0; 
    RMS_PQ_Info.sum_INV_AC_Vol              = 0;
    //��·��ر�����ʼ��

    INV_Ctrl_Info.short_Vol_Peak_Cnt        = 0;
    INV_Ctrl_Info.short_Start_Cnt           = 0;
    INV_Ctrl_Info.short_Start_Flag          = INV_ACOUT_SCP_START_FLAG;//��ʼ̬�����ö�·���� 

    RMS_PQ_Info.periodDot_Val       = COM_RMS_DOT_VAL;
    RMS_PQ_Info.dot_Reciprocal      = COM_RMS_DOT_RECIPROCAL;  

    INV_PID_DCIM.ref = 0;    
    PFC_StartCount = 0;  

    PFC_PID_Vol.err_Integral = PFC_VOL_ERR_INTEGRAL_MIN;//��ʼ��ʱ��PFC��ѹ��·������С���ֵ
    PFC_PID_Vol.kp          = PFC_VOL_KP;
    PFC_VBUS_READY_OK_Cnt   = 0;
    
    PFC_VPID_Info.u32kp_Min         = PFC_VPID_KP_MIN;   //���غ�Kpֵ�����仯����Сֵ 
    PFC_VPID_Info.u32kp_Max         = PFC_VPID_KP_MAX;        //����ʱKpֵ���
    PFC_VPID_Info.u32kp_Hold        = PFC_VPID_KP_HOLD_INIT; //�Ŵ���kp��ʼֵ
    PFC_VPID_Info.u16kp_Step_Val    = PFC_VPID_KP_STEP_VAL; //ÿ�β���ֵ  
    PFC_VPID_Info.u8kp_Shift_Val    = PFC_VPID_KP_SHIFT_VAL;//������λֵ 
    PFC_VPID_Info.u16kp_I_Induc_RMS_Val = PFC_I_INDUC_RMS_VAL; //PFC��ѹ��KP�仯ʱ���ж���������е�����Чֵ-��:1A*512   
}


/*************************************************
Description: INV_Variableinit
Input      : 
Return     : 
Others     : �Ӵ���״̬�л�����̬������ʼ��
*************************************************/
void INV_Run_VariableInit(void)
{  
    COM_Ctr_Info.PWM_Enable = 0;//������ʹ���ź�
    INV_Ctrl_Info.test_Cnt = 0;
    UPS_Ctr_Info.lock_Phase_OK_Cnt  = 0;//����OK����
   
    //�����е�ת����л�ʱ�䣬��˷���PFC��ʼ����
    for(i16_variableI= 0;i16_variableI<680;i16_variableI++)
    {
        if(i16_variableI<600)
        {
            RepeatOut[i16_variableI] = 0;
            RepeatErr[i16_variableI] = 0;
        }
        INV_AC_Vol_table[i16_variableI] = 0;
        PFC_AC_Vol_table[i16_variableI] = 0;
    }    
    RMS_PQ_Info.out_RMS_INV_AC_Vol          = 0; 
    RMS_PQ_Info.out_RMS_INV_AC_Vol_Hold     = 0;
    RMS_PQ_Info.sum_INV_AC_Vol              = 0;
    RMS_PQ_Info.sum_INV_AC_Vol_Hold         = 0;
    RMS_PQ_Info.out_RMS_PFC_AC_Vol          = 0; 
    RMS_PQ_Info.out_RMS_PFC_AC_Vol_Hold     = 0;
    RMS_PQ_Info.sum_PFC_AC_Vol              = 0;
    RMS_PQ_Info.sum_PFC_AC_Vol_Hold         = 0; 
    
 
    
    UPS_Ctr_Info.PWM_Period         = 0;//UPS������µ�����
    UPS_Ctr_Info.delta_CMP_Val      = 0;//�����ѹ�������ѹ������CMP����������ֵ
    UPS_Ctr_Info.delta_P            = 0;//�������������Ĳ�ֵ*50
    UPS_Ctr_Info.delta_T            = 0;//

    UPS_Ctr_Info.lock_Phase_Cnt     = 0;
    
    UPS_Ctr_Info.lock_Phase_OK      = 0;//����OK
    UPS_Ctr_Info.lock_Phase_OK_Cnt  = 0;//����OK����
    UPS_Ctr_Info.V_ACIN_OK          = 0;//�е����OK��־
    UPS_Ctr_Info.V_ACIN_OK_Cnt      = 0;//�е����OK���� 
    
    UPS_Ctr_Info.V_ACIN_NOK_Cnt_P   = 0;//�������е����NOK����  
    UPS_Ctr_Info.V_ACIN_NOK_Cnt_N   = 0;//�������е����NOK����
    
    UPS_Ctr_Info.V_ACIN_HYS_ADV     = 0;//1���е粨���ͺ�2���е粨�γ�ǰ
    UPS_Ctr_Info.PFC_CMP_Val        = 0;//���������ѹ����ʱ��CMP��������
    
    UPS_Ctr_Info.flag1              = 0; 
    UPS_Ctr_Info.flag2              = 0; 
    UPS_Ctr_Info.flag3              = 0;   

    INV_Repeat_Info.err             = 0; 
    INV_Repeat_Info.out             = 0;    
    INV_PID_Vol.out                 = 0;
    INV_PID_Cur.out                 = 0;    
    INV_PID_Vol.err_Integral        = 0;
    INV_PID_Cur.err_Integral        = 0;

    INV_Ctrl_Info.DCIM_Sum      = 0;
    INV_Ctrl_Info.DCIM_Val_Fir  = 0;         
    INV_PID_DCIM.out            = 0;    
    INV_PID_DCIM.err_Integral   = 0;  
    
    
    INV_Ctrl_Info.AC_Vol_AMP_Target = 0;
    PFC_Ctrl_Info.DCDC_CHG_Out      = 0;
    
    #if INV_AC_VOL_FREQ_SELECT == INV_AC_VOL_FREQ_50HZ
        INV_Ctrl_Info.SineTab_Ptr           = &Sine_Table_50Hz[0];
        INV_Ctrl_Info.periodDot_Val         = SPWMWAVE_DOT_50Hz;
        INV_Ctrl_Info.periodDot_Val_Half    = SPWMWAVE_DOT_50Hz_2;
    
        INV_Repeat_Info.periodDot_Val           = SPWMWAVE_DOT_50Hz;     
        INV_Ctrl_Info.short_Vol_Peak_Time_Val   = SPWMWAVE_DOT_50Hz_4;//��������
    
        RMS_PQ_Info.dot_Reciprocal              = SPWMWAVE_DOT_RECIPROCAL_50Hz;
 
    
    #else
        INV_Ctrl_Info.SineTab_Ptr           = &Sine_Table_60Hz[0];
        INV_Ctrl_Info.periodDot_Val         = SPWMWAVE_DOT_60Hz;
        INV_Ctrl_Info.periodDot_Val_Half    = SPWMWAVE_DOT_60Hz_2;
    
        INV_Repeat_Info.periodDot_Val           = SPWMWAVE_DOT_60Hz;     
        INV_Ctrl_Info.short_Vol_Peak_Time_Val   = SPWMWAVE_DOT_60Hz_4;//��������

        RMS_PQ_Info.dot_Reciprocal              = SPWMWAVE_DOT_RECIPROCAL_60Hz;
        RMS_PQ_Info.periodDot_Val               = SPWMWAVE_DOT_60Hz;    
    
    #endif


    //--------------------------------------------------------------------------------------
    #if ( OPERATING_MODE == NORMAL_MODE )			 		
        INV_Ctrl_Info.AC_Vol_AMP_Target_Ref     = INV_AC_VOL_AMP_VAL_NORMAL_REF;
    #else
        INV_Ctrl_Info.AC_Vol_AMP_Target_Ref     = INV_AC_VOL_AMP_VAL_DEBUG_REF; 
    #endif	

    
    INV_Parall_Info.Flag.masterFlag         = 0;		//Ĭ�Ϸ�����������ȷ�ϱ���״̬  
    INV_Parall_Info.Flag.slaveFlag          = 0;		//Ĭ�ϷǴӻ�������ȷ�ϱ���״̬      
    INV_Parall_Info.Flag.syn_Freq_OK        = 0;       
    INV_Parall_Info.Flag.selectStateOK      = 0;
    INV_Parall_Info.slave_Cnt               = 0;
    INV_Parall_Info.master_Cnt              = 0;
    INV_Parall_Info.selectSlave_Cnt         = 0;    
    INV_Parall_Info.judge_Vol_RMS_Val       = INV_JUDGE_VOL_RMS_VAL;
    
    
    RMS_PQ_Info.out_RMS_INV_AC_Vol          = 0;  
    RMS_PQ_Info.out_RMS_PFC_AC_Vol          = 0;      
    RMS_PQ_Info.out_RMS_CurInduc            = 0;
    RMS_PQ_Info.out_RMS_CurLoad             = 0;    
    RMS_PQ_Info.out_P                       = 0;
    RMS_PQ_Info.periodDot_Cnt               = 0;    
    INV_PID_Power.ref                       = INV_POWER_REF;//���ʻ��ο�ֵ��ʼ��
 
    
    INV_Ctrl_Info.short_Start_Time_Val      = INV_ACOUT_SCP_START_TIME;//(ms)
    INV_Ctrl_Info.short_Start_Flag          = INV_ACOUT_SCP_START_FLAG;//��ʼ̬�����ö�·����
    INV_Ctrl_Info.PWM_Freq_Init             = INV_PWM_FREQ;
    INV_Ctrl_Info.PWM_Period_Init           = INV_PWM_PERIOD;
    INV_Ctrl_Info.PWM_Period                = INV_PWM_PERIOD;
    INV_Ctrl_Info.mode_Operate              = OPERATING_MODE;
    INV_Ctrl_Info.mode_PID_Loop             = INV_PID_LOOP_MODE;
    INV_Ctrl_Info.mode_Repeat_Ctrl          = INV_REPEAT_CTRL_SELECT;//�ظ�����ʹ��
    INV_Ctrl_Info.mode_Power_Ctrl           = INV_POWER_CTRL_SELECT;//�޹��ʿ���ʹ�� 
    INV_Ctrl_Info.mode_PQ_Droop_Ctrl        = INV_PQ_DROOP_CTRL_SELECT;//PQ�´�����ʹ��     
    INV_Ctrl_Info.mode_AC_Freq_Select       = INV_AC_VOL_FREQ_SELECT;//50Hz/60Hz 
    INV_Ctrl_Info.mode_AC_Software_Ctrl     = INV_SOFTWARE_CTRL_SELECT;//AC�������������
   
    INV_Ctrl_Info.virtual_Res_Coeff         = INV_VIRTUAL_RES;//�����迹  
    INV_Ctrl_Info.virtual_Res_Coeff_Min     = INV_VIRTUAL_RES_MIN;//�����迹 
    
    INV_Ctrl_Info.curLoop_Up                = INV_CUR_UP_LIMIT;
    INV_Ctrl_Info.curLoop_Dn                = INV_CUR_DN_LIMIT;  
    INV_Ctrl_Info.PWM_Duty_Up               = SPWMDUTY_UP_LIMIT;
    INV_Ctrl_Info.PWM_Duty_Dn               = SPWMDUTY_DN_LIMIT;      

    
    INV_Ctrl_Info.SS_Step_Value             = INV_SOFTWARE_STEP_VALUE;//AC������ʱ����ֵϵ��ÿ�β���ֵ        
    INV_Ctrl_Info.SS_Shift_Value            = INV_SOFTWARE_SHIFT_VALUE;//AC������ʱ����ֵϵ��������λֵ       
    INV_Ctrl_Info.SS_AMP_Target_Hold        = INV_SOFTWARE_HOLD_INIT;//�Ŵ��ķ�ֵϵ����ʼֵ 

    INV_Ctrl_Info.short_Vol_Peak_Cnt        = 0;
    INV_Ctrl_Info.short_Start_Cnt           = 0;

    //׼��̬��ر������
    StartCheck_Flag_Info.bit.AC_Vol_High_OK = E_FALSE;
    StartCheck_Flag_Info.bit.AC_Vol_Low_OK  = E_FALSE; 
    StartCheck_Flag_Info.bit.FREQ_High_OK   = E_FALSE;
    StartCheck_Flag_Info.bit.FREQ_Low_OK    = E_FALSE;     
    StartCheck_Flag_Info.bit.vBus_High_OK   = E_FALSE;
    StartCheck_Flag_Info.bit.vBus_Low_OK    = E_FALSE;  

    RMS_PQ_Info.periodDot_Val       = COM_RMS_DOT_VAL;
    RMS_PQ_Info.dot_Reciprocal      = COM_RMS_DOT_RECIPROCAL;
    
    INV_PID_DCIM.ref = 0;
    
    
    INV_Ctrl_Info.PWM_Period_43Hz = (COM_MCU_CLK  / (INV_Ctrl_Info.periodDot_Val * PLL_VACOUT_FREQ_MIN*2)) + INV_DEADTIME - 2;   //1/fk* (m+2�Cp)* 2
    INV_Ctrl_Info.PWM_Period_67Hz = (COM_MCU_CLK  / (INV_Ctrl_Info.periodDot_Val * PLL_VACOUT_FREQ_MAX*2)) + INV_DEADTIME - 2;

}



/*-------------------------------------------------------------------------------------
 *  No more.
 *------------------------------------------------------------------------------------*/


