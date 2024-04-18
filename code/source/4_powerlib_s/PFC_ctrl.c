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
#include "sys_state_machine.h"
/***************************************************************************/

/*************************************************
Description: PFC_VBus_RefCalc
Input      : 
Return     : 
Others     : ĸ�ߵ�ѹ�ο�ֵ�߼�����
*************************************************/
#define PFC_Reverse_BufferValue  15 //�趨����ֵ
int16_t PFC_Reverse_BufferCount = 0;//����ʱ��ֹ������������ֵ
int32_t PFC_Zero_DutySS_Count = 0;//���㷴��ʱ��ռ�ձȻ������������ڼ�������
uint8_t PFC_CHG_Out_StepVal = 20;
uint8_t PFC_Reverse_Flag = 0; //����㻻���־

void  PFC_VBus_RefCalc(void)
{
    #if 1
        //�����仯�ο�ֵ
        if((PFC_Ctrl_Info.DCDC_CHG_Out - UARTx_DC_Info.vBus_SetVal) <= -PFC_CHG_Out_StepVal )
        {
            PFC_Ctrl_Info.DCDC_CHG_Out += PFC_CHG_Out_StepVal;
        } 
        else  if((PFC_Ctrl_Info.DCDC_CHG_Out - UARTx_DC_Info.vBus_SetVal) >= PFC_CHG_Out_StepVal)
        {
            PFC_Ctrl_Info.DCDC_CHG_Out -= PFC_CHG_Out_StepVal;        
        } 
        else
        {   
            PFC_Ctrl_Info.DCDC_CHG_Out = UARTx_DC_Info.vBus_SetVal;////DCDC����Ŀ��Ƶ�ѹֵ(��ѹ��ͨ�Ŵ���)
        }    
    #else 
        PFC_Ctrl_Info.DCDC_CHG_Out = 1600;   
    #endif

    //Vbus��ref�����Сֵ����
    if(PFC_Ctrl_Info.DCDC_CHG_Out > PFC_Ctrl_Info.vBUS_Ref_Max )
    {
        PFC_Ctrl_Info.DCDC_CHG_Out = PFC_Ctrl_Info.vBUS_Ref_Max;
    }            
    
    if(PFC_Ctrl_Info.DCDC_CHG_Out < PFC_Ctrl_Info.vBUS_Ref_Min)
    {
        PFC_Ctrl_Info.DCDC_CHG_Out = PFC_Ctrl_Info.vBUS_Ref_Min;
    } 
    PFC_Ref_Info.u32SS_Set_Val = PFC_Ctrl_Info.DCDC_CHG_Out<<PFC_Ref_Info.u8SS_Shift_Val;//Vbus�Ĳο�����ֵ�Ŵ���ֵ

    
    //ע�⣺���߼����������ƶ�λ�ã�
    //ÿ�ι���ʱReverse_count�������趨ֵ��ʼ�����´ι��㣬��ֹ���㸽���������������㸽������������������        
    if(PFC_Reverse_BufferCount <= PFC_Reverse_BufferValue)
        PFC_Reverse_BufferCount  ++;//�������㸽������������������
    
    //������Ǵ���
    if( - ADSample_Info.PFC_AC_Vol_AD_FIR >= 0 && PFC_Reverse_Flag == 0&&(PFC_Reverse_BufferCount >= PFC_Reverse_BufferValue))               
    {
        PFC_Reverse_Flag = 1;  //����㷴����λ
        PFC_Reverse_BufferCount = 0;//����ʱ����������Reverse_count����ֵ
        PFC_Zero_DutySS_Count = 0;//����ʱ������������ֵ����
    }
    else if( - ADSample_Info.PFC_AC_Vol_AD_FIR <= 0 && PFC_Reverse_Flag == 1&&(PFC_Reverse_BufferCount >= PFC_Reverse_BufferValue))
    {
        PFC_Reverse_Flag = 0;   
        PFC_Reverse_BufferCount = 0;    
        PFC_Zero_DutySS_Count = 0;          
    }
    
}


/*************************************************
Description: PFC_Ctrl
Input      : 
Return     : 
Others     : PFC����
*************************************************/
int32_t PFC_vBus_Hold,PFC_vBus_Fir = 0;
void PFC_Ctrl(void)
{
    int32_t Duty_Out = 0;      
    //PFC��ѹ����PI
    if(PFC_Ctrl_Info.RMS_InductorCur > PFC_VPID_Info.u16kp_I_Induc_RMS_Val)//PFC��ѹ��KP�仯ʱ���ж���������е�����Чֵ
    {
        if(PFC_PID_Vol.kp > PFC_VPID_Info.u32kp_Min)           
            PFC_VPID_Info.u32kp_Hold -= PFC_VPID_Info.u16kp_Step_Val;   
    }
    else
    {
        if(PFC_PID_Vol.kp < PFC_VPID_Info.u32kp_Max)             
            PFC_VPID_Info.u32kp_Hold += PFC_VPID_Info.u16kp_Step_Val;          
    } 
    PFC_PID_Vol.kp = PFC_VPID_Info.u32kp_Hold>>PFC_VPID_Info.u8kp_Shift_Val;
    
    
    /***************************************************************************/
    /*-------------------��ѹ���ƻ�·����������--------------------------------*/    
    
    if(PFC_Ref_Info.u32SS_vBus_Hold < PFC_Ref_Info.u32SS_Set_Val)
    {      
        PFC_Ref_Info.u32SS_vBus_Hold +=PFC_Ref_Info.u16SS_Step_Val;
    }
    else
    {
        PFC_Ref_Info.u32SS_vBus_Hold = PFC_Ref_Info.u32SS_Set_Val;
    }
    
    /***************************************************************************/
    /*-------------------��ѹ���ο�����----------------------------------------*/
    PFC_PID_Vol.ref       = PFC_Ref_Info.u32SS_vBus_Hold >> PFC_Ref_Info.u8SS_Shift_Val;        
    
    /***************************************************************************/
    /*-------------------��ѹ������--------------------------------------------*/ 
    //����ĸ���ж��Ƿ�ͻ����ʱ����������ۻ����
    PFC_PID_Vol.fdb       = PFC_Ctrl_Info.AD_VBus;
    PFC_PID_Vol.Calc(&PFC_PID_Vol);

    /***************************************************************************/
    /*-------------------����������--------------------------------------------*/
    PFC_PID_Cur.ref     = (User_Divider(PFC_PID_Vol.out * PFC_Ctrl_Info.AD_VolPeak, PFC_Ctrl_Info.AC_VolBase))>>2 ;
//    PFC_PID_Cur.ref     = PFC_PID_Vol.out * PFC_Ctrl_Info.AD_VolPeak>>10 ;

    PFC_PID_Cur.fdb     = PFC_Ctrl_Info.AD_InducCurPeak;          
    PFC_PID_Cur.Calc(&PFC_PID_Cur);
     
    
    //ռ�ձ�ǰ��
    PFC_Ctrl_Info.feedforward_Duty = User_Divider((PFC_Ctrl_Info.AD_VolPeak << 12), PFC_Ctrl_Info.AD_VBus);
//    PFC_Ctrl_Info.feedforward_Duty = PFC_Ctrl_Info.AD_VolPeak*5*PFC_Ctrl_Info.RMS_Vol>>10;
//    PFC_Ctrl_Info.feedforward_Duty = User_Divider((PFC_Ctrl_Info.AD_VolPeak*PFC_Ctrl_Info.RMS_Vol << 3), PFC_Ctrl_Info.AD_VBus);
//    PFC_Ctrl_Info.feedforward_Duty =User_Divider(ABSFUN((PLL_Ctrl_Info_V_ACIN.i32SinTheta>>12)*PFC_Ctrl_Info.RMS_Vol*181),PFC_Ctrl_Info.AD_VBus>>2);
//    PFC_Ctrl_Info.feedforward_Duty =PFC_Ctrl_Info.AD_VolPeak*5*34>>5;
//    PFC_Ctrl_Info.feedforward_Duty =PFC_Ctrl_Info.AD_VolPeak*5>>1;
    if(PFC_Ctrl_Info.feedforward_Duty >  4096)
        PFC_Ctrl_Info.feedforward_Duty =  4096;   
    
    Duty_Out = (4096 - PFC_Ctrl_Info.feedforward_Duty + PFC_PID_Cur.out);
    
//    Duty_Out =PFC_PID_Cur.out;
    if(Duty_Out > 4090)
        Duty_Out = 4090;
    if(Duty_Out < 0)
        Duty_Out = 0; 

    
    /***************************************************************************/
    /*-------------------ռ�ձȸ���--------------------------------------------*/        
    PFC_Ctrl_Info.PWM_Duty   =  (int32_t)(( Duty_Out * PFC_Ctrl_Info.PWM_Period ) >> 12)  ;//����ռ�ձ�                                  
 
    //����㸽����ռ�ձ�ֵ����
    if(PFC_Ctrl_Info.active_Power < 970)
        PFC_Ctrl_Info.Zero_DutySS = PFC_Ctrl_Info.PWM_Period*10 >> 6;
    else  if(PFC_Ctrl_Info.active_Power > 1000)
        PFC_Ctrl_Info.Zero_DutySS = PFC_Ctrl_Info.PWM_Period*10 >> 7; 
    
    //ռ�ձȻ�����
    if(PFC_Zero_DutySS_Count < 15 && ((PFC_Ctrl_Info.Zero_DutySS + PFC_Zero_DutySS_Count * PFC_Ctrl_Info.Zero_DutySS) <PFC_Ctrl_Info.PWM_Duty ))//����У����־
    {
        PFC_Zero_DutySS_Count ++;
        PFC_Ctrl_Info.PWM_Duty = PFC_Ctrl_Info.Zero_DutySS + PFC_Zero_DutySS_Count * PFC_Ctrl_Info.Zero_DutySS;            
    }  
        
}

