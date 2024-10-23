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
#include "sys_state_machine.h"
#include "sys_mcu_header.h"
/***************************************************************************/

/*************************************************
Description: COM_Get_Protect_Flag
Input      : 
Return     : 
Others     : ���ϱ����ж��߼�
*************************************************/
void COM_Get_Protect_Flag( int Check_Value, Protect_Check_Var_t * Check_Info,unsigned short int* protect_code )
{
    if (( *protect_code & Check_Info->status_Val ) != E_FALSE )//����˱�����־�Ѿ���λ�����˳����
    {
        return;
    }

    if ( Check_Info->compare_Type == E_MAX )//��ֵΪ��Сֵ
    {
        if ( Check_Value > Check_Info->ref_Val)
        {	

            Check_Info->period_Cnt++;
            if ( Check_Info->period_Cnt >= 0x0FFFFFFF ) 
            {
                Check_Info->period_Cnt = 0x0FFFFFFF;
            }
            if ( Check_Info->period_Cnt >= Check_Info->period_Val )
            {
                Check_Info->period_Cnt = 0;
                *protect_code |= Check_Info->status_Val;
            }
        }
        else if ( Check_Value < Check_Info->hysteretic_Val )
        {
            Check_Info->period_Cnt = 0;
        }
    }
    /*------------------------------------------------------------------------------------*/
    else if ( Check_Info->compare_Type == E_MIN )//��ֵΪ���ֵ
    {
        if ( Check_Value < Check_Info->ref_Val )//�ﵽ������ֵҪ��
        {
            Check_Info->period_Cnt++;
            if ( Check_Info->period_Cnt >= 0x0FFFFFFF ) 
            {
                Check_Info->period_Cnt = 0x0FFFFFFF;
            }
            if ( Check_Info->period_Cnt >= Check_Info->period_Val )
            {
                Check_Info->period_Cnt  = 0;        //ʱ�����ֵ����
                *protect_code |= Check_Info->status_Val;	
            }
        }
        else if ( Check_Value > Check_Info->hysteretic_Val )//���û�����㷧ֵ����
        {
            Check_Info->period_Cnt = 0;//ʱ�����ֵ����
        }
    }
    else
    {
         return;//������
    }
}

/*************************************************
Description: Fault_State_Check
Input      : 
Return     : 
Others     : ���ϱ�����������
*************************************************/
void Fault_State_Check(void)
{
    if (COM_Ctr_Info.INV_PFC_Mode_Select == INV_MODE&& COM_RUN_STATE == State_Context.state_Value)//INVʹ��ģʽ���ŵ�״̬
    {
        /*------------------------------------------------------------------------------------*/				
        /*---------------------����ʱ���ζ�·�ź�50ms���ѹ����100V���ö�·����---------------*/
         if(COM_AD_Data_Info.VACOUT_RMS_Val_Fir > INV_ACOUT_SCP_START_VAL || \
            INV_Ctrl_Info.short_Start_Cnt >= INV_Ctrl_Info.short_Start_Time_Val) //�����ѹ����100V << 2 ��������ʱ�����ѹһֱ����5V����50ms  ���־���ö�·����
         {
             INV_Ctrl_Info.short_Start_Flag = 0;   //0:���ö�·����             
         }
     }
    
    #if OPERATING_MODE == DEBUG_MODE
        return;
    #endif
    
    if (SysClockBase_ms.faultCheck_1ms != E_TRUE || System_ProtectFlag_Info.all != 0) return;
    
    if (COM_Ctr_Info.INV_PFC_Mode_Select == PFC_MODE)//PFCʹ��ģʽ�����״̬            
    {     
        /*-------------------------PFC״̬�¹��ر���--------------------------------------------*/
        COM_Get_Protect_Flag( COM_AD_Data_Info.VACIN_PFC_Power , &PFC_P_OLP1_Info   , &System_ProtectFlag_Info.all );
        COM_Get_Protect_Flag( COM_AD_Data_Info.VACIN_PFC_Power , &PFC_P_OLP2_Info   , &System_ProtectFlag_Info.all );

        /*-------------------------PFC״̬�µ�й�������----------------------------------------*/
        COM_Get_Protect_Flag( COM_AD_Data_Info.iInduc_RMS_Val_Fir , &PFC_RMS_OCP1_Info   , &System_ProtectFlag_Info.all );
        COM_Get_Protect_Flag( COM_AD_Data_Info.iInduc_RMS_Val_Fir , &PFC_RMS_OCP2_Info   , &System_ProtectFlag_Info.all );
    }
    else if (COM_Ctr_Info.INV_PFC_Mode_Select == INV_MODE)//INVʹ��ģʽ���ŵ�״̬
    {
        /*-------------------------��·����-----------------------------------------------------*/
        if(INV_Ctrl_Info.short_Start_Flag == 0 && COM_RUN_STATE == State_Context.state_Value)//��·������־��������ö�·����
        {
            COM_Get_Protect_Flag( COM_AD_Data_Info.VACOUT_RMS_Val_Fir , &INV_ACOUT_SCP_Info , &System_ProtectFlag_Info.all );
            
            if(ADSample_Info.INV_AC_Vol_AD_FIR > 0)            
            {
                COM_Get_Protect_Flag( ADSample_Info.INV_AC_Vol_AD_FIR , &INV_ACOUT_P_SCP_Info , &System_ProtectFlag_Info.all );  		
            }
            else
            {
                COM_Get_Protect_Flag( -ADSample_Info.INV_AC_Vol_AD_FIR , &INV_ACOUT_P_SCP_Info , &System_ProtectFlag_Info.all );  		
            }
        }   
        /*-------------------------�����ѹ��ѹ����-----------------------------------------------*/
        COM_Get_Protect_Flag( COM_AD_Data_Info.VACOUT_RMS_Val_Fir , &INV_ACOUT_OVP_Info , &System_ProtectFlag_Info.all );

        /*-------------------------�����ѹǷѹ---------------------------------------------------*/
        COM_Get_Protect_Flag( COM_AD_Data_Info.VACOUT_RMS_Val_Fir , &INV_ACOUT_LVP_Info , &System_ProtectFlag_Info.all );
        
        /*-------------------------�ŵ�ʱ���ر���-----------------------------------------------*/
        COM_Get_Protect_Flag( COM_AD_Data_Info.VACOUT_ActivePower , &INV_P_OLP1_Info   , &System_ProtectFlag_Info.all );
        COM_Get_Protect_Flag( COM_AD_Data_Info.VACOUT_ActivePower , &INV_P_OLP2_Info   , &System_ProtectFlag_Info.all );
        COM_Get_Protect_Flag( COM_AD_Data_Info.VACOUT_ActivePower , &INV_P_OLP3_Info   , &System_ProtectFlag_Info.all );
        COM_Get_Protect_Flag( COM_AD_Data_Info.VACOUT_ActivePower , &INV_P_OLP4_Info   , &System_ProtectFlag_Info.all );
        
        COM_Get_Protect_Flag( COM_AD_Data_Info.VACOUT_ApparentPower , &INV_S_OLP1_Info   , &System_ProtectFlag_Info.all );
        COM_Get_Protect_Flag( COM_AD_Data_Info.VACOUT_ApparentPower , &INV_S_OLP2_Info   , &System_ProtectFlag_Info.all );
        COM_Get_Protect_Flag( COM_AD_Data_Info.VACOUT_ApparentPower , &INV_S_OLP3_Info   , &System_ProtectFlag_Info.all );
        COM_Get_Protect_Flag( COM_AD_Data_Info.VACOUT_ApparentPower , &INV_S_OLP4_Info   , &System_ProtectFlag_Info.all );
      
        /*-------------------------�ŵ�ʱ��й�������--------------------------------------------*/
        COM_Get_Protect_Flag( COM_AD_Data_Info.iInduc_RMS_Val_Fir , &INV_RMS_OCP1_Info , &System_ProtectFlag_Info.all );
        COM_Get_Protect_Flag( COM_AD_Data_Info.iInduc_RMS_Val_Fir , &INV_RMS_OCP2_Info , &System_ProtectFlag_Info.all );
        COM_Get_Protect_Flag( COM_AD_Data_Info.iInduc_RMS_Val_Fir , &INV_RMS_OCP3_Info , &System_ProtectFlag_Info.all );
        COM_Get_Protect_Flag( COM_AD_Data_Info.iInduc_RMS_Val_Fir , &INV_RMS_OCP4_Info , &System_ProtectFlag_Info.all );

        COM_Get_Protect_Flag( ABSFUN(ADSample_Info.curLoad_AD_FIR) , &INV_IST_OCP1_Info , &System_ProtectFlag_Info.all );
		
        /*-------------------------ĸ��Ƿѹ��Ᵽ��-----------------------------------------------*/
        COM_Get_Protect_Flag( COM_AD_Data_Info.vBus_Val_Fir  , &INV_VBus_LVP1_Info  , &System_ProtectFlag_Info.all );
        COM_Get_Protect_Flag( COM_AD_Data_Info.vBus_Val_Fir  , &INV_VBus_LVP2_Info  , &System_ProtectFlag_Info.all );
        COM_Get_Protect_Flag( COM_AD_Data_Info.vBus_Val_Fir  , &INV_VBus_LVP3_Info  , &System_ProtectFlag_Info.all );                
    }

    /*-------------------------ĸ�߹�ѹ��Ᵽ��-----------------------------------------------*/
    COM_Get_Protect_Flag( COM_AD_Data_Info.vBus_Val_Fir , &COM_VBus_OVP1_Info   , &System_ProtectFlag_Info.all );
    COM_Get_Protect_Flag( COM_AD_Data_Info.vBus_Val_Fir , &COM_VBus_OVP2_Info   , &System_ProtectFlag_Info.all );
    COM_Get_Protect_Flag( COM_AD_Data_Info.vBus_Val_Fir , &COM_VBus_OVP3_Info   , &System_ProtectFlag_Info.all );

    /*---------------------������Դ��ѹ����---------------------------------------------------------------------*/		
    COM_Get_Protect_Flag( COM_AD_Data_Info.auxPower_Val_Fir , &COM_AuxPower_OVP_Info , &System_ProtectFlag_Info.all );	  		
    /*---------------------������ԴǷѹ����---------------------------------------------------------------------*/		
    COM_Get_Protect_Flag( COM_AD_Data_Info.auxPower_Val_Fir , &COM_AuxPower_LVP_Info , &System_ProtectFlag_Info.all );


    /*-------------------------���±���-------------------------------------------------------*/
    COM_Get_Protect_Flag( COM_AD_Data_Info.temp_NTC_Val_Fir , &COM_OTP1_Info , &System_ProtectFlag_Info.all );
    COM_Get_Protect_Flag( COM_AD_Data_Info.temp_NTC_Val_Fir , &COM_OTP2_Info , &System_ProtectFlag_Info.all );	

    /*-------------------------VREF����-------------------------------------------------------*/
    COM_Get_Protect_Flag( COM_AD_Data_Info.vRef_Val_Fir , &COM_Vref_OVP_Info , &System_ProtectFlag_Info.all );
    COM_Get_Protect_Flag( COM_AD_Data_Info.vRef_Val_Fir , &COM_Vref_LVP_Info , &System_ProtectFlag_Info.all );	
    

    SysClockBase_ms.faultCheck_1ms = 0;	    //���ϼ��1ms ʱ�ӱ������  				
}

/*************************************************
Description: Fault_State_Deal
Input      : 
Return     : 
Others     : 
*************************************************/
void Fault_State_Deal(void)
{
}

/*************************************************
Description: Protect_Release
Input      : 
Return     : 
Others     : �������ʱ��ʼ����ر���
*************************************************/
void Protect_Release(void)
{
}


