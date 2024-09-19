/*****************************************************************************
Copyright (C) 2021 China Micro Semiconductor Limited Company. All Rights Reserved.
@Description: 
@Author     : 
@Version    : 0.0.1  
@History    : 
@Attention  ：
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
Others     : 故障保护判断逻辑
*************************************************/
void COM_Get_Protect_Flag( int Check_Value, Protect_Check_Var_t * Check_Info,unsigned short int* protect_code )
{
    if (( *protect_code & Check_Info->status_Val ) != E_FALSE )//如果此保护标志已经置位，则退出检查
    {
        return;
    }

    if ( Check_Info->compare_Type == E_MAX )//阀值为最小值
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
    else if ( Check_Info->compare_Type == E_MIN )//阀值为最大值
    {
        if ( Check_Value < Check_Info->ref_Val )//达到保护阀值要求
        {
            Check_Info->period_Cnt++;
            if ( Check_Info->period_Cnt >= 0x0FFFFFFF ) 
            {
                Check_Info->period_Cnt = 0x0FFFFFFF;
            }
            if ( Check_Info->period_Cnt >= Check_Info->period_Val )
            {
                Check_Info->period_Cnt  = 0;        //时间计数值清零
                *protect_code |= Check_Info->status_Val;	
            }
        }
        else if ( Check_Value > Check_Info->hysteretic_Val )//如果没有满足阀值条件
        {
            Check_Info->period_Cnt = 0;//时间计数值清零
        }
    }
    else
    {
         return;//错误处理
    }
}

/*************************************************
Description: Fault_State_Check
Input      : 
Return     : 
Others     : 故障保护功能启用
*************************************************/
void Fault_State_Check(void)
{
    if (COM_Ctr_Info.INV_PFC_Mode_Select == INV_MODE&& COM_RUN_STATE == State_Context.state_Value)//INV使能模式：放电状态
    {
        /*------------------------------------------------------------------------------------*/				
        /*---------------------启机时屏蔽短路信号50ms或电压大于100V启用短路保护---------------*/
         if(COM_AD_Data_Info.VACOUT_RMS_Val_Fir > INV_ACOUT_SCP_START_VAL || \
            INV_Ctrl_Info.short_Start_Cnt >= INV_Ctrl_Info.short_Start_Time_Val) //输出电压大于100V << 2 或者启机时输出电压一直低于5V超过50ms  清标志启用短路保护
         {
             INV_Ctrl_Info.short_Start_Flag = 0;   //0:启用短路功能             
         }
     }
    
    #if OPERATING_MODE == DEBUG_MODE
        return;
    #endif
    
    if (SysClockBase_ms.faultCheck_1ms != E_TRUE || System_ProtectFlag_Info.all != 0) return;
    
    if (COM_Ctr_Info.INV_PFC_Mode_Select == PFC_MODE)//PFC使能模式：充电状态            
    {     
        /*-------------------------PFC状态下过载保护--------------------------------------------*/
        COM_Get_Protect_Flag( COM_AD_Data_Info.VACIN_PFC_Power , &PFC_P_OLP1_Info   , &System_ProtectFlag_Info.all );
        COM_Get_Protect_Flag( COM_AD_Data_Info.VACIN_PFC_Power , &PFC_P_OLP2_Info   , &System_ProtectFlag_Info.all );

        /*-------------------------PFC状态下电感过流保护----------------------------------------*/
        COM_Get_Protect_Flag( COM_AD_Data_Info.iInduc_RMS_Val_Fir , &PFC_RMS_OCP1_Info   , &System_ProtectFlag_Info.all );
        COM_Get_Protect_Flag( COM_AD_Data_Info.iInduc_RMS_Val_Fir , &PFC_RMS_OCP2_Info   , &System_ProtectFlag_Info.all );
    }
    else if (COM_Ctr_Info.INV_PFC_Mode_Select == INV_MODE)//INV使能模式：放电状态
    {
        /*-------------------------短路保护-----------------------------------------------------*/
        if(INV_Ctrl_Info.short_Start_Flag == 0 && COM_RUN_STATE == State_Context.state_Value)//短路启机标志清零后启用短路保护
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
        /*-------------------------输出电压过压保护-----------------------------------------------*/
        COM_Get_Protect_Flag( COM_AD_Data_Info.VACOUT_RMS_Val_Fir , &INV_ACOUT_OVP_Info , &System_ProtectFlag_Info.all );

        /*-------------------------输出电压欠压---------------------------------------------------*/
        COM_Get_Protect_Flag( COM_AD_Data_Info.VACOUT_RMS_Val_Fir , &INV_ACOUT_LVP_Info , &System_ProtectFlag_Info.all );
        
        /*-------------------------放电时过载保护-----------------------------------------------*/
        COM_Get_Protect_Flag( COM_AD_Data_Info.VACOUT_ActivePower , &INV_P_OLP1_Info   , &System_ProtectFlag_Info.all );
        COM_Get_Protect_Flag( COM_AD_Data_Info.VACOUT_ActivePower , &INV_P_OLP2_Info   , &System_ProtectFlag_Info.all );
        COM_Get_Protect_Flag( COM_AD_Data_Info.VACOUT_ActivePower , &INV_P_OLP3_Info   , &System_ProtectFlag_Info.all );
        COM_Get_Protect_Flag( COM_AD_Data_Info.VACOUT_ActivePower , &INV_P_OLP4_Info   , &System_ProtectFlag_Info.all );
        
        COM_Get_Protect_Flag( COM_AD_Data_Info.VACOUT_ApparentPower , &INV_S_OLP1_Info   , &System_ProtectFlag_Info.all );
        COM_Get_Protect_Flag( COM_AD_Data_Info.VACOUT_ApparentPower , &INV_S_OLP2_Info   , &System_ProtectFlag_Info.all );
        COM_Get_Protect_Flag( COM_AD_Data_Info.VACOUT_ApparentPower , &INV_S_OLP3_Info   , &System_ProtectFlag_Info.all );
        COM_Get_Protect_Flag( COM_AD_Data_Info.VACOUT_ApparentPower , &INV_S_OLP4_Info   , &System_ProtectFlag_Info.all );
      
        /*-------------------------放电时电感过流保护--------------------------------------------*/
        COM_Get_Protect_Flag( COM_AD_Data_Info.iInduc_RMS_Val_Fir , &INV_RMS_OCP1_Info , &System_ProtectFlag_Info.all );
        COM_Get_Protect_Flag( COM_AD_Data_Info.iInduc_RMS_Val_Fir , &INV_RMS_OCP2_Info , &System_ProtectFlag_Info.all );
        COM_Get_Protect_Flag( COM_AD_Data_Info.iInduc_RMS_Val_Fir , &INV_RMS_OCP3_Info , &System_ProtectFlag_Info.all );
        COM_Get_Protect_Flag( COM_AD_Data_Info.iInduc_RMS_Val_Fir , &INV_RMS_OCP4_Info , &System_ProtectFlag_Info.all );

        COM_Get_Protect_Flag( ABSFUN(ADSample_Info.curLoad_AD_FIR) , &INV_IST_OCP1_Info , &System_ProtectFlag_Info.all );
		
        /*-------------------------母线欠压检测保护-----------------------------------------------*/
        COM_Get_Protect_Flag( COM_AD_Data_Info.vBus_Val_Fir  , &INV_VBus_LVP1_Info  , &System_ProtectFlag_Info.all );
        COM_Get_Protect_Flag( COM_AD_Data_Info.vBus_Val_Fir  , &INV_VBus_LVP2_Info  , &System_ProtectFlag_Info.all );
        COM_Get_Protect_Flag( COM_AD_Data_Info.vBus_Val_Fir  , &INV_VBus_LVP3_Info  , &System_ProtectFlag_Info.all );                
    }

    /*-------------------------母线过压检测保护-----------------------------------------------*/
    COM_Get_Protect_Flag( COM_AD_Data_Info.vBus_Val_Fir , &COM_VBus_OVP1_Info   , &System_ProtectFlag_Info.all );
    COM_Get_Protect_Flag( COM_AD_Data_Info.vBus_Val_Fir , &COM_VBus_OVP2_Info   , &System_ProtectFlag_Info.all );
    COM_Get_Protect_Flag( COM_AD_Data_Info.vBus_Val_Fir , &COM_VBus_OVP3_Info   , &System_ProtectFlag_Info.all );

    /*---------------------辅助电源过压保护---------------------------------------------------------------------*/		
    COM_Get_Protect_Flag( COM_AD_Data_Info.auxPower_Val_Fir , &COM_AuxPower_OVP_Info , &System_ProtectFlag_Info.all );	  		
    /*---------------------辅助电源欠压保护---------------------------------------------------------------------*/		
    COM_Get_Protect_Flag( COM_AD_Data_Info.auxPower_Val_Fir , &COM_AuxPower_LVP_Info , &System_ProtectFlag_Info.all );


    /*-------------------------过温保护-------------------------------------------------------*/
    COM_Get_Protect_Flag( COM_AD_Data_Info.temp_NTC_Val_Fir , &COM_OTP1_Info , &System_ProtectFlag_Info.all );
    COM_Get_Protect_Flag( COM_AD_Data_Info.temp_NTC_Val_Fir , &COM_OTP2_Info , &System_ProtectFlag_Info.all );	

    /*-------------------------VREF保护-------------------------------------------------------*/
    COM_Get_Protect_Flag( COM_AD_Data_Info.vRef_Val_Fir , &COM_Vref_OVP_Info , &System_ProtectFlag_Info.all );
    COM_Get_Protect_Flag( COM_AD_Data_Info.vRef_Val_Fir , &COM_Vref_LVP_Info , &System_ProtectFlag_Info.all );	
    

    SysClockBase_ms.faultCheck_1ms = 0;	    //故障检测1ms 时钟标记清零  				
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
Others     : 解除保护时初始化相关变量
*************************************************/
void Protect_Release(void)
{
}


