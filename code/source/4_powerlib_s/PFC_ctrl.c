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

#include "sys_mcu_header.h"
#include "sys_define_param.h"
#include "sys_state_machine.h"
/***************************************************************************/

/*************************************************
Description: PFC_VBus_RefCalc
Input      : 
Return     : 
Others     : 母线电压参考值逻辑处理
*************************************************/
#define PFC_Reverse_BufferValue  15 //设定计数值
int16_t PFC_Reverse_BufferCount = 0;//过零时防止二次误动作计数值
int32_t PFC_Zero_DutySS_Count = 0;//过零反相时，占空比缓启动开关周期计数变量
uint8_t PFC_CHG_Out_StepVal = 20;
uint8_t PFC_Reverse_Flag = 0; //过零点换向标志

void  PFC_VBus_RefCalc(void)
{
    #if 1
        //缓慢变化参考值
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
            PFC_Ctrl_Info.DCDC_CHG_Out = UARTx_DC_Info.vBus_SetVal;////DCDC传输的控制电压值(升压侧通信传输)
        }    
    #else 
        PFC_Ctrl_Info.DCDC_CHG_Out = 1600;   
    #endif

    //Vbus的ref最大最小值限制
    if(PFC_Ctrl_Info.DCDC_CHG_Out > PFC_Ctrl_Info.vBUS_Ref_Max )
    {
        PFC_Ctrl_Info.DCDC_CHG_Out = PFC_Ctrl_Info.vBUS_Ref_Max;
    }            
    
    if(PFC_Ctrl_Info.DCDC_CHG_Out < PFC_Ctrl_Info.vBUS_Ref_Min)
    {
        PFC_Ctrl_Info.DCDC_CHG_Out = PFC_Ctrl_Info.vBUS_Ref_Min;
    } 
    PFC_Ref_Info.u32SS_Set_Val = PFC_Ctrl_Info.DCDC_CHG_Out<<PFC_Ref_Info.u8SS_Shift_Val;//Vbus的参考给定值放大后的值

    
    //注意：此逻辑不可随意移动位置，
    //每次过零时Reverse_count计数到设定值开始启用下次过零，防止过零附近误动作；解决过零点附近第三个波反相问题        
    if(PFC_Reverse_BufferCount <= PFC_Reverse_BufferValue)
        PFC_Reverse_BufferCount  ++;//解决过零点附近第三个波反相问题
    
    //过零点标记处理
    if( - ADSample_Info.PFC_AC_Vol_AD_FIR >= 0 && PFC_Reverse_Flag == 0&&(PFC_Reverse_BufferCount >= PFC_Reverse_BufferValue))               
    {
        PFC_Reverse_Flag = 1;  //过零点反相标记位
        PFC_Reverse_BufferCount = 0;//过零时清误动作变量Reverse_count计数值
        PFC_Zero_DutySS_Count = 0;//过零时将缓启动计数值清零
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
Others     : PFC控制
*************************************************/
int32_t PFC_vBus_Hold,PFC_vBus_Fir = 0;
void PFC_Ctrl(void)
{
    int32_t Duty_Out = 0;      
    //PFC电压环变PI
    if(PFC_Ctrl_Info.RMS_InductorCur > PFC_VPID_Info.u16kp_I_Induc_RMS_Val)//PFC电压环KP变化时的判定条件：电感电流有效值
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
    /*-------------------电压控制环路缓启动处理--------------------------------*/    
    
    if(PFC_Ref_Info.u32SS_vBus_Hold < PFC_Ref_Info.u32SS_Set_Val)
    {      
        PFC_Ref_Info.u32SS_vBus_Hold +=PFC_Ref_Info.u16SS_Step_Val;
    }
    else
    {
        PFC_Ref_Info.u32SS_vBus_Hold = PFC_Ref_Info.u32SS_Set_Val;
    }
    
    /***************************************************************************/
    /*-------------------电压环参考给定----------------------------------------*/
    PFC_PID_Vol.ref       = PFC_Ref_Info.u32SS_vBus_Hold >> PFC_Ref_Info.u8SS_Shift_Val;        
    
    /***************************************************************************/
    /*-------------------电压环控制--------------------------------------------*/ 
    //根据母线判断是否突加载时，清除积分累积误差
    PFC_PID_Vol.fdb       = PFC_Ctrl_Info.AD_VBus;
    PFC_PID_Vol.Calc(&PFC_PID_Vol);

    /***************************************************************************/
    /*-------------------电流环控制--------------------------------------------*/
    PFC_PID_Cur.ref     = (User_Divider(PFC_PID_Vol.out * PFC_Ctrl_Info.AD_VolPeak, PFC_Ctrl_Info.AC_VolBase))>>2 ;
//    PFC_PID_Cur.ref     = PFC_PID_Vol.out * PFC_Ctrl_Info.AD_VolPeak>>10 ;

    PFC_PID_Cur.fdb     = PFC_Ctrl_Info.AD_InducCurPeak;          
    PFC_PID_Cur.Calc(&PFC_PID_Cur);
     
    
    //占空比前馈
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
    /*-------------------占空比更新--------------------------------------------*/        
    PFC_Ctrl_Info.PWM_Duty   =  (int32_t)(( Duty_Out * PFC_Ctrl_Info.PWM_Period ) >> 12)  ;//更新占空比                                  
 
    //过零点附近缓占空比值处理
    if(PFC_Ctrl_Info.active_Power < 970)
        PFC_Ctrl_Info.Zero_DutySS = PFC_Ctrl_Info.PWM_Period*10 >> 6;
    else  if(PFC_Ctrl_Info.active_Power > 1000)
        PFC_Ctrl_Info.Zero_DutySS = PFC_Ctrl_Info.PWM_Period*10 >> 7; 
    
    //占空比缓启动
    if(PFC_Zero_DutySS_Count < 15 && ((PFC_Ctrl_Info.Zero_DutySS + PFC_Zero_DutySS_Count * PFC_Ctrl_Info.Zero_DutySS) <PFC_Ctrl_Info.PWM_Duty ))//计数校正标志
    {
        PFC_Zero_DutySS_Count ++;
        PFC_Ctrl_Info.PWM_Duty = PFC_Ctrl_Info.Zero_DutySS + PFC_Zero_DutySS_Count * PFC_Ctrl_Info.Zero_DutySS;            
    }  
        
}

