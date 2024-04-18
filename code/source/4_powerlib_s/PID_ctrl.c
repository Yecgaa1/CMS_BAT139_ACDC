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
#include "PID_ctrl.h"
/***************************************************************************/
/*************************************************
Description: PFC_PID_Ctrl
Input      : 
Return     : 
Others     : 位置式PID控制逻辑
*************************************************/
void PFC_PID_Ctrl(PID_Ctrl_Var_t *PID_Info)
{   
    PID_Info->err = PID_Info->ref - PID_Info->fdb;
    
    PID_Info->ctrl_Period_Cnt ++;//控制周期累加 
    if(PID_Info->ctrl_Period_Cnt >= PID_Info->ctrl_Period_Val)    
    {
        PID_Info->ctrl_Period_Cnt = 0;//控制周期计数值清0       

        //误差限制
//        if ( PID_Info->err > PID_Info->err_Max ) PID_Info->err = PID_Info->err_Max;
//        if ( PID_Info->err < PID_Info->err_Min ) PID_Info->err = PID_Info->err_Min;

        //积分误差累计
        PID_Info->err_Integral += PID_Info->err;    
        //积分限幅
        if(PID_Info->err_Integral >= PID_Info->err_Integral_Max)
        {
            PID_Info->err_Integral = PID_Info->err_Integral_Max;
        }
        if(PID_Info->err_Integral < PID_Info->err_Integral_Min)
        {
            PID_Info->err_Integral = PID_Info->err_Integral_Min;
        }

        PID_Info->up = (PID_Info->kp  * PID_Info->err) ; //比例部分作用
        PID_Info->ui = (PID_Info->ki  * PID_Info->err_Integral) ;//积分部分作用  
        /*------------------------------------------------------------------------------------*/
        PID_Info->out = (PID_Info->up + PID_Info->ui )>>12;
        

        /*----------------------------------PID输出限幅--------------------------------------------*/
        if ( PID_Info->out > PID_Info->out_Max ) PID_Info->out = PID_Info->out_Max;
        if ( PID_Info->out < PID_Info->out_Min ) PID_Info->out = PID_Info->out_Min;
    }    
}

/*************************************************
Description: PID_Ctrl
Input      : 
Return     : 
Others     : 位置式PID控制逻辑
*************************************************/
void PID_Ctrl(PID_Ctrl_Var_t *PID_Info)
{   
    PID_Info->ctrl_Period_Cnt ++;//控制周期累加 
    if(PID_Info->ctrl_Period_Cnt >= PID_Info->ctrl_Period_Val)    
    {
        PID_Info->ctrl_Period_Cnt = 0;//控制周期计数值清0
        
//        PID_Info->err_n = PID_Info->err; 
        PID_Info->err = PID_Info->ref - PID_Info->fdb;

//        //误差限制
//        if ( PID_Info->err > PID_Info->err_Max ) PID_Info->err = PID_Info->err_Max;
//        if ( PID_Info->err < PID_Info->err_Min ) PID_Info->err = PID_Info->err_Min;

        //积分误差累计
        PID_Info->err_Integral += PID_Info->err;    
        //积分限幅
        if(PID_Info->err_Integral >= PID_Info->err_Integral_Max)
        {
            PID_Info->err_Integral = PID_Info->err_Integral_Max;
        }
        if(PID_Info->err_Integral < PID_Info->err_Integral_Min)
        {
            PID_Info->err_Integral = PID_Info->err_Integral_Min;
        }

        PID_Info->up = (PID_Info->kp  * PID_Info->err) ; //比例部分作用
        PID_Info->ui = (PID_Info->ki  * PID_Info->err_Integral) ;//积分部分作用  
//        PID_Info->ud =  PID_Info->kd  * (PID_Info->err - PID_Info->err_n );//微分部分作用
        /*------------------------------------------------------------------------------------*/
        PID_Info->out = (PID_Info->up + PID_Info->ui )>>15;
        

        /*----------------------------------PID输出限幅--------------------------------------------*/
        if ( PID_Info->out > PID_Info->out_Max ) PID_Info->out = PID_Info->out_Max;
        if ( PID_Info->out < PID_Info->out_Min ) PID_Info->out = PID_Info->out_Min;
    }    
}

/*************************************************
Description: PID_Increase_Ctrl
Input      : 
Return     : 
Others     : 增量式PID控制逻辑
*************************************************/
void PID_Increase_Ctrl(PID_Ctrl_Var_t *PID_Info)
{    
    PID_Info->ctrl_Period_Cnt ++;//控制周期累加 
    if(PID_Info->ctrl_Period_Cnt >= PID_Info->ctrl_Period_Val)    
    {
        PID_Info->ctrl_Period_Cnt = 0;//控制周期计数值清0
            
        PID_Info->err_n2    = PID_Info->err_n;
        PID_Info->err_n     = PID_Info->err; 
        PID_Info->err       = PID_Info->ref - PID_Info->fdb;

        //误差限制
        if ( PID_Info->err > PID_Info->err_Max ) PID_Info->err = PID_Info->err_Max;
        if ( PID_Info->err < PID_Info->err_Min ) PID_Info->err = PID_Info->err_Min;

        PID_Info->up = PID_Info->kp *(PID_Info->err - PID_Info->err_n) >> 15; //比例部分作用
        PID_Info->ui = PID_Info->ki * PID_Info->err >> 15;//积分部分作用  
        PID_Info->ud = PID_Info->kd *(PID_Info->err - 2* PID_Info->err_n + PID_Info->err_n2)>>15;//微分部分作用
        /*------------------------------------------------------------------------------------*/
        PID_Info->out = (PID_Info->up + PID_Info->ui + PID_Info->ud);
        
        /*----------------------------------PID输出限幅--------------------------------------------*/
        if ( PID_Info->out > PID_Info->out_Max ) PID_Info->out = PID_Info->out_Max;
        if ( PID_Info->out < PID_Info->out_Min ) PID_Info->out = PID_Info->out_Min; 
    }        
}



//#pragma arm section
