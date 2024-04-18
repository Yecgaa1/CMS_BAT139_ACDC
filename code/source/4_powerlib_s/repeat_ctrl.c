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
#include "repeat_ctrl.h"
#include "stdint.h"
/***************************************************************************/

int16_t	RepeatOut[600] = {0};
int16_t	RepeatErr[600] = {0};
/*************************************************
Description: INV_Repeat_Ctrl
Input      : 
Return     : 
Others     : 重复控制
*************************************************/
//#pragma arm section code = "RAMCODE"
//__attribute__((section("RAMCODE")))
void INV_Repeat_Ctrl(Repeat_Ctrl_Var_t *Repeat_Info)
{
    //N：一个周期内的采样次数  ;  K：相位补偿次数
    //N是重复信号一个周期对应的拍数，也是数组长度，Kr是幅值补偿系数，k是相位补偿拍数，Q是稳定补偿系数
    //写成差分方程形式为：y(m) - 0.95 * y(m - N) = Kr * e(m - N +K)
    /*------------------------------------------------------------------------------------*/		
    /*--------------------------------重复控制--------------------------------------------*/
    /*---------------y(m) = Q * y(m - N) + Kr * e(m - N + K)------------------------------*/	
    //y(m - N)：相当于上个重复周期内的y(m)值   
    //y(m) = Q * y(m - N) + Kr * e(m - N + K)：本次重复周期内的y(m)值
    
    Repeat_Info->err = (Repeat_Info->ref - Repeat_Info->fdb);   
    //误差值限制
//    if(Repeat_Info->err > Repeat_Info->err_Max)    Repeat_Info->err =  Repeat_Info->err_Max;
//    if(Repeat_Info->err < Repeat_Info->err_Min)    Repeat_Info->err =  Repeat_Info->err_Min;
    

    if(Repeat_Info->periodDot_Cnt < (Repeat_Info->periodDot_Val - Repeat_Info->lag_Point))
    {
        Repeat_Info->out = (Repeat_Info->Qr_Coeff * RepeatOut[Repeat_Info->periodDot_Cnt] + \
                            Repeat_Info->Kr_Coeff * RepeatErr[Repeat_Info->lag_Point + Repeat_Info->periodDot_Cnt]) >> 12;
    }
    else
    {
        Repeat_Info->out = (Repeat_Info->Qr_Coeff * RepeatOut[Repeat_Info->periodDot_Cnt] + \
                            Repeat_Info->Kr_Coeff * RepeatErr[Repeat_Info->periodDot_Cnt - \
                            Repeat_Info->periodDot_Val + Repeat_Info->lag_Point]) >> 12;
    }
    RepeatOut[Repeat_Info->periodDot_Cnt] = Repeat_Info->out;
    RepeatErr[Repeat_Info->periodDot_Cnt] = Repeat_Info->err;

    //嵌入式重复控制输出误差值  
    Repeat_Info->out += Repeat_Info->err;
    
    //输出值限制
    if(Repeat_Info->out > Repeat_Info->out_Max)    Repeat_Info->out =  Repeat_Info->out_Max;
    if(Repeat_Info->out < Repeat_Info->out_Min)    Repeat_Info->out =  Repeat_Info->out_Min;    
}


//#pragma arm section
