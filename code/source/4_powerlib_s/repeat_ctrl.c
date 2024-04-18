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
#include "repeat_ctrl.h"
#include "stdint.h"
/***************************************************************************/

int16_t	RepeatOut[600] = {0};
int16_t	RepeatErr[600] = {0};
/*************************************************
Description: INV_Repeat_Ctrl
Input      : 
Return     : 
Others     : �ظ�����
*************************************************/
//#pragma arm section code = "RAMCODE"
//__attribute__((section("RAMCODE")))
void INV_Repeat_Ctrl(Repeat_Ctrl_Var_t *Repeat_Info)
{
    //N��һ�������ڵĲ�������  ;  K����λ��������
    //N���ظ��ź�һ�����ڶ�Ӧ��������Ҳ�����鳤�ȣ�Kr�Ƿ�ֵ����ϵ����k����λ����������Q���ȶ�����ϵ��
    //д�ɲ�ַ�����ʽΪ��y(m) - 0.95 * y(m - N) = Kr * e(m - N +K)
    /*------------------------------------------------------------------------------------*/		
    /*--------------------------------�ظ�����--------------------------------------------*/
    /*---------------y(m) = Q * y(m - N) + Kr * e(m - N + K)------------------------------*/	
    //y(m - N)���൱���ϸ��ظ������ڵ�y(m)ֵ   
    //y(m) = Q * y(m - N) + Kr * e(m - N + K)�������ظ������ڵ�y(m)ֵ
    
    Repeat_Info->err = (Repeat_Info->ref - Repeat_Info->fdb);   
    //���ֵ����
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

    //Ƕ��ʽ�ظ�����������ֵ  
    Repeat_Info->out += Repeat_Info->err;
    
    //���ֵ����
    if(Repeat_Info->out > Repeat_Info->out_Max)    Repeat_Info->out =  Repeat_Info->out_Max;
    if(Repeat_Info->out < Repeat_Info->out_Min)    Repeat_Info->out =  Repeat_Info->out_Min;    
}


//#pragma arm section
