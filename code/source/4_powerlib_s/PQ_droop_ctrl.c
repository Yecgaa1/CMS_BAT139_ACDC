/*****************************************************************************
Copyright (C) 2021 China Micro Semiconductor Limited Company. All Rights Reserved.
@Description: �жϺ������ܴ���
@Author     : 
@Version    : 0.0.1  
@History    : 
@Attention  ��
*****************************************************************************/

/****************************************************************************/
/*----include files---------------------------------------------------------*/
#include "PQ_droop_ctrl.h"
/***************************************************************************/

//#pragma arm section code = "RAMCODE"
//__attribute__((section("RAMCODE")))

/*************************************************
Description: INV_PQ_Droop_Ctrl(PQ�´�����)
Input      : 
Return     : 
Others     ://W = W0 - m1*P + n1*Q;
            //E = E0 - m2*P - n2*Q;
*************************************************/
void INV_PQ_Droop_Ctrl(INV_PQ_Droop_Ctrl_Var_t *PQ_Droop_Info)
{           
    //���߲�����PQ�´�����
    //PQ�´��������Ӧ��Ƶ�ʺ͵�ѹ��ֵ
    PQ_Droop_Info->omiga_Out = ( PQ_Droop_Info->omigaCoeff_P * PQ_Droop_Info->P_In - \
                                   PQ_Droop_Info->omigaCoeff_Q * PQ_Droop_Info->Q_In )>>12 ; 
    
    PQ_Droop_Info->AMP_Out   = ( PQ_Droop_Info->AMPCoeff_P * PQ_Droop_Info->P_In + \
                                   PQ_Droop_Info->AMPCoeff_Q * PQ_Droop_Info->Q_In ) >> 12; 
    
    //���Ƶ�ѹƵ�ʿɱ䷶Χ
    if(PQ_Droop_Info->omiga_Out < (PQ_Droop_Info->omiga_OutMin))
        PQ_Droop_Info->omiga_Out = PQ_Droop_Info->omiga_OutMin;
    if(PQ_Droop_Info->omiga_Out > PQ_Droop_Info->omiga_OutMax)
        PQ_Droop_Info->omiga_Out = PQ_Droop_Info->omiga_OutMax; 
    
     //���Ƶ�ѹ��ֵ�ɱ䷶Χ       
    if(PQ_Droop_Info->AMP_Out < PQ_Droop_Info->AMP_OutMin)
        PQ_Droop_Info->AMP_Out = PQ_Droop_Info->AMP_OutMin;
    if(PQ_Droop_Info->AMP_Out > PQ_Droop_Info->AMP_OutMax)
        PQ_Droop_Info->AMP_Out = PQ_Droop_Info->AMP_OutMax;      
    
}


//#pragma arm section
