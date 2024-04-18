/*****************************************************************************
Copyright (C) 2021 China Micro Semiconductor Limited Company. All Rights Reserved.
@Description: 
@Author     : 
@Version    : 0.0.1  
@History    : 
@Attention  ：
*****************************************************************************/

#ifndef __PLL_CTRL_H
#define __PLL_CTRL_H

#include "stdint.h"
#include "sys_define_config.h"


typedef struct
{
    int32_t	i32Theta_delta ;
    int16_t i16Dot_Cnt;
    int16_t i16sin[580];
    int16_t i16cos[580];
    int16_t i16Valpha;
    int16_t i16Vbeta;
    int32_t	i32Theta ;
    int32_t i32SinTheta;    //计算的正弦值
    int32_t i32CosTheta;    //计算的余弦值  
    int16_t i16PI_Kp ;
    int16_t i16PI_Ki ;    
    int16_t i16PI_Error ;
    int32_t i32PI_ErrIntegral;
    int32_t i32PI_ErrIntegralMax;
    int32_t i32PI_ErrIntegralMin;
    int16_t i16PI_OutMax;
    int16_t i16PI_OutMin;
    int16_t i16PI_Out;
}PLL_Ctrl_Var_t;
 
extern PLL_Ctrl_Var_t PLL_Ctrl_Info_V_ACIN;
extern PLL_Ctrl_Var_t PLL_Ctrl_Info_V_ACOUT;;
void PLL_Ctrl(PLL_Ctrl_Var_t *PLL_Info);
void PLL_Ctrl_Param(PLL_Ctrl_Var_t *PLL_Info);                                
                                                           
typedef struct
{
    int32_t	i32Theta ;
    int32_t i32SinTheta;    //计算的正弦值
    int32_t i32CosTheta;    //计算的余弦值  

}PFC_3Theta_Var_t;
extern PFC_3Theta_Var_t PFC_3Theta_Info;                                                                
                            
#endif 
/*-------------------------------------------------------------------------------------
 *  No more.
 *------------------------------------------------------------------------------------*/
