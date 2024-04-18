/*****************************************************************************
Copyright (C) 2021 China Micro Semiconductor Limited Company. All Rights Reserved.
@Description: 
@Author     : 
@Version    : 0.0.1  
@History    : 
@Attention  ：
*****************************************************************************/

#ifndef __REPEAT_CTRL_H
#define __REPEAT_CTRL_H

                                  
//#define     INV_REPEAT_LAG_POINT                    (8)//相位补偿数（电感2.1mH）                            
#define     INV_REPEAT_LAG_POINT                    (6)//相位补偿数（电感1.1mH）
#define     INV_REPEAT_QR_COEFF                     (3940) //4096 * 0.95稳定补偿系数
#define     INV_REPEAT_KR_COEFF                     (1638) //4096 * 0.4 幅值补偿系数
#define     INV_REPEAT_ERR_MAX					    (10)
#define     INV_REPEAT_ERR_MIN					    (-10)
#define     INV_REPEAT_OUT_MAX                      (9000)    
#define     INV_REPEAT_OUT_MIN                      (-9000)
                              
typedef struct {
    int     ref;
    int     fdb;
    int     err;  
    int     out;
    int     err_Max;
    int     err_Min;
    int     out_Max;
    int     out_Min;
    int     lag_Point;//滞后补偿点数    
    int     Qr_Coeff;//稳定补偿系数
    int     Kr_Coeff;//幅值补偿系数     
    int     periodDot_Val;//周期设定点数
    int     periodDot_Cnt;//周期计数值  
    void     (*Calc)();     // Pointer to calculation function
}Repeat_Ctrl_Var_t;
#define REPEAT_CTRL_DEFAULTS {  0,\
                                0,\
                                0,\
                                0,\
                                INV_REPEAT_ERR_MAX,\
                                INV_REPEAT_ERR_MIN,\
                                INV_REPEAT_OUT_MAX,\
                                INV_REPEAT_OUT_MIN,\
                                INV_REPEAT_LAG_POINT,\
                                INV_REPEAT_QR_COEFF,\
                                INV_REPEAT_KR_COEFF,\
                                0,\
                                0,\
                        (void *)0 }
void INV_Repeat_Ctrl(Repeat_Ctrl_Var_t *Repeat_Info);
extern Repeat_Ctrl_Var_t    INV_Repeat_Info;

#endif 
/*-------------------------------------------------------------------------------------
 *  No more.
 *------------------------------------------------------------------------------------*/
