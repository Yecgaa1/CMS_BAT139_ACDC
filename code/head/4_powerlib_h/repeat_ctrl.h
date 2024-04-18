/*****************************************************************************
Copyright (C) 2021 China Micro Semiconductor Limited Company. All Rights Reserved.
@Description: 
@Author     : 
@Version    : 0.0.1  
@History    : 
@Attention  ��
*****************************************************************************/

#ifndef __REPEAT_CTRL_H
#define __REPEAT_CTRL_H

                                  
//#define     INV_REPEAT_LAG_POINT                    (8)//��λ�����������2.1mH��                            
#define     INV_REPEAT_LAG_POINT                    (6)//��λ�����������1.1mH��
#define     INV_REPEAT_QR_COEFF                     (3940) //4096 * 0.95�ȶ�����ϵ��
#define     INV_REPEAT_KR_COEFF                     (1638) //4096 * 0.4 ��ֵ����ϵ��
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
    int     lag_Point;//�ͺ󲹳�����    
    int     Qr_Coeff;//�ȶ�����ϵ��
    int     Kr_Coeff;//��ֵ����ϵ��     
    int     periodDot_Val;//�����趨����
    int     periodDot_Cnt;//���ڼ���ֵ  
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
