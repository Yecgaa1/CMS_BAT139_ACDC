/*****************************************************************************
Copyright (C) 2021 China Micro Semiconductor Limited Company. All Rights Reserved.
@Description: 
@Author     : 
@Version    : 0.0.1  
@History    : 
@Attention  ��
*****************************************************************************/

#ifndef __PID_CTRL_H
#define __PID_CTRL_H

#include "stdint.h"
#include "sys_define_config.h"

/*-------------------INV��ѹ������---------------------------------------*/
#if INV_PID_LOOP_MODE  ==   INV_DOUBLE_LOOP //��ѹ����˫����� 
    #define  INV_VOL_KP                                 (4700)
    #define  INV_VOL_KI                                 (4700)//˫������ 
    #define  INV_VOL_OUT_MAX                            (32767)    
    #define  INV_VOL_OUT_MIN                            (-32767)
#else
    #define  INV_VOL_KP                                 (3000)
    #define  INV_VOL_KI                                 (3000)   
    #define  INV_VOL_OUT_MAX                            (4000)  
    #define  INV_VOL_OUT_MIN                            (-4000)
#endif
#define     INV_VOL_KD                                  (0)
#define     INV_VOL_ERR_MAX						        (29900)
#define     INV_VOL_ERR_MIN							    (-29900)
#define     INV_VOL_ERR_INTEGRAL_FACT                   (int32_t)(32767.0/INV_VOL_KI+0.5)    
#define     INV_VOL_ERR_INTEGRAL_MAX                    (INV_CUR_UP_LIMIT*INV_VOL_ERR_INTEGRAL_FACT)    
#define     INV_VOL_ERR_INTEGRAL_MIN                    (INV_CUR_DN_LIMIT*INV_VOL_ERR_INTEGRAL_FACT)
#define     INV_VOL_PERIOD_VALUE                        (0)

/*-------------------INV����������---------------------------------------*/
#define     INV_CUR_KP                                  35000//(12500)
#define     INV_CUR_KI                                  20000//(3000)
#define     INV_CUR_KD                                  (0)
#define     INV_CUR_ERR_MAX							    (32767)
#define     INV_CUR_ERR_MIN							    (-32767)
#define     INV_CUR_OUT_MAX                             (4000)    
#define     INV_CUR_OUT_MIN                             (-4000)
#define     INV_CUR_ERR_INTEGRAL_MAX                     (INV_CUR_OUT_MAX*32767/INV_CUR_KI)    
#define     INV_CUR_ERR_INTEGRAL_MIN                    (INV_CUR_OUT_MIN*32767/INV_CUR_KI)
#define     INV_CUR_PERIOD_VALUE                        (0)


/*-------------------DCIM����------------------------------------------*/
#define     INV_DCIM_KP                                 (19900)
#define     INV_DCIM_KI                                 (1500)
#define     INV_DCIM_KD                                 (0)
#define     INV_DCIM_ERR_MAX				            (32767)
#define     INV_DCIM_ERR_MIN						    (-32767)
#define     INV_DCIM_ERR_INTEGRAL_MAX                   (5350)    
#define     INV_DCIM_ERR_INTEGRAL_MIN                   (-5350)
#define     INV_DCIM_OUT_MAX                            (700)    
#define     INV_DCIM_OUT_MIN                            (-700)
#define     INV_DCIM_PERIOD_VALUE                       (0)


/*-------------------INV���ʻ�����---------------------------------------*/
#define     INV_POWER_REF                               (3620)//(W)
#define     INV_POWER_KP                                (4000)
#define     INV_POWER_KI                                (4000)
#define     INV_POWER_KD                                (0)
#define     INV_POWER_ERR_MAX						    (400)
#define     INV_POWER_ERR_MIN						    (-400)
#define     INV_POWER_OUT_MAX                           (0)  
#define     INV_POWER_OUT_MIN                           (-700)
#define     INV_POWER_ERR_INTEGRAL_MAX                  (0)    
#define     INV_POWER_ERR_INTEGRAL_MIN                  (INV_POWER_OUT_MIN*32767/INV_POWER_KI)
#define     INV_POWER_PERIOD_VALUE                      (400)


/*-------------------PFC��ѹ������---------------------------------------*/
//PFC��ѹ������
#define     PFC_VOL_KP                                  (12000)
#define     PFC_VOL_KI                                  (400)
#define     PFC_VOL_KD                                  (0)
#define     PFC_VOL_ERR_MAX						        (500)
#define     PFC_VOL_ERR_MIN							    (-1000)
#define     PFC_VOL_OUT_MAX                             (850)    //����RMS:1A*62.5��14A*62.5 = 875
#define     PFC_VOL_OUT_MIN                             (-8096)
#define     PFC_VOL_ERR_INTEGRAL_MAX                    (PFC_VOL_OUT_MAX*4096/PFC_VOL_KI)    
#define     PFC_VOL_ERR_INTEGRAL_MIN                    (PFC_VOL_OUT_MIN*4096/PFC_VOL_KI)//�����ƿ���ĸ�ߵ�ѹ��
//#define     PFC_VOL_PERIOD_VALUE                        (12)//19.2KHz
#define     PFC_VOL_PERIOD_VALUE                        (20)//24KHz

/*-------------------PFC����������--------------------------------------*/
//PFC����������
#define     PFC_CUR_KP                                  (11900)//(44900)
#define     PFC_CUR_KI                                  (1200)//(2500)
#define     PFC_CUR_KD                                  (0)
#define     PFC_CUR_ERR_MAX							    (4096)
#define     PFC_CUR_ERR_MIN						        (-4096)
#define     PFC_CUR_OUT_MAX                             (4090)    
#define     PFC_CUR_OUT_MIN                             (-4090)
#define     PFC_CUR_ERR_INTEGRAL_MAX                    (PFC_CUR_OUT_MAX*4096/PFC_CUR_KI)   
#define     PFC_CUR_ERR_INTEGRAL_MIN                    (0)
#define     PFC_CUR_PERIOD_VALUE                        (0)

//��PI������
#define PFC_I_INDUC_RMS_VAL                             (int32_t)(0.8 * COM_REAL_IINDUC_RMS_SCAL)//PFC��ѹ��KP�仯ʱ���ж���������е�����Чֵ-��:1A*COM_REAL_IINDUC_RMS_SCAL
#define PFC_VPID_KP_STEP_VAL                            (1)//ÿ�β���ֵ
#define PFC_VPID_KP_SHIFT_VAL                           (0)//������λֵ
#define PFC_VPID_KP_MIN			                        (1400 )//���غ�Kpֵ�����仯����Сֵ
#define PFC_VPID_KP_MAX			                        (12000 )//����ʱKpֵ���
#define PFC_VPID_KP_HOLD_INIT			                (PFC_VPID_KP_MAX << PFC_VPID_KP_SHIFT_VAL)//�Ŵ���ref��ʼֵ

typedef struct { 
                uint16_t     u16kp_Step_Val;  
                uint8_t      u8kp_Shift_Val; 
                uint32_t     u32kp_Hold;    
                uint32_t     u32kp_Min;  
                uint32_t     u32kp_Max;
                uint16_t     u16kp_I_Induc_RMS_Val;//PFC��ѹ��KP�仯ʱ���ж���������е�����Чֵ--��:1A*512
}PFC_VPID_Var_t; //PFC��ѹ����PI����
extern  PFC_VPID_Var_t    PFC_VPID_Info;  

/*-------------------PLL����--------------------------------------*/
#define     PLL_KP                                      (22900)
#define     PLL_KI                                      (200)
#define     PLL_KD                                      (0)
#define     PLL_ERR_MAX							        (4096)
#define     PLL_ERR_MIN						            (-4096)
#define     PLL_OUT_MAX                                 (4090)    
#define     PLL_OUT_MIN                                 (-4090)
#define     PLL_ERR_INTEGRAL_MAX                        (PLL_OUT_MAX*32767/PLL_KI)    
#define     PLL_ERR_INTEGRAL_MIN                        (PLL_OUT_MIN*32767/PLL_KI)
#define     PLL_PERIOD_VALUE                            (0)
/*------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------*/
typedef struct { 
                    int32_t  ref;                  // ����: ��������
                    int16_t  fdb;                  // ����: ��������
                    int32_t  err;                  // ����: ��ǰ���
                    int32_t  err_n;                // ����: �ϴ����
                    int32_t  err_n2;  					   // ����: ���ϴ����
                    int32_t  err_Integral;				 // ����: �����ۼ����	
                    int32_t  kp;                   // ����: ����ϵ��          
                    int32_t  ki;                   // ����: ����ϵ��        
                    int32_t  kd; 	                 // ����: ΢��ϵ��
                    int32_t  err_Max;              // ����: ��ǰ������޷�          
                    int32_t  err_Min;              // ����: ��ǰ������޷� 
                    int32_t  err_Integral_Max;           // ����: ����������޷�
                    int32_t  err_Integral_Min;           // ����: ����������޷�
                    int32_t  out_Max;          // ����: PID������޷�          
                    int32_t  out_Min;     	   // ����: PID������޷� 
                    int32_t  up;                   // ����: �������
                    int32_t  ui;                   // ����: �������	
                    int32_t  ud;                   // ����: �������		
                    int32_t  out;                  // ����: PID���             
                    uint16_t  ctrl_Period_Cnt;
                    uint16_t  ctrl_Period_Val;    // ����: ��������   
                   void  (*Calc)();	/* Pointer to calculation function */                          
			   } PID_Ctrl_Var_t;

              
/*-----------------------------------------------------------------------------
Default initalizer for the PARK object.
-----------------------------------------------------------------------------*/
#define INV_VOL_LOOP_DEFAULTS { 0,                              \
                                0,                              \
                                0,                              \
                                0,                              \
                                0,                              \
                                0,                              \
                                INV_VOL_KP,                     \
                                INV_VOL_KI,                     \
                                INV_VOL_KD,                     \
                                INV_VOL_ERR_MAX,                \
                                INV_VOL_ERR_MIN,                \
                                INV_VOL_ERR_INTEGRAL_MAX,       \
                                INV_VOL_ERR_INTEGRAL_MIN,       \
                                INV_VOL_OUT_MAX,                \
                                INV_VOL_OUT_MIN,                \
                                0,                              \
                                0,                              \
                                0,                              \
                                0,                              \
                                0,                              \
                                INV_VOL_PERIOD_VALUE,           \
                                (void (*)( unsigned int ))PID_Ctrl}

						
                                                             
#define INV_CUR_LOOP_DEFAULTS { 0,                              \
                                0,                              \
                                0,                              \
                                0,                              \
                                0,                              \
                                0,                              \
                                INV_CUR_KP,                     \
                                INV_CUR_KI,                     \
                                INV_CUR_KD,                     \
                                INV_CUR_ERR_MAX,                \
                                INV_CUR_ERR_MIN,                \
                                INV_CUR_ERR_INTEGRAL_MAX,       \
                                INV_CUR_ERR_INTEGRAL_MIN,       \
                                INV_CUR_OUT_MAX,                \
                                INV_CUR_OUT_MIN,                \
                                0,                              \
                                0,                              \
                                0,                              \
                                0,                              \
                                0,                              \
                                INV_CUR_PERIOD_VALUE,           \
                                (void (*)( unsigned int ))PID_Ctrl}
                              
#define INV_DCIM_LOOP_DEFAULTS { 0,                              \
                                0,                              \
                                0,                              \
                                0,                              \
                                0,                              \
                                0,                              \
                                INV_DCIM_KP,                     \
                                INV_DCIM_KI,                     \
                                INV_DCIM_KD,                     \
                                INV_DCIM_ERR_MAX,                \
                                INV_DCIM_ERR_MIN,                \
                                INV_DCIM_ERR_INTEGRAL_MAX,       \
                                INV_DCIM_ERR_INTEGRAL_MIN,       \
                                INV_DCIM_OUT_MAX,                \
                                INV_DCIM_OUT_MIN,                \
                                0,                              \
                                0,                              \
                                0,                              \
                                0,                              \
                                0,                              \
                                INV_DCIM_PERIOD_VALUE,           \
                                (void (*)( unsigned int ))PID_Ctrl}

#define INV_POWER_LOOP_DEFAULTS {INV_POWER_REF,                 \
                                0,                              \
                                0,                              \
                                0,                              \
                                0,                              \
                                0,                              \
                                INV_POWER_KP,                   \
                                INV_POWER_KI,                   \
                                INV_POWER_KD,                   \
                                INV_POWER_ERR_MAX,              \
                                INV_POWER_ERR_MIN,              \
                                INV_POWER_ERR_INTEGRAL_MAX,     \
                                INV_POWER_ERR_INTEGRAL_MIN,     \
                                INV_POWER_OUT_MAX,              \
                                INV_POWER_OUT_MIN,              \
                                0,                              \
                                0,                              \
                                0,                              \
                                0,                              \
                                0,                              \
                                INV_POWER_PERIOD_VALUE,         \
                                (void (*)( unsigned int ))PID_Ctrl}
                                                                

#define PFC_VOL_LOOP_DEFAULTS {0,                            \
                                0,                            \
                                0,                            \
                                0,                            \
                                0,                            \
                                0,                            \
                                PFC_VOL_KP,         \
                                PFC_VOL_KI,         \
                                PFC_VOL_KD,         \
                                PFC_VOL_ERR_MAX,     \
                                PFC_VOL_ERR_MIN,     \
                                PFC_VOL_ERR_INTEGRAL_MAX,  \
                                PFC_VOL_ERR_INTEGRAL_MIN,  \
                                PFC_VOL_OUT_MAX,     \
                                PFC_VOL_OUT_MIN,     \
                                0,                            \
                                0,                            \
                                0,                            \
                                0,                            \
                                0,                            \
                                PFC_VOL_PERIOD_VALUE,    \
                                (void (*)( unsigned int ))PFC_PID_Ctrl}

															
#define PFC_CUR_LOOP_DEFAULTS {0,                            \
                                0,                            \
                                0,                            \
                                0,                            \
                                0,                            \
                                0,                            \
                                PFC_CUR_KP,         \
                                PFC_CUR_KI,         \
                                PFC_CUR_KD,         \
                                PFC_CUR_ERR_MAX,     \
                                PFC_CUR_ERR_MIN,     \
                                PFC_CUR_ERR_INTEGRAL_MAX,  \
                                PFC_CUR_ERR_INTEGRAL_MIN,  \
                                PFC_CUR_OUT_MAX,     \
                                PFC_CUR_OUT_MIN,     \
                                0,                            \
                                0,                            \
                                0,                            \
                                0,                            \
                                0,                            \
                                PFC_CUR_PERIOD_VALUE,    \
                                (void (*)( unsigned int ))PFC_PID_Ctrl}
                              
  
#define PLL_LOOP_DEFAULTS {0,                            \
                                0,                            \
                                0,                            \
                                0,                            \
                                0,                            \
                                0,                            \
                                PLL_KP,         \
                                PLL_KI,         \
                                PLL_KD,         \
                                PLL_ERR_MAX,     \
                                PLL_ERR_MIN,     \
                                PLL_ERR_INTEGRAL_MAX,  \
                                PLL_ERR_INTEGRAL_MIN,  \
                                PLL_OUT_MAX,     \
                                PLL_OUT_MIN,     \
                                0,                            \
                                0,                            \
                                0,                            \
                                0,                            \
                                0,                            \
                                PLL_PERIOD_VALUE,    \
                                (void (*)( unsigned int ))PID_Ctrl}

void PID_Ctrl(PID_Ctrl_Var_t *PID_Info);
void PFC_PID_Ctrl(PID_Ctrl_Var_t *PID_Info);
                                
extern PID_Ctrl_Var_t INV_PID_Vol;
extern PID_Ctrl_Var_t INV_PID_Cur; 
extern PID_Ctrl_Var_t INV_PID_DCIM;                                 
extern PID_Ctrl_Var_t INV_PID_Power;
extern PID_Ctrl_Var_t PFC_PID_Vol;
extern PID_Ctrl_Var_t PFC_PID_Cur; 
extern PID_Ctrl_Var_t PLL_PID;                                                                 

                                                                
                            
#endif 
/*-------------------------------------------------------------------------------------
 *  No more.
 *------------------------------------------------------------------------------------*/
