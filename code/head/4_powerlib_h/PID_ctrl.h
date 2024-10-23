/*****************************************************************************
Copyright (C) 2021 China Micro Semiconductor Limited Company. All Rights Reserved.
@Description: 
@Author     : 
@Version    : 0.0.1  
@History    : 
@Attention  ：
*****************************************************************************/

#ifndef __PID_CTRL_H
#define __PID_CTRL_H

#include "stdint.h"
#include "sys_define_config.h"

/*-------------------INV电压环参数---------------------------------------*/
#if INV_PID_LOOP_MODE  ==   INV_DOUBLE_LOOP //电压电流双环输出 
    #define  INV_VOL_KP                                 (4700)
    #define  INV_VOL_KI                                 (4700)//双环参数 
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

/*-------------------INV电流环参数---------------------------------------*/
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


/*-------------------DCIM参数------------------------------------------*/
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


/*-------------------INV功率环参数---------------------------------------*/
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


/*-------------------PFC电压环参数---------------------------------------*/
//PFC电压环参数
#define     PFC_VOL_KP                                  (12000)
#define     PFC_VOL_KI                                  (400)
#define     PFC_VOL_KD                                  (0)
#define     PFC_VOL_ERR_MAX						        (500)
#define     PFC_VOL_ERR_MIN							    (-1000)
#define     PFC_VOL_OUT_MAX                             (850)    //限流RMS:1A*62.5；14A*62.5 = 875
#define     PFC_VOL_OUT_MIN                             (-8096)
#define     PFC_VOL_ERR_INTEGRAL_MAX                    (PFC_VOL_OUT_MAX*4096/PFC_VOL_KI)    
#define     PFC_VOL_ERR_INTEGRAL_MIN                    (PFC_VOL_OUT_MIN*4096/PFC_VOL_KI)//可限制空载母线电压振荡
//#define     PFC_VOL_PERIOD_VALUE                        (12)//19.2KHz
#define     PFC_VOL_PERIOD_VALUE                        (20)//24KHz

/*-------------------PFC电流环参数--------------------------------------*/
//PFC电流环参数
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

//变PI缓启动
#define PFC_I_INDUC_RMS_VAL                             (int32_t)(0.8 * COM_REAL_IINDUC_RMS_SCAL)//PFC电压环KP变化时的判定条件：电感电流有效值-如:1A*COM_REAL_IINDUC_RMS_SCAL
#define PFC_VPID_KP_STEP_VAL                            (1)//每次步进值
#define PFC_VPID_KP_SHIFT_VAL                           (0)//缩放移位值
#define PFC_VPID_KP_MIN			                        (1400 )//带载后Kp值缓慢变化至最小值
#define PFC_VPID_KP_MAX			                        (12000 )//空载时Kp值最大
#define PFC_VPID_KP_HOLD_INIT			                (PFC_VPID_KP_MAX << PFC_VPID_KP_SHIFT_VAL)//放大后的ref初始值

typedef struct { 
                uint16_t     u16kp_Step_Val;  
                uint8_t      u8kp_Shift_Val; 
                uint32_t     u32kp_Hold;    
                uint32_t     u32kp_Min;  
                uint32_t     u32kp_Max;
                uint16_t     u16kp_I_Induc_RMS_Val;//PFC电压环KP变化时的判定条件：电感电流有效值--如:1A*512
}PFC_VPID_Var_t; //PFC电压环变PI缓启
extern  PFC_VPID_Var_t    PFC_VPID_Info;  

/*-------------------PLL参数--------------------------------------*/
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
                    int32_t  ref;                  // 输入: 给定输入
                    int16_t  fdb;                  // 输入: 反馈输入
                    int32_t  err;                  // 变量: 当前误差
                    int32_t  err_n;                // 变量: 上次误差
                    int32_t  err_n2;  					   // 变量: 上上次误差
                    int32_t  err_Integral;				 // 变量: 积分累计误差	
                    int32_t  kp;                   // 参数: 比例系数          
                    int32_t  ki;                   // 参数: 积分系数        
                    int32_t  kd; 	                 // 参数: 微分系数
                    int32_t  err_Max;              // 参数: 当前误差上限幅          
                    int32_t  err_Min;              // 参数: 当前误差下限幅 
                    int32_t  err_Integral_Max;           // 参数: 积分输出上限幅
                    int32_t  err_Integral_Min;           // 参数: 积分输出下限幅
                    int32_t  out_Max;          // 参数: PID输出上限幅          
                    int32_t  out_Min;     	   // 参数: PID输出下限幅 
                    int32_t  up;                   // 变量: 比例输出
                    int32_t  ui;                   // 变量: 积分输出	
                    int32_t  ud;                   // 变量: 积分输出		
                    int32_t  out;                  // 变量: PID输出             
                    uint16_t  ctrl_Period_Cnt;
                    uint16_t  ctrl_Period_Val;    // 参数: 控制周期   
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
