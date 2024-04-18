
/*****************************************************************************
Copyright (C) 2021 China Micro Semiconductor Limited Company. All Rights Reserved.
@Description: 系统配置
@Author     : 
@Version    : 0.0.1  
@History    : 
@Attention  ：
*****************************************************************************/

#ifndef __SYS_DEFINE_CONFIG_H
#define	__SYS_DEFINE_CONFIG_H


#define COM_MCU_CLK 	                            (64000000L)//MCU主频
/***************************************************************************/
/*-------------------INV模式时PWM开关周期设置------------------------------*/
/***************************************************************************/
#define INV_PWM_FREQ	                            (21000L)// PWM frequency[Hz]
#define INV_DEADTIME			                    ((int32_t)((0 * COM_MCU_CLK)) / 1000000L)
#if 1
    //中心互补对称周期值
    #define INV_PWM_PERIOD	                        ((int32_t)(COM_MCU_CLK  / (INV_PWM_FREQ * 2)) + INV_DEADTIME - 2)   //1/fk* (m+2–p)* 2
    #define INV_PWM_PERIOD_HALF 	                (INV_PWM_PERIOD >> 1)
#else
    //锯齿波周期值
    #define INV_PWM_PERIOD	                        ((int32_t)(COM_MCU_CLK  / (INV_PWM_FREQ )) -1)  //1/fk * (m+1) = period = 1/Carry_FREQ   =>  m + 1 = fk/Carry_FREQ  锯齿波   
    #define INV_PWM_PERIOD_HALF 	                (INV_PWM_PERIOD >> 1)
#endif
/***************************************************************************/

/***************************************************************************/
/*-------------------PFC模式时PWM开关周期设置------------------------------*/
/***************************************************************************/
#define PFC_PWM_FREQ	                            (24000)// PWM frequency[Hz]
#define PFC_DEADTIME			                    ((int32_t)((0 * COM_MCU_CLK)) / 1000000L)
#if 1
    //中心互补对称周期值
    #define PFC_PWM_PERIOD	                        ((int32_t)(COM_MCU_CLK  / (PFC_PWM_FREQ * 2)) + PFC_DEADTIME - 2)   //1/fk* (m+2–p)* 2
    #define PFC_PWM_PERIOD_HALF 	                (PFC_PWM_PERIOD >> 1)
#else
    //锯齿波周期值
    #define PFC_PWM_PERIOD	                        ((int32_t)(COM_MCU_CLK  / (PFC_PWM_FREQ )) -1)  //1/fk * (m+1) = period = 1/Carry_FREQ   =>  m + 1 = fk/Carry_FREQ  锯齿波   
    #define PFC_PWM_PERIOD_HALF 	                (PFC_PWM_PERIOD >> 1)
#endif
/***************************************************************************/
/*-------------------PFC独立工作或与DC侧联动工作---------------------------*/
#define PFC_SINGLE_DISABLE                          (0)//PFC与前级DC侧联动，电压环路ref值由DC侧给定(正常充电时)
#define PFC_SINGLE_ENABLE                           (1)//PFC独立工作，给固定的电压环路ref值(注意：需断开主变压器)
#define PFC_SINGLE_CTRL_SELECT                      (PFC_SINGLE_DISABLE) 

/*-------------------INV模式或者PFC模式------------------------------------*/
#define FREE_MODE                                   (0)
#define INV_MODE                                    (1)
#define PFC_MODE                                    (2) 

/***************************************************************************/
/*-------------------工作模式设置------------------------------------------*/
/***************************************************************************/
#define  NORMAL_MODE                                (0)//闭环工作：启用保护功能
#define  DEBUG_MODE                                 (1)//开环工作：关闭保护功能
#define  TEST_MODE                                  (2)
#define  OPERATING_MODE                             (NORMAL_MODE)
/***************************************************************************/


/***************************************************************************/
/*-------------------逆变器控制参数设置------------------------------------*/
/***************************************************************************/
/*-------------------VACOUT输出电压缓启使能--------------------------------*/
#define INV_SOFTWARE_DISABLE                        (0)
#define INV_SOFTWARE_ENABLE                         (1)              
#define INV_SOFTWARE_CTRL_SELECT                    (INV_SOFTWARE_DISABLE)//是否启用输出电压缓启功能

/*-------------------PID单双环选择-----------------------------------------*/
#define INV_SINGLE_LOOP                             (0)
#define INV_DOUBLE_LOOP                             (1)              
#define INV_PID_LOOP_MODE                           (INV_DOUBLE_LOOP)

/*-------------------重复控制使能------------------------------------------*/
#define INV_REPEAT_DISABLE                          (0)
#define INV_REPEAT_ENABLE                           (1)
#define INV_REPEAT_CTRL_SELECT                      (INV_REPEAT_ENABLE)//1：开启重复控制；0：关闭重复控制  

/*-------------------限功率控制使能----------------------------------------*/
#define INV_POWER_DISABLE                           (0)
#define INV_POWER_ENABLE                            (1)
#define INV_POWER_CTRL_SELECT                       (INV_POWER_DISABLE)//1：开启限功率控制；0：关闭限功率控制  
      
/*-------------------PQ下垂控制使能----------------------------------------*/
#define INV_PQ_DROOP_DISABLE                        (0)
#define INV_PQ_DROOP_ENABLE                         (1)
#define INV_PQ_DROOP_CTRL_SELECT                    (INV_PQ_DROOP_DISABLE)//1：开启PQ下垂控制；0：关闭PQ下垂控制 

/*-------------------逆变器输出电压频率------------------------------------*/
#define  INV_AC_VOL_FREQ_50HZ                       (50)
#define  INV_AC_VOL_FREQ_60HZ                       (60)               
#define  INV_AC_VOL_FREQ_SELECT                     (INV_AC_VOL_FREQ_50HZ)
/***************************************************************************/

/*-------------------逆变器输出电压有效值----------------------------------*/
#define  INV_AC_VOL_OUT_120                         (120)
#define  INV_AC_VOL_OUT_230                         (230)               
#define  INV_AC_VOL_OUT_SELECT                      (INV_AC_VOL_OUT_230)
/***************************************************************************/


/***************************************************************************/
/*-------------------硬件参数配置------------------------------------------*/
/***************************************************************************/
/*-------------------采样比例设置--------------------------------------*/
#define HW_ADC_REF                                  (5.0)   // AD参考工作电压                                                                                                                            
#define HW_VBUS_GAIN                                (133.33)// 母线电压倍数                      
#define HW_AC_VOL_GAIN                              (200.0) // 输出电压放大倍数 
#define HW_CUR_LOAD_GAIN                            (20.00) // 负载电流放大倍数 
#define HW_CUR_INDUC_GAIN                           (14.70) // 电感电流放大倍数 
#define HW_AUX_POWER_GAIN                           (6.0)   // 辅助电源电压倍数  

/*-------------------采样基值设置------------------------------------------*/
#define COM_VBUS_BASE                               ((int32_t)(HW_ADC_REF * HW_VBUS_GAIN))
#define COM_AC_VOL_BASE                             ((int32_t)(HW_ADC_REF * HW_AC_VOL_GAIN))//采样比例 * AD参考电压
#define COM_CUR_LOAD_BASE                           ((int32_t)(HW_ADC_REF * HW_CUR_LOAD_GAIN))
#define COM_CUR_INDUC_BASE                          ((int32_t)(HW_ADC_REF * HW_CUR_INDUC_GAIN))
#define COM_AUX_POWER_BASE                          ((int32_t)(HW_ADC_REF * HW_AUX_POWER_GAIN))
#define HW_TEMP_DIVIDE_RES                          (10.00)// 温度采样分压电阻值(KΩ)

/*-------------------缩放倍数设置------------------------------------------*/
#define COM_REAL_AUXPOWER_SCAL                      (100)//辅助电源电压真实值缩放系数：8.52V*100 = 852
#define COM_REAL_VBUS_SCAL                          ( 10)//母线电压真实值缩放系数：299.5V*10 = 2995
#define COM_REAL_VACIN_RMS_SCAL                     ( 10)//市电输入电压有效值的真实值缩放系数：218.5V*10 = 2185 
#define COM_REAL_VACOUT_RMS_SCAL                    ( 10)//逆变输出电压有效值的真实值缩放系数
#define COM_REAL_VACIN_FREQ_SCAL                    ( 10)//市电输入电压频率的真实值缩放系数：50.2*10 = 502 
#define COM_REAL_VACOUT_FREQ_SCAL                   ( 10)//逆变输出电压频率的真实值缩放系数
#define COM_REAL_ILOAD_RMS_SCAL                     (100)//负载电流有效值的真实值缩放系数：11.52V*100 = 1152
#define COM_REAL_IINDUC_RMS_SCAL                    (100)//电感电流有效值的真实值缩放系数


/***************************************************************************/
/*-------------------IO端口宏定义------------------------------------------*/
/***************************************************************************/
//LED指示
#define LED_GREEN_ON                                ( PORT->PCLR7 = (1<<0))
#define LED_GREEN_OFF                               ( PORT->PSET7 = (1<<0))

//INV_继电器1
#define INV_RY1_ENABLE				                ( PORT->PSET2 = (1<<0))
#define INV_RY1_DISABLE  			                ( PORT->PCLR2 = (1<<0))
#define INV_RY1_STATE   	                        ( (PORT->PREAD2  &(1<<0))>>0)  

//INV_继电器3
#define INV_RY3_ENABLE				                ( PORT->PSET2 = (1<<1))
#define INV_RY3_DISABLE  			                ( PORT->PCLR2 = (1<<1))

//PFC_继电器2
#define PFC_RY2_ENABLE				                ( PORT->PSET13 = (1<<0))
#define PFC_RY2_DISABLE  			                ( PORT->PCLR13 = (1<<0))

//发送INV工作信号ToDCDC侧
#define INV_START_ENABLE			                ( PORT->PSET7 = (1<<5))
#define INV_START_DISABLE  			                ( PORT->PCLR7 = (1<<5))
#define INV_START_STATE   	                        ((PORT->PREAD7  &(1<<5))>>5)  

//接收DCDC侧工作信号
#define DCDC_WORK_STATE   	                        ((PORT->PREAD3  &(1<<1))>>1)  

//INV开关按键输入信号
#define INV_ON_SWITCH   	                        ((PORT->PREAD6  &(1<<3))>>3)  

//母线过压信号 To DCDC侧
#define VBUS_OVER_ENABLE				            ( PORT->PSET3 = (1<<0))
/***************************************************************************/

#endif 

