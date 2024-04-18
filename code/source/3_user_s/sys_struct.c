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
#include "BAT32G139.h"
#include "sys_define_param.h"
#include "sys_struct.h"
#include "sys_define_config.h"
#include "sys_mcu_header.h"
/***************************************************************************/


/*------------------------------------------------------------------------------------*/
//PFC过载变量定义
/*------------------------------------------------------------------------------------*/
Protect_Check_Var_t     PFC_P_OLP1_Info 		        = { E_OLP,E_MAX,PFC_P_OLP1_TIME,0,\
                                                            PFC_P_OLP1_VAL,PFC_P_OLP1_VAL_BACK };

Protect_Check_Var_t     PFC_P_OLP2_Info 		        = { E_OLP,E_MAX,PFC_P_OLP2_TIME,0, \
                                                            PFC_P_OLP2_VAL,PFC_P_OLP2_VAL_BACK };

/*------------------------------------------------------------------------------------*/
//PFC过流变量定义
/*------------------------------------------------------------------------------------*/																													 
Protect_Check_Var_t     PFC_RMS_OCP1_Info 		        = { E_I_INDUC_OCP,E_MAX,PFC_RMS_OCP1_TIME,0,\
                                                            PFC_RMS_OCP1_VAL,PFC_RMS_OCP1_VAL_BACK };

Protect_Check_Var_t     PFC_RMS_OCP2_Info 		        = { E_I_INDUC_OCP,E_MAX,PFC_RMS_OCP2_TIME,0,\
                                                            PFC_RMS_OCP2_VAL,PFC_RMS_OCP2_VAL_BACK };   

                                                           
/*------------------------------------------------------------------------------------*/
//COM母线过压变量定义
/*------------------------------------------------------------------------------------*/
Protect_Check_Var_t     COM_VBus_OVP1_Info 		        = { E_VBUS_OVP,E_MAX,COM_VBUS_OVP1_TIME,0,\
                                                            COM_VBUS_OVP1_VAL ,COM_VBUS_OVP1_VAL_BACK };

Protect_Check_Var_t     COM_VBus_OVP2_Info 	            = { E_VBUS_OVP,E_MAX,COM_VBUS_OVP2_TIME,0, \
                                                            COM_VBUS_OVP2_VAL ,COM_VBUS_OVP2_VAL_BACK };

Protect_Check_Var_t     COM_VBus_OVP3_Info              = { E_VBUS_OVP,E_MAX,COM_VBUS_OVP3_TIME,0, \
                                                            COM_VBUS_OVP3_VAL ,COM_VBUS_OVP3_VAL_BACK };

/*------------------------------------------------------------------------------------*/
//INV母线欠压变量定义
/*------------------------------------------------------------------------------------*/
Protect_Check_Var_t     INV_VBus_LVP1_Info 		        = { E_VBUS_LVP,E_MIN,INV_VBUS_LVP1_TIME,0,\
                                                            INV_VBUS_LVP1_VAL ,INV_VBUS_LVP1_VAL_BACK };

Protect_Check_Var_t     INV_VBus_LVP2_Info 		        = { E_VBUS_LVP,E_MIN,INV_VBUS_LVP2_TIME,0, \
                                                            INV_VBUS_LVP2_VAL ,INV_VBUS_LVP2_VAL_BACK };

Protect_Check_Var_t     INV_VBus_LVP3_Info 	            = { E_VBUS_LVP,E_MIN,INV_VBUS_LVP3_TIME,0, \
                                                            INV_VBUS_LVP3_VAL ,INV_VBUS_LVP3_VAL_BACK };

/*------------------------------------------------------------------------------------*/
//INV过载变量定义
/*------------------------------------------------------------------------------------*/
Protect_Check_Var_t     INV_P_OLP1_Info 		        = { E_OLP,E_MAX,INV_P_OLP1_TIME,0,\
                                                            INV_P_OLP1_VAL ,INV_P_OLP1_VAL_BACK };

Protect_Check_Var_t     INV_P_OLP2_Info 		        = { E_OLP,E_MAX,INV_P_OLP2_TIME,0,\
                                                            INV_P_OLP2_VAL ,INV_P_OLP2_VAL_BACK };

Protect_Check_Var_t     INV_P_OLP3_Info 		        = { E_OLP,E_MAX,INV_P_OLP3_TIME,0,\
                                                            INV_P_OLP3_VAL ,INV_P_OLP3_VAL_BACK };

Protect_Check_Var_t     INV_P_OLP4_Info 		        = { E_OLP,E_MAX,INV_P_OLP4_TIME,0,\
                                                            INV_P_OLP4_VAL ,INV_P_OLP4_VAL_BACK };

                                                            
Protect_Check_Var_t     INV_S_OLP1_Info 		        = { E_OLP,E_MAX,INV_S_OLP1_TIME,0,\
                                                            INV_S_OLP1_VAL ,INV_S_OLP1_VAL_BACK };

Protect_Check_Var_t     INV_S_OLP2_Info 		        = { E_OLP,E_MAX,INV_S_OLP2_TIME,0,\
                                                            INV_S_OLP2_VAL ,INV_S_OLP2_VAL_BACK };

Protect_Check_Var_t     INV_S_OLP3_Info 		        = { E_OLP,E_MAX,INV_S_OLP3_TIME,0,\
                                                            INV_S_OLP3_VAL ,INV_S_OLP3_VAL_BACK };

Protect_Check_Var_t     INV_S_OLP4_Info 		        = { E_OLP,E_MAX,INV_S_OLP4_TIME,0,\
                                                            INV_S_OLP4_VAL ,INV_S_OLP4_VAL_BACK };                                                         
/*------------------------------------------------------------------------------------*/
//INV过流变量定义
/*------------------------------------------------------------------------------------*/																													 
Protect_Check_Var_t     INV_RMS_OCP1_Info 		        = { E_I_INDUC_OCP,E_MAX,INV_RMS_OCP1_TIME,0,\
                                                            INV_RMS_OCP1_VAL,INV_RMS_OCP1_VAL_BACK };

Protect_Check_Var_t     INV_RMS_OCP2_Info 		        = { E_I_INDUC_OCP,E_MAX,INV_RMS_OCP2_TIME,0,\
                                                            INV_RMS_OCP2_VAL,INV_RMS_OCP2_VAL_BACK };

Protect_Check_Var_t     INV_RMS_OCP3_Info 		        = { E_I_INDUC_OCP,E_MAX,INV_RMS_OCP3_TIME,0,\
                                                            INV_RMS_OCP3_VAL,INV_RMS_OCP3_VAL_BACK };

Protect_Check_Var_t     INV_RMS_OCP4_Info 		        = { E_I_INDUC_OCP,E_MAX,INV_RMS_OCP4_TIME,0,\
                                                            INV_RMS_OCP4_VAL,INV_RMS_OCP4_VAL_BACK };

/*------------------------------------------------------------------------------------*/
//INV输出过欠压变量定义
/*------------------------------------------------------------------------------------*/
Protect_Check_Var_t     INV_ACOUT_OVP_Info 		        = { E_ACOUT_OVP,E_MAX,INV_ACOUT_OVP_TIME,0,\
                                                            INV_ACOUT_OVP_VAL,INV_ACOUT_OVP_VAL_BACK };																															
	
Protect_Check_Var_t     INV_ACOUT_LVP_Info 		        = { E_ACOUT_LVP,E_MIN,INV_ACOUT_LVP_TIME,0,\
                                                            INV_ACOUT_LVP_VAL,INV_ACOUT_LVP_VAL_BACK };	

/*------------------------------------------------------------------------------------*/
//INV输出短路变量定义
/*------------------------------------------------------------------------------------*/	
Protect_Check_Var_t     INV_ACOUT_SCP_Info              = { E_ACOUT_SCP,E_MIN,INV_V_ACOUT_SCP_TIME,0,\
                                                            INV_V_ACOUT_SCP_VAL,INV_V_ACOUT_SCP_VAL_BACK };	

Protect_Check_Var_t     INV_ACOUT_P_SCP_Info            = { E_ACOUT_SCP,E_MIN,INV_V_ACOUT_P_SCP_TIME,0,\
                                                            INV_V_ACOUT_P_SCP_VAL,INV_V_ACOUT_P_SCP_VAL_BACK };	
                                                            
/*------------------------------------------------------------------------------------*/
																																
/*------------------------------------------------------------------------------------*/
//COM辅助电源过欠压变量定义
/*------------------------------------------------------------------------------------*/	
Protect_Check_Var_t     COM_AuxPower_OVP_Info 	        = { E_AUX_POWER_OVP,E_MAX,COM_AUX_POWER_OVP_TIME,0,\
                                                            COM_AUX_POWER_OVP_VAL,COM_AUX_POWER_OVP_VAL_BACK };

Protect_Check_Var_t     COM_AuxPower_LVP_Info 	        = { E_AUX_POWER_LVP,E_MIN,COM_AUX_POWER_LVP_TIME,0,\
                                                            COM_AUX_POWER_LVP_VAL,COM_AUX_POWER_LVP_VAL_BACK };

/*------------------------------------------------------------------------------------*/
//VREF参考电压异常变量定义
/*------------------------------------------------------------------------------------*/                                                            
Protect_Check_Var_t     COM_Vref_OVP_Info               = { E_VREF_ERR,E_MAX,COM_VREF_OVP_TIME,0,\
                                                            COM_VREF_OVP_VAL,COM_VREF_OVP_VAL_BACK };

Protect_Check_Var_t     COM_Vref_LVP_Info               = { E_VREF_ERR,E_MIN,COM_VREF_LVP_TIME,0,\
                                                            COM_VREF_LVP_VAL,COM_VREF_LVP_VAL_BACK };
                                                            
/*------------------------------------------------------------------------------------*/
//COM过温变量定义
/*------------------------------------------------------------------------------------*/	
Protect_Check_Var_t     COM_OTP1_Info 	                = { E_OTP,E_MIN,COM_OTP1_TIME,0,\
                                                            COM_OTP1_VAL,COM_OTP1_VAL_BACK };		

Protect_Check_Var_t     COM_OTP2_Info 	                = { E_OTP,E_MIN,COM_OTP2_TIME,0,\
                                                            COM_OTP2_VAL,COM_OTP2_VAL_BACK };

/*------------------------------------------------------------------------------------*/
//开机启动条件检测变量定义
/*------------------------------------------------------------------------------------*/
Protect_Check_Var_t     INV_StartCheck_VBus_Up         = { E_START_CHECK_VBUS_HIGH,E_MIN,INV_START_CHECK_VBUS_UP_TIME,0,\
                                                            INV_START_CHECK_VBUS_UP,INV_START_CHECK_VBUS_UP_BACK };

Protect_Check_Var_t     INV_StartCheck_VBus_Down       = { E_START_CHECK_VBUS_LOW,E_MAX,INV_START_CHECK_VBUS_DN_TIME,0,\
                                                            INV_START_CHECK_VBUS_DN,INV_START_CHECK_VBUS_DN_BACK };	

Protect_Check_Var_t     COM_StartCheck_AuxPower_Up     = { E_START_CHECK_AUX_POWER_HIGH,E_MIN,COM_START_CHECK_AUX_POWER_UP_TIME,0,\
                                                            COM_START_CHECK_AUX_POWER_UP,COM_START_CHECK_AUX_POWER_UP_BACK };

Protect_Check_Var_t     COM_StartCheck_AuxPower_Down   = { E_START_CHECK_AUX_POWER_LOW,E_MAX,COM_START_CHECK_AUX_POWER_DN_TIME,0,\
                                                            COM_START_CHECK_AUX_POWER_DN,COM_START_CHECK_AUX_POWER_DN_BACK };	

Protect_Check_Var_t     PFC_StartCheck_AC_VOL_Up       = { E_START_CHECK_AC_VOL_HIGH,E_MIN,PFC_START_CHECK_AC_VOL_UP_TIME,0,\
                                                            PFC_START_CHECK_AC_VOL_UP,PFC_START_CHECK_AC_VOL_UP_BACK };

Protect_Check_Var_t     PFC_StartCheck_AC_VOL_Down     = { E_START_CHECK_AC_VOL_LOW,E_MAX,PFC_START_CHECK_AC_VOL_DN_TIME,0,\
                                                            PFC_START_CHECK_AC_VOL_DN,PFC_START_CHECK_AC_VOL_DN_BACK };	

Protect_Check_Var_t     PFC_StartCheck_FREQ_Up         = { E_START_CHECK_FREQ_HIGH,E_MIN,PFC_START_CHECK_FREQ_UP_TIME,0,\
                                                            PFC_START_CHECK_FREQ_UP,PFC_START_CHECK_FREQ_UP_BACK };

Protect_Check_Var_t     PFC_StartCheck_FREQ_Down       = { E_START_CHECK_FREQ_LOW,E_MAX,PFC_START_CHECK_FREQ_DN_TIME,0,\
                                                            PFC_START_CHECK_FREQ_DN,PFC_START_CHECK_FREQ_DN_BACK };	

Protect_Check_Var_t     COM_StartCheck_Temp_Up         = { E_START_CHECK_TEMP_HIGH,E_MAX,COM_START_CHECK_TEMP_UP_TIME,0,\
                                                            COM_START_CHECK_TEMP_UP,COM_START_CHECK_TEMP_UP_BACK };
                                                           
/*------------------------------------------------------------------------------------*/
//AD采样校准定义
/*------------------------------------------------------------------------------------*/
AD_Correct_Var_t        AD_Correct_V_ACOUT              = {0,0,0,COM_V_ACOUT_AD_CORRECT_UP,COM_V_ACOUT_AD_CORRECT_MID,\
                                                            COM_V_ACOUT_AD_CORRECT_DN,COM_V_ACOUT_AD_CORRECT_MID};

AD_Correct_Var_t        AD_Correct_V_ACIN               = {0,0,0,COM_V_ACIN_AD_CORRECT_UP,COM_V_ACIN_AD_CORRECT_MID,\
                                                            COM_V_ACIN_AD_CORRECT_DN,COM_V_ACIN_AD_CORRECT_MID};

AD_Correct_Var_t        AD_Correct_I_Induc              = {0,0,0,COM_I_INDUC_AD_CORRECT_UP,COM_I_INDUC_AD_CORRECT_MID,\
                                                            COM_I_INDUC_AD_CORRECT_DN,COM_I_INDUC_AD_CORRECT_MID};

AD_Correct_Var_t        AD_Correct_I_Load               = {0,0,0,COM_I_LOAD_AD_CORRECT_UP,COM_I_LOAD_AD_CORRECT_MID,\
                                                            COM_I_LOAD_AD_CORRECT_DN,COM_I_LOAD_AD_CORRECT_MID};

AD_Correct_Var_t        AD_Correct_Vref                 = {0,0,0,COM_VREF_AD_CORRECT_UP,COM_VREF_AD_CORRECT_MID,\
                                                            COM_VREF_AD_CORRECT_DN,COM_VREF_AD_CORRECT_MID};
                                                    
/*------------------------------------------------------------------------------------*/
//PID变量声明定义
/*------------------------------------------------------------------------------------*/
PID_Ctrl_Var_t          INV_PID_Vol         = INV_VOL_LOOP_DEFAULTS;
PID_Ctrl_Var_t          INV_PID_Cur         = INV_CUR_LOOP_DEFAULTS;
PID_Ctrl_Var_t          INV_PID_Power       = INV_POWER_LOOP_DEFAULTS;
PID_Ctrl_Var_t          INV_PID_DCIM        = INV_DCIM_LOOP_DEFAULTS;
PID_Ctrl_Var_t          PFC_PID_Vol         = PFC_VOL_LOOP_DEFAULTS;
PID_Ctrl_Var_t          PFC_PID_Cur         = PFC_CUR_LOOP_DEFAULTS;

PID_Ctrl_Var_t          PLL_PID             = PLL_LOOP_DEFAULTS;
PFC_Ctrl_Var_t          PFC_Ctrl_Info;
PFC_Ref_Var_t           PFC_Ref_Info;
PFC_VPID_Var_t          PFC_VPID_Info; 
/*------------------------------------------------------------------------------------*/
//并机变量声明定义
/*------------------------------------------------------------------------------------*/
INV_Parall_Var_t	    INV_Parall_Info;

/*------------------------------------------------------------------------------------*/
//重复控制变量声明定义
/*------------------------------------------------------------------------------------*/                                                  
Repeat_Ctrl_Var_t       INV_Repeat_Info     = REPEAT_CTRL_DEFAULTS;  
                                                  
/*------------------------------------------------------------------------------------*/
//PQ下垂变量声明定义
/*------------------------------------------------------------------------------------*/
INV_PQ_Droop_Ctrl_Var_t INV_PQ_Droop_Info   = INV_PQ_DROOP_CTRL_DEFAULTS;  

                                               
/*------------------------------------------------------------------------------------*/
//功率、电压电流有效值变量定义
/*------------------------------------------------------------------------------------*/																												                                            
RMS_PQ_Var_t            RMS_PQ_Info         = {0};

/*------------------------------------------------------------------------------------*/
//AD采样变量定义
/*------------------------------------------------------------------------------------*/	
ADSample_Var_t          ADSample_Info       = {0}; 


/*------------------------------------------------------------------------------------*/
//按键检测变量定义
/*------------------------------------------------------------------------------------*/
Key_Var_t          	    Key_SW_FreqChoose   = {0,0,0,0,COM_KEY_FREQ_CHOOSE_ACTIVE_LEVEL,COM_KEY_FREQ_CHOOSE_C_DELAY_VAL,COM_KEY_FREQ_CHOOSE_R_DELAY_VAL,(void (*)( unsigned int ))Key_Calc};

Key_Var_t          	    Key_SW_InvOn	    = {0,0,0,0,COM_KEY_INV_ON_ACTIVE_LEVEL,COM_KEY_INV_ON_C_DELAY_VAL,COM_KEY_INV_ON_R_DELAY_VAL,(void (*)( unsigned int ))Key_Calc};



/*------------------------------------------------------------------------------------*/
//系统MS时基变量定义
/*------------------------------------------------------------------------------------*/
SysClockBase_t          SysClockBase_ms;


/*------------------------------------------------------------------------------------*/
//系统保护、自检变量定义
/*------------------------------------------------------------------------------------*/																												
StartCheck_Flag_u       StartCheck_Flag_Info    = {0};
System_Protect_Flag_u   System_ProtectFlag_Info = {0};		


/*------------------------------------------------------------------------------------*/
//逆变器控制变量定义
/*------------------------------------------------------------------------------------*/
INV_Ctrl_Var_t          INV_Ctrl_Info;

/*------------------------------------------------------------------------------------*/
//UARTx心跳监测变量定义
/*------------------------------------------------------------------------------------*/
UARTx_Var_t             UART1_Info          = {0,1,0,0,UART1_HEART_VAL,{0},{0},{0},{0},0,UART1_SEND_PERIOD_VAL,0xffff,0xffff,0,0,0,0,0,0,0};
UARTx_DC_Var_t		    UARTx_DC_Info       ={0};
/*------------------------------------------------------------------------------------*/
//COM共用控制变量定义
/*------------------------------------------------------------------------------------*/
COM_Ctr_Var_t		    COM_Ctr_Info        = {0}; 

/*------------------------------------------------------------------------------------*/
//UPS共用控制变量定义
/*------------------------------------------------------------------------------------*/
UPS_Ctr_Var_t		    UPS_Ctr_Info        = {0}; 

LED_Ctr_Var_t		    LED_Ctr_Info        = {0}; 

InvToDC_Data_Var_t		InvToDC_Data_Info   = {0}; 

COM_AD_Data_Var_t		COM_AD_Data_Info    = {0};

ADSampleDMA_Var_t		ADSampleDMA_Info    = {0};
/*------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------*/
/*  No more.                                                                           */
/*-------------------------------------------------------------------------------------*/
