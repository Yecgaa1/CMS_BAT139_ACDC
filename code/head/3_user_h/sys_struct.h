
/*****************************************************************************
Copyright (C) 2021 China Micro Semiconductor Limited Company. All Rights Reserved.
@Description: 
@Author     : 
@Version    : 0.0.1  
@History    : 
@Attention  ：
*****************************************************************************/

#ifndef __SYS_STRUCT_H
#define __SYS_STRUCT_H

#include "stdint.h"

/***************************************************************************/
/*-----------------------------常用数学工具宏定义--------------------------*/
/***************************************************************************/
#define ABSFUN(Value)			                    (Value)>=0?(Value):-(Value)

#define MAXFUN(varone,vartwo)                       (varone)>(vartwo)?(varone):(vartwo)

#define UPDNLMTFUN(Var,Max,Min)	                    {(Var)=((Var)>=(Max))?(Max):(Var);(Var)=((Var)<=(Min))?(Min):(Var);}

//中断中调用
#define DFILTER(filnum,newinputval,oldoutputval)	(((((int32_t)(newinputval)<<16)-(oldoutputval))>>((filnum)+1))+(oldoutputval)) 

//非中断中调用
#define DFILTER_N(filnum1,newinputval1,oldoutputval1)	(((((int32_t)(newinputval1)<<16)-(oldoutputval1))>>((filnum1)+1))+(oldoutputval1)) 

extern const short int Sine_Table_50Hz[];
extern const short int Sine_Table_60Hz[];

//******************系统中用到的枚举常量**********************************
typedef enum
{
  	E_FALSE = 0,
	E_TRUE  = 1,
	E_FAIL  = 2
}
Logic_Value_e;

typedef enum
{
  	E_DISABLE,	
	E_ENABLE
}
Sys_Enable_e;

typedef enum
{
    E_KEY_CLOSE = 0  ,//按键闭合
    E_KEY_RELEASE	= 1//按键释放
}
KEY_STATE_e;//开/关两种状态

typedef enum
{
	E_MIN = 0,	   //进入保护时极限值为最小值，大于此值时有效
  	E_MAX = 1,	   //进入保护时极限值为最大值，小于此值时有效
  	E_EQUAL
}
Protect_Limit_Type_e;

typedef struct 
{
    uint8_t	        sys_PWM_Base:1;			    //PWM中断时基
    uint8_t	        sys_1ms:1;					//1ms时基有效位
    uint8_t	        LED_1ms:1;				    //LED1ms时基
    uint8_t	        sys_Mode_1ms:1;				//工作模式1ms时基
    uint8_t         faultCheck_1ms:1;		    //故障1ms时基有效位	
    uint8_t         readyCheck_1ms:1;		    //	
    uint8_t         waitCheck_1ms:1;		    //	    
    uint8_t         Resbits:2;				    //预留位
}SysClockBase_t;
extern SysClockBase_t  SysClockBase_ms;

/***************************************************************************/
/***************************************************************************/
typedef enum
{
    E_OLP                   = ( 1 << 0  ),//0位       
    E_VBUS_OVP              = ( 1 << 1  ),//1位
    E_VBUS_LVP              = ( 1 << 2  ),//2位
    E_ACOUT_OVP             = ( 1 << 3  ),//3位
    E_ACOUT_LVP             = ( 1 << 4  ),//4位
    E_ACOUT_SCP             = ( 1 << 5  ),//5位    
    E_AUX_POWER_OVP         = ( 1 << 6  ),//6位
    E_AUX_POWER_LVP         = ( 1 << 7  ),//7位    
    E_OTP                   = ( 1 << 8  ),//8位
    E_I_LOAD_OCP            = ( 1 << 9  ),//9位
    E_I_INDUC_OCP           = ( 1 << 10 ),//10位
    E_SYS_INIT_FAIL         = ( 1 << 11 ),//11位
    E_DCDC_ERR              = ( 1 << 12 ),//12位
    E_COMMUNICA_ERR         = ( 1 << 13 ),//13位
    E_VREF_ERR              = ( 1 << 14 ),//14位    
}
ProtectFlag_Position_e;
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
struct System_Protect_Flag_BITS
{
    unsigned short int OLP:             1;//0位  
    unsigned short int vBus_OVP:        1;//2
    unsigned short int vBus_LVP:        1;//4
    unsigned short int ACOUT_OVP:       1;//8
    unsigned short int ACOUT_LVP:       1;//16
    unsigned short int ACOUT_SCP:       1;//32
    unsigned short int auxPower_OVP:    1;//64
    unsigned short int auxPower_LVP:    1;//128
    unsigned short int OTP:             1;//256
    unsigned short int I_Load_OCP:      1;//512
    unsigned short int I_Induc_OCP:     1;//1024
    unsigned short int sys_Init_Fail:   1;//2048
    unsigned short int DCDC_ERR:        1;//4096
    unsigned short int communica_ERR:   1;//8192
    unsigned short int vref_ERR:        1;//16384
    unsigned short int rsvd:            2;

};
typedef union _System_Protect_Flag
{
	unsigned short int                   all;
	struct System_Protect_Flag_BITS      bit;
}System_Protect_Flag_u;
extern System_Protect_Flag_u System_ProtectFlag_Info;

//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
typedef enum
{
    E_START_CHECK_VBUS_HIGH             = ( 1 << 0  ),//0位
    E_START_CHECK_VBUS_LOW              = ( 1 << 1  ),
    E_START_CHECK_AUX_POWER_HIGH        = ( 1 << 2  ),
    E_START_CHECK_AUX_POWER_LOW         = ( 1 << 3  ),
    E_START_CHECK_AC_VOL_HIGH           = ( 1 << 4  ),
    E_START_CHECK_AC_VOL_LOW            = ( 1 << 5  ),
    E_START_CHECK_FREQ_HIGH             = ( 1 << 6  ),
    E_START_CHECK_FREQ_LOW              = ( 1 << 7  ),
    E_START_CHECK_TEMP_HIGH             = ( 1 << 8  )
}StartCheckFlag_Position_e;
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
struct StartCheck_Flag_BITS
{
    unsigned short int vBus_High_OK:        1;//0位 1
    unsigned short int vBus_Low_OK:         1;//2
    unsigned short int auxPower_High_OK:    1;//4
    unsigned short int auxPower_Low_OK:     1;//8
    unsigned short int AC_Vol_High_OK:      1;//16
    unsigned short int AC_Vol_Low_OK:       1;//32
    unsigned short int FREQ_High_OK:        1;//64
    unsigned short int FREQ_Low_OK:         1;//128
    unsigned short int temp_High_OK:        1;//256    
    unsigned short int rsvd:                7;
};
typedef union _StartCheck_Flag
{
    unsigned short int                   all;
    struct StartCheck_Flag_BITS          bit;
}StartCheck_Flag_u;
extern StartCheck_Flag_u StartCheck_Flag_Info;
/***************************************************************************/


/***************************************************************************/
/*--Protect flag information-----------------------------------------------*/
/***************************************************************************/
typedef struct _Protect_Check_Info//保护的进入/处理、和恢复的数据结构
{
    unsigned int short  status_Val;//保护对应的状态标志值
    unsigned int short  compare_Type;//比较类型：最大or最小
    unsigned int        period_Val;//计数多少次保护设定值
    unsigned int        period_Cnt;//计数值，唯一的一个变量，其它的都是初始化后不再改变
             int        ref_Val;//比较的参考值
             int        hysteretic_Val;//迟滞值
}
Protect_Check_Var_t;

extern Protect_Check_Var_t      INV_P_OLP1_Info;
extern Protect_Check_Var_t      INV_P_OLP2_Info;
extern Protect_Check_Var_t      INV_P_OLP3_Info;
extern Protect_Check_Var_t      INV_P_OLP4_Info;

extern Protect_Check_Var_t      INV_S_OLP1_Info;
extern Protect_Check_Var_t      INV_S_OLP2_Info;
extern Protect_Check_Var_t      INV_S_OLP3_Info;
extern Protect_Check_Var_t      INV_S_OLP4_Info;

extern Protect_Check_Var_t      PFC_P_OLP1_Info;
extern Protect_Check_Var_t      PFC_P_OLP2_Info;
																													 
extern Protect_Check_Var_t      PFC_RMS_OCP1_Info;
extern Protect_Check_Var_t      PFC_RMS_OCP2_Info; 
                                                            
extern Protect_Check_Var_t      INV_ACOUT_OVP_Info;
extern Protect_Check_Var_t      INV_ACOUT_LVP_Info;
extern Protect_Check_Var_t      INV_ACOUT_SCP_Info;
extern Protect_Check_Var_t      INV_ACOUT_P_SCP_Info;
extern Protect_Check_Var_t      COM_VBus_OVP1_Info;
extern Protect_Check_Var_t      COM_VBus_OVP2_Info;
extern Protect_Check_Var_t      COM_VBus_OVP3_Info;

extern Protect_Check_Var_t      INV_VBus_LVP1_Info;
extern Protect_Check_Var_t      INV_VBus_LVP2_Info;
extern Protect_Check_Var_t      INV_VBus_LVP3_Info;

extern Protect_Check_Var_t      INV_RMS_OCP1_Info;
extern Protect_Check_Var_t      INV_RMS_OCP2_Info;
extern Protect_Check_Var_t      INV_RMS_OCP3_Info;
extern Protect_Check_Var_t      INV_RMS_OCP4_Info;

extern Protect_Check_Var_t      INV_IST_OCP1_Info;


extern Protect_Check_Var_t      COM_AuxPower_OVP_Info;
extern Protect_Check_Var_t      COM_AuxPower_LVP_Info;

extern Protect_Check_Var_t      COM_OTP1_Info;
extern Protect_Check_Var_t      COM_OTP2_Info;

extern Protect_Check_Var_t      COM_Vref_OVP_Info;
extern Protect_Check_Var_t      COM_Vref_LVP_Info;

extern Protect_Check_Var_t      PFC_StartCheck_FREQ_Up;
extern Protect_Check_Var_t      PFC_StartCheck_FREQ_Down;	
extern Protect_Check_Var_t      PFC_StartCheck_AC_VOL_Up;
extern Protect_Check_Var_t      PFC_StartCheck_AC_VOL_Down;	
extern Protect_Check_Var_t      INV_StartCheck_VBus_Up;
extern Protect_Check_Var_t      INV_StartCheck_VBus_Down;
extern Protect_Check_Var_t      COM_StartCheck_AuxPower_Up;
extern Protect_Check_Var_t      COM_StartCheck_AuxPower_Down;
extern Protect_Check_Var_t      COM_StartCheck_Temp_Up;
/***************************************************************************/



/*------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------*/
struct _ADSample_RefCorrect_Flag
{
    unsigned short int ADRef_Correct_Ok:  1;
    unsigned short int Rsvd:              15;
};
union ADSample_RefCorrect_Flag
{
    unsigned short int                    all;
    struct _ADSample_RefCorrect_Flag      bit; 
};
typedef struct _ADSample_RefCorrect
{
    union ADSample_RefCorrect_Flag        Flag; 
    int64_t                               sum;
    int                                   sum_Cnt;
    int                                   up_Limit;
    int                                   mid_Limit;
    int                                   dn_Limit;
    int                                   out_Val;
}AD_Correct_Var_t;
extern AD_Correct_Var_t   AD_Correct_V_ACOUT;
extern AD_Correct_Var_t   AD_Correct_V_ACIN;
extern AD_Correct_Var_t   AD_Correct_I_Induc;
extern AD_Correct_Var_t   AD_Correct_I_Load;
extern AD_Correct_Var_t   AD_Correct_Vref;

/*------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------*/
typedef struct 
{      
    int  INV_AC_Vol_AD;
    int  INV_AC_Vol_Hold;            
    int  INV_AC_Vol_AD_FIR; 
    int  PFC_AC_Vol_AD;
    int  PFC_AC_Vol_Hold;            
    int  PFC_AC_Vol_AD_FIR;         
    int  curLoad_AD;
    int  curLoad_Hold;
    int  curLoad_AD_FIR;                    
    int  curInduc_AD;
    int  curInduc_Hold;
    int  curInduc_AD_FIR;        
    int  vBus_AD;
    int  vBus_Hold;
    int  vBus_AD_FIR;           
    int  auxPower_AD;
    int  auxPower_AD_FIR;
    int  temp_NTC_AD;
    int  temp_NTC_AD_FIR;
    int  ref_AD;
    int  ref_AD_Fir;
} ADSample_Var_t;
extern ADSample_Var_t    ADSample_Info;
       
typedef struct {  
    unsigned  int        periodDot_Val;
    //unsigned  int        periodDot_Val_4;
    unsigned  int        periodDot_Cnt;
    int        dot_Reciprocal;
    int        INV_AC_Vol_Peak;
    int        PFC_AC_Vol_Peak;
    int        curLoad_Peak;
    int        curInduc_Peak;
    int        sum_P;
    int        sum_Q;   
    int        sum_INV_AC_Vol; 
    int        sum_INV_AC_Vol_Hold;
    int        sum_PFC_AC_Vol;
    int        sum_PFC_AC_Vol_Hold;
    int        sum_CurLoad;  
    int        sum_CurInduc;  
    int        out_P;
    int        out_P_Hold;
    int        out_P_FIR;    
    int        out_Q;

    int        out_S;    
    int        out_S_Hold;
    int        out_S_FIR;
    
    int        out_RMS_INV_AC_Vol;
    int        out_RMS_INV_AC_Vol_Hold;
    int        out_RMS_INV_AC_Vol_FIR;
    int        out_RMS_PFC_AC_Vol;
    int        out_RMS_PFC_AC_Vol_Hold;
    int        out_RMS_PFC_AC_Vol_FIR;                                
    int        out_RMS_CurLoad;
    int        out_RMS_CurLoad_Hold;
    int        out_RMS_CurLoad_FIR; 
    int        out_RMS_CurInduc;
    int        out_RMS_CurInduc_Hold;
    int        out_RMS_CurInduc_FIR; 
}RMS_PQ_Var_t;


/*------------------------------------------------------------------------------
Prototypes for the functions in PQ_Calc.C
------------------------------------------------------------------------------*/
extern RMS_PQ_Var_t    RMS_PQ_Info;  
void RMS_PQ_Calc(RMS_PQ_Var_t *rms_PQ);
void RMS_PQ_Update(RMS_PQ_Var_t *rms_PQ);
                             

/***************************************************************************/
typedef struct {
    unsigned short int in;//电平输入状态
    unsigned short int close_Cnt;//按键按下计数
    unsigned short int release_Cnt;//按键松开计数
    unsigned short int out_State;//闭合或者断开
    unsigned short int ref_Active_Level;//按键有效电平设置
    unsigned short int c_Delay_Val;//按下防抖设定值
    unsigned short int r_Delay_Val;//松开防抖设定值
    void  (*Calc)();
} Key_Var_t;
               
/*-----------------------------------------------------------------------------
Default initalizer for the PARK object.
-----------------------------------------------------------------------------*/

#define KeyInfo_DEFAULTS {  0, \
                            0, \
                            0, \
                            0, \
                            0, \
                            0, \
                            (void (*)( unsigned int ))Key_Calc }

/*------------------------------------------------------------------------------
Prototypes for the functions in PARK.C
------------------------------------------------------------------------------*/
void Key_Calc( Key_Var_t *key );
extern Key_Var_t	    Key_SW_FreqChoose;
extern Key_Var_t        Key_SW_InvOn;
 
/*------------------------------------------------------------------------------------*/
/*--------------------------------并机结构体定义--------------------------------------*/                             
struct _INV_Parall_Flag_t
{
    uint16_t		masterFlag:         1;			    //主机标志位 1:					bit0    
    uint16_t		slaveFlag:          1;			    //从机标志位 1:				    bit1    
    uint16_t		syn_Freq_OK:        1;			    //检测到有效的工频同步信号	    bit2
    uint16_t		selectStateOK:      1;			    //本机状态确认是主机或从机OK    bit3
    uint16_t        Resbits:12;							//预留位		
};

typedef struct 
{
    struct _INV_Parall_Flag_t           Flag; 
    int                                 master_Cnt;
    int                                 slave_Cnt;
    int                                 selectSlave_Cnt;
    int                                 mid_Limit;
    int                                 down_Limit;
    int                                 judge_Vol_RMS_Val;//区分主从机交流电压设定		
}INV_Parall_Var_t;
extern  INV_Parall_Var_t		INV_Parall_Info; 


/***************************************************************************/
/*----UARTx心跳监测结构体定义-----------------------------------------------------*/
/***************************************************************************/
typedef struct UARTx_Info
{
     uint8_t  heart_Status;
     uint8_t  heart_OK_Status;
    uint16_t  heart0_Cnt;     //心跳为0时计数值
    uint16_t  heart1_Cnt;     //心跳为1时计数值
    uint16_t  heart_Val;      //心跳正常阈值
    uint8_t   RXD_B[60];//接收的字节数组 (Byte)
    int32_t   RXD_W[30];//接收的字数组 (Word)
    uint8_t   TXD_B[30];//发送的字节数组 (Byte)
    int32_t   TXD_W[15];//发送的字数组 (Word)    
    uint16_t  TXD_Period_Cnt;
    uint16_t  TXD_Period_Val;// 发送控制周期 
    uint16_t  TXD_CRC_Init;//发送CRC初始值
    uint16_t  RXD_CRC_Init;//接收CRC初始值    
    uint16_t  TXD_CRC_Calc;//Tx计算的CRC校验码    
    uint16_t  RXD_CRC_Calc;//Rx计算的CRC校验码
    uint8_t   RXD_CRC_Cnt;//接收时CRC处理计数
    uint16_t  RXD_CRC_CkCd;//接收的CRC校验码：Check Code
    uint8_t   TXD_Len;//发送字节长度
    uint8_t   RXD_Len;//接收字节长度
    uint8_t   RXD_Flag;//接收完成标志   
}UARTx_Var_t;
extern UARTx_Var_t    UART1_Info;

/*------------------------------------------------------------------------------------*/
/*--------------------------------UARTx_DC结构体定义--------------------------------------*/
typedef struct 
{ 

    uint16_t    err_Code;//DC侧故障代码
    uint8_t     mode_State;//DC侧工作模式：放电或者充电
 
    uint8_t     ready_State;//DC侧准备状态
    uint16_t    vBus_SetVal;//DC侧给定PFC母线电压值
    uint8_t     CHG_FinishState;//充电完成状态值 
}UARTx_DC_Var_t;
extern  UARTx_DC_Var_t		UARTx_DC_Info; 

/*------------------------------------------------------------------------------------*/
/*--------------------------------INV侧ToDC侧数据结构体定义--------------------------------------*/
typedef struct 
{ 
    uint16_t                                err_Code;//故障码
    uint8_t                                 mode_State;//工作模式：放电或者充电 
    uint8_t                                 ready_State;//准备状态
    uint16_t                                VBusAD_Val;//母线电压值
    uint16_t                                VACOUT_RMS;//逆变输出电压值（RMS） 
    uint16_t                                VACOUT_Freq;//逆变输出电压频率（Hz）
    uint16_t                                VACOUT_Power;//逆变输出功率（VA）
    uint16_t                                VACIN_RMS;//市电输入电压值（RMS）    
    uint16_t                                VACIN_Freq;//市电输入电压频率（Hz）
    uint16_t                                VACIN_PFC_Power;//市电输入PFC功率（W）
    uint16_t                                VACIN_BypassPower;//市电输入旁路功率（VA）   
}InvToDC_Data_Var_t;
extern  InvToDC_Data_Var_t		InvToDC_Data_Info; 

/*------------------------------------------------------------------------------------*/
/*--------------------------------LED结构体定义--------------------------------------*/
typedef struct 
{ 
    uint8_t     flag_1ms;  
    uint16_t    period_Cnt;// 周期时间计数
    uint16_t    on_Val;//点亮时间设定
    uint16_t    period_Val;//闪烁周期设定  
    uint8_t     cycle_Cnt;// 循环次数计数
    uint8_t     cycle_Val;//循环次数设定    
    
}LED_Ctr_Var_t;
extern  LED_Ctr_Var_t		LED_Ctr_Info; 



/*------------------------------------------------------------------------------------*/
/*--------------------------------COM结构体定义--------------------------------------*/
typedef struct 
{ 
    int     DC_ERR_CODE;//INV开关使能标记；1：开逆变器；0：关逆变器      
    int     INV_Enable_Flag;//INV开关使能标记；1：开逆变器；0：关逆变器
    int     PFC_FREQ_Cnt;//输入交流电压频率检测计数
    int     PFC_FREQ_TimeVal;//输入交流电压频率检测设定时间
    int     PFC_FREQ_NOKCnt;//输入交流电压频率NOK检测计数
    int     PFC_FREQ_NOKTimeVal;//输入交流电压频率NOK检测设定时间
    int     PFC_FREQ_State;//输入电压频率状态  
    int     PFC_AC_Vol_OK_Cnt;//输入交流电压检测计数
    int     PFC_AC_Vol_OK_TimeVal;//输入交流电压检测设定时间
    int     PFC_AC_Vol_NOK_Cnt;//输入交流电压检测计数
    int     PFC_AC_Vol_NOK_TimeVal;//输入交流电压检测设定时间   
    int     NO_Mode_OK_Cnt;//模式为空状态检测计数
    int     NO_Mode_OK_TimeVal;//模式为空状态设定时间     
    int     INV_PFC_Mode_Select;//工作模式为逆变模式或者PFC模式
    int     operate_Mode;//开环工作：无保护功能；闭环工作：启用保护功能
    uint8_t PWM_Enable;//使能驱动标
    uint8_t EPWM_Init_Mode;//1:EPWM初始化为INV开关频率 ；2：EPWM初始化为PFC开关频率   
}COM_Ctr_Var_t;
extern  COM_Ctr_Var_t		COM_Ctr_Info; 


/*------------------------------------------------------------------------------------*/
/*--------------------------------UPS结构体定义--------------------------------------*/
typedef struct 
{   
    int16_t PWM_Period;//UPS计算更新的周期
    int16_t delta_CMP_Val;//输出电压与输入电压过零点的CMP点数计数差值
    int32_t delta_P;//输出与输入过零点的差值*50
    int32_t delta_P_Hold;//输出与输入过零点的差值*50
    int32_t delta_P_Fir;//输出与输入过零点的差值*50
    int16_t delta_T;//
    int16_t PFC_CMP_Val;//保存输入电压掉电时的CMP点数计数   
    uint8_t lock_Phase_OK;//锁相OK
    uint16_t lock_Phase_OK_Cnt;//锁相OK计数
    uint8_t lock_Phase_Cnt;//锁相时间计数
    uint16_t lock_Phase_Val;//锁相最长时间，强制锁相完成  
    uint8_t V_ACIN_OK;//市电接入OK标志
    uint8_t V_ACIN_OK_Cnt;//市电接入OK计数    
    uint8_t V_ACIN_NOK;//市电接入NOK标志    
    uint16_t V_ACIN_NOK_Cnt_P;//正半周市电接入NOK计数
    uint16_t V_ACIN_NOK_Cnt_N;//负半周市电接入NOK计数
    uint16_t V_ACIN_NOK_Cnt;//正负半周市电接入NOK计数
    uint8_t V_ACIN_HYS_ADV;//1：市电波形滞后；2：市电波形超前    
    uint8_t flag1; 
    uint8_t flag2; 
    uint8_t flag3; 
    
}UPS_Ctr_Var_t;
extern  UPS_Ctr_Var_t		UPS_Ctr_Info; 

/*------------------------------------------------------------------------------------*/
/*---------------------------------AD采样真实值处理定义-------------------------------*/
typedef struct 
{ 
    int32_t     vBus_Val;//母线电压
    int32_t     vBus_Sum;//母线电压
    int32_t     vBus_Hold;//  
    int32_t     vBus_Val_Fir;//母线电压 
   
    uint16_t    vRef_Val;//基准VRef电压  
    uint16_t    vRef_Val_Fir;//基准VRef电压     
    uint16_t    auxPower_Val;//辅助电源电压 
    uint16_t    auxPower_Val_Fir;//辅助电源电压      
    uint16_t    iLoad_RMS_Val;//负载电流有效值
    uint16_t    iLoad_RMS_Val_Fir;//负载电流有效值     
    uint16_t    iInduc_RMS_Val;//电感电流有效值 
    uint16_t    iInduc_RMS_Val_Fir;//电感电流有效值  
    uint16_t    VACIN_RMS_Val;//市电输入电压有效值  
    uint16_t    VACIN_RMS_Val_Fir;//市电输入电压有效值 
    uint16_t    VACOUT_RMS_Val;//逆变输出电压有效值 
    uint16_t    VACOUT_RMS_Val_Fir;//逆变输出电压有效值 
    uint16_t    VACIN_Freq_Val;//市电输入电压频率 
    uint16_t    VACIN_Freq_Val_Fir;//市电输入电压频率 
    uint16_t    VACOUT_Freq_Val;//逆变输出电压频率  
    uint16_t    VACOUT_Freq_Val_Fir;//逆变输出电压频率     
    uint16_t    temp_NTC_Val;//器件温度
    uint16_t    temp_NTC_Val_Fir;//器件温度    

    uint16_t    VACOUT_ApparentPower;//逆变输出视在功率（VA）
    uint16_t    VACOUT_ActivePower;//逆变输出有功功率（W）
    uint16_t    VACIN_PFC_Power;//市电输入PFC功率（W）
    uint16_t    VACIN_BypassPower;//市电输入旁路功率（VA）
    
}COM_AD_Data_Var_t;
extern  COM_AD_Data_Var_t		COM_AD_Data_Info; 

/*------------------------------------------------------------------------------------*/
/*--------------------------------ADC硬件触发采样定义---------------------------------*/
typedef struct 
{   
    uint8_t     u8Flag;//
    uint16_t    u16Buff[4];//采样数据缓存
}ADSampleDMA_Var_t;
extern  ADSampleDMA_Var_t		ADSampleDMA_Info; 

/***************************************************************************/


#endif


