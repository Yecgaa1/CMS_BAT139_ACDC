
/*****************************************************************************
Copyright (C) 2021 China Micro Semiconductor Limited Company. All Rights Reserved.
@Description: 
@Author     : 
@Version    : 0.0.1  
@History    : 
@Attention  ��
*****************************************************************************/

#ifndef __SYS_STRUCT_H
#define __SYS_STRUCT_H

#include "stdint.h"

/***************************************************************************/
/*-----------------------------������ѧ���ߺ궨��--------------------------*/
/***************************************************************************/
#define ABSFUN(Value)			                    (Value)>=0?(Value):-(Value)

#define MAXFUN(varone,vartwo)                       (varone)>(vartwo)?(varone):(vartwo)

#define UPDNLMTFUN(Var,Max,Min)	                    {(Var)=((Var)>=(Max))?(Max):(Var);(Var)=((Var)<=(Min))?(Min):(Var);}

//�ж��е���
#define DFILTER(filnum,newinputval,oldoutputval)	(((((int32_t)(newinputval)<<16)-(oldoutputval))>>((filnum)+1))+(oldoutputval)) 

//���ж��е���
#define DFILTER_N(filnum1,newinputval1,oldoutputval1)	(((((int32_t)(newinputval1)<<16)-(oldoutputval1))>>((filnum1)+1))+(oldoutputval1)) 

extern const short int Sine_Table_50Hz[];
extern const short int Sine_Table_60Hz[];

//******************ϵͳ���õ���ö�ٳ���**********************************
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
    E_KEY_CLOSE = 0  ,//�����պ�
    E_KEY_RELEASE	= 1//�����ͷ�
}
KEY_STATE_e;//��/������״̬

typedef enum
{
	E_MIN = 0,	   //���뱣��ʱ����ֵΪ��Сֵ�����ڴ�ֵʱ��Ч
  	E_MAX = 1,	   //���뱣��ʱ����ֵΪ���ֵ��С�ڴ�ֵʱ��Ч
  	E_EQUAL
}
Protect_Limit_Type_e;

typedef struct 
{
    uint8_t	        sys_PWM_Base:1;			    //PWM�ж�ʱ��
    uint8_t	        sys_1ms:1;					//1msʱ����Чλ
    uint8_t	        LED_1ms:1;				    //LED1msʱ��
    uint8_t	        sys_Mode_1ms:1;				//����ģʽ1msʱ��
    uint8_t         faultCheck_1ms:1;		    //����1msʱ����Чλ	
    uint8_t         readyCheck_1ms:1;		    //	
    uint8_t         waitCheck_1ms:1;		    //	    
    uint8_t         Resbits:2;				    //Ԥ��λ
}SysClockBase_t;
extern SysClockBase_t  SysClockBase_ms;

/***************************************************************************/
/***************************************************************************/
typedef enum
{
    E_OLP                   = ( 1 << 0  ),//0λ       
    E_VBUS_OVP              = ( 1 << 1  ),//1λ
    E_VBUS_LVP              = ( 1 << 2  ),//2λ
    E_ACOUT_OVP             = ( 1 << 3  ),//3λ
    E_ACOUT_LVP             = ( 1 << 4  ),//4λ
    E_ACOUT_SCP             = ( 1 << 5  ),//5λ    
    E_AUX_POWER_OVP         = ( 1 << 6  ),//6λ
    E_AUX_POWER_LVP         = ( 1 << 7  ),//7λ    
    E_OTP                   = ( 1 << 8  ),//8λ
    E_I_LOAD_OCP            = ( 1 << 9  ),//9λ
    E_I_INDUC_OCP           = ( 1 << 10 ),//10λ
    E_SYS_INIT_FAIL         = ( 1 << 11 ),//11λ
    E_DCDC_ERR              = ( 1 << 12 ),//12λ
    E_COMMUNICA_ERR         = ( 1 << 13 ),//13λ
    E_VREF_ERR              = ( 1 << 14 ),//14λ    
}
ProtectFlag_Position_e;
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
struct System_Protect_Flag_BITS
{
    unsigned short int OLP:             1;//0λ  
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
    E_START_CHECK_VBUS_HIGH             = ( 1 << 0  ),//0λ
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
    unsigned short int vBus_High_OK:        1;//0λ 1
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
typedef struct _Protect_Check_Info//�����Ľ���/�����ͻָ������ݽṹ
{
    unsigned int short  status_Val;//������Ӧ��״̬��־ֵ
    unsigned int short  compare_Type;//�Ƚ����ͣ����or��С
    unsigned int        period_Val;//�������ٴα����趨ֵ
    unsigned int        period_Cnt;//����ֵ��Ψһ��һ�������������Ķ��ǳ�ʼ�����ٸı�
             int        ref_Val;//�ȽϵĲο�ֵ
             int        hysteretic_Val;//����ֵ
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
    unsigned short int in;//��ƽ����״̬
    unsigned short int close_Cnt;//�������¼���
    unsigned short int release_Cnt;//�����ɿ�����
    unsigned short int out_State;//�պϻ��߶Ͽ�
    unsigned short int ref_Active_Level;//������Ч��ƽ����
    unsigned short int c_Delay_Val;//���·����趨ֵ
    unsigned short int r_Delay_Val;//�ɿ������趨ֵ
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
/*--------------------------------�����ṹ�嶨��--------------------------------------*/                             
struct _INV_Parall_Flag_t
{
    uint16_t		masterFlag:         1;			    //������־λ 1:					bit0    
    uint16_t		slaveFlag:          1;			    //�ӻ���־λ 1:				    bit1    
    uint16_t		syn_Freq_OK:        1;			    //��⵽��Ч�Ĺ�Ƶͬ���ź�	    bit2
    uint16_t		selectStateOK:      1;			    //����״̬ȷ����������ӻ�OK    bit3
    uint16_t        Resbits:12;							//Ԥ��λ		
};

typedef struct 
{
    struct _INV_Parall_Flag_t           Flag; 
    int                                 master_Cnt;
    int                                 slave_Cnt;
    int                                 selectSlave_Cnt;
    int                                 mid_Limit;
    int                                 down_Limit;
    int                                 judge_Vol_RMS_Val;//�������ӻ�������ѹ�趨		
}INV_Parall_Var_t;
extern  INV_Parall_Var_t		INV_Parall_Info; 


/***************************************************************************/
/*----UARTx�������ṹ�嶨��-----------------------------------------------------*/
/***************************************************************************/
typedef struct UARTx_Info
{
     uint8_t  heart_Status;
     uint8_t  heart_OK_Status;
    uint16_t  heart0_Cnt;     //����Ϊ0ʱ����ֵ
    uint16_t  heart1_Cnt;     //����Ϊ1ʱ����ֵ
    uint16_t  heart_Val;      //����������ֵ
    uint8_t   RXD_B[60];//���յ��ֽ����� (Byte)
    int32_t   RXD_W[30];//���յ������� (Word)
    uint8_t   TXD_B[30];//���͵��ֽ����� (Byte)
    int32_t   TXD_W[15];//���͵������� (Word)    
    uint16_t  TXD_Period_Cnt;
    uint16_t  TXD_Period_Val;// ���Ϳ������� 
    uint16_t  TXD_CRC_Init;//����CRC��ʼֵ
    uint16_t  RXD_CRC_Init;//����CRC��ʼֵ    
    uint16_t  TXD_CRC_Calc;//Tx�����CRCУ����    
    uint16_t  RXD_CRC_Calc;//Rx�����CRCУ����
    uint8_t   RXD_CRC_Cnt;//����ʱCRC�������
    uint16_t  RXD_CRC_CkCd;//���յ�CRCУ���룺Check Code
    uint8_t   TXD_Len;//�����ֽڳ���
    uint8_t   RXD_Len;//�����ֽڳ���
    uint8_t   RXD_Flag;//������ɱ�־   
}UARTx_Var_t;
extern UARTx_Var_t    UART1_Info;

/*------------------------------------------------------------------------------------*/
/*--------------------------------UARTx_DC�ṹ�嶨��--------------------------------------*/
typedef struct 
{ 

    uint16_t    err_Code;//DC����ϴ���
    uint8_t     mode_State;//DC�๤��ģʽ���ŵ���߳��
 
    uint8_t     ready_State;//DC��׼��״̬
    uint16_t    vBus_SetVal;//DC�����PFCĸ�ߵ�ѹֵ
    uint8_t     CHG_FinishState;//������״ֵ̬ 
}UARTx_DC_Var_t;
extern  UARTx_DC_Var_t		UARTx_DC_Info; 

/*------------------------------------------------------------------------------------*/
/*--------------------------------INV��ToDC�����ݽṹ�嶨��--------------------------------------*/
typedef struct 
{ 
    uint16_t                                err_Code;//������
    uint8_t                                 mode_State;//����ģʽ���ŵ���߳�� 
    uint8_t                                 ready_State;//׼��״̬
    uint16_t                                VBusAD_Val;//ĸ�ߵ�ѹֵ
    uint16_t                                VACOUT_RMS;//��������ѹֵ��RMS�� 
    uint16_t                                VACOUT_Freq;//��������ѹƵ�ʣ�Hz��
    uint16_t                                VACOUT_Power;//���������ʣ�VA��
    uint16_t                                VACIN_RMS;//�е������ѹֵ��RMS��    
    uint16_t                                VACIN_Freq;//�е������ѹƵ�ʣ�Hz��
    uint16_t                                VACIN_PFC_Power;//�е�����PFC���ʣ�W��
    uint16_t                                VACIN_BypassPower;//�е�������·���ʣ�VA��   
}InvToDC_Data_Var_t;
extern  InvToDC_Data_Var_t		InvToDC_Data_Info; 

/*------------------------------------------------------------------------------------*/
/*--------------------------------LED�ṹ�嶨��--------------------------------------*/
typedef struct 
{ 
    uint8_t     flag_1ms;  
    uint16_t    period_Cnt;// ����ʱ�����
    uint16_t    on_Val;//����ʱ���趨
    uint16_t    period_Val;//��˸�����趨  
    uint8_t     cycle_Cnt;// ѭ����������
    uint8_t     cycle_Val;//ѭ�������趨    
    
}LED_Ctr_Var_t;
extern  LED_Ctr_Var_t		LED_Ctr_Info; 



/*------------------------------------------------------------------------------------*/
/*--------------------------------COM�ṹ�嶨��--------------------------------------*/
typedef struct 
{ 
    int     DC_ERR_CODE;//INV����ʹ�ܱ�ǣ�1�����������0���������      
    int     INV_Enable_Flag;//INV����ʹ�ܱ�ǣ�1�����������0���������
    int     PFC_FREQ_Cnt;//���뽻����ѹƵ�ʼ�����
    int     PFC_FREQ_TimeVal;//���뽻����ѹƵ�ʼ���趨ʱ��
    int     PFC_FREQ_NOKCnt;//���뽻����ѹƵ��NOK������
    int     PFC_FREQ_NOKTimeVal;//���뽻����ѹƵ��NOK����趨ʱ��
    int     PFC_FREQ_State;//�����ѹƵ��״̬  
    int     PFC_AC_Vol_OK_Cnt;//���뽻����ѹ������
    int     PFC_AC_Vol_OK_TimeVal;//���뽻����ѹ����趨ʱ��
    int     PFC_AC_Vol_NOK_Cnt;//���뽻����ѹ������
    int     PFC_AC_Vol_NOK_TimeVal;//���뽻����ѹ����趨ʱ��   
    int     NO_Mode_OK_Cnt;//ģʽΪ��״̬������
    int     NO_Mode_OK_TimeVal;//ģʽΪ��״̬�趨ʱ��     
    int     INV_PFC_Mode_Select;//����ģʽΪ���ģʽ����PFCģʽ
    int     operate_Mode;//�����������ޱ������ܣ��ջ����������ñ�������
    uint8_t PWM_Enable;//ʹ��������
    uint8_t EPWM_Init_Mode;//1:EPWM��ʼ��ΪINV����Ƶ�� ��2��EPWM��ʼ��ΪPFC����Ƶ��   
}COM_Ctr_Var_t;
extern  COM_Ctr_Var_t		COM_Ctr_Info; 


/*------------------------------------------------------------------------------------*/
/*--------------------------------UPS�ṹ�嶨��--------------------------------------*/
typedef struct 
{   
    int16_t PWM_Period;//UPS������µ�����
    int16_t delta_CMP_Val;//�����ѹ�������ѹ������CMP����������ֵ
    int32_t delta_P;//�������������Ĳ�ֵ*50
    int32_t delta_P_Hold;//�������������Ĳ�ֵ*50
    int32_t delta_P_Fir;//�������������Ĳ�ֵ*50
    int16_t delta_T;//
    int16_t PFC_CMP_Val;//���������ѹ����ʱ��CMP��������   
    uint8_t lock_Phase_OK;//����OK
    uint16_t lock_Phase_OK_Cnt;//����OK����
    uint8_t lock_Phase_Cnt;//����ʱ�����
    uint16_t lock_Phase_Val;//�����ʱ�䣬ǿ���������  
    uint8_t V_ACIN_OK;//�е����OK��־
    uint8_t V_ACIN_OK_Cnt;//�е����OK����    
    uint8_t V_ACIN_NOK;//�е����NOK��־    
    uint16_t V_ACIN_NOK_Cnt_P;//�������е����NOK����
    uint16_t V_ACIN_NOK_Cnt_N;//�������е����NOK����
    uint16_t V_ACIN_NOK_Cnt;//���������е����NOK����
    uint8_t V_ACIN_HYS_ADV;//1���е粨���ͺ�2���е粨�γ�ǰ    
    uint8_t flag1; 
    uint8_t flag2; 
    uint8_t flag3; 
    
}UPS_Ctr_Var_t;
extern  UPS_Ctr_Var_t		UPS_Ctr_Info; 

/*------------------------------------------------------------------------------------*/
/*---------------------------------AD������ʵֵ������-------------------------------*/
typedef struct 
{ 
    int32_t     vBus_Val;//ĸ�ߵ�ѹ
    int32_t     vBus_Sum;//ĸ�ߵ�ѹ
    int32_t     vBus_Hold;//  
    int32_t     vBus_Val_Fir;//ĸ�ߵ�ѹ 
   
    uint16_t    vRef_Val;//��׼VRef��ѹ  
    uint16_t    vRef_Val_Fir;//��׼VRef��ѹ     
    uint16_t    auxPower_Val;//������Դ��ѹ 
    uint16_t    auxPower_Val_Fir;//������Դ��ѹ      
    uint16_t    iLoad_RMS_Val;//���ص�����Чֵ
    uint16_t    iLoad_RMS_Val_Fir;//���ص�����Чֵ     
    uint16_t    iInduc_RMS_Val;//��е�����Чֵ 
    uint16_t    iInduc_RMS_Val_Fir;//��е�����Чֵ  
    uint16_t    VACIN_RMS_Val;//�е������ѹ��Чֵ  
    uint16_t    VACIN_RMS_Val_Fir;//�е������ѹ��Чֵ 
    uint16_t    VACOUT_RMS_Val;//��������ѹ��Чֵ 
    uint16_t    VACOUT_RMS_Val_Fir;//��������ѹ��Чֵ 
    uint16_t    VACIN_Freq_Val;//�е������ѹƵ�� 
    uint16_t    VACIN_Freq_Val_Fir;//�е������ѹƵ�� 
    uint16_t    VACOUT_Freq_Val;//��������ѹƵ��  
    uint16_t    VACOUT_Freq_Val_Fir;//��������ѹƵ��     
    uint16_t    temp_NTC_Val;//�����¶�
    uint16_t    temp_NTC_Val_Fir;//�����¶�    

    uint16_t    VACOUT_ApparentPower;//���������ڹ��ʣ�VA��
    uint16_t    VACOUT_ActivePower;//�������й����ʣ�W��
    uint16_t    VACIN_PFC_Power;//�е�����PFC���ʣ�W��
    uint16_t    VACIN_BypassPower;//�е�������·���ʣ�VA��
    
}COM_AD_Data_Var_t;
extern  COM_AD_Data_Var_t		COM_AD_Data_Info; 

/*------------------------------------------------------------------------------------*/
/*--------------------------------ADCӲ��������������---------------------------------*/
typedef struct 
{   
    uint8_t     u8Flag;//
    uint16_t    u16Buff[4];//�������ݻ���
}ADSampleDMA_Var_t;
extern  ADSampleDMA_Var_t		ADSampleDMA_Info; 

/***************************************************************************/


#endif


