
/*****************************************************************************
Copyright (C) 2021 China Micro Semiconductor Limited Company. All Rights Reserved.
@Description: 
@Author     : 
@Version    : 0.0.1  
@History    : 
@Attention  ��
*****************************************************************************/


#ifndef __SYS_DEFINE_PARAM_H
#define	__SYS_DEFINE_PARAM_H

/****************************************************************************/
/*	include files
*****************************************************************************/
#include "sys_define_config.h"
/****************************************************************************/
/*------------------------------------------------------------------------------------*/
//ͨ�Ų�������
/*------------------------------------------------------------------------------------*/
#define UART1_HEART_VAL                                 (200)//ms
#define UART1_SEND_PERIOD_VAL                           (20)//���ͼ��ʱ���趨(ms)  1/(19200)Baud*10*30
#define UART1_SEND_NUM                                  (30)//2�ֽ�֡ͷ+12�ֽ�����λ+2�ֽ�У��λ
#define UART1_RECE_NUM                                  (UART1_SEND_NUM*2)//2�ֽ�֡ͷ+12�ֽ�����λ+2�ֽ�У��λ

/*------------------------------------------------------------------------------------*/
//INV�����ѹ��ֵϵ��
/*------------------------------------------------------------------------------------*/
#define INV_AC_VOL_AMP_VAL_NORMAL_REF                   (2530)//(0 ~ 4096)//�ջ��Ĺ���ϵ��
#define INV_AC_VOL_AMP_VAL_DEBUG_REF                    (3096)//(0 ~ 4096)//�����Ĺ���ϵ��

/*------------------------------------------------------------------------------------*/
//��������ѹ��������ز���
/*------------------------------------------------------------------------------------*/
#define INV_SOFTWARE_STEP_VALUE                         (30)//AC������ʱ����ֵϵ��ÿ�β���ֵ
#define INV_SOFTWARE_SHIFT_VALUE                        ( 6)//AC������ʱ����ֵϵ��������λֵ
#define INV_SOFTWARE_HOLD_INIT                          (1150 << INV_SOFTWARE_SHIFT_VALUE)//�Ŵ��ķ�ֵϵ����ʼֵ

/*------------------------------------------------------------------------------------*/
//Ƶ���޶�������50HZ   = 20.0ms    20.0ms/52us = 384   50Hz
/*------------------------------------------------------------------------------------*/
#define PFC_FREQ_TIME_MAX                               (PFC_PWM_FREQ/44)//(Hz)
#define	INV_FREQ_TIME_MAX                               (INV_PWM_FREQ/30)

#define	INV_FREQ_TIME_UP_50HZ                           (INV_PWM_FREQ/47)//(Hz)
#define	INV_FREQ_TIME_DN_50HZ                           (INV_PWM_FREQ/53)
#define	INV_FREQ_TIME_UP_60HZ                           (INV_PWM_FREQ/57)
#define	INV_FREQ_TIME_DN_60HZ                           (INV_PWM_FREQ/63)

#define INV_JUDGE_VOL_RMS_VAL                           (40 * COM_REAL_VACOUT_RMS_SCAL)//��ʵ��Чֵ(V)*COM_REAL_VACOUT_RMS_SCAL 

//--------------------------------------------------------------------
/*��Чֵ�����������*/
//--------------------------------------------------------------------
#define COM_RMS_DOT_VAL                                 (642)//����ֵ
#define COM_RMS_DOT_RECIPROCAL                          (32768/COM_RMS_DOT_VAL)//�ֶ�ֵ 32768/642 = 51
//--------------------------------------------------------------------

//�����Ƶ��ʱ�仯�������޷�Χ
#define PLL_VACOUT_FREQ_MIN                             (43)//(Hz)
#define PLL_VACOUT_FREQ_MAX                             (67)//(Hz)


//PFC��ѹ��·�ο�����
#define PFC_VBUS_REF_MAX			                    (480 << 2)//ĸ�ߵ�ѹ��ʵֵ(V)*4
#define PFC_VBUS_REF_MIN			                    (375 << 2)

//PFC��������ʱĸ�ߵ�ѹrefֵ
//ע�⣺��Ͽ�����ѹ��
#define PFC_SINGLE_VBUS_VAL_REF                         (400 << 2)//ĸ�ߵ�ѹ��ʵֵ(V)*4

//PFCֱ��ĸ�ߵ�ѹ��ֵ�趨
#define PFC_VBUS_UpLIMIT_VAL			                (495 * COM_REAL_VBUS_SCAL)//PFC��ѹ�г�����ֵ�ر�PWM ---ĸ�ߵ�ѹ��ʵֵ(V)*COM_REAL_VBUS_SCAL

//PFCֱ��ĸ�ߵ�ѹref���������ͳ�ʼֵ����
#define PFC_VBUS_STEP_VAL                               (3)//PFC������ʱ��refÿ�β���ֵ
#define PFC_VBUS_SHIFT_VAL                              (6)//PFC������ʱ��ref������λֵ
#define PFC_VBUS_REF			                        (375L << 2 )//ĸ�߲ο���ѹ��403V   
#define PFC_VBUS_REF_INIT_SS			                (PFC_VBUS_REF << PFC_VBUS_SHIFT_VAL)//�Ŵ���ref��ʼֵ

//�е�Ԥ���VBUS��ѹ�ж�����
#define PFC_VBUS_PRECHARGE_VAL			                (int16_t)(165 * 1.414 * COM_REAL_VBUS_SCAL)//165VAC*1.414*COM_REAL_VBUS_SCAL

//PFCģʽĸ���ж���������Ready_OK�ź���DC��
#define PFC_VBUS_READY_OK_MIN			                (300 * COM_REAL_VBUS_SCAL)//ĸ�ߵ�ѹ��ʵֵ(V)*COM_REAL_VBUS_SCAL
#define PFC_VBUS_READY_OK_CNT_VAL                       (50)//����ֵ

/*------------------------------------------------------------------------------------*/
//��������¶Ȳ�������
/*------------------------------------------------------------------------------------*/
#define COM_START_CHECK_TEMP_UP_RES                     (2.240)//�趨�¶�70���ӦNTC����ֵ(K��)
#define COM_START_CHECK_TEMP_UP_BACK_RES                (1.678)//�趨�¶�80���ӦNTC����ֵ(K��)
#define COM_START_CHECK_TEMP_UP                         (int32_t)((COM_START_CHECK_TEMP_UP_RES/(COM_START_CHECK_TEMP_UP_RES+HW_TEMP_DIVIDE_RES)*4096))  
#define COM_START_CHECK_TEMP_UP_BACK                    (int32_t)((COM_START_CHECK_TEMP_UP_BACK_RES/(COM_START_CHECK_TEMP_UP_BACK_RES+HW_TEMP_DIVIDE_RES)*4096)) 
#define COM_START_CHECK_TEMP_UP_TIME		            (700)//��ʱʱ��(ms)

/*------------------------------------------------------------------------------------*/
//������Դ�����޲�������
/*------------------------------------------------------------------------------------*/
#define COM_AUX_POWER_MAX                               (9.0)//(V)
#define COM_AUX_POWER_MIN                               ( 5.5)
#define COM_AUX_POWER_VALBACK                           ( 0.3)//����ʹ�õĻز�ֵ
/*------------------------------------------------------------------------------------*/
//������⸨����Դ��������
/*------------------------------------------------------------------------------------*/
#define COM_START_CHECK_AUX_POWER_UP                    (int32_t)( COM_AUX_POWER_MAX * COM_REAL_AUXPOWER_SCAL)//������Դ���ǵ�ѹ(V)*COM_REAL_AUXPOWER_SCAL   
#define COM_START_CHECK_AUX_POWER_UP_BACK               (int32_t)((COM_AUX_POWER_MAX + COM_AUX_POWER_VALBACK) * COM_REAL_AUXPOWER_SCAL)
#define COM_START_CHECK_AUX_POWER_UP_TIME		        (700)//��ʱʱ��(ms)

#define COM_START_CHECK_AUX_POWER_DN                    (int32_t)( COM_AUX_POWER_MIN * COM_REAL_AUXPOWER_SCAL)	
#define COM_START_CHECK_AUX_POWER_DN_BACK               (int32_t)((COM_AUX_POWER_MIN - COM_AUX_POWER_VALBACK) * COM_REAL_AUXPOWER_SCAL)
#define COM_START_CHECK_AUX_POWER_DN_TIME			    (700)//��ʱʱ��(ms)

/*------------------------------------------------------------------------------------*/
//������Դ��ѹ/Ƿѹ����
/*------------------------------------------------------------------------------------*/
#define COM_AUX_POWER_OVP_TIME                          (100)//��ʱʱ��(ms))
#define COM_AUX_POWER_OVP_VAL                           (int32_t)( COM_AUX_POWER_MAX * COM_REAL_AUXPOWER_SCAL)//������Դ���ǵ�ѹ(V)*COM_REAL_AUXPOWER_SCAL   
#define COM_AUX_POWER_OVP_VAL_BACK                      (int32_t)((COM_AUX_POWER_MAX - COM_AUX_POWER_VALBACK)* COM_REAL_AUXPOWER_SCAL)   

#define COM_AUX_POWER_LVP_TIME                          (200)//��ʱʱ��(ms) 
#define COM_AUX_POWER_LVP_VAL                           (int32_t)( COM_AUX_POWER_MIN * COM_REAL_AUXPOWER_SCAL)    
#define COM_AUX_POWER_LVP_VAL_BACK                      (int32_t)((COM_AUX_POWER_MIN + COM_AUX_POWER_VALBACK) * COM_REAL_AUXPOWER_SCAL)


#if ( INV_AC_VOL_OUT_SELECT == INV_AC_VOL_OUT_120 )		
    /*------------------------------------------------------------------------------------*/
    //������������ѹƵ�ʲ�������
    /*------------------------------------------------------------------------------------*/
    #define VACIN_FREQ_MAX                              ( 65)//(Hz)
    #define VACIN_FREQ_MIN                              ( 45)
    #define VACIN_FREQ_VALBACK                          (0.5)//����ʹ�õĻز�ֵ(Hz)

    #define PFC_START_CHECK_FREQ_UP                     ( VACIN_FREQ_MAX * COM_REAL_VACIN_FREQ_SCAL)//65HZ
    #define PFC_START_CHECK_FREQ_UP_BACK                ((VACIN_FREQ_MAX + VACIN_FREQ_VALBACK)* COM_REAL_VACIN_FREQ_SCAL)
    #define PFC_START_CHECK_FREQ_UP_TIME			    (100)

    #define PFC_START_CHECK_FREQ_DN                     ( VACIN_FREQ_MIN * COM_REAL_VACIN_FREQ_SCAL)//45HZ
    #define PFC_START_CHECK_FREQ_DN_BACK                ((VACIN_FREQ_MIN - VACIN_FREQ_VALBACK)* COM_REAL_VACIN_FREQ_SCAL)
    #define PFC_START_CHECK_FREQ_DN_TIME			    (100)

    /*------------------------------------------------------------------------------------*/
    //�е������ѹ��������
    //��Чֵ��RMS
    /*------------------------------------------------------------------------------------*/
    #define VACIN_RMS_MAX                               (150)//(V)
    #define VACIN_RMS_MIN                               (80)//(V)
    #define VACIN_RMS_VALBACK                           ( 3)//����ʹ�õĻز�ֵ(V)
    /*------------------------------------------------------------------------------------*/
    //������������ѹ��������
    /*------------------------------------------------------------------------------------*/
    #define PFC_START_CHECK_AC_VOL_UP                   ( VACIN_RMS_MAX * COM_REAL_VACIN_RMS_SCAL )
    #define PFC_START_CHECK_AC_VOL_UP_BACK              ((VACIN_RMS_MAX + VACIN_RMS_VALBACK) * COM_REAL_VACIN_RMS_SCAL )
    #define PFC_START_CHECK_AC_VOL_UP_TIME			    (100)

    #define PFC_START_CHECK_AC_VOL_DN                   ( VACIN_RMS_MIN * COM_REAL_VACIN_RMS_SCAL )
    #define PFC_START_CHECK_AC_VOL_DN_BACK              ((VACIN_RMS_MIN - VACIN_RMS_VALBACK) * COM_REAL_VACIN_RMS_SCAL )
    #define PFC_START_CHECK_AC_VOL_DN_TIME	            (100)

    /*------------------------------------------------------------------------------------*/
    //PFCģʽ���е���ϵ����ж�����  ��ֵ��peak(P)
    /*------------------------------------------------------------------------------------*/
    #define PFC_V_ACIN_P_DN_NOK_4			            ((int32_t)(((VACIN_RMS_MIN*1.000*0.9) *1.414 * 4096) / COM_AC_VOL_BASE))
    //sin(2pi*(480/8/480))=0.707
    #define PFC_V_ACIN_P_DN_NOK_8		                ((int32_t)(((VACIN_RMS_MIN*0.707*0.9) *1.414 * 4096) / COM_AC_VOL_BASE))
    //sin(2pi*(480/16/480))=0.38
    #define PFC_V_ACIN_P_DN_NOK_16		                ((int32_t)(((VACIN_RMS_MIN*0.382*0.9) *1.414 * 4096) / COM_AC_VOL_BASE))

    /*------------------------------------------------------------------------------------*/
    //�������ĸ�߲�������
    /*------------------------------------------------------------------------------------*/
    #define INV_START_CHECK_VBUS_MAX                    (220)//(V)
    #define INV_START_CHECK_VBUS_MIN                    (150)//(V)
    #define INV_START_CHECK_VBUS_VALBACK                ( 5)//����ʹ�õĻز�ֵ(V)
    
    #define INV_START_CHECK_VBUS_UP                     ( INV_START_CHECK_VBUS_MAX * COM_REAL_VBUS_SCAL )//ĸ�ߵ�ѹ��ʵֵ(V)*COM_REAL_VBUS_SCAL  
    #define INV_START_CHECK_VBUS_UP_BACK                ((INV_START_CHECK_VBUS_MAX + INV_START_CHECK_VBUS_VALBACK) * COM_REAL_VBUS_SCAL )
    #define INV_START_CHECK_VBUS_UP_TIME		        (10)//��ʱʱ��(ms)

    #define INV_START_CHECK_VBUS_DN                     ( INV_START_CHECK_VBUS_MIN * COM_REAL_VBUS_SCAL )	
    #define INV_START_CHECK_VBUS_DN_BACK                ((INV_START_CHECK_VBUS_MIN - INV_START_CHECK_VBUS_VALBACK) * COM_REAL_VBUS_SCAL )
    #define INV_START_CHECK_VBUS_DN_TIME			    (10)//��ʱʱ��(ms)

    /*------------------------------------------------------------------------------------*/
    //ĸ�߹�Ƿѹ��������
    /*------------------------------------------------------------------------------------*/
    #define COM_VBUS_OVP1                               (240)//(V)
    #define COM_VBUS_OVP2                               (245)//(V)
    #define COM_VBUS_OVP3                               (245)//(V)
    #define COM_VBUS_LVP1                               (155)//(V)
    #define COM_VBUS_LVP2                               (140)//(V)
    #define COM_VBUS_LVP3                               (135)//(V)    
    #define COM_VBUS_VP_VALBACK                         ( 5)//����ʹ�õĻز�ֵ(V)
    
    #define COM_VBUS_OVP1_TIME		                    (30)//��ʱʱ��(ms)
    #define COM_VBUS_OVP2_TIME			                (30)//��ʱʱ��(ms))
    #define COM_VBUS_OVP3_TIME			                (30)//��ʱʱ��(ms)
    #define COM_VBUS_OVP1_VAL			                ( COM_VBUS_OVP1 * COM_REAL_VBUS_SCAL )//ĸ�ߵ�ѹ��ʵֵ(V)*COM_REAL_VBUS_SCAL
    #define COM_VBUS_OVP2_VAL			                ( COM_VBUS_OVP2 * COM_REAL_VBUS_SCAL )
    #define COM_VBUS_OVP3_VAL			                ( COM_VBUS_OVP3 * COM_REAL_VBUS_SCAL )
    #define COM_VBUS_OVP1_VAL_BACK                      ((COM_VBUS_OVP1 - COM_VBUS_VP_VALBACK) * COM_REAL_VBUS_SCAL )
    #define COM_VBUS_OVP2_VAL_BACK			            ((COM_VBUS_OVP2 - COM_VBUS_VP_VALBACK) * COM_REAL_VBUS_SCAL )
    #define COM_VBUS_OVP3_VAL_BACK			            ((COM_VBUS_OVP3 - COM_VBUS_VP_VALBACK) * COM_REAL_VBUS_SCAL )

    #define INV_VBUS_LVP1_TIME			                (2000)//��ʱʱ��(ms)
    #define INV_VBUS_LVP2_TIME			                (1000)//��ʱʱ��(ms)
    #define INV_VBUS_LVP3_TIME			                (10)//��ʱʱ��(ms)
    #define INV_VBUS_LVP1_VAL			                ( COM_VBUS_LVP1 * COM_REAL_VBUS_SCAL )//ĸ�ߵ�ѹ��ʵֵ(V)*COM_REAL_VBUS_SCAL
    #define INV_VBUS_LVP2_VAL			                ( COM_VBUS_LVP2 * COM_REAL_VBUS_SCAL )
    #define INV_VBUS_LVP3_VAL		                    ( COM_VBUS_LVP3 * COM_REAL_VBUS_SCAL )
    #define INV_VBUS_LVP1_VAL_BACK                      ((COM_VBUS_LVP1 - COM_VBUS_VP_VALBACK) * COM_REAL_VBUS_SCAL )
    #define INV_VBUS_LVP2_VAL_BACK			            ((COM_VBUS_LVP2 - COM_VBUS_VP_VALBACK) * COM_REAL_VBUS_SCAL )
    #define INV_VBUS_LVP3_VAL_BACK			            ((COM_VBUS_LVP3 - COM_VBUS_VP_VALBACK) * COM_REAL_VBUS_SCAL )

    /*------------------------------------------------------------------------------------*/
    //�����ѹ/Ƿѹ��������
    //��Чֵ��RMS
    //����������OVP
    /*------------------------------------------------------------------------------------*/
    #define INV_VACOUT_RMS_OVP                          (135)//(V)
    #define INV_VACOUT_RMS_LVP                          (100)//(V)
    #define INV_VACOUT_RMS_VP_VALBACK                   ( 5)//����ʹ�õĻز�ֵ(V)
    
    #define INV_ACOUT_OVP_TIME                          ( 300)//��ʱʱ��(ms)        
    #define INV_ACOUT_OVP_VAL                           ( INV_VACOUT_RMS_OVP * COM_REAL_VACOUT_RMS_SCAL)//��ʵ��Чֵ(V)*COM_REAL_VACOUT_RMS_SCAL  
    #define INV_ACOUT_OVP_VAL_BACK                      ((INV_VACOUT_RMS_OVP - INV_VACOUT_RMS_VP_VALBACK) * COM_REAL_VACOUT_RMS_SCAL)  

    #define INV_ACOUT_LVP_TIME                          ( 1500)//��ʱʱ��(ms) 
    #define INV_ACOUT_LVP_VAL                           ( INV_VACOUT_RMS_LVP * COM_REAL_VACOUT_RMS_SCAL) 
    #define INV_ACOUT_LVP_VAL_BACK                      ((INV_VACOUT_RMS_LVP + INV_VACOUT_RMS_VP_VALBACK) * COM_REAL_VACOUT_RMS_SCAL)      

    /*------------------------------------------------------------------------------------*/
    //PFC��е���������������
    //��Чֵ��RMS
    //����������OCP
    /*------------------------------------------------------------------------------------*/
    #define PFC_INDUC_RMS_OCP1                          (14.0)//(A)
    #define PFC_INDUC_RMS_OCP2                          (14.0)//(A)
    #define PFC_INDUC_RMS_OCP_VALBACK                   ( 0.4)//����ʹ�õĻز�ֵ(A)

    #define PFC_RMS_OCP1_TIME                           (3000)//(ms) 
    #define PFC_RMS_OCP2_TIME                           (1000)//(ms) 
    #define PFC_RMS_OCP1_VAL                            (int32_t)(PFC_INDUC_RMS_OCP1 * COM_REAL_IINDUC_RMS_SCAL)//��ʵ������Чֵ(A)*COM_REAL_IINDUC_RMS_SCAL
    #define PFC_RMS_OCP2_VAL                            (int32_t)(INV_INDUC_RMS_OCP2 * COM_REAL_IINDUC_RMS_SCAL)   
    #define PFC_RMS_OCP1_VAL_BACK                       (int32_t)((PFC_INDUC_RMS_OCP1 - PFC_INDUC_RMS_OCP_VALBACK) * COM_REAL_IINDUC_RMS_SCAL)
    #define PFC_RMS_OCP2_VAL_BACK                       (int32_t)((PFC_INDUC_RMS_OCP1 - PFC_INDUC_RMS_OCP_VALBACK) * COM_REAL_IINDUC_RMS_SCAL)  

    /*------------------------------------------------------------------------------------*/
    /*------------------------------------------------------------------------------------*/
    //INV��е�����Чֵ��������
    //��Чֵ��RMS
    //����������OCP
    /*------------------------------------------------------------------------------------*/
    #define INV_INDUC_RMS_OCP1                          (20.0)//(A)
    #define INV_INDUC_RMS_OCP2                          (24.0)//(A)
    #define INV_INDUC_RMS_OCP3                          (28.0)//(A)
    #define INV_INDUC_RMS_OCP4                          (38.0)//(A)
    #define INV_INDUC_RMS_OCP_VALBACK                   ( 0.4)//����ʹ�õĻز�ֵ(A)

    #define INV_RMS_OCP1_TIME                           (40000)//��ʱʱ��(ms)
    #define INV_RMS_OCP2_TIME                           (8000)//��ʱʱ��(ms)
    #define INV_RMS_OCP3_TIME                           (12000)//��ʱʱ��(ms)
    #define INV_RMS_OCP4_TIME                           (12000)//��ʱʱ��(ms)

    #define INV_RMS_OCP1_VAL                            (int32_t)(INV_INDUC_RMS_OCP1 * COM_REAL_IINDUC_RMS_SCAL)//��ʵ������Чֵ(A)*COM_REAL_IINDUC_RMS_SCAL
    #define INV_RMS_OCP2_VAL                            (int32_t)(INV_INDUC_RMS_OCP2 * COM_REAL_IINDUC_RMS_SCAL)   
    #define INV_RMS_OCP3_VAL                            (int32_t)(INV_INDUC_RMS_OCP3 * COM_REAL_IINDUC_RMS_SCAL)//��ʵ������Чֵ(A)*COM_REAL_IINDUC_RMS_SCAL
    #define INV_RMS_OCP4_VAL                            (int32_t)(INV_INDUC_RMS_OCP4 * COM_REAL_IINDUC_RMS_SCAL)   

    #define INV_RMS_OCP1_VAL_BACK                       (int32_t)((INV_INDUC_RMS_OCP1 - INV_INDUC_RMS_OCP_VALBACK) * COM_REAL_IINDUC_RMS_SCAL)
    #define INV_RMS_OCP2_VAL_BACK                       (int32_t)((INV_INDUC_RMS_OCP1 - INV_INDUC_RMS_OCP_VALBACK) * COM_REAL_IINDUC_RMS_SCAL)  
    #define INV_RMS_OCP3_VAL_BACK                       (int32_t)((INV_INDUC_RMS_OCP1 - INV_INDUC_RMS_OCP_VALBACK) * COM_REAL_IINDUC_RMS_SCAL)
    #define INV_RMS_OCP4_VAL_BACK                       (int32_t)((INV_INDUC_RMS_OCP1 - INV_INDUC_RMS_OCP_VALBACK) * COM_REAL_IINDUC_RMS_SCAL)  

#else
    /*------------------------------------------------------------------------------------*/
    //������������ѹƵ�ʲ�������
    /*------------------------------------------------------------------------------------*/
    #define VACIN_FREQ_MAX                              ( 63)//(Hz) ���50HZʱ���ɽ����Ƶ��������65H(�ж�ִ��ʱ������)�����60Hzʱ��������65Hz
    #define VACIN_FREQ_MIN                              ( 45)
    #define VACIN_FREQ_VALBACK                          (0.5)//����ʹ�õĻز�ֵ(Hz)

    #define PFC_START_CHECK_FREQ_UP                     ( VACIN_FREQ_MAX * COM_REAL_VACIN_FREQ_SCAL)//65HZ
    #define PFC_START_CHECK_FREQ_UP_BACK                ((VACIN_FREQ_MAX + VACIN_FREQ_VALBACK)* COM_REAL_VACIN_FREQ_SCAL)
    #define PFC_START_CHECK_FREQ_UP_TIME			    (100)

    #define PFC_START_CHECK_FREQ_DN                     ( VACIN_FREQ_MIN * COM_REAL_VACIN_FREQ_SCAL)//45HZ
    #define PFC_START_CHECK_FREQ_DN_BACK                ((VACIN_FREQ_MIN - VACIN_FREQ_VALBACK)* COM_REAL_VACIN_FREQ_SCAL)
    #define PFC_START_CHECK_FREQ_DN_TIME			    (100)

    /*------------------------------------------------------------------------------------*/
    //�е������ѹ��������
    //��Чֵ��RMS
    /*------------------------------------------------------------------------------------*/
    #define VACIN_RMS_MAX                               (274)//(V)
    #define VACIN_RMS_MIN                               (153)//(V)
    #define VACIN_RMS_VALBACK                           ( 3)//����ʹ�õĻز�ֵ(V)
    /*------------------------------------------------------------------------------------*/
    //������������ѹ��������
    /*------------------------------------------------------------------------------------*/
    #define PFC_START_CHECK_AC_VOL_UP                   ( VACIN_RMS_MAX * COM_REAL_VACIN_RMS_SCAL )
    #define PFC_START_CHECK_AC_VOL_UP_BACK              ((VACIN_RMS_MAX + VACIN_RMS_VALBACK) * COM_REAL_VACIN_RMS_SCAL )
    #define PFC_START_CHECK_AC_VOL_UP_TIME			    (100)

    #define PFC_START_CHECK_AC_VOL_DN                   ( VACIN_RMS_MIN * COM_REAL_VACIN_RMS_SCAL )
    #define PFC_START_CHECK_AC_VOL_DN_BACK              ((VACIN_RMS_MIN - VACIN_RMS_VALBACK) * COM_REAL_VACIN_RMS_SCAL )
    #define PFC_START_CHECK_AC_VOL_DN_TIME	            (100)

    /*------------------------------------------------------------------------------------*/
    //PFCģʽ���е���ϵ����ж�����  ��ֵ��peak(P)
    /*------------------------------------------------------------------------------------*/
    #define PFC_V_ACIN_P_DN_NOK_4			            ((int32_t)(((VACIN_RMS_MIN*1.000*0.9) *1.414 * 4096) / COM_AC_VOL_BASE))
    //sin(2pi*(480/8/480))=0.707
    #define PFC_V_ACIN_P_DN_NOK_8		                ((int32_t)(((VACIN_RMS_MIN*0.707*0.9) *1.414 * 4096) / COM_AC_VOL_BASE))
    //sin(2pi*(480/16/480))=0.38
    #define PFC_V_ACIN_P_DN_NOK_16		                ((int32_t)(((VACIN_RMS_MIN*0.382*0.9) *1.414 * 4096) / COM_AC_VOL_BASE))


    /*------------------------------------------------------------------------------------*/
    //�������ĸ�߲�������
    /*------------------------------------------------------------------------------------*/
    #define INV_START_CHECK_VBUS_MAX                    (490)//(V)
    #define INV_START_CHECK_VBUS_MIN                    (300)//(V)
    #define INV_START_CHECK_VBUS_VALBACK                ( 5)//����ʹ�õĻز�ֵ(V)
    
    #define INV_START_CHECK_VBUS_UP                     ( INV_START_CHECK_VBUS_MAX * COM_REAL_VBUS_SCAL )//ĸ�ߵ�ѹ��ʵֵ(V)*COM_REAL_VBUS_SCAL  
    #define INV_START_CHECK_VBUS_UP_BACK                ((INV_START_CHECK_VBUS_MAX + INV_START_CHECK_VBUS_VALBACK) * COM_REAL_VBUS_SCAL )
    #define INV_START_CHECK_VBUS_UP_TIME		        (10)//��ʱʱ��(ms)

    #define INV_START_CHECK_VBUS_DN                     ( INV_START_CHECK_VBUS_MIN * COM_REAL_VBUS_SCAL )	
    #define INV_START_CHECK_VBUS_DN_BACK                ((INV_START_CHECK_VBUS_MIN - INV_START_CHECK_VBUS_VALBACK) * COM_REAL_VBUS_SCAL )
    #define INV_START_CHECK_VBUS_DN_TIME			    (10)//��ʱʱ��(ms)

    /*------------------------------------------------------------------------------------*/
    //ĸ�߹�Ƿѹ��������
    /*------------------------------------------------------------------------------------*/
    #define COM_VBUS_OVP1                               (490)//(V)
    #define COM_VBUS_OVP2                               (490)//(V)
    #define COM_VBUS_OVP3                               (490)//(V)
    #define COM_VBUS_LVP1                               (285)//(V)
    #define COM_VBUS_LVP2                               (280)//(V)
    #define COM_VBUS_LVP3                               (275)//(V)    
    #define COM_VBUS_VP_VALBACK                         ( 5)//����ʹ�õĻز�ֵ(V)
    
    #define COM_VBUS_OVP1_TIME		                    (30)//��ʱʱ��(ms)
    #define COM_VBUS_OVP2_TIME			                (30)//��ʱʱ��(ms))
    #define COM_VBUS_OVP3_TIME			                (30)//��ʱʱ��(ms)
    #define COM_VBUS_OVP1_VAL			                ( COM_VBUS_OVP1 * COM_REAL_VBUS_SCAL )//ĸ�ߵ�ѹ��ʵֵ(V)*COM_REAL_VBUS_SCAL
    #define COM_VBUS_OVP2_VAL			                ( COM_VBUS_OVP2 * COM_REAL_VBUS_SCAL )
    #define COM_VBUS_OVP3_VAL			                ( COM_VBUS_OVP3 * COM_REAL_VBUS_SCAL )
    #define COM_VBUS_OVP1_VAL_BACK                      ((COM_VBUS_OVP1 - COM_VBUS_VP_VALBACK) * COM_REAL_VBUS_SCAL )
    #define COM_VBUS_OVP2_VAL_BACK			            ((COM_VBUS_OVP2 - COM_VBUS_VP_VALBACK) * COM_REAL_VBUS_SCAL )
    #define COM_VBUS_OVP3_VAL_BACK			            ((COM_VBUS_OVP3 - COM_VBUS_VP_VALBACK) * COM_REAL_VBUS_SCAL )

    #define INV_VBUS_LVP1_TIME			                (2000)//��ʱʱ��(ms)
    #define INV_VBUS_LVP2_TIME			                (1000)//��ʱʱ��(ms)
    #define INV_VBUS_LVP3_TIME			                (10)//��ʱʱ��(ms)
    #define INV_VBUS_LVP1_VAL			                ( COM_VBUS_LVP1 * COM_REAL_VBUS_SCAL )//ĸ�ߵ�ѹ��ʵֵ(V)*COM_REAL_VBUS_SCAL
    #define INV_VBUS_LVP2_VAL			                ( COM_VBUS_LVP2 * COM_REAL_VBUS_SCAL )
    #define INV_VBUS_LVP3_VAL		                    ( COM_VBUS_LVP3 * COM_REAL_VBUS_SCAL )
    #define INV_VBUS_LVP1_VAL_BACK                      ((COM_VBUS_LVP1 - COM_VBUS_VP_VALBACK) * COM_REAL_VBUS_SCAL )
    #define INV_VBUS_LVP2_VAL_BACK			            ((COM_VBUS_LVP2 - COM_VBUS_VP_VALBACK) * COM_REAL_VBUS_SCAL )
    #define INV_VBUS_LVP3_VAL_BACK			            ((COM_VBUS_LVP3 - COM_VBUS_VP_VALBACK) * COM_REAL_VBUS_SCAL )


    /*------------------------------------------------------------------------------------*/
    //�����ѹ/Ƿѹ��������
    //��Чֵ��RMS
    //����������OVP
    /*------------------------------------------------------------------------------------*/
    #define VACOUT_RMS_OVP                              (255)//(V)
    #define VACOUT_RMS_LVP                              (160)//(V)
    #define VACOUT_RMS_VP_VALBACK                       ( 5)//����ʹ�õĻز�ֵ(V)
    
    #define INV_ACOUT_OVP_TIME                          ( 300)//��ʱʱ��(ms)        
    #define INV_ACOUT_OVP_VAL                           ( VACOUT_RMS_OVP * COM_REAL_VACOUT_RMS_SCAL)//��ʵ��Чֵ(V)*COM_REAL_VACOUT_RMS_SCAL  
    #define INV_ACOUT_OVP_VAL_BACK                      ((VACOUT_RMS_OVP - VACOUT_RMS_VP_VALBACK) * COM_REAL_VACOUT_RMS_SCAL)  

    #define INV_ACOUT_LVP_TIME                          ( 1500)//��ʱʱ��(ms) 
    #define INV_ACOUT_LVP_VAL                           ( VACOUT_RMS_LVP * COM_REAL_VACOUT_RMS_SCAL) 
    #define INV_ACOUT_LVP_VAL_BACK                      ((VACOUT_RMS_LVP + VACOUT_RMS_VP_VALBACK) * COM_REAL_VACOUT_RMS_SCAL)      
    
    /*------------------------------------------------------------------------------------*/
    //PFC��е���������������
    //��Чֵ��RMS
    //����������OCP
    /*------------------------------------------------------------------------------------*/
    #define PFC_INDUC_RMS_OCP1                          (24.0)//(A)
    #define PFC_INDUC_RMS_OCP2                          (24.0)//(A)
    #define PFC_INDUC_RMS_OCP_VALBACK                   ( 0.4)//����ʹ�õĻز�ֵ(A)

    #define PFC_RMS_OCP1_TIME                           (3000)//(ms) 
    #define PFC_RMS_OCP2_TIME                           (1000)//(ms) 
    #define PFC_RMS_OCP1_VAL                            (int32_t)(PFC_INDUC_RMS_OCP1 * COM_REAL_IINDUC_RMS_SCAL)//��ʵ������Чֵ(A)*COM_REAL_IINDUC_RMS_SCAL
    #define PFC_RMS_OCP2_VAL                            (int32_t)(INV_INDUC_RMS_OCP2 * COM_REAL_IINDUC_RMS_SCAL)   
    #define PFC_RMS_OCP1_VAL_BACK                       (int32_t)((PFC_INDUC_RMS_OCP1 - PFC_INDUC_RMS_OCP_VALBACK) * COM_REAL_IINDUC_RMS_SCAL)
    #define PFC_RMS_OCP2_VAL_BACK                       (int32_t)((PFC_INDUC_RMS_OCP1 - PFC_INDUC_RMS_OCP_VALBACK) * COM_REAL_IINDUC_RMS_SCAL)  

    /*------------------------------------------------------------------------------------*/
    /*------------------------------------------------------------------------------------*/
    //INV��е�����Чֵ��������
    //��Чֵ��RMS
    //����������OCP
    /*------------------------------------------------------------------------------------*/
    #define INV_INDUC_RMS_OCP1                          (10.0)//(A)
    #define INV_INDUC_RMS_OCP2                          (0.9)//(A)
    #define INV_INDUC_RMS_OCP3                          (14.0)//(A)
    #define INV_INDUC_RMS_OCP4                          (18.0)//(A)
    #define INV_INDUC_RMS_OCP_VALBACK                   ( 0.4)//����ʹ�õĻز�ֵ(A)

    #define INV_RMS_OCP1_TIME                           (40000)//��ʱʱ��(ms)
    #define INV_RMS_OCP2_TIME                           (1000)//��ʱʱ��(ms)
    #define INV_RMS_OCP3_TIME                           (12000)//��ʱʱ��(ms)
    #define INV_RMS_OCP4_TIME                           (12000)//��ʱʱ��(ms)

    #define INV_RMS_OCP1_VAL                            (int32_t)(INV_INDUC_RMS_OCP1 * COM_REAL_IINDUC_RMS_SCAL)//��ʵ������Чֵ(A)*COM_REAL_IINDUC_RMS_SCAL
    #define INV_RMS_OCP2_VAL                            (int32_t)(INV_INDUC_RMS_OCP2 * COM_REAL_IINDUC_RMS_SCAL)   
    #define INV_RMS_OCP3_VAL                            (int32_t)(INV_INDUC_RMS_OCP3 * COM_REAL_IINDUC_RMS_SCAL)//��ʵ������Чֵ(A)*COM_REAL_IINDUC_RMS_SCAL
    #define INV_RMS_OCP4_VAL                            (int32_t)(INV_INDUC_RMS_OCP4 * COM_REAL_IINDUC_RMS_SCAL)   

    #define INV_RMS_OCP1_VAL_BACK                       (int32_t)((INV_INDUC_RMS_OCP1 - INV_INDUC_RMS_OCP_VALBACK) * COM_REAL_IINDUC_RMS_SCAL)
    #define INV_RMS_OCP2_VAL_BACK                       (int32_t)((INV_INDUC_RMS_OCP1 - INV_INDUC_RMS_OCP_VALBACK) * COM_REAL_IINDUC_RMS_SCAL)  
    #define INV_RMS_OCP3_VAL_BACK                       (int32_t)((INV_INDUC_RMS_OCP1 - INV_INDUC_RMS_OCP_VALBACK) * COM_REAL_IINDUC_RMS_SCAL)
    #define INV_RMS_OCP4_VAL_BACK                       (int32_t)((INV_INDUC_RMS_OCP1 - INV_INDUC_RMS_OCP_VALBACK) * COM_REAL_IINDUC_RMS_SCAL)  


#endif	

/*------------------------------------------------------------------------------------*/
//�����·��������
//��·������SCP(short circuit protect)
/*------------------------------------------------------------------------------------*/
#define VACOUT_RMS_SCP                                  ( 20)//(V)
#define VACOUT_RMS_SCP_VALBACK                          ( 15)//����ʹ�õĻز�ֵ(V)
    
#define INV_V_ACOUT_SCP_TIME                            ( 55)//��ʱʱ��(ms)      	
#define INV_V_ACOUT_SCP_VAL                             ( VACOUT_RMS_SCP * COM_REAL_VACOUT_RMS_SCAL)//��ʵ��Чֵ(V)*COM_REAL_VACOUT_RMS_SCAL
#define INV_V_ACOUT_SCP_VAL_BACK                        ((VACOUT_RMS_SCP + VACOUT_RMS_SCP_VALBACK) * COM_REAL_VACOUT_RMS_SCAL)  

#define INV_V_ACOUT_P_SCP_TIME                          ( 55)//��ʱʱ��(ms)      	
#define INV_V_ACOUT_P_SCP_VAL                           ( VACOUT_RMS_SCP*1.414*4096/COM_AC_VOL_BASE)
#define INV_V_ACOUT_P_SCP_VAL_BACK                      ((VACOUT_RMS_SCP + VACOUT_RMS_SCP_VALBACK)*1.414*4096/COM_AC_VOL_BASE)//�����ѹ˲ʱֵ(peak)�趨��ֵ   

//���ö�·��������趨ֵ
#define INV_ACOUT_SCP_START_FLAG                        ( 1)//1����ʼ̬�����ö�·���ܣ�0����ʼ̬���ö�·����
#define INV_ACOUT_SCP_START_TIME                        (60)//��ʱʱ��(ms)  
#define INV_ACOUT_SCP_START_VAL                         (100 * COM_REAL_VACOUT_RMS_SCAL)//��ʵ��Чֵ(V)*COM_REAL_VACOUT_RMS_SCAL


/*------------------------------------------------------------------------------------*/
//PFC����P(W)��������
/*------------------------------------------------------------------------------------*/
#define PFC_P_OLP1                                      (2400)//(W)
#define PFC_P_OLP2                                      (2450)//(W)
#define PFC_P_OLP_VALBACK                               ( 30)//����ʹ�õĻز�ֵ(W)

#define PFC_P_OLP1_TIME                                 (10000)//(ms)
#define PFC_P_OLP2_TIME                                 (2000)//(ms)
#define PFC_P_OLP1_VAL                                  (PFC_P_OLP1)
#define PFC_P_OLP2_VAL                                  (PFC_P_OLP2)
#define PFC_P_OLP1_VAL_BACK                             (PFC_P_OLP1 - PFC_P_OLP_VALBACK)
#define PFC_P_OLP2_VAL_BACK                             (PFC_P_OLP2 - PFC_P_OLP_VALBACK)


/*------------------------------------------------------------------------------------*/
//INV-�й�����P(W)���ز�������
/*------------------------------------------------------------------------------------*/
#define INV_P_OLP1                                      (2100)//(W)
#define INV_P_OLP2                                      (2500)//(W)
#define INV_P_OLP3                                      (2800)//(W)
#define INV_P_OLP4                                      (3600)//(W)
#define INV_P_OLP_VALBACK                               ( 30)//����ʹ�õĻز�ֵ(W)

#define INV_P_OLP1_TIME                                 (40000)//��ʱʱ��(ms)   
#define INV_P_OLP2_TIME                                 (8000)//��ʱʱ��(ms)    
#define INV_P_OLP3_TIME	                                (2000)//��ʱʱ��(ms)     
#define INV_P_OLP4_TIME	                                (2000)//��ʱʱ��(ms)     

#define INV_P_OLP1_VAL                                  (INV_P_OLP1)     
#define INV_P_OLP2_VAL                                  (INV_P_OLP2)      
#define INV_P_OLP3_VAL	                                (INV_P_OLP3)      
#define INV_P_OLP4_VAL	                                (INV_P_OLP4) 

#define INV_P_OLP1_VAL_BACK                             (INV_P_OLP1 - INV_P_OLP_VALBACK)
#define INV_P_OLP2_VAL_BACK                             (INV_P_OLP2 - INV_P_OLP_VALBACK)
#define INV_P_OLP3_VAL_BACK	                            (INV_P_OLP3 - INV_P_OLP_VALBACK)
#define INV_P_OLP4_VAL_BACK	                            (INV_P_OLP4 - INV_P_OLP_VALBACK)


/*------------------------------------------------------------------------------------*/
//���ڹ���-S(VA)���ز�������
/*------------------------------------------------------------------------------------*/
#define INV_S_OLP1                                      (2200)//(VA)
#define INV_S_OLP2                                      (2600)//(VA)
#define INV_S_OLP3                                      (2900)//(VA)
#define INV_S_OLP4                                      (3800)//(VA)
#define INV_S_OLP_VALBACK                               ( 30)//����ʹ�õĻز�ֵ(VA)

#define INV_S_OLP1_TIME                                 (120000)//��ʱʱ��(ms)   
#define INV_S_OLP2_TIME                                 (20000)//��ʱʱ��(ms)    
#define INV_S_OLP3_TIME	                                (2000)//��ʱʱ��(ms)     
#define INV_S_OLP4_TIME	                                (1000)//��ʱʱ��(ms)     

#define INV_S_OLP1_VAL                                  (INV_S_OLP1)    
#define INV_S_OLP2_VAL                                  (INV_S_OLP2)      
#define INV_S_OLP3_VAL	                                (INV_S_OLP3)      
#define INV_S_OLP4_VAL	                                (INV_S_OLP4) 

#define INV_S_OLP1_VAL_BACK                             (INV_S_OLP1 - INV_S_OLP_VALBACK)
#define INV_S_OLP2_VAL_BACK                             (INV_S_OLP2 - INV_S_OLP_VALBACK)
#define INV_S_OLP3_VAL_BACK	                            (INV_S_OLP3 - INV_S_OLP_VALBACK)
#define INV_S_OLP4_VAL_BACK	                            (INV_S_OLP4 - INV_S_OLP_VALBACK)


/*------------------------------------------------------------------------------------*/
//��������
/*------------------------------------------------------------------------------------*/	
#define COM_OTP1_TIME                                   (500)//��ʱʱ��(ms)
#define COM_OTP2_TIME                                   (500)//��ʱʱ��(ms)
#define COM_OTP1_RES                                    (1.678)//�趨�¶�80���ӦNTC����ֵ(K��)
#define COM_OTP2_RES                                    (1.275)//�趨�¶�90���ӦNTC����ֵ(K��)
#define COM_OTP1_BACK_RES                               (2.240)//�趨�¶�70���ӦNTC����ֵ(K��)
#define COM_OTP2_BACK_RES                               (2.240)//�趨�¶�70���ӦNTC����ֵ(K��)                          
#define COM_OTP1_VAL                                    (int32_t)((COM_OTP1_RES/(COM_OTP1_RES+HW_TEMP_DIVIDE_RES)*4096))
#define COM_OTP2_VAL                                    (int32_t)((COM_OTP2_RES/(COM_OTP2_RES+HW_TEMP_DIVIDE_RES)*4096))
#define COM_OTP1_VAL_BACK                               (int32_t)((COM_OTP1_BACK_RES/(COM_OTP1_BACK_RES+HW_TEMP_DIVIDE_RES)*4096))
#define COM_OTP2_VAL_BACK                               (int32_t)((COM_OTP2_BACK_RES/(COM_OTP2_BACK_RES+HW_TEMP_DIVIDE_RES)*4096))	
/*------------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------------*/
//VREF�ο���ѹ����������
/*------------------------------------------------------------------------------------*/	
#define COM_VREF_MAX                                    (2248)
#define COM_VREF_MIN                                    (1848)
#define COM_VREF_VALBACK                                ( 80)//����ʹ�õĻز�ֵ

#define COM_VREF_OVP_TIME                               (10)//��ʱʱ��(ms)
#define COM_VREF_OVP_VAL                                (COM_VREF_MAX)//
#define COM_VREF_OVP_VAL_BACK                           (COM_VREF_MAX - COM_VREF_VALBACK)

#define COM_VREF_LVP_TIME                               (10)//��ʱʱ��(ms)
#define COM_VREF_LVP_VAL                                (COM_VREF_MIN)//
#define COM_VREF_LVP_VAL_BACK                           (COM_VREF_MIN + COM_VREF_VALBACK)	
/*------------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------------*/
//������������
/*------------------------------------------------------------------------------------*/
#define COM_KEY_FREQ_CHOOSE_ACTIVE_LEVEL                (0)//��Ч��ƽ����0����1
#define COM_KEY_FREQ_CHOOSE_C_DELAY_VAL                 (200)//����������ʱʱ��(ms)
#define COM_KEY_FREQ_CHOOSE_R_DELAY_VAL                 (100)//�ɿ�������ʱʱ��(ms)

#define COM_KEY_INV_ON_ACTIVE_LEVEL                     (0)//��Ч��ƽ����0����1
#define COM_KEY_INV_ON_C_DELAY_VAL                      (250)//����������ʱʱ��(ms)
#define COM_KEY_INV_ON_R_DELAY_VAL                      (100)//�ɿ�������ʱʱ��(ms)
/*-------------------��ʱʱ���趨ֵ-----------------------------------*/
#define COM_INV_ON_DELAY_TIME                           (500)//ms


/*------------------------------------------------------------------------------------*/
//AD�ο�У׼ֵ��������
/*------------------------------------------------------------------------------------*/
#define COM_BASE_AD_CORRECT_VAL                         (2048)//Ԥ���׼ֵ
#define COM_BASE_AD_CORRECT_VALBACK                     ( 150)

//VREF_ADУ��ֵ
#define COM_VREF_AD_CORRECT_UP                          (COM_BASE_AD_CORRECT_VAL + COM_BASE_AD_CORRECT_VALBACK)
#define COM_VREF_AD_CORRECT_MID                         (COM_BASE_AD_CORRECT_VAL)
#define COM_VREF_AD_CORRECT_DN                          (COM_BASE_AD_CORRECT_VAL - COM_BASE_AD_CORRECT_VALBACK)

//INV��ѹ����ADУ��ֵ
#define COM_V_ACOUT_AD_CORRECT_UP                       (COM_BASE_AD_CORRECT_VAL + COM_BASE_AD_CORRECT_VALBACK)
#define COM_V_ACOUT_AD_CORRECT_MID                      (COM_BASE_AD_CORRECT_VAL)
#define COM_V_ACOUT_AD_CORRECT_DN                       (COM_BASE_AD_CORRECT_VAL - COM_BASE_AD_CORRECT_VALBACK)

//PFC��ѹ����ADУ��ֵ
#define COM_V_ACIN_AD_CORRECT_UP                        (COM_BASE_AD_CORRECT_VAL + COM_BASE_AD_CORRECT_VALBACK)
#define COM_V_ACIN_AD_CORRECT_MID                       (COM_BASE_AD_CORRECT_VAL)
#define COM_V_ACIN_AD_CORRECT_DN                        (COM_BASE_AD_CORRECT_VAL - COM_BASE_AD_CORRECT_VALBACK)

//��е�������ADУ��ֵ
#define COM_I_INDUC_AD_CORRECT_UP                       (COM_BASE_AD_CORRECT_VAL + COM_BASE_AD_CORRECT_VALBACK)
#define COM_I_INDUC_AD_CORRECT_MID                      (COM_BASE_AD_CORRECT_VAL)
#define COM_I_INDUC_AD_CORRECT_DN                       (COM_BASE_AD_CORRECT_VAL - COM_BASE_AD_CORRECT_VALBACK)

//���ص�������ADУ��ֵ
#define COM_I_LOAD_AD_CORRECT_UP                        (COM_BASE_AD_CORRECT_VAL + COM_BASE_AD_CORRECT_VALBACK)
#define COM_I_LOAD_AD_CORRECT_MID                       (COM_BASE_AD_CORRECT_VAL)
#define COM_I_LOAD_AD_CORRECT_DN                        (COM_BASE_AD_CORRECT_VAL - COM_BASE_AD_CORRECT_VALBACK)
      
/*------------------------------------------------------------------------------------*/
//���ұ��������
/*------------------------------------------------------------------------------------*/
#if(INV_PWM_FREQ == 9600)
    #define SPWMWAVE_DOT_50Hz                           (192)                   
    #define SPWMWAVE_DOT_50Hz_2                         (96)                  
    #define SPWMWAVE_DOT_50Hz_4                         (48)                  
    #define SPWMWAVE_DOT_RECIPROCAL_50Hz                (171)//_IQ(1/192) * 32768
    #define SINE_FREQ_VALUE_50Hz                        (50)

    #define SPWMWAVE_DOT_60Hz                           (160)
    #define SPWMWAVE_DOT_60Hz_2                         (80)
    #define SPWMWAVE_DOT_60Hz_4                         (40)
    #define SPWMWAVE_DOT_RECIPROCAL_60Hz                (205)//_IQ(1/160*32768)  9.6K
    #define SINE_FREQ_VALUE_60Hz                        (60)//hz
#elif(INV_PWM_FREQ == 12000)
    #define SPWMWAVE_DOT_50Hz                           (240)                   
    #define SPWMWAVE_DOT_50Hz_2                         (120)                  
    #define SPWMWAVE_DOT_50Hz_4                         (60)                  
    #define SPWMWAVE_DOT_RECIPROCAL_50Hz                (137)//_IQ(1/240) * 32768
    #define SINE_FREQ_VALUE_50Hz                        (50)  
    

    #define SPWMWAVE_DOT_60Hz                           (200)
    #define SPWMWAVE_DOT_60Hz_2                         (100)
    #define SPWMWAVE_DOT_60Hz_4                         (50)
    #define SPWMWAVE_DOT_RECIPROCAL_60Hz                (164)//_IQ(1/320)  12K
    #define SINE_FREQ_VALUE_60Hz                        (60)//hz
#elif(INV_PWM_FREQ == 19200)
    #define SPWMWAVE_DOT_50Hz                           (384)              
    #define SPWMWAVE_DOT_50Hz_2                         (192)               
    #define SPWMWAVE_DOT_50Hz_4                         (96)              
    #define SPWMWAVE_DOT_RECIPROCAL_50Hz                (85)//_IQ(1/192) * 32768
    #define SINE_FREQ_VALUE_50Hz                        (50)//hz

    #define SPWMWAVE_DOT_60Hz                           (320)
    #define SPWMWAVE_DOT_60Hz_2                         (160)
    #define SPWMWAVE_DOT_60Hz_4                         (80)
    #define SPWMWAVE_DOT_RECIPROCAL_60Hz                (102)//_IQ(1/320*32768)
    #define SINE_FREQ_VALUE_60Hz                        (60)//hz
    
#elif(INV_PWM_FREQ == 21000)    
    #define SPWMWAVE_DOT_50Hz                           (420)              
    #define SPWMWAVE_DOT_50Hz_2                         (210)               
    #define SPWMWAVE_DOT_50Hz_4                         (105)              
    #define SPWMWAVE_DOT_RECIPROCAL_50Hz                (78)               
    #define SINE_FREQ_VALUE_50Hz                        (50)   

    #define SPWMWAVE_DOT_60Hz                           (350)
    #define SPWMWAVE_DOT_60Hz_2                         (175)
    #define SPWMWAVE_DOT_60Hz_4                         (87)
    #define SPWMWAVE_DOT_RECIPROCAL_60Hz                (94)//_IQ(1/350) * 32768  
    #define SINE_FREQ_VALUE_60Hz                        (60)//hz    

#elif(INV_PWM_FREQ == 24000)    
    #define SPWMWAVE_DOT_50Hz                           (480)              
    #define SPWMWAVE_DOT_50Hz_2                         (240)               
    #define SPWMWAVE_DOT_50Hz_4                         (120)              
    #define SPWMWAVE_DOT_RECIPROCAL_50Hz                (68)//_IQ(1/480) * 32768  24Khz
    #define SINE_FREQ_VALUE_50Hz                        (50)   
    
    #define SPWMWAVE_DOT_60Hz                           (400)
    #define SPWMWAVE_DOT_60Hz_2                         (200)
    #define SPWMWAVE_DOT_60Hz_4                         (100)
    #define SPWMWAVE_DOT_RECIPROCAL_60Hz                (82)//_IQ(1/400) * 32768  24K
    #define SINE_FREQ_VALUE_60Hz                        (60)//hz    
#endif


/*------------------------------------------------------------------------------------*/
//�����迹ֵ�趨
/*------------------------------------------------------------------------------------*/
#define INV_VIRTUAL_RES                                 (0)
#define INV_VIRTUAL_RES_MIN                             (0)


/*------------------------------------------------------------------------------------*/
//PQ�´�����
/*------------------------------------------------------------------------------------*/
#define PI                                              (3.1415)//3.1415*4096
#define PERIOD_COEFF                                    (int32_t)(COM_MCU_CLK * 2 * PI)

#if ( INV_AC_VOL_FREQ_SELECT == INV_AC_VOL_FREQ_50HZ )
    #define OMIGA_REF                                   (int32_t)(2 * PI * INV_AC_VOL_FREQ_50HZ)//2*PI*f_base
#else
    #define OMIGA_REF                                   (int32_t)(2 * PI * INV_AC_VOL_FREQ_60HZ)//2*PI*f_base
#endif


#endif 

