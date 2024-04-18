
/*****************************************************************************
Copyright (C) 2021 China Micro Semiconductor Limited Company. All Rights Reserved.
@Description: ϵͳ����
@Author     : 
@Version    : 0.0.1  
@History    : 
@Attention  ��
*****************************************************************************/

#ifndef __SYS_DEFINE_CONFIG_H
#define	__SYS_DEFINE_CONFIG_H


#define COM_MCU_CLK 	                            (64000000L)//MCU��Ƶ
/***************************************************************************/
/*-------------------INVģʽʱPWM������������------------------------------*/
/***************************************************************************/
#define INV_PWM_FREQ	                            (21000L)// PWM frequency[Hz]
#define INV_DEADTIME			                    ((int32_t)((0 * COM_MCU_CLK)) / 1000000L)
#if 1
    //���Ļ����Գ�����ֵ
    #define INV_PWM_PERIOD	                        ((int32_t)(COM_MCU_CLK  / (INV_PWM_FREQ * 2)) + INV_DEADTIME - 2)   //1/fk* (m+2�Cp)* 2
    #define INV_PWM_PERIOD_HALF 	                (INV_PWM_PERIOD >> 1)
#else
    //��ݲ�����ֵ
    #define INV_PWM_PERIOD	                        ((int32_t)(COM_MCU_CLK  / (INV_PWM_FREQ )) -1)  //1/fk * (m+1) = period = 1/Carry_FREQ   =>  m + 1 = fk/Carry_FREQ  ��ݲ�   
    #define INV_PWM_PERIOD_HALF 	                (INV_PWM_PERIOD >> 1)
#endif
/***************************************************************************/

/***************************************************************************/
/*-------------------PFCģʽʱPWM������������------------------------------*/
/***************************************************************************/
#define PFC_PWM_FREQ	                            (24000)// PWM frequency[Hz]
#define PFC_DEADTIME			                    ((int32_t)((0 * COM_MCU_CLK)) / 1000000L)
#if 1
    //���Ļ����Գ�����ֵ
    #define PFC_PWM_PERIOD	                        ((int32_t)(COM_MCU_CLK  / (PFC_PWM_FREQ * 2)) + PFC_DEADTIME - 2)   //1/fk* (m+2�Cp)* 2
    #define PFC_PWM_PERIOD_HALF 	                (PFC_PWM_PERIOD >> 1)
#else
    //��ݲ�����ֵ
    #define PFC_PWM_PERIOD	                        ((int32_t)(COM_MCU_CLK  / (PFC_PWM_FREQ )) -1)  //1/fk * (m+1) = period = 1/Carry_FREQ   =>  m + 1 = fk/Carry_FREQ  ��ݲ�   
    #define PFC_PWM_PERIOD_HALF 	                (PFC_PWM_PERIOD >> 1)
#endif
/***************************************************************************/
/*-------------------PFC������������DC����������---------------------------*/
#define PFC_SINGLE_DISABLE                          (0)//PFC��ǰ��DC����������ѹ��·refֵ��DC�����(�������ʱ)
#define PFC_SINGLE_ENABLE                           (1)//PFC�������������̶��ĵ�ѹ��·refֵ(ע�⣺��Ͽ�����ѹ��)
#define PFC_SINGLE_CTRL_SELECT                      (PFC_SINGLE_DISABLE) 

/*-------------------INVģʽ����PFCģʽ------------------------------------*/
#define FREE_MODE                                   (0)
#define INV_MODE                                    (1)
#define PFC_MODE                                    (2) 

/***************************************************************************/
/*-------------------����ģʽ����------------------------------------------*/
/***************************************************************************/
#define  NORMAL_MODE                                (0)//�ջ����������ñ�������
#define  DEBUG_MODE                                 (1)//�����������رձ�������
#define  TEST_MODE                                  (2)
#define  OPERATING_MODE                             (NORMAL_MODE)
/***************************************************************************/


/***************************************************************************/
/*-------------------��������Ʋ�������------------------------------------*/
/***************************************************************************/
/*-------------------VACOUT�����ѹ����ʹ��--------------------------------*/
#define INV_SOFTWARE_DISABLE                        (0)
#define INV_SOFTWARE_ENABLE                         (1)              
#define INV_SOFTWARE_CTRL_SELECT                    (INV_SOFTWARE_DISABLE)//�Ƿ����������ѹ��������

/*-------------------PID��˫��ѡ��-----------------------------------------*/
#define INV_SINGLE_LOOP                             (0)
#define INV_DOUBLE_LOOP                             (1)              
#define INV_PID_LOOP_MODE                           (INV_DOUBLE_LOOP)

/*-------------------�ظ�����ʹ��------------------------------------------*/
#define INV_REPEAT_DISABLE                          (0)
#define INV_REPEAT_ENABLE                           (1)
#define INV_REPEAT_CTRL_SELECT                      (INV_REPEAT_ENABLE)//1�������ظ����ƣ�0���ر��ظ�����  

/*-------------------�޹��ʿ���ʹ��----------------------------------------*/
#define INV_POWER_DISABLE                           (0)
#define INV_POWER_ENABLE                            (1)
#define INV_POWER_CTRL_SELECT                       (INV_POWER_DISABLE)//1�������޹��ʿ��ƣ�0���ر��޹��ʿ���  
      
/*-------------------PQ�´�����ʹ��----------------------------------------*/
#define INV_PQ_DROOP_DISABLE                        (0)
#define INV_PQ_DROOP_ENABLE                         (1)
#define INV_PQ_DROOP_CTRL_SELECT                    (INV_PQ_DROOP_DISABLE)//1������PQ�´����ƣ�0���ر�PQ�´����� 

/*-------------------����������ѹƵ��------------------------------------*/
#define  INV_AC_VOL_FREQ_50HZ                       (50)
#define  INV_AC_VOL_FREQ_60HZ                       (60)               
#define  INV_AC_VOL_FREQ_SELECT                     (INV_AC_VOL_FREQ_50HZ)
/***************************************************************************/

/*-------------------����������ѹ��Чֵ----------------------------------*/
#define  INV_AC_VOL_OUT_120                         (120)
#define  INV_AC_VOL_OUT_230                         (230)               
#define  INV_AC_VOL_OUT_SELECT                      (INV_AC_VOL_OUT_230)
/***************************************************************************/


/***************************************************************************/
/*-------------------Ӳ����������------------------------------------------*/
/***************************************************************************/
/*-------------------������������--------------------------------------*/
#define HW_ADC_REF                                  (5.0)   // AD�ο�������ѹ                                                                                                                            
#define HW_VBUS_GAIN                                (133.33)// ĸ�ߵ�ѹ����                      
#define HW_AC_VOL_GAIN                              (200.0) // �����ѹ�Ŵ��� 
#define HW_CUR_LOAD_GAIN                            (20.00) // ���ص����Ŵ��� 
#define HW_CUR_INDUC_GAIN                           (14.70) // ��е����Ŵ��� 
#define HW_AUX_POWER_GAIN                           (6.0)   // ������Դ��ѹ����  

/*-------------------������ֵ����------------------------------------------*/
#define COM_VBUS_BASE                               ((int32_t)(HW_ADC_REF * HW_VBUS_GAIN))
#define COM_AC_VOL_BASE                             ((int32_t)(HW_ADC_REF * HW_AC_VOL_GAIN))//�������� * AD�ο���ѹ
#define COM_CUR_LOAD_BASE                           ((int32_t)(HW_ADC_REF * HW_CUR_LOAD_GAIN))
#define COM_CUR_INDUC_BASE                          ((int32_t)(HW_ADC_REF * HW_CUR_INDUC_GAIN))
#define COM_AUX_POWER_BASE                          ((int32_t)(HW_ADC_REF * HW_AUX_POWER_GAIN))
#define HW_TEMP_DIVIDE_RES                          (10.00)// �¶Ȳ�����ѹ����ֵ(K��)

/*-------------------���ű�������------------------------------------------*/
#define COM_REAL_AUXPOWER_SCAL                      (100)//������Դ��ѹ��ʵֵ����ϵ����8.52V*100 = 852
#define COM_REAL_VBUS_SCAL                          ( 10)//ĸ�ߵ�ѹ��ʵֵ����ϵ����299.5V*10 = 2995
#define COM_REAL_VACIN_RMS_SCAL                     ( 10)//�е������ѹ��Чֵ����ʵֵ����ϵ����218.5V*10 = 2185 
#define COM_REAL_VACOUT_RMS_SCAL                    ( 10)//��������ѹ��Чֵ����ʵֵ����ϵ��
#define COM_REAL_VACIN_FREQ_SCAL                    ( 10)//�е������ѹƵ�ʵ���ʵֵ����ϵ����50.2*10 = 502 
#define COM_REAL_VACOUT_FREQ_SCAL                   ( 10)//��������ѹƵ�ʵ���ʵֵ����ϵ��
#define COM_REAL_ILOAD_RMS_SCAL                     (100)//���ص�����Чֵ����ʵֵ����ϵ����11.52V*100 = 1152
#define COM_REAL_IINDUC_RMS_SCAL                    (100)//��е�����Чֵ����ʵֵ����ϵ��


/***************************************************************************/
/*-------------------IO�˿ں궨��------------------------------------------*/
/***************************************************************************/
//LEDָʾ
#define LED_GREEN_ON                                ( PORT->PCLR7 = (1<<0))
#define LED_GREEN_OFF                               ( PORT->PSET7 = (1<<0))

//INV_�̵���1
#define INV_RY1_ENABLE				                ( PORT->PSET2 = (1<<0))
#define INV_RY1_DISABLE  			                ( PORT->PCLR2 = (1<<0))
#define INV_RY1_STATE   	                        ( (PORT->PREAD2  &(1<<0))>>0)  

//INV_�̵���3
#define INV_RY3_ENABLE				                ( PORT->PSET2 = (1<<1))
#define INV_RY3_DISABLE  			                ( PORT->PCLR2 = (1<<1))

//PFC_�̵���2
#define PFC_RY2_ENABLE				                ( PORT->PSET13 = (1<<0))
#define PFC_RY2_DISABLE  			                ( PORT->PCLR13 = (1<<0))

//����INV�����ź�ToDCDC��
#define INV_START_ENABLE			                ( PORT->PSET7 = (1<<5))
#define INV_START_DISABLE  			                ( PORT->PCLR7 = (1<<5))
#define INV_START_STATE   	                        ((PORT->PREAD7  &(1<<5))>>5)  

//����DCDC�๤���ź�
#define DCDC_WORK_STATE   	                        ((PORT->PREAD3  &(1<<1))>>1)  

//INV���ذ��������ź�
#define INV_ON_SWITCH   	                        ((PORT->PREAD6  &(1<<3))>>3)  

//ĸ�߹�ѹ�ź� To DCDC��
#define VBUS_OVER_ENABLE				            ( PORT->PSET3 = (1<<0))
/***************************************************************************/

#endif 

