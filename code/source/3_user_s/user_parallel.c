/*****************************************************************************
Copyright (C) 2021 China Micro Semiconductor Limited Company. All Rights Reserved.
@Description: 
@Author     : 
@Version    : 0.0.1  
@History    : 
@Attention  ��
*****************************************************************************/

/****************************************************************************/
/*----include files---------------------------------------------------------*/
#include "sys_mcu_header.h"
/***************************************************************************/

/*************************************************
Description: INV_MasterSlaveSelect
Input      : 
Return     : 
Others     : �������������ǰ�ⲿ�����ѹȷ�ϸû�Ϊ�������Ǵӻ�
*************************************************/
void INV_MasterSlaveSelect(void)
{
    if (SysClockBase_ms.sys_1ms == 1)
    {
        if (INV_Parall_Info.Flag.masterFlag == 0 && INV_Parall_Info.Flag.slaveFlag == 0)
        {
            if (COM_AD_Data_Info.VACOUT_RMS_Val_Fir > INV_Parall_Info.judge_Vol_RMS_Val )//�����ѹ��Чֵ����30V
            {
                INV_Parall_Info.slave_Cnt++;
            }
            else
            {
               INV_Parall_Info.master_Cnt++;
            }
  
            if (INV_Parall_Info.slave_Cnt > 500)//(ms)
            {
                INV_Parall_Info.Flag.slaveFlag = 1; //�ӻ�
                INV_Parall_Info.slave_Cnt = 0;	//��������λ
            }

            if (INV_Parall_Info.master_Cnt > 500 )   //(ms)
            {
               INV_Parall_Info.Flag.masterFlag = 1; //����
               INV_Parall_Info.Flag.selectStateOK = 1;//��ʾ��ȷ�ϱ���״̬Ϊ������ӻ�
               INV_Parall_Info.master_Cnt = 0;	//��������λ
            }
        }

        
        if ((INV_Parall_Info.Flag.slaveFlag == 1) && (INV_Parall_Info.Flag.selectStateOK == 0))	//�ӻ����ȴ�������ȫ�������������ӻ�
        {
            INV_Parall_Info.selectSlave_Cnt++;
            if (INV_Parall_Info.selectSlave_Cnt > 50)														//ȷ������ܲ�
            {
                INV_Parall_Info.selectSlave_Cnt = 0;
                if(INV_Parall_Info.Flag.syn_Freq_OK == 1)												//ȷ�Ϲ�Ƶͬ���ź���Ч
                {
                    INV_Parall_Info.Flag.selectStateOK = 1;//��ʾ��ȷ�ϱ���״̬Ϊ������ӻ�
                }
            } 

        }
    }
}

/*************************************************
Description: INV_AC_Vol_FreCalc
Input      : 
Return     : 
Others     : �Ƚ������������ѹƵ��   �����ѹ��2.5V�Ƚ��������������������Ƶ��
*************************************************/
int16_t     INV_TimeCntVal = 0;//���ڼ���ֵ
uint16_t	u16_INV_Freq_Cnt = 0;
uint8_t     INV_flag_cnt,INV_flag,INV_flag_cnt1 = 0;//�ɼ����ڱ�� ��ͬʱ����ֵ����
void  INV_AC_Vol_FreCalc(void)
{ 
    //(CMP->COMPMDR) & 0X80 == 0;//��ʾVCIN10���Ƚ���1�Ļ�׼��ѹ�����߱Ƚ���1ֹͣ���У���״̬λ0��  _00_COMP1_FLAG_REFERENCE_0
    //(CMP->COMPMDR) & 0X80 == 0X80;// ��ʾVCIN10���Ƚ���1�Ļ�׼��ѹ����״̬λֵΪ1��  _80_COMP1_FLAG_REFERENCE_1

    if((((CMP->COMPMDR) & 0X80) == 0X80) && INV_flag == 0)//�Ƚ���0�жϱ�־״ֵ̬ 	
    {               
        INV_flag_cnt1 = 0;
        INV_flag_cnt ++;
        if(INV_flag_cnt>1)
        {     
            INV_flag_cnt = 0;            
            INV_TimeCntVal = u16_INV_Freq_Cnt;
            INV_flag = 1;//�ɼ����ڱ�� ����ʼ����
            u16_INV_Freq_Cnt = 0;
        }
    }
    else if((((CMP->COMPMDR) & 0X80) == 0) && INV_flag == 1)//�Ƚ���1״ֵ̬    
    {    
        
        INV_flag_cnt = 0;
        INV_flag_cnt1 ++;
        if(INV_flag_cnt1>1)
        {     
            INV_flag_cnt1 = 0;         
            INV_flag = 0;
        }
    }
    
    INV_Parall_Info.Flag.syn_Freq_OK = 0;
    u16_INV_Freq_Cnt++;//ʱ�����   ��ʾ���ٸ�PWM����
    if (u16_INV_Freq_Cnt > INV_FREQ_TIME_MAX)
    {
        u16_INV_Freq_Cnt =INV_FREQ_TIME_MAX;
    }

    INV_Ctrl_Info.AC_Vol_Freq = INV_TimeCntVal;    

    //���趨Ƶ�ʷ�Χ�������λƵ��ͬ���ź�
    if((INV_Ctrl_Info.AC_Vol_Freq < INV_FREQ_TIME_UP_50HZ) && (INV_Ctrl_Info.AC_Vol_Freq > INV_FREQ_TIME_DN_50HZ))//
    {
        INV_Parall_Info.Flag.syn_Freq_OK = 1;//Ƶ��ͬ�����OK
    }
    else
    {
        INV_Parall_Info.Flag.syn_Freq_OK = 0; 
    }
}

/*************************************************
Description: PFC_AC_Vol_FreCalc
Input      : 
Return     : 
Others     : �Ƚ������������ѹƵ��   �����ѹ��2.5V�Ƚ��������������������Ƶ��
*************************************************/
int16_t     PFC_TimeCntVal = 0;//���ڼ���ֵ
uint16_t	PFC_Freq_Time_Cnt = 0;
uint8_t     PFC_flag,PFC_flag_cnt,PFC_flag_cnt1 = 0;//�ɼ����ڱ�� ��ͬʱ����ֵ����
extern uint16_t	u16_INV_Freq_Cnt;
void  PFC_AC_Vol_FreCalc(void)
{   
    //(CMP->COMPMDR) & 0X08 == 0;//��ʾVCIN0���Ƚ���0�Ļ�׼��ѹ�����߱Ƚ���0ֹͣ���У���״̬λ0��  _00_COMP0_FLAG_REFERENCE_0
    //(CMP->COMPMDR) & 0X08 == 0X08;// ��ʾVCIN0���Ƚ���0�Ļ�׼��ѹ����״̬λֵΪ1��  _08_COMP0_FLAG_REFERENCE_1

    if((((CMP->COMPMDR) & 0X08) == 0X08) && PFC_flag == 0)//�Ƚ���0�жϱ�־״ֵ̬ 	
    {                  
        PFC_flag_cnt1 = 0;
        PFC_flag_cnt ++;
        if(PFC_flag_cnt>1)
        {
            INV_Ctrl_Info.PWM_Freq_Init_Temp = TMC->TC;
            TMC->TCCR2 &= (uint8_t)~_01_TMC_COUNTING_START;
            TMC->TC = 0;           
            TMC->TCCR2 |= _01_TMC_COUNTING_START;
            UPS_Ctr_Info.delta_CMP_Val = u16_INV_Freq_Cnt;
            
            PFC_flag_cnt = 0;
            PFC_TimeCntVal = PFC_Freq_Time_Cnt;
            PFC_flag = 1;//�ɼ����ڱ�� ����ʼ����
            PFC_Freq_Time_Cnt = 0;           
        }
    }
    else if((((CMP->COMPMDR) & 0X08) == 0) && PFC_flag == 1)//�Ƚ���0״ֵ̬    
    {    
        PFC_flag_cnt = 0;
        PFC_flag_cnt1++;
        if(PFC_flag_cnt1>1)
        {       
            PFC_flag_cnt1 = 0; 
            PFC_flag = 0;            
        }
    }
		
    PFC_Freq_Time_Cnt++;//ʱ�����   ��ʾ���ٸ�PWM����
    if (PFC_Freq_Time_Cnt > PFC_FREQ_TIME_MAX)
    {
        PFC_Freq_Time_Cnt =PFC_FREQ_TIME_MAX;
    }

    PFC_Ctrl_Info.AC_Vol_Freq = PFC_TimeCntVal;       
}




