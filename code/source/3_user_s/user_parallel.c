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
#include "sys_mcu_header.h"
/***************************************************************************/

/*************************************************
Description: INV_MasterSlaveSelect
Input      : 
Return     : 
Others     : 根据逆变器开启前外部输出电压确认该机为主机还是从机
*************************************************/
void INV_MasterSlaveSelect(void)
{
    if (SysClockBase_ms.sys_1ms == 1)
    {
        if (INV_Parall_Info.Flag.masterFlag == 0 && INV_Parall_Info.Flag.slaveFlag == 0)
        {
            if (COM_AD_Data_Info.VACOUT_RMS_Val_Fir > INV_Parall_Info.judge_Vol_RMS_Val )//输出电压有效值大于30V
            {
                INV_Parall_Info.slave_Cnt++;
            }
            else
            {
               INV_Parall_Info.master_Cnt++;
            }
  
            if (INV_Parall_Info.slave_Cnt > 500)//(ms)
            {
                INV_Parall_Info.Flag.slaveFlag = 1; //从机
                INV_Parall_Info.slave_Cnt = 0;	//计数器复位
            }

            if (INV_Parall_Info.master_Cnt > 500 )   //(ms)
            {
               INV_Parall_Info.Flag.masterFlag = 1; //主机
               INV_Parall_Info.Flag.selectStateOK = 1;//表示已确认本机状态为主机或从机
               INV_Parall_Info.master_Cnt = 0;	//计数器复位
            }
        }

        
        if ((INV_Parall_Info.Flag.slaveFlag == 1) && (INV_Parall_Info.Flag.selectStateOK == 0))	//从机，等待主机完全启动后再启动从机
        {
            INV_Parall_Info.selectSlave_Cnt++;
            if (INV_Parall_Info.selectSlave_Cnt > 50)														//确认五个周波
            {
                INV_Parall_Info.selectSlave_Cnt = 0;
                if(INV_Parall_Info.Flag.syn_Freq_OK == 1)												//确认工频同步信号有效
                {
                    INV_Parall_Info.Flag.selectStateOK = 1;//表示已确认本机状态为主机或从机
                }
            } 

        }
    }
}

/*************************************************
Description: INV_AC_Vol_FreCalc
Input      : 
Return     : 
Others     : 比较器计算输出电压频率   输出电压与2.5V比较输出连续方波，计算其频率
*************************************************/
int16_t     INV_TimeCntVal = 0;//周期计数值
uint16_t	u16_INV_Freq_Cnt = 0;
uint8_t     INV_flag_cnt,INV_flag,INV_flag_cnt1 = 0;//采集周期标记 ，同时计数值清零
void  INV_AC_Vol_FreCalc(void)
{ 
    //(CMP->COMPMDR) & 0X80 == 0;//表示VCIN10＜比较器1的基准电压，或者比较器1停止运行，该状态位0。  _00_COMP1_FLAG_REFERENCE_0
    //(CMP->COMPMDR) & 0X80 == 0X80;// 表示VCIN10＞比较器1的基准电压，该状态位值为1。  _80_COMP1_FLAG_REFERENCE_1

    if((((CMP->COMPMDR) & 0X80) == 0X80) && INV_flag == 0)//比较器0中断标志状态值 	
    {               
        INV_flag_cnt1 = 0;
        INV_flag_cnt ++;
        if(INV_flag_cnt>1)
        {     
            INV_flag_cnt = 0;            
            INV_TimeCntVal = u16_INV_Freq_Cnt;
            INV_flag = 1;//采集周期标记 ，开始计数
            u16_INV_Freq_Cnt = 0;
        }
    }
    else if((((CMP->COMPMDR) & 0X80) == 0) && INV_flag == 1)//比较器1状态值    
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
    u16_INV_Freq_Cnt++;//时间计数   表示多少个PWM周期
    if (u16_INV_Freq_Cnt > INV_FREQ_TIME_MAX)
    {
        u16_INV_Freq_Cnt =INV_FREQ_TIME_MAX;
    }

    INV_Ctrl_Info.AC_Vol_Freq = INV_TimeCntVal;    

    //在设定频率范围内则可置位频率同步信号
    if((INV_Ctrl_Info.AC_Vol_Freq < INV_FREQ_TIME_UP_50HZ) && (INV_Ctrl_Info.AC_Vol_Freq > INV_FREQ_TIME_DN_50HZ))//
    {
        INV_Parall_Info.Flag.syn_Freq_OK = 1;//频率同步标记OK
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
Others     : 比较器计算输出电压频率   输出电压与2.5V比较输出连续方波，计算其频率
*************************************************/
int16_t     PFC_TimeCntVal = 0;//周期计数值
uint16_t	PFC_Freq_Time_Cnt = 0;
uint8_t     PFC_flag,PFC_flag_cnt,PFC_flag_cnt1 = 0;//采集周期标记 ，同时计数值清零
extern uint16_t	u16_INV_Freq_Cnt;
void  PFC_AC_Vol_FreCalc(void)
{   
    //(CMP->COMPMDR) & 0X08 == 0;//表示VCIN0＜比较器0的基准电压，或者比较器0停止运行，该状态位0。  _00_COMP0_FLAG_REFERENCE_0
    //(CMP->COMPMDR) & 0X08 == 0X08;// 表示VCIN0＞比较器0的基准电压，该状态位值为1。  _08_COMP0_FLAG_REFERENCE_1

    if((((CMP->COMPMDR) & 0X08) == 0X08) && PFC_flag == 0)//比较器0中断标志状态值 	
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
            PFC_flag = 1;//采集周期标记 ，开始计数
            PFC_Freq_Time_Cnt = 0;           
        }
    }
    else if((((CMP->COMPMDR) & 0X08) == 0) && PFC_flag == 1)//比较器0状态值    
    {    
        PFC_flag_cnt = 0;
        PFC_flag_cnt1++;
        if(PFC_flag_cnt1>1)
        {       
            PFC_flag_cnt1 = 0; 
            PFC_flag = 0;            
        }
    }
		
    PFC_Freq_Time_Cnt++;//时间计数   表示多少个PWM周期
    if (PFC_Freq_Time_Cnt > PFC_FREQ_TIME_MAX)
    {
        PFC_Freq_Time_Cnt =PFC_FREQ_TIME_MAX;
    }

    PFC_Ctrl_Info.AC_Vol_Freq = PFC_TimeCntVal;       
}




