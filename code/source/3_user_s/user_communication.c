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
#include "user_communication.h"
#include "bat32g139.h"
#include "sys_mcu_header.h"
#include "sys_state_machine.h"
#include "sys_define_param.h"
#include "string.h"
#include "change.h"
/***************************************************************************/

uint8_t u8_SendDataDebug[9] = {0};	
/*************************************************
Description: Function_TxSendDebug
Input      : 
Return     : 
Others     : 调试串口数据处理
*************************************************/
uint8_t txBuf[24] = {0};
uint8_t u8txBuf1,u8txBuf2,u8txBuf3,u8txBuf4 ;
float num[5] = {0};
uint8_t u8_senddebug = 0;
void Function_TxSendDebug(int32_t data1,int32_t data2,int32_t data3,int32_t data4,int32_t data5)
{
    num[0] = data1;
    num[1] = data2;
    num[2] = data3;
    num[3] = data4; 
		num[4] = data5; 
    memcpy(txBuf,(uint8_t *)num,20);
	
    txBuf[20] = 0x00;
    txBuf[21] = 0x00;
    txBuf[22] = 0x80;
    txBuf[23] = 0x7f;

	
        DMAVEC->CTRL[0].DMSAR = (uint32_t)(txBuf+1);
    DMAVEC->CTRL[0].DMACT = 23;                                      //传输8个数据
    DMA->DMAEN1 |= 1<<4;   //uart1                                  //使能传输(UART1)
    SCI0->TXD0 = txBuf[0]; 		
}


/*************************************************
Description: COM_Uartx_HeartCheck
Input      : 
Return     : 
Others     : UARTx心跳监测
*************************************************/
void COM_Uartx_HeartCheck(UARTx_Var_t *uart_Info)
{
    int Heart_Value = uart_Info->heart_Val;
    
    /*------------------------------------------------------------------------------------*/
    /*---------------------UARTx接收数据通信状态位计数------------------------------------*/			
    if(uart_Info->heart_Status == 1)//心跳位为1时计数值
    {
        uart_Info->heart0_Cnt = 0;				
        if(uart_Info->heart1_Cnt <= Heart_Value)					
        {
            uart_Info->heart1_Cnt ++;
        }
    }
    else if(uart_Info->heart_Status == 0 )//心跳位为0时计数值
    {
        uart_Info->heart1_Cnt = 0;				
        if(uart_Info->heart0_Cnt <= Heart_Value)					
        {
            uart_Info->heart0_Cnt ++;
        }
    }		
        
    /*------------------------------------------------------------------------------------*/				
    /*---------------------UARTx通信心跳位逻辑判断处理------------------------------------*/	
    if(uart_Info->heart0_Cnt >= Heart_Value || uart_Info->heart1_Cnt >= Heart_Value)  //PWM中断周期为20us * 200 = 4ms
    {
        uart_Info->heart_OK_Status =0; //接收数据通信异常时Status_Heart_OK状态为 0
    }
    else 
    {
        uart_Info->heart_OK_Status = 1;//接收数据通信正常时Status_Heart_OK状态为 1
    }
}


/*************************************************
Description: crc16_modbus CRC校验 
Poly        : 0x8005
Init        : 0xFFFF
Refin       : True
Refout      : True
Xorout      : 0x0000
Others      : CRC-16/MODBUS       x16+x15+x2+1
*************************************************/
uint16_t crc16_modbus(uint8_t *data, uint16_t length, uint16_t CRC_Init)
{
    uint8_t i;

    while(length--)
    {
        CRC_Init ^= *data++;            // crc ^= *data; data++;
        for (i = 0; i < 8; ++i)
        {
            if (CRC_Init & 1)
                CRC_Init = (CRC_Init >> 1) ^ 0xA001;        // 0xA001 = reverse 0x8005
            else
                CRC_Init = (CRC_Init >> 1);
        }
    }
    return CRC_Init;
}


/*************************************************
Description: UART1_INV_DataSend
Input      : 
Return     : 
Others     : 逆变侧数据发送处理
*************************************************/  
void UART1_INV_DataSend(int32_t *tx_buf)
{
 
    //更新数据
    for(int i=0;i<((UART1_SEND_NUM>>1)-1);i++)
    {
        UART1_Info.TXD_B[i*2    ] = (uint8_t)( tx_buf[i] );  
        UART1_Info.TXD_B[i*2 + 1] = (uint8_t)( tx_buf[i] >> 8 );	 
    }

    //CRC校验码获得处理
    for(int i=0;i<(UART1_SEND_NUM-2);i++)
    {
        UART1_Info.TXD_CRC_Calc = crc16_modbus(&UART1_Info.TXD_B[i],1,UART1_Info.TXD_CRC_Init ); 
        UART1_Info.TXD_CRC_Init = UART1_Info.TXD_CRC_Calc;          	 
    }    
    UART1_Info.TXD_B[ UART1_SEND_NUM-2 ]   = (uint8_t)( UART1_Info.TXD_CRC_Calc );
    UART1_Info.TXD_B[ UART1_SEND_NUM-1 ]   = (uint8_t)( UART1_Info.TXD_CRC_Calc >> 8 );    
    UART1_Info.TXD_CRC_Init                 = 0xffff;//CRC初始值给定
        
 
	//if((SCI0->SSR01 & (_0040_SCI_UNDER_EXECUTE | _0020_SCI_VALID_STORED)) == 0)
	{
        DMAVEC->CTRL[2].DMSAR = (uint32_t)(void *)&UART1_Info.TXD_B[1];
        DMAVEC->CTRL[2].DMACT = UART1_SEND_NUM-1;                                      //传输15个数据
        DMA->DMAEN1 |= 1<<6;   //uart1_TXD                                 //使能传输(UART1)
        
		SCI0->TXD1 = UART1_Info.TXD_B[0];  //操作UART1的数据发送寄存器，把数据依次发送完成
	}
}

/*************************************************
Description: INV_DataSend
Input      : 
Return     : 
Others     : 逆变器数据发送给升压侧
*************************************************/  
uint16_t PFC_VBUS_READY_OK_Cnt = 0;
void INV_DataSend(void)
{
    //发送至DC侧的电压/功率/频率数据处理
    InvToDC_Data_Info.err_Code      = System_ProtectFlag_Info.all;//故障码
    InvToDC_Data_Info.mode_State    = COM_Ctr_Info.INV_PFC_Mode_Select;//工作模式：放电或者充电     
    InvToDC_Data_Info.VBusAD_Val    = COM_AD_Data_Info.vBus_Val_Fir;//真实母线电压值*COM_REAL_VBUS_SCAL 
    InvToDC_Data_Info.VACOUT_RMS    = COM_AD_Data_Info.VACOUT_RMS_Val_Fir;//逆变输出电压值（RMS）真实有效值*COM_REAL_VACOUT_RMS_SCAL 
    InvToDC_Data_Info.VACOUT_Freq   = COM_AD_Data_Info.VACOUT_Freq_Val_Fir;//逆变输出电压频率（Hz）真实频率*COM_REAL_VACOUT_FREQ_SCAL
    InvToDC_Data_Info.VACIN_RMS     = COM_AD_Data_Info.VACIN_RMS_Val_Fir;//市电输入电压值（RMS）真实有效值*COM_REAL_VACIN_RMS_SCAL  
    InvToDC_Data_Info.VACIN_Freq        = COM_AD_Data_Info.VACIN_Freq_Val_Fir;//市电输入电压频率（Hz）真实频率*COM_REAL_VACIN_FREQ_SCAL
    InvToDC_Data_Info.VACOUT_Power      = COM_AD_Data_Info.VACOUT_ApparentPower;//逆变输出视在功率（VA）负载电流有效值*电压有效值    
    InvToDC_Data_Info.VACIN_PFC_Power   = COM_AD_Data_Info.VACIN_PFC_Power;//市电输入PFC功率（W）----电感电流*输入电压
    InvToDC_Data_Info.VACIN_BypassPower = COM_AD_Data_Info.VACIN_BypassPower;//市电输入旁路功率（VA）负载电流有效值*电压有效值


    if(COM_Ctr_Info.INV_PFC_Mode_Select == INV_MODE)//INV模式：放电
    {
        /*------------------------------------------------------------------------------------*/			
        /*------------------发送逆变器数据给升压控制板----------------------------------------*/ 
        //逆变器状态值判断
        if((COM_AD_Data_Info.auxPower_Val_Fir > COM_AUX_POWER_LVP_VAL &&  \
            COM_AD_Data_Info.auxPower_Val_Fir < COM_AUX_POWER_OVP_VAL)&& \
            (System_ProtectFlag_Info.all == 0) ) //判断逆变工作电源在开机检测范围内 
        {         
            UART1_Info.TXD_W[6] = 0;//工作电源状态OK为1
            INV_START_ENABLE;//发送逆变准备好信号
        }
        else 
        {            
            UART1_Info.TXD_W[6] = 0;//
            INV_START_DISABLE;
        }
        
        UART1_Info.TXD_W[0] = 0x55aa;//2字节帧头
        UART1_Info.TXD_W[1] = InvToDC_Data_Info.mode_State ; //逆变工作模式或者PFC工作模式    
        UART1_Info.TXD_W[2] = InvToDC_Data_Info.err_Code;//故障代码
        UART1_Info.TXD_W[3] = InvToDC_Data_Info.VACOUT_RMS;	//逆变输出电压值（RMS）真实有效值*COM_REAL_VACOUT_RMS_SCAL
        UART1_Info.TXD_W[4] = InvToDC_Data_Info.VACOUT_Power;//逆变输出功率（W）  
        UART1_Info.TXD_W[5] = InvToDC_Data_Info.VBusAD_Val;//真实母线电压值*COM_REAL_VBUS_SCAL       
        UART1_Info.TXD_W[7] = InvToDC_Data_Info.VACOUT_Freq;//逆变输出电压频率:真实频率*COM_REAL_VACOUT_FREQ_SCAL
        UART1_Info.TXD_W[8] = InvToDC_Data_Info.VACIN_RMS;//市电输入电压值（RMS）真实有效值*COM_REAL_VACIN_RMS_SCAL
        UART1_Info.TXD_W[9] = InvToDC_Data_Info.VACIN_PFC_Power;//市电输入PFC功率（W）----电感电流*输入电压	//
        UART1_Info.TXD_W[10] = InvToDC_Data_Info.VACIN_BypassPower;//市电输入旁路功率（W）--负载电流有效值*输入电压有效值
        UART1_Info.TXD_W[11] = InvToDC_Data_Info.VACIN_Freq;//市电输入电压频率：真实频率*COM_REAL_VACIN_FREQ_SCAL   
    }
    else if(COM_Ctr_Info.INV_PFC_Mode_Select == PFC_MODE)//PFC模式：充电
    {       
        if( COM_RUN_STATE == State_Context.state_Value && \
            COM_AD_Data_Info.vBus_Val_Fir >= PFC_VBUS_READY_OK_MIN )      
        {
            if(PFC_VBUS_READY_OK_Cnt<=PFC_VBUS_READY_OK_CNT_VAL)
                PFC_VBUS_READY_OK_Cnt++;
            if(PFC_VBUS_READY_OK_Cnt>=PFC_VBUS_READY_OK_CNT_VAL)
            {
                UART1_Info.TXD_W[6] = 1;//
                INV_START_DISABLE;            
            }
        }
        else
        {
            UART1_Info.TXD_W[6] = 0;//
            INV_START_DISABLE;
            PFC_VBUS_READY_OK_Cnt = 0;
        }
        UART1_Info.TXD_W[0] = 0x55aa;//2字节帧头
        UART1_Info.TXD_W[1] = InvToDC_Data_Info.mode_State;//逆变工作模式或者PFC工作模式    
        UART1_Info.TXD_W[2] = InvToDC_Data_Info.err_Code;//故障代码
        UART1_Info.TXD_W[3] = InvToDC_Data_Info.VACOUT_RMS;//逆变输出电压值（RMS）真实有效值*COM_REAL_VACOUT_RMS_SCAL
        UART1_Info.TXD_W[4] = InvToDC_Data_Info.VACOUT_Power;//逆变输出功率（W）  
        UART1_Info.TXD_W[5] = InvToDC_Data_Info.VBusAD_Val;//真实母线电压值*COM_REAL_VBUS_SCAL
        UART1_Info.TXD_W[7] = InvToDC_Data_Info.VACOUT_Freq;//逆变输出电压频率:真实频率*COM_REAL_VACOUT_FREQ_SCAL
        UART1_Info.TXD_W[8] = InvToDC_Data_Info.VACIN_RMS;//市电输入电压值（RMS）真实有效值*COM_REAL_VACIN_RMS_SCAL
        UART1_Info.TXD_W[9] = InvToDC_Data_Info.VACIN_PFC_Power;//市电输入PFC功率（W）----电感电流*输入电压
        UART1_Info.TXD_W[10] = InvToDC_Data_Info.VACIN_BypassPower;//市电输入旁路功率（W）--负载电流有效值*输入电压有效值
        UART1_Info.TXD_W[11] = InvToDC_Data_Info.VACIN_Freq;//市电输入电压频率：真实频率*COM_REAL_VACIN_FREQ_SCAL     
    }
    else if(COM_Ctr_Info.INV_PFC_Mode_Select == FREE_MODE)//FREE模式：空闲等待模式
    {
        INV_START_DISABLE;
        
        UART1_Info.TXD_W[0] = 0x55aa;//2字节帧头    
        UART1_Info.TXD_W[1] = InvToDC_Data_Info.mode_State ; //逆变工作模式或者PFC工作模式    
        UART1_Info.TXD_W[2] = InvToDC_Data_Info.err_Code;//故障代码
        UART1_Info.TXD_W[3] = 0;	//
        UART1_Info.TXD_W[4] = 0;	//  
        UART1_Info.TXD_W[5] = InvToDC_Data_Info.VBusAD_Val;//真实母线电压值*COM_REAL_VBUS_SCAL 
        UART1_Info.TXD_W[6] = 0;	//  
        UART1_Info.TXD_W[7] = 0;	//
        UART1_Info.TXD_W[8] = 0;	//
        UART1_Info.TXD_W[9] = 0;	//
        UART1_Info.TXD_W[10] = 0;	//
        UART1_Info.TXD_W[11] = 0;	//         
    }

    //调用UART1的发送数据格式        
    UART1_INV_DataSend(UART1_Info.TXD_W);     
}


/*************************************************
Description: User_UART_View
Input      : 
Return     : 
Others     : 用于串口调试
*************************************************/

extern uint16_t	PFC_Freq_Time_Cnt;
extern uint8_t PFC_Reverse_Flag ; //过零点换向标志

extern uint8_t     PFC_flag,INV_flag;

extern const short int Sine_Table_50Hz[];
extern const short int Sine_Table_60Hz[];
extern uint16_t	u16_INV_Freq_Cnt;
extern int virtual_Res_Coeff_temp;
extern uint8_t *UART1_RXD_B_Temp;

extern int16_t PFC_StartCount;
void User_UART_View(void)
{
    /*------------------------------------------------------------------------------------*/			
    /*---------------------UART0串口调试--------------------------------------------------*/					

  //  Function_TxSendDebug(INV_PID_Cur.ref,INV_PID_Cur.fdb,INV_PID_Vol.ref,INV_PID_Vol.fdb);//
//    Function_TxSendDebug(INV_Ctrl_Info.AC_Vol_Peak,INV_Ctrl_Info.curLoad_Peak,INV_Ctrl_Info.curInduc_Peak,Sine_Table_50Hz[INV_Ctrl_Info.periodDot_Cnt]);//
    //Function_TxSendDebug(ADSample_Info.curLoad_AD,ADSample_Info.ref_AD_Fir,ADSample_Info.temp_NTC_AD_FIR,ADSample_Info.auxPower_AD_FIR);//
    //Function_TxSendDebug(ADSample_Info.curLoad_AD,ADSample_Info.curInduc_AD,ADSample_Info.INV_AC_Vol_AD,ADSample_Info.PFC_AC_Vol_AD);//

    //Function_TxSendDebug(INV_PID_Cur.ref,INV_PID_Cur.fdb,INV_PID_Cur.out,INV_PID_Cur.ui);//
    //Function_TxSendDebug(INV_PID_Vol.ref,INV_PID_Vol.fdb,INV_PID_Vol.out,INV_PID_Vol.ui);//
    
//    Function_TxSendDebug(INV_PID_Vol.ref,INV_Ctrl_Info.AC_Vol_Peak,INV_PID_Cur.ref,INV_Ctrl_Info.curInduc_Peak);//
//    Function_TxSendDebug(INV_PID_Vol.ref,INV_Ctrl_Info.AC_Vol_Peak,INV_PID_Vol.out,INV_PID_Vol.ui);//
    
//        Function_TxSendDebug(INV_PID_Vol.ref,INV_Ctrl_Info.AC_Vol_Peak,ADSample_Info.ref_AD_Fir,INV_Ctrl_Info.curInduc_Peak);//
//////    Function_TxSendDebug(INV_PID_Cur.ref,INV_Ctrl_Info.curInduc_Peak,INV_PID_Vol.ref,INV_Ctrl_Info.AC_Vol_Peak);//
////    Function_TxSendDebug(State_Context.state_Value*1000,COM_Ctr_Info.INV_PFC_Mode_Select*1000+DCDC_WORK_STATE*100 , ADSample_Info.vBus_AD_FIR ,INV_START_STATE*1000);//

    //Function_TxSendDebug(RMS_PQ_Info.out_RMS_INV_AC_Vol,INV_PID_Vol.fdb,INV_PID_Cur.ref,INV_PID_Cur.fdb);//
   
    //Function_TxSendDebug(INV_Ctrl_Info.AC_Vol_Peak,Sine_Table_50Hz[INV_Ctrl_Info.periodDot_Cnt],INV_PID_Cur.ref,INV_PID_Cur.fdb);//
    
    //Function_TxSendDebug(INV_Ctrl_Info.curInduc_Peak,INV_PID_Vol.fdb,INV_PID_DCIM.fdb,INV_PID_DCIM.out);//
    //Function_TxSendDebug(ADSample_Info.INV_AC_Vol_AD_FIR,ADSample_Info.PFC_AC_Vol_AD_FIR,RMS_PQ_Info.out_RMS_PFC_AC_Vol,INV_Ctrl_Info.periodDot_Val);//

    //Function_TxSendDebug(COM_Ctr_Info.INV_PFC_Mode_Select*1000,State_Context.state_Value*1000,ADSample_Info.PFC_AC_Vol_AD_FIR,UPS_Ctr_Info.V_ACIN_NOK*1000);//
//    Function_TxSendDebug(COM_AD_Data_Info.vRef_Val_Fir,COM_AD_Data_Info.temp_NTC_Val_Fir,COM_AD_Data_Info.auxPower_Val_Fir,COM_AD_Data_Info.vBus_Val_Fir);//
//    Function_TxSendDebug(COM_AD_Data_Info.VACIN_Freq_Val_Fir + COM_Ctr_Info.PFC_FREQ_State*1000,COM_Ctr_Info.PFC_FREQ_NOKCnt*10,COM_Ctr_Info.INV_PFC_Mode_Select*1000+PFC_Ctrl_Info.AC_Vol_Freq,State_Context.state_Value*1000+COM_Ctr_Info.PFC_FREQ_State*100);//
//    Function_TxSendDebug(COM_AD_Data_Info.iLoad_RMS_Val_Fir,COM_AD_Data_Info.iInduc_RMS_Val_Fir,COM_AD_Data_Info.VACIN_PFC_Power,COM_AD_Data_Info.VACOUT_Power);//

//       Function_TxSendDebug(COM_AD_Data_Info.VACIN_Freq_Val_Fir + UPS_Ctr_Info.lock_Phase_OK*1000,COM_Ctr_Info.PFC_AC_Vol_OK_Cnt*10,COM_Ctr_Info.INV_PFC_Mode_Select*1000+PFC_Ctrl_Info.AC_Vol_Freq,State_Context.state_Value*1000+UPS_Ctr_Info.V_ACIN_OK*100);//
//       Function_TxSendDebug(AD_Correct_V_ACOUT.Flag.bit.ADRef_Correct_Ok+AD_Correct_I_Load.Flag.bit.ADRef_Correct_Ok*1000+\
//       AD_Correct_V_ACIN.Flag.bit.ADRef_Correct_Ok*10+AD_Correct_I_Induc.Flag.bit.ADRef_Correct_Ok*100\
//       ,AD_Correct_Vref.Flag.bit.ADRef_Correct_Ok*10,UART1_Info.RXD_W[0],System_ProtectFlag_Info.all);//
//              Function_TxSendDebug(COM_AD_Data_Info.iLoad_RMS_Val_Fir,COM_AD_Data_Info.iInduc_RMS_Val_Fir,COM_AD_Data_Info.VACIN_RMS_Val_Fir,COM_AD_Data_Info.VACOUT_RMS_Val_Fir);//
    
//         Function_TxSendDebug(ADSample_Info.curLoad_AD_FIR,ADSample_Info.curInduc_AD_FIR,ADSample_Info.INV_AC_Vol_AD_FIR,COM_AD_Data_Info.VACOUT_RMS_Val_Fir);//

//         Function_TxSendDebug(UART1_Info.TXD_W[5],System_ProtectFlag_Info.all,ADSample_Info.INV_AC_Vol_AD_FIR,COM_AD_Data_Info.VACOUT_RMS_Val_Fir);//
				// Function_TxSendDebug(COM_Ctr_Info.INV_PFC_Mode_Select*100+State_Context.state_Value*1000,PFC_PID_Vol.fdb, UARTx_DC_Info.vBus_SetVal,PFC_PID_Vol.ref,ADSample_Info.INV_AC_Vol_AD_FIR);//			 
       Function_TxSendDebug_INT(123);
       Function_TxSendDebug_Float(127.01);
}
