/***********************************************************************************************************************
* Copyright (C) All rights reserved.
***********************************************************************************************************************/

/***********************************************************************************************************************
* @file    adc_user.c
* @brief   This file implements device driver for ADC module.
* @version 1.0.0 
* @date    2019/12/24
***********************************************************************************************************************/

/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "BAT32G139.h"
#include "adc.h"
/* Start user code for include. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
#include "userdefine.h"
#include "sys_mcu_header.h"
#include "elc.h"
#include "sys_state_machine.h"
#include "sys_mcu_header.h"
/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/
//void IRQ21_Handler(void) __attribute__((alias("adc_interrupt")));

/* Start user code for pragma. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */
volatile uint8_t  * gp_u1_adc_buf;           /* adc higher 8 bit result buffer address */
volatile uint16_t * gp_u2_adc_buf;           /* adc 12 bit result buffer address */
volatile uint32_t   g_AdcIntTaken;           /* adc interrupt flag */
volatile int16_t    g_temperature = 25;        /* chip temperature */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: adc_interrupt
* @brief  ADC interrupt service routine
* @param  None
* @return None
***********************************************************************************************************************/
//uint8_t u8ADC_flag = 0;
//extern int ad_ad_cnt ;
//uint16_t u16ADC_buff[4];
//void adc_interrupt(void)
//{
//  //  LED_GREEN_ON;

//    INTC_ClearPendingIRQ(ADC_IRQn);     /* clear INTCMP0 interrupt flag */

//        if(u8ADC_flag == 0)
//        {
//            ELC_Start(_0C_ELC_EVENT_TMUDF1, _01_ELC_EVENT_LINK_AD);		//TMM1的下溢触发ADC采样
//            		
//            ADSample_Info.curLoad_AD            = u16ADC_buff[0];
//            ADSample_Info.ref_AD_Fir            = u16ADC_buff[1];
//            ADSample_Info.temp_NTC_AD_FIR       = u16ADC_buff[2];
//            ADSample_Info.auxPower_AD_FIR       = u16ADC_buff[3];     

//            if ( E_FALSE == AD_Correct_Vref.Flag.bit.ADRef_Correct_Ok )
//            {
//                AD_Correct_Vref.sum_Cnt++;
//                AD_Correct_Vref.sum += ADSample_Info.ref_AD_Fir;
//                if ( AD_Correct_Vref.sum_Cnt >= 1024 )
//                {
//                    AD_Correct_Vref.out_Val = AD_Correct_Vref.sum >> 10;
//                    AD_Correct_Vref.sum_Cnt = 0;
//                    AD_Correct_Vref.sum     = 0;
//                    
//                    if (( AD_Correct_Vref.out_Val < AD_Correct_Vref.up_Limit   ) &&\
//                        ( AD_Correct_Vref.out_Val > AD_Correct_Vref.dn_Limit ))
//                    {
//                        AD_Correct_Vref.Flag.bit.ADRef_Correct_Ok = E_TRUE;
//                    }
//                }
//            }	
//            if ( E_FALSE == AD_Correct_I_Load.Flag.bit.ADRef_Correct_Ok )
//            {
//                AD_Correct_I_Load.sum_Cnt++;
//                AD_Correct_I_Load.sum += ADSample_Info.curLoad_AD;				
//                if ( AD_Correct_I_Load.sum_Cnt >= 128 )
//                {
//                    AD_Correct_I_Load.out_Val   = AD_Correct_I_Load.sum >> 7;
//                    AD_Correct_I_Load.sum_Cnt = 0;
//                    AD_Correct_I_Load.sum     = 0;
//                    

//                    if (( AD_Correct_I_Load.out_Val < AD_Correct_I_Load.up_Limit   ) && \
//                        ( AD_Correct_I_Load.out_Val > AD_Correct_I_Load.dn_Limit ))
//                    {
//                        AD_Correct_I_Load.Flag.bit.ADRef_Correct_Ok = E_TRUE;
//                    }	
//                }        
//            }    
//            u8ADC_flag = 1;
//            ADC->ADS = _07_AD_INPUT_CHANNEL_7;  // 2通道扫描模式时：选择AN3和AN4通道
//            DMAVEC->CTRL[5].DMACT = 4;
//            DMAVEC->CTRL[5].DMDAR = (uint32_t)(void *)&u16ADC_buff[0];
//            
//            DMA->DMAEN1 |= 1 << 2;//DMA使能 
//        }
//        else
//        {
//            ELC_Start(_08_ELC_EVENT_TMIMFA0, _01_ELC_EVENT_LINK_AD);		//TMM0的上溢触发ADC采样；
//            
//            ADSample_Info.vBus_AD       = u16ADC_buff[0];
//            //母线电压数据处理		
//            ADSample_Info.vBus_Hold = DFILTER(1,ADSample_Info.vBus_AD,ADSample_Info.vBus_Hold);
//            ADSample_Info.vBus_AD_FIR = ADSample_Info.vBus_Hold >> 16;	
//            
//            ADSample_Info.curInduc_AD   = u16ADC_buff[1];  
//            ADSample_Info.INV_AC_Vol_AD = u16ADC_buff[2];
//            ADSample_Info.PFC_AC_Vol_AD = u16ADC_buff[3];
//            
//            
//            //PFC输入电压数据处理	
//            ADSample_Info.PFC_AC_Vol_Hold = (ADSample_Info.PFC_AC_Vol_AD - AD_Correct_Vref.out_Val);//减去偏置电压
//            ADSample_Info.PFC_AC_Vol_AD_FIR = (ADSample_Info.PFC_AC_Vol_AD_FIR * 1596 + ADSample_Info.PFC_AC_Vol_Hold * 2500)>> 12;	


//            //输出电压数据处理	
//            ADSample_Info.INV_AC_Vol_Hold = ADSample_Info.INV_AC_Vol_AD - AD_Correct_Vref.out_Val;//减去偏置电压   
//            ADSample_Info.INV_AC_Vol_AD_FIR = (ADSample_Info.INV_AC_Vol_AD_FIR * 1596 + ADSample_Info.INV_AC_Vol_Hold * 2500)>> 12;	

//            //电感电流数据处理
//            ADSample_Info.curInduc_Hold = ADSample_Info.curInduc_AD - AD_Correct_I_Induc.out_Val; //减去偏置电压
//            ADSample_Info.curInduc_AD_FIR = (ADSample_Info.curInduc_AD_FIR * 593 >> 12) + (ADSample_Info.curInduc_Hold * 3503 >> 12);

// 
//        //逆变电压滞后输入电压
//        if( COM_Ctr_Info.INV_PFC_Mode_Select == PFC_MODE && Run_Deal.flag.bit.OpenDriver_Ok == E_TRUE)
//        //if( COM_Ctr_Info.INV_PFC_Mode_Select == PFC_MODE )
//        { 
//            if(ad_ad_cnt<600)
//                ad_ad_cnt++;
//            if(ad_ad_cnt>=600)
//                ADSample_Info.PFC_AC_Vol_AD_FIR = ADSample_Info.INV_AC_Vol_AD_FIR;        
//        }
//        else
//        {
//            ad_ad_cnt = 0;
//        } 
//        
//            //偏置值校正
//            if ( E_FALSE == AD_Correct_I_Induc.Flag.bit.ADRef_Correct_Ok )
//            {
//                AD_Correct_I_Induc.sum_Cnt++;
//                AD_Correct_I_Induc.sum += ADSample_Info.curInduc_AD;				
//                if ( AD_Correct_I_Induc.sum_Cnt >= 128 )
//                {
//                    AD_Correct_I_Induc.out_Val   = AD_Correct_I_Induc.sum >> 7;
//                    AD_Correct_I_Induc.sum_Cnt = 0;
//                    AD_Correct_I_Induc.sum     = 0;
//                    

//                    if (( AD_Correct_I_Induc.out_Val < AD_Correct_I_Induc.up_Limit   ) || \
//                        ( AD_Correct_I_Induc.out_Val > AD_Correct_I_Induc.dn_Limit ))
//                    {
//                        AD_Correct_I_Induc.Flag.bit.ADRef_Correct_Ok = E_TRUE;
//                    }	
//                }
//            }

//            if ( E_FALSE == AD_Correct_V_ACIN.Flag.bit.ADRef_Correct_Ok )
//            {
//                AD_Correct_V_ACIN.sum_Cnt++;
//                AD_Correct_V_ACIN.sum += ADSample_Info.PFC_AC_Vol_AD;
//                if ( AD_Correct_V_ACIN.sum_Cnt >= 768 )
//                {
//                    AD_Correct_V_ACIN.out_Val = AD_Correct_V_ACIN.sum*1365 >> 20;//未开电源锁，直接上市电时，市电采样校准处理
//                    AD_Correct_V_ACIN.sum_Cnt = 0;
//                    AD_Correct_V_ACIN.sum     = 0;
//                    
//                    if (( AD_Correct_V_ACIN.out_Val < AD_Correct_V_ACIN.up_Limit   ) &&\
//                        ( AD_Correct_V_ACIN.out_Val > AD_Correct_V_ACIN.dn_Limit ))
//                    {
//                        AD_Correct_V_ACIN.Flag.bit.ADRef_Correct_Ok = E_TRUE;
//                    }
//                }
//            }

//  
//    
//            if ( E_FALSE == AD_Correct_V_ACOUT.Flag.bit.ADRef_Correct_Ok )
//            {
//                AD_Correct_V_ACOUT.sum_Cnt++;
//                AD_Correct_V_ACOUT.sum += ADSample_Info.INV_AC_Vol_AD;
//                if ( AD_Correct_V_ACOUT.sum_Cnt >= 128 )
//                {
//                    AD_Correct_V_ACOUT.out_Val = AD_Correct_V_ACOUT.sum >> 7;
//                    AD_Correct_V_ACOUT.sum_Cnt = 0;
//                    AD_Correct_V_ACOUT.sum     = 0;
//                    
//                    if (( AD_Correct_V_ACOUT.out_Val < AD_Correct_V_ACOUT.up_Limit   ) &&\
//                        ( AD_Correct_V_ACOUT.out_Val > AD_Correct_V_ACOUT.dn_Limit ))
//                    {
//                        AD_Correct_V_ACOUT.Flag.bit.ADRef_Correct_Ok = E_TRUE;
//                    }
//                }
//            }            
//            
//            u8ADC_flag = 0;
//            ADC->ADS = _03_AD_INPUT_CHANNEL_3; // 2通道扫描模式时：选择AN3和AN4通道
//            DMAVEC->CTRL[5].DMACT = 4;
//            DMAVEC->CTRL[5].DMDAR = (uint32_t)(void *)&u16ADC_buff[0];            
//            DMA->DMAEN1 |= 1 << 2;//DMA使能         
//        }
// 
//    //LED_GREEN_OFF;

//}

/***********************************************************************************************************************
* Function Name: adc_get_temperature
* @brief  ADC Get temperature
* @param  dat - the adc conversion value of ADC_TEMPERSENSOR0 channel
* @return temperature
***********************************************************************************************************************/
#if 0
double adc_get_temperature(int16_t dat)
{
    double temp;    /* temperature value */

    temp = (double)(TSN->TSN25 - dat) * 60 / (TSN->TSN25 - TSN->TSN85) + 25; /* 12bit dat */

    return (temp);
}
#else
int16_t adc_get_temperature(int16_t dat)
{
    int16_t temp;   /* temperature value */

    temp = (int16_t)(TSN->TSN25 - dat) * 60 / (TSN->TSN25 - TSN->TSN85) + 25; /* 12bit dat */

    return (temp);
}
#endif

/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
