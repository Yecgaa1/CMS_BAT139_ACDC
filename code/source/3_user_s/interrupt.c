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
#include "sys_define_param.h"
#include "user_function.h"
#include "sys_state_machine.h"
#include "elc.h"
#include "change.h"
/***************************************************************************/
void IRQ27_Handler(void) __attribute__((alias("tmm0_interrupt")));
void IRQ28_Handler(void) __attribute__((alias("tmm1_interrupt")));
void IRQ14_Handler(void) __attribute__((alias("uart1_interrupt_receive")));
void IRQ21_Handler(void) __attribute__((alias("adc_interrupt")));

/*************************************************
Description: PFC_Deal
Input      : 
Return     : 
Others     : PFC���ܳ�����
*************************************************/
#define PFC_START_VALUE 500    //�趨����ֵ
uint16_t PFC_StartCount = 0;//�ڹ�����ѹ��Χ��ʱ��ʱһ�κ�ʼ����PFC����
extern uint8_t PFC_Reverse_Flag; //����㻻���־
uint16_t u16AD_Cnt = 0 ;
void PFC_Deal(void)
{     
    //PFC�������������̶��ĵ�ѹ��·refֵ(ע�⣺��Ͽ�����ѹ��)
    #if PFC_SINGLE_CTRL_SELECT == PFC_SINGLE_ENABLE 
        UARTx_DC_Info.vBus_SetVal       = PFC_SINGLE_VBUS_VAL_REF;//����PFCĸ�ߵ�ѹREF
    #endif

    //ĸ�ߵ�ѹ�ο�ֵ��������
    PFC_VBus_RefCalc();
     
    //����ѹ�ͺ������ѹ
    if( COM_Ctr_Info.INV_PFC_Mode_Select == PFC_MODE && Run_Deal.flag.bit.OpenDriver_Ok == E_TRUE)
    { 
        if(u16AD_Cnt<600)
            u16AD_Cnt++;
        if(u16AD_Cnt>=600)
            ADSample_Info.PFC_AC_Vol_AD_FIR = ADSample_Info.INV_AC_Vol_AD_FIR;        
    }
    else
    {
        u16AD_Cnt = 0;
    } 
              
    /*------------------------------------------------------------------------------------*/				
    /*---------------------�����㷨ִ�м�ռ�ձȸ���---------------------------------------*/	
    if( COM_RUN_STATE == State_Context.state_Value && Run_Deal.flag.bit.OpenDriver_Ok == E_TRUE )
    {
        //��������ֵ
        PFC_Ctrl_Info.AD_VBus           = ( ADSample_Info.vBus_AD_FIR * COM_VBUS_BASE) >> 10;   
       
        PFC_Ctrl_Info.AD_VolPeak        = ( ABSFUN(ADSample_Info.PFC_AC_Vol_AD_FIR))* COM_AC_VOL_BASE >> 10;         
//        PFC_Ctrl_Info.AD_VolPeak        =  ABSFUN(PLL_Ctrl_Info_V_ACIN.i32SinTheta*23>>9);        
      
        PFC_Ctrl_Info.AD_InducCurPeak   = ((ABSFUN(ADSample_Info.curInduc_AD_FIR))* COM_CUR_INDUC_BASE  >> 7);
        
        //���������ѹ��Чֵ�仯�����ֵ
        PFC_Ctrl_Info.AC_VolBase = PFC_Ctrl_Info.RMS_Vol  >> 1;//�����ѹ��ֵ��220V*1.414 =    1.414*128 = 180.99
        if(PFC_Ctrl_Info.AC_VolBase < 250)
        {
            PFC_Ctrl_Info.AC_VolBase = 250;
        }

        //�жϳ���Ƿ����
        if( PFC_Ctrl_Info.DCDC_CHG_State == 0)              
        {
            if( PFC_StartCount <= PFC_START_VALUE   &&\
                ADSample_Info.PFC_AC_Vol_AD_FIR<100 &&\
                ADSample_Info.PFC_AC_Vol_AD_FIR>-100 )
            {
                PFC_StartCount++;
            }
        }
        else
        {
            PFC_StartCount = 0;//��������ֵ���� 
            PFC_Ctrl_Info.PWM_Duty = 0;//ռ�ձ�Ϊ��ʱ�ر���ѹ 
                                    
            //�ر�PWM���
            TMM->TMOER1 = _01_TMM_TMIOA0_OUTPUT_DISABLE | _02_TMM_TMIOB0_OUTPUT_DISABLE | _00_TMM_TMIOC0_OUTPUT_ENABLE | _08_TMM_TMIOD0_OUTPUT_DISABLE |
              _10_TMM_TMIOA1_OUTPUT_DISABLE | _20_TMM_TMIOB1_OUTPUT_DISABLE | _40_TMM_TMIOC1_OUTPUT_DISABLE | _80_TMM_TMIOD1_OUTPUT_DISABLE;

            PORT->P1    &= ~(1 << 3);    /* P13 output low level */ 
            PORT->P1    &= ~(1 << 5);    /* P15 output low level */ 
            PORT->P1    &= ~(1 << 2);    /* P13 output low level */ 
            PORT->P1    &= ~(1 << 4);    /* P15 output low level */ 
            
            //���½����ʼ���׶�
            Initial_Deal.flag.bit.Initial_Ok        = E_FALSE;
            Ready_Deal.flag.bit.Ready_Ok            = E_FALSE;
            Run_Deal.flag.bit.OpenDriver_Ok         = E_FALSE;
            
            State_Context.flag.bit.initial_Ok       = E_FALSE;            
            State_Context.flag.bit.ready_Ok         = E_FALSE;
            PFC_Ctrl_Info.vBus_Ref_SS               = PFC_VBUS_REF_INIT_SS;//��ѹ���������ο�ֵ���� 
            PFC_Ref_Info.u32SS_vBus_Hold            = PFC_VBUS_REF_INIT_SS;  //PFC��ѹ���������ο�ֵ����             
            State_Context.state_Value               = COM_WAITING_STATE;//COM_READY_STATE;          
        }
               
        if(PFC_StartCount >= PFC_START_VALUE)
        {
            if(COM_Ctr_Info.PWM_Enable == 0)
            {
                //��PFC״̬ʱPWM��
                TMM->TMOER1 =   _01_TMM_TMIOA0_OUTPUT_DISABLE | _02_TMM_TMIOB0_OUTPUT_DISABLE | _00_TMM_TMIOC0_OUTPUT_ENABLE | _08_TMM_TMIOD0_OUTPUT_DISABLE |
                                _00_TMM_TMIOA1_OUTPUT_ENABLE | _20_TMM_TMIOB1_OUTPUT_DISABLE | _00_TMM_TMIOC1_OUTPUT_ENABLE | _80_TMM_TMIOD1_OUTPUT_DISABLE;
                COM_Ctr_Info.PWM_Enable = 1;
            }  
            //PFC���Ƴ���ִ��
            PFC_Ctrl(); 
        }
        
                   
        //ռ�ձȸ���
        if(COM_AD_Data_Info.vBus_Val < PFC_VBUS_UpLIMIT_VAL) // ĸ��С���趨ֵʱ������PFC��ѹ  
        {
           // if(PFC_Ctrl_Info.RMS_InductorCur > 50)//����ʱ����ռ�ձ�,����ë�̼��� (1A*COM_REAL_IINDUC_RMS_SCAL)
            {
                if(PFC_Ctrl_Info.PWM_Duty > (PFC_Ctrl_Info.PWM_Period-61))//�������㴦���⻬
                {
                    PFC_Ctrl_Info.PWM_Duty = (PFC_Ctrl_Info.PWM_Period-61);
                }              
            }                                 
        }
        else    //ĸ�ߴ����趨ĸ�ߵ�ѹʱ��ֱ�ӹر����� 
        {
            PFC_Ctrl_Info.PWM_Duty = PFC_Ctrl_Info.PWM_Duty_Dn;  //ռ�ձ�Ϊ��ʱ�ر���ѹ             
            
            //��ĸ�ߵ�ѹ�����趨ֵʱ���������ۻ������С���趨ֵ���ӿ���Ӧ
            if(PFC_PID_Vol.err_Integral > 0)
            {
                PFC_PID_Vol.err_Integral = PFC_PID_Vol.err_Integral>>1;//�����ۼ�ֵ��ʼ�� 
            }                
        }
        
        
        PFC_Ctrl_Info.PWM_DutyB = PFC_Ctrl_Info.PWM_Duty ;           
        if(PFC_Reverse_Flag==1)
        {
            PFC_Ctrl_Info.PWM_DutyB = PFC_Ctrl_Info.PWM_Period - PFC_Ctrl_Info.PWM_DutyB ;
        }     

        //���������Сռ�ձ�
        if(PFC_Ctrl_Info.PWM_DutyB > PFC_Ctrl_Info.PWM_Duty_Up)
        {
            PFC_Ctrl_Info.PWM_DutyB = PFC_Ctrl_Info.PWM_Duty_Up; 
        }
        if(PFC_Ctrl_Info.PWM_DutyB < PFC_Ctrl_Info.PWM_Duty_Dn)
        {
            PFC_Ctrl_Info.PWM_DutyB = PFC_Ctrl_Info.PWM_Duty_Dn; 
        }
        
        //ռ�ձȸ���
        TMM->TMGRC1 = PFC_Ctrl_Info.PWM_DutyB;//  TMGRC1�Ĵ���ΪTMGRA1�Ĵ����Ļ���Ĵ���                        
    }
}

/*************************************************
Description: INV_Deal
Input      : 
Return     : 
Others     : INV���ܳ�����
*************************************************/
void INV_Deal(void)
{
    //��������ֵ   
    INV_Ctrl_Info.vBus              =   ADSample_Info.vBus_AD_FIR;    
    INV_Ctrl_Info.AC_Vol_Peak       = - ADSample_Info.INV_AC_Vol_AD_FIR * 2 ;	
    INV_Ctrl_Info.curInduc_Peak     = - (ADSample_Info.curInduc_AD_FIR * 3>>1);	        
    
    /*------------------------------------------------------------------------------------*/				
    /*---------------------�����㷨ִ�м�ռ�ձȸ���---------------------------------------*/	
    if( COM_RUN_STATE == State_Context.state_Value && Run_Deal.flag.bit.OpenDriver_Ok == E_TRUE )
    {            
        //�����е����ദ��
        INV_Lock_Phase();  //ע�⣺�����������ֵ������INV��·����֮ǰִ��  
            
        //������
        INV_Ctrl();
         
        //ռ�ձȡ����ڸ���          
        TMM->TMGRA0 = INV_Ctrl_Info.PWM_Period;	//Set pwm period
        TMM->TMGRD0 = INV_Ctrl_Info.PWM_DutyB;//PWM2�����ռ�ձ�����   TMGRC1�Ĵ���ΪTMGRA1�Ĵ����Ļ���Ĵ���         
        TMM->TMGRC1 = INV_Ctrl_Info.PWM_Duty; //PWM2�����ռ�ձ�����   TMGRC1�Ĵ���ΪTMGRA1�Ĵ����Ļ���Ĵ���          
      
        if(COM_Ctr_Info.PWM_Enable == 0&&COM_AD_Data_Info.vBus_Val_Fir > Enable_PWM)
        {
            // //��INV״̬ʱPWM��
            // TMM->TMOER1 = _01_TMM_TMIOA0_OUTPUT_DISABLE | _00_TMM_TMIOB0_OUTPUT_ENABLE | _00_TMM_TMIOC0_OUTPUT_ENABLE | _00_TMM_TMIOD0_OUTPUT_ENABLE |
            //               _00_TMM_TMIOA1_OUTPUT_ENABLE | _20_TMM_TMIOB1_OUTPUT_DISABLE | _00_TMM_TMIOC1_OUTPUT_ENABLE | _80_TMM_TMIOD1_OUTPUT_DISABLE;	
            
            // INV_Ctrl_Info.periodDot_Cnt = 0;
            // COM_Ctr_Info.PWM_Enable     = 1;
        }
    }
}


/*************************************************
Description: tmm1_interrupt(�����������ж�)
Input      : 
Return     : 
Others     : 
*************************************************/
uint8_t  tmsr1_temp,tmier1_temp;
int User_UART_View_cnt11 = 0;
void tmm1_interrupt(void)  
{
    /*------------------------------------------------------------------------------------*/
    /*---------------------PWM�Ĵ���ֵ�ݴ�------------------------------------------------*/	
    tmsr1_temp = TMM->TMSR1;
    tmier1_temp = TMM->TMIER1;
    /*------------------------------------------------------------------------------------*/
    /*---------------------���TMM0�жϹ���-----------------------------------------------*/		 
    INTC_ClearPendingIRQ(TMM1_IRQn); /* clear INTTMM0 interrupt flag */	
    TMM->TMIER1 = 0x00U;  

    //����Ƶ�ʼ��  
    PFC_AC_Vol_FreCalc();
    

    //�е������ѹ����
    PLL_Ctrl_Info_V_ACIN.i16Dot_Cnt++;
    if(PLL_Ctrl_Info_V_ACIN.i16Dot_Cnt>=PFC_Ctrl_Info.AC_Vol_Freq)
        PLL_Ctrl_Info_V_ACIN.i16Dot_Cnt = 0;
    
    PLL_Ctrl_Info_V_ACIN.i16Valpha = -ADSample_Info.PFC_AC_Vol_AD_FIR;   
    if(PLL_Ctrl_Info_V_ACIN.i16Dot_Cnt>=(PFC_Ctrl_Info.AC_Vol_Freq>>2))
        PLL_Ctrl_Info_V_ACIN.i16cos[PLL_Ctrl_Info_V_ACIN.i16Dot_Cnt-(PFC_Ctrl_Info.AC_Vol_Freq>>2)] = PLL_Ctrl_Info_V_ACIN.i16Valpha;
    else
        PLL_Ctrl_Info_V_ACIN.i16cos[PLL_Ctrl_Info_V_ACIN.i16Dot_Cnt-(PFC_Ctrl_Info.AC_Vol_Freq>>2)+PFC_Ctrl_Info.AC_Vol_Freq] = PLL_Ctrl_Info_V_ACIN.i16Valpha;
    
    PLL_Ctrl_Info_V_ACIN.i16Vbeta = PLL_Ctrl_Info_V_ACIN.i16cos[PLL_Ctrl_Info_V_ACIN.i16Dot_Cnt];
    PLL_Ctrl(&PLL_Ctrl_Info_V_ACIN);    
    
    //ע�⣺�ڴ�֮ǰ�ĳ���λ�ò���������ִ��ʱ�䲻�ɱ仯
    if(COM_Ctr_Info.INV_PFC_Mode_Select == INV_MODE)//INVģʽ���ŵ�
    {         
        INV_Deal();
    }
    else if(COM_Ctr_Info.INV_PFC_Mode_Select == PFC_MODE)//PFCģʽ�����
    {	        
        PFC_Deal();    
    } 
       
     //���Ƶ�ʼ��  
    INV_AC_Vol_FreCalc(); 

    //�е��Ƿ�OK�߼�����
    COM_UPS_Deal();

    if(COM_Ctr_Info.INV_PFC_Mode_Select == INV_MODE)//
    {
        //�����ѹ����
        PLL_Ctrl_Info_V_ACOUT.i16Valpha = ADSample_Info.INV_AC_Vol_AD_FIR;    
        if(INV_Ctrl_Info.periodDot_Cnt>=(INV_Ctrl_Info.periodDot_Val>>2))
        {
            PLL_Ctrl_Info_V_ACOUT.i16cos[INV_Ctrl_Info.periodDot_Cnt-(INV_Ctrl_Info.periodDot_Val>>2)] = PLL_Ctrl_Info_V_ACOUT.i16Valpha;
        }
        else
        {
            PLL_Ctrl_Info_V_ACOUT.i16cos[INV_Ctrl_Info.periodDot_Cnt-(INV_Ctrl_Info.periodDot_Val>>2)+INV_Ctrl_Info.periodDot_Val] = PLL_Ctrl_Info_V_ACOUT.i16Valpha;
        }
        PLL_Ctrl_Info_V_ACOUT.i16Vbeta = PLL_Ctrl_Info_V_ACOUT.i16cos[INV_Ctrl_Info.periodDot_Cnt];

        PLL_Ctrl(&PLL_Ctrl_Info_V_ACOUT);            
    }      
    
    /*------------------------------------------------------------------------------------*/		
    /*-------------------------���ʡ���Чֵ����-------------------------------------------*/     
    if( COM_RUN_STATE == State_Context.state_Value && Run_Deal.flag.bit.OpenDriver_Ok == E_TRUE )
    {   
        RMS_PQ_Info.INV_AC_Vol_Peak     = ADSample_Info.INV_AC_Vol_AD_FIR;
    }
    else
    {
        RMS_PQ_Info.out_RMS_INV_AC_Vol_Hold = 0;
        RMS_PQ_Info.sum_INV_AC_Vol          = 0;
        RMS_PQ_Info.INV_AC_Vol_Peak         = 0;  
    }
    RMS_PQ_Info.PFC_AC_Vol_Peak     = ADSample_Info.PFC_AC_Vol_AD_FIR;     
    RMS_PQ_Info.curLoad_Peak        = -ADSample_Info.curLoad_AD_FIR;   
    RMS_PQ_Info.curInduc_Peak       = ADSample_Info.curInduc_AD_FIR;     
    RMS_PQ_Calc(&RMS_PQ_Info);
    

    /*------------------------------------------------------------------------------------*/			
    /*------------------------���Ҳ�����ֵ����----------------------------------------*/
    INV_Ctrl_Info.periodDot_Cnt++;
    if( INV_Ctrl_Info.periodDot_Cnt >= INV_Ctrl_Info.periodDot_Val )
    {
        INV_Ctrl_Info.periodDot_Cnt     = 0;	
    }	
    
 
    /*------------------------------------------------------------------------------------*/			
    /*------------------------UARTx���ڵ���-----------------------------------------------*/
    User_UART_View_cnt11++;    
    if(User_UART_View_cnt11>22)   
    {    
       User_UART_View();
        User_UART_View_cnt11 = 0;
    }   

    /*-------------------------�����Ӧ��־λ��ʹ���ж�-----------------------------------*/			
//    TMM->TMSR1 = tmsr1_temp & (uint8_t)~_01_TMM1_INTA_GENERATE_FLAG;//�����TMGRA0ƥ��������жϱ�־λ
    TMM->TMSR1 = 0;//�����־λ
    TMM->TMIER1 = tmier1_temp; 
}


/*************************************************
Description: adc_interrupt(ADC�ж�)
Input      : 
Return     : 
Others     : 
*************************************************/
void adc_interrupt(void)
{
    INTC_ClearPendingIRQ(ADC_IRQn);     /* clear INTCMP0 interrupt flag */

    if(ADSampleDMA_Info.u8Flag == 0)
    {
        //ELC_Start(_0C_ELC_EVENT_TMUDF1, _01_ELC_EVENT_LINK_AD);		//TMM1�����紥��ADC����
        *(&ELC->ELSELR00 + _0C_ELC_EVENT_TMUDF1) = _01_ELC_EVENT_LINK_AD;            
                
        ADSample_Info.curLoad_AD            = ADSampleDMA_Info.u16Buff[0];
        ADSample_Info.ref_AD_Fir            = ADSampleDMA_Info.u16Buff[1];
        ADSample_Info.temp_NTC_AD_FIR       = ADSampleDMA_Info.u16Buff[2];
        ADSample_Info.auxPower_AD_FIR       = ADSampleDMA_Info.u16Buff[3];     

        if ( E_FALSE == AD_Correct_Vref.Flag.bit.ADRef_Correct_Ok )
        {
            AD_Correct_Vref.sum_Cnt++;
            AD_Correct_Vref.sum += ADSample_Info.ref_AD_Fir;
            if ( AD_Correct_Vref.sum_Cnt >= 1024 )
            {
                AD_Correct_Vref.out_Val = AD_Correct_Vref.sum >> 10;
                AD_Correct_Vref.sum_Cnt = 0;
                AD_Correct_Vref.sum     = 0;
                
                if (( AD_Correct_Vref.out_Val < AD_Correct_Vref.up_Limit   ) &&\
                    ( AD_Correct_Vref.out_Val > AD_Correct_Vref.dn_Limit ))
                {
                    AD_Correct_Vref.Flag.bit.ADRef_Correct_Ok = E_TRUE;
                }
            }
        }	
        if ( E_FALSE == AD_Correct_I_Load.Flag.bit.ADRef_Correct_Ok )
        {
            AD_Correct_I_Load.sum_Cnt++;
            AD_Correct_I_Load.sum += ADSample_Info.curLoad_AD;				
            if ( AD_Correct_I_Load.sum_Cnt >= 128 )
            {
                AD_Correct_I_Load.out_Val   = AD_Correct_I_Load.sum >> 7;
                AD_Correct_I_Load.sum_Cnt = 0;
                AD_Correct_I_Load.sum     = 0;
                

                if (( AD_Correct_I_Load.out_Val < AD_Correct_I_Load.up_Limit   ) && \
                    ( AD_Correct_I_Load.out_Val > AD_Correct_I_Load.dn_Limit ))
                {
                    AD_Correct_I_Load.Flag.bit.ADRef_Correct_Ok = E_TRUE;
                }	
            }        
        }    
        ADSampleDMA_Info.u8Flag = 1;
        ADC->ADS = _07_AD_INPUT_CHANNEL_7;  // 
        DMAVEC->CTRL[5].DMACT = 4;
        DMAVEC->CTRL[5].DMDAR = (uint32_t)(void *)&ADSampleDMA_Info.u16Buff[0];
        
        DMA->DMAEN1 |= 1 << 2;//DMAʹ�� 
    }
    else
    {
        //ELC_Start(_08_ELC_EVENT_TMIMFA0, _01_ELC_EVENT_LINK_AD);		//TMM0�����紥��ADC������
        *(&ELC->ELSELR00 + _08_ELC_EVENT_TMIMFA0) = _01_ELC_EVENT_LINK_AD;            
        ADSample_Info.vBus_AD_FIR   = ADSampleDMA_Info.u16Buff[0];
        ADSample_Info.curInduc_AD   = ADSampleDMA_Info.u16Buff[1];  
        ADSample_Info.INV_AC_Vol_AD = ADSampleDMA_Info.u16Buff[2];
        ADSample_Info.PFC_AC_Vol_AD = ADSampleDMA_Info.u16Buff[3];
            
        //ƫ��ֵУ��
        if ( E_FALSE == AD_Correct_I_Induc.Flag.bit.ADRef_Correct_Ok )
        {
            AD_Correct_I_Induc.sum_Cnt++;
            AD_Correct_I_Induc.sum += ADSample_Info.curInduc_AD;				
            if ( AD_Correct_I_Induc.sum_Cnt >= 128 )
            {
                AD_Correct_I_Induc.out_Val   = AD_Correct_I_Induc.sum >> 7;
                AD_Correct_I_Induc.sum_Cnt = 0;
                AD_Correct_I_Induc.sum     = 0;
                                    
                if (( AD_Correct_I_Induc.out_Val < AD_Correct_I_Induc.up_Limit   ) && \
                    ( AD_Correct_I_Induc.out_Val > AD_Correct_I_Induc.dn_Limit ))
                {
                    AD_Correct_I_Induc.Flag.bit.ADRef_Correct_Ok = E_TRUE;
                }	
            }
        }

        if ( E_FALSE == AD_Correct_V_ACIN.Flag.bit.ADRef_Correct_Ok )
        {
            AD_Correct_V_ACIN.sum_Cnt++;
            AD_Correct_V_ACIN.sum += ADSample_Info.PFC_AC_Vol_AD;
            if ( AD_Correct_V_ACIN.sum_Cnt >= 768 )
            {
                AD_Correct_V_ACIN.out_Val = AD_Correct_V_ACIN.sum*1365 >> 20;//δ����Դ����ֱ�����е�ʱ���е����У׼����
                AD_Correct_V_ACIN.sum_Cnt = 0;
                AD_Correct_V_ACIN.sum     = 0;
                
                if (( AD_Correct_V_ACIN.out_Val < AD_Correct_V_ACIN.up_Limit   ) &&\
                    ( AD_Correct_V_ACIN.out_Val > AD_Correct_V_ACIN.dn_Limit ))
                {
                    AD_Correct_V_ACIN.Flag.bit.ADRef_Correct_Ok = E_TRUE;
                }
            }
        }

        if ( E_FALSE == AD_Correct_V_ACOUT.Flag.bit.ADRef_Correct_Ok )
        {
            AD_Correct_V_ACOUT.sum_Cnt++;
            AD_Correct_V_ACOUT.sum += ADSample_Info.INV_AC_Vol_AD;
            if ( AD_Correct_V_ACOUT.sum_Cnt >= 128 )
            {
                AD_Correct_V_ACOUT.out_Val = AD_Correct_V_ACOUT.sum >> 7;
                AD_Correct_V_ACOUT.sum_Cnt = 0;
                AD_Correct_V_ACOUT.sum     = 0;
                
                if (( AD_Correct_V_ACOUT.out_Val < AD_Correct_V_ACOUT.up_Limit   ) &&\
                    ( AD_Correct_V_ACOUT.out_Val > AD_Correct_V_ACOUT.dn_Limit ))
                {
                    AD_Correct_V_ACOUT.Flag.bit.ADRef_Correct_Ok = E_TRUE;
                }
            }
        }            

        //ĸ�ߵ�ѹ���ݴ���	   
//        ADSample_Info.vBus_Hold = DFILTER(4,ADSample_Info.vBus_AD,ADSample_Info.vBus_Hold);
//        ADSample_Info.vBus_AD_FIR = ADSample_Info.vBus_Hold >> 16;	 

        //���ص������ݴ���
        ADSample_Info.curLoad_AD_FIR = ADSample_Info.curLoad_AD - AD_Correct_Vref.out_Val; //��ȥƫ�õ�ѹ
        
        //�����ѹ���ݴ���	
//        ADSample_Info.INV_AC_Vol_Hold = ADSample_Info.INV_AC_Vol_AD - AD_Correct_Vref.out_Val;//��ȥƫ�õ�ѹ   
//        ADSample_Info.INV_AC_Vol_AD_FIR = (ADSample_Info.INV_AC_Vol_AD_FIR * 1596 + ADSample_Info.INV_AC_Vol_Hold * 2500)>> 12;	
        ADSample_Info.INV_AC_Vol_AD_FIR = ADSample_Info.INV_AC_Vol_AD - AD_Correct_Vref.out_Val;//��ȥƫ�õ�ѹ   
        
        //PFC�����ѹ���ݴ���	
//        ADSample_Info.PFC_AC_Vol_Hold = (ADSample_Info.PFC_AC_Vol_AD - AD_Correct_Vref.out_Val);//��ȥƫ�õ�ѹ
//        ADSample_Info.PFC_AC_Vol_AD_FIR = (ADSample_Info.PFC_AC_Vol_AD_FIR * 1596 + ADSample_Info.PFC_AC_Vol_Hold * 2500)>> 12;	
        ADSample_Info.PFC_AC_Vol_AD_FIR = (ADSample_Info.PFC_AC_Vol_AD - AD_Correct_Vref.out_Val);//��ȥƫ�õ�ѹ

        //��е������ݴ���
//        ADSample_Info.curInduc_Hold = ADSample_Info.curInduc_AD - AD_Correct_Vref.out_Val;//AD_Correct_I_Induc.out_Val; //��ȥƫ�õ�ѹ
//        ADSample_Info.curInduc_AD_FIR = (ADSample_Info.curInduc_AD_FIR * 1593 >> 12) + (ADSample_Info.curInduc_Hold * 2503 >> 12);
        ADSample_Info.curInduc_AD_FIR = ADSample_Info.curInduc_AD - AD_Correct_Vref.out_Val;//AD_Correct_I_Induc.out_Val; //��ȥƫ�õ�ѹ


        ADSampleDMA_Info.u8Flag = 0;
        ADC->ADS = _03_AD_INPUT_CHANNEL_3; // 4ͨ��ɨ��ģʽʱ��ѡ��AN3/AN4/AN5/AN6ͨ��
        DMAVEC->CTRL[5].DMACT = 4;
        DMAVEC->CTRL[5].DMDAR = (uint32_t)(void *)&ADSampleDMA_Info.u16Buff[0];            
        DMA->DMAEN1 |= 1 << 2;//DMAʹ��   
    }
}


/*************************************************
Description: SysTick_Handler
Input      : 
Return     : 
Others     : ϵͳ��ʱ���жϷ�����
*************************************************/
int count_err = 0;
void SysTick_Handler(void)
{
    SysClockBase_ms.sys_1ms = 1;  
    SysClockBase_ms.LED_1ms = 1;	     
    SysClockBase_ms.faultCheck_1ms = 1;//���ϼ��1ms ʱ�ӱ��  
    SysClockBase_ms.waitCheck_1ms = 1;    
    SysClockBase_ms.readyCheck_1ms = 1;     
    SysClockBase_ms.sys_Mode_1ms = 1;

    if(UART1_Info.TXD_Period_Cnt<=UART1_SEND_PERIOD_VAL)
        UART1_Info.TXD_Period_Cnt++;
  
    //DCDC�ഫ������߼�����
    if( UARTx_DC_Info.err_Code != 0 && \
        System_ProtectFlag_Info.all == 0  &&\
        INV_Ctrl_Info.mode_Operate == NORMAL_MODE)
    {          
        count_err++;
    }
    else if(count_err > 0 )
    {
        count_err --;
    }
    if( count_err > 20 )
    {
        if(COM_Ctr_Info.DC_ERR_CODE ==0)
        COM_Ctr_Info.DC_ERR_CODE = UARTx_DC_Info.err_Code;
        
        System_ProtectFlag_Info.all |= E_DCDC_ERR;
    }
    
    
   //ǰ������Ƿ����
   if( UARTx_DC_Info.CHG_FinishState == 1)//��ֹ����ж�
   {           
        PFC_Ctrl_Info.DCDC_CHG_State = 1;   
   }
   else if( UARTx_DC_Info.CHG_FinishState == 0)//��Ҫ����ж�
   {         
        PFC_Ctrl_Info.DCDC_CHG_State = 0;
   }    
    
    
    //����̬�ж����һֱû��������ö�·
    if(COM_Ctr_Info.INV_PFC_Mode_Select == INV_MODE)//INVģʽ���ŵ�
    {
        if( (COM_RUN_STATE == State_Context.state_Value && \
            Run_Deal.flag.bit.OpenDriver_Ok == E_TRUE   &&\
            INV_Ctrl_Info.short_Start_Cnt <= INV_Ctrl_Info.short_Start_Time_Val)) //��������̬�趨ʱ��ǿ�ƿ���
        {
            INV_Ctrl_Info.short_Start_Cnt ++;
        }
    }
}

/*************************************************
Description: HardFault_Handler
Input      : 
Return     : 
Others     : Ӳ�����жϷ�����
*************************************************/
void HardFault_Handler(void)
{
    //�ر�PWM
    INV_RY1_DISABLE;
    PFC_RY2_DISABLE;
    INV_RY3_DISABLE;        
    
    COM_PWM_Disable();	//�ر�PWM��� 
    State_Context.state_Value = COM_FAULT_STATE;
}



/*************************************************
Description: uart1_interrupt_receive
Input      : 
Return     : 
Others     : uart1�����жϣ�DCDC�����ݽ���
*************************************************/
void uart1_interrupt_receive(void)
{
    volatile uint8_t rx_data;
    volatile uint8_t err_type;
		
    INTC_ClearPendingIRQ(SR1_IRQn);//������жϱ�־λ
    err_type    = (uint8_t)(SCI0->SSR03 & 0x0007U);
    SCI0->SIR03 = (uint16_t)err_type;	
}
/*************************************************
Description: tmm0_interrupt(�������ڵ�����ж�)
Input      : 
Return     : 
Others     : 
*************************************************/
void tmm0_interrupt(void)
{
    uint8_t tmsr0_temp,tmier0_temp;
    /*------------------------------------------------------------------------------------*/
    /*---------------------PWM�Ĵ���ֵ�ݴ�------------------------------------------------*/	
    tmsr0_temp = TMM->TMSR0;
    tmier0_temp = TMM->TMIER0;

    /*------------------------------------------------------------------------------------*/
    /*---------------------���TMM0�жϹ���-----------------------------------------------*/		 
    INTC_ClearPendingIRQ(TMM0_IRQn); /* clear INTTMM0 interrupt flag */	
    TMM->TMIER0 = 0x00U;  
        
    /*------------------------------------------------------------------------------------*/		
    /*------------------------�����Ӧ��־λ��ʹ���ж�------------------------------------*/			
    TMM->TMSR0 = tmsr0_temp & (uint8_t)~_01_TMM0_INTA_GENERATE_FLAG;//�����TMGRA0ƥ��������жϱ�־λ
    TMM->TMSR0 = 0;//�����־λ
    TMM->TMIER0 = tmier0_temp;
}

