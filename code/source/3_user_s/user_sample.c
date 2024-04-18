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
#include "sys_state_machine.h"
#include "sys_mcu_header.h"
/***************************************************************************/
        
        
/*************************************************
Description: AD_CorrectInit
Input      : 
Return     : 
Others     : ����У׼
*************************************************/
void AD_CorrectInit(void) 
{ 
	/*
	(1)����ADCʱ��
	*/
    CGC->PER0 |= CGC_PER0_ADCEN_Msk;    /* enables input clock supply */
    ADC->ADM0  = 0x00U;                 /* disable AD conversion and clear ADM0 register */
	
	/*
	(2)����ADCͨ��ʹ��
	*/	
    #if 1	  
        PORT->PMC2 |= (1 << 3);   /* Set ANI3 (P23)  pin */   //���ص�������    
        PORT->PMC2 |= (1 << 4);   /* Set ANI4 (P24)  pin */   //2.5V ref����   
        PORT->PMC2 |= (1 << 5);   /* Set ANI5 (P25)  pin */   //�¶Ȳ���    
        PORT->PMC2 |= (1 << 6);   /* Set ANI6 (P26)  pin */   //������Դ����    
        PORT->PMC2 |= (1 << 7);   /* Set ANI7 (P27)  pin */   //BUS��ѹ����
        PORT->PMC1 |= (1 << 1);   /* Set ANI8 (P11)  pin */   //��е�������          
        PORT->PMC1 |= (1 << 0);   /* Set ANI9 (P10)  pin */   //��������ѹ����
        PORT->PMC0 |= (1 << 1);   /* Set ANI10 (P01)  pin */   //UPS�����ѹ����    
    
    #else 
            ADC_PORT_SETTING();//����ANI00 - ANI15ͨ��
    #endif
	/*
	(3)����ADC����ģʽ
	*/	
    /* AD operation mode: select or scan mode  */
    //ADC->ADM0 = _10_AD_CONVERSION_CLOCK_8 | _00_AD_COMPARATOR_DISABLE;
    ADC->ADM0 = _20_AD_CONVERSION_CLOCK_2 | _00_AD_COMPARATOR_DISABLE;    //Fclk /1  ת��ʱ��Ƶ�� ;ת��ʱ��Ϊ45/Fclk

    /* AD conversion mode setting */
    ADC->ADM1 = _00_AD_HISPEED;  //���ٱ任ģʽ
		
    //����ʱ��������Ϊ13.5 * Fclk
    ADC->ADNSMP = _0D_AD_CHARGE_DISCHARGE_13CLK;   //����ʱ��������Ϊ13.5 * Fclk
		
    //ADת��ͨ��ģʽ����		   
        ADC->ADM1 |= _00_AD_OPERMODE_SELECT;  //ADת��ͨ������Ϊѡ��ģʽ

    // ADת��ģʽ����
        ADC->ADM1 |= _08_AD_CONVMODE_ONESHOT;  // ADת��ģʽ����Ϊ����ת��ģʽ

    // AD�ο���ѹ����
    /* AD reference voltage setting */
    ADC->ADM2 = _00_AD_POSITIVE_VDD | _00_AD_NEGATIVE_VSS | _00_AD_AREA_MODE_1 ;

    // AD�ο���ѹ����
    /* AD trigger selection */
    ADC->ADTRG = _00_AD_TRIGGER_SOFTWARE;   //�������ģʽ



    // AD������ֵ����	
    /* AD comversion result comprision upper limit setting */
    ADC->ADUL = _FF_AD_ADUL_VALUE;   //�Ƚ�AD�ɼ���12λ���ݵĸ߰�λ
    /* AD comversion result comprision lower limit setting */
    ADC->ADLL = _00_AD_ADLL_VALUE;


    ADC->ADM0 |= ADCE;          // ʹ��A/D��ѹ�Ƚ��������С�

    User_DelayTime_ms(500);

    //ANI4 ��׼��ѹ2.5V����
    ADC->ADS = ADC_CHANNEL_4; //ָ��ģ������ͨ����
    ADC->ADM0 |= ADCS; //����ת������
    while((ADC->ADM0 & ADCS) != 0);  //�ȴ�AD�жϱ��Ϊ1
    ADSample_Info.ref_AD = ADC->ADCR;  // ��ȡ12λA/Dת�����
    ADSample_Info.ref_AD_Fir = (ADSample_Info.ref_AD_Fir * 1593 >> 12) + (ADSample_Info.ref_AD* 2503 >> 12);

        
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
    
    //ANI8 �����е�������  ��ȡANI15ͨ��ֵ
    ADC->ADS = ADC_CHANNEL_8; //ָ��ģ������ͨ����
    ADC->ADM0 |= ADCS; //����ת������
    while((ADC->ADM0 & ADCS) != 0);  //�ȴ�AD�жϱ��Ϊ1
    ADSample_Info.curInduc_AD = ADC->ADCR;  // ��ȡ12λA/Dת�����

    //ANI3 ������ص�������  ��ȡANI7ͨ��ֵ
    ADC->ADS = ADC_CHANNEL_3; //ָ��ģ������ͨ����
    ADC->ADM0 |= ADCS; //����ת������
    while((ADC->ADM0 & ADCS) != 0);  //�ȴ�AD�жϱ��Ϊ1
    ADSample_Info.curLoad_AD =  ADC->ADCR;  // ���ص������е�����λ�෴��ʹ��ʱע��

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
    
    
    //ANI10 PFC���뽻����ѹ����
    ADC->ADS = ADC_CHANNEL_10; //ָ��ģ������ͨ����
    ADC->ADM0 |= ADCS; //����ת������
    while((ADC->ADM0 & ADCS) != 0);  //�ȴ�AD�жϱ��Ϊ1
    ADSample_Info.PFC_AC_Vol_AD = ADC->ADCR ;  // ��ȡ12λA/Dת�����
	
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


    
    //ANI9 ��������������ѹ����
    ADC->ADS = ADC_CHANNEL_9; //ָ��ģ������ͨ����
    ADC->ADM0 |= ADCS; //����ת������
    while((ADC->ADM0 & ADCS) != 0);  //�ȴ�AD�жϱ��Ϊ1
    ADSample_Info.INV_AC_Vol_AD = ADC->ADCR ;  // ��ȡ12λA/Dת�����
	
    
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

}


/*************************************************
Description: COM_Vref_Sample
Input      : 
Return     : 
Others     : ��׼��ѹ2.5V����
*************************************************/
void COM_Vref_Sample(void)
{
    //ANI4 ��׼��ѹ2.5V����
    ADC->ADS = ADC_CHANNEL_4; //ָ��ģ������ͨ����
    ADC->ADM0 |= ADCS; //����ת������
    while((ADC->ADM0 & ADCS) != 0);  //�ȴ�AD�жϱ��Ϊ1
    ADSample_Info.ref_AD = ADC->ADCR;  // ��ȡ12λA/Dת�����
    ADSample_Info.ref_AD_Fir = (ADSample_Info.ref_AD_Fir * 1593 >> 12) + (ADSample_Info.ref_AD* 2503 >> 12);

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
}

 
/*************************************************
Description: COM_CurSample
Input      : 
Return     : 
Others     : �ô�ʹ���˵��ݵ����ع�����
                     u(k) - u(k-1)
            ic = C * --------------
                          Ts
			 ���ǵ������ѹ�������ܵĸ��ţ��ع���ʹ�õ�ͨ�˲��������˲�����
*************************************************/
void COM_CurSample(void)
{
    //ANI8 �����е�������  ��ȡANI15ͨ��ֵ
    ADC->ADS = ADC_CHANNEL_8; //ָ��ģ������ͨ����
    ADC->ADM0 |= ADCS; //����ת������
    while((ADC->ADM0 & ADCS) != 0);  //�ȴ�AD�жϱ��Ϊ1
    ADSample_Info.curInduc_AD = ADC->ADCR;  // ��ȡ12λA/Dת�����

    //ANI3 ������ص�������  ��ȡANI7ͨ��ֵ
    ADC->ADS = ADC_CHANNEL_3; //ָ��ģ������ͨ����
    ADC->ADM0 |= ADCS; //����ת������
    while((ADC->ADM0 & ADCS) != 0);  //�ȴ�AD�жϱ��Ϊ1
    ADSample_Info.curLoad_AD =  ADC->ADCR;  // ���ص������е�����λ�෴��ʹ��ʱע��

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

    //��е������ݴ���
    ADSample_Info.curInduc_Hold = ADSample_Info.curInduc_AD - AD_Correct_Vref.out_Val; //��ȥƫ�õ�ѹ
    ADSample_Info.curInduc_AD_FIR = (ADSample_Info.curInduc_AD_FIR * 593 >> 12) + (ADSample_Info.curInduc_Hold * 3503 >> 12);

    //���ص������ݴ���
//    ADSample_Info.curLoad_Hold = ADSample_Info.curLoad_AD - AD_Correct_Vref.out_Val; //��ȥƫ�õ�ѹ
//    ADSample_Info.curLoad_AD_FIR = (ADSample_Info.curLoad_AD_FIR * 593 >> 12) + (ADSample_Info.curLoad_Hold * 3503 >> 12);	 

    ADSample_Info.curLoad_AD_FIR = ADSample_Info.curLoad_AD - AD_Correct_Vref.out_Val; //��ȥƫ�õ�ѹ
	
}  


/*************************************************
Description: COM_VolSample
Input      : 
Return     : 
Others     : ĸ�ߵ�ѹ�������ѹ���������ݴ���
*************************************************/
int ad_ad_cnt = 0;
void COM_VolSample(void)
{
    //ANI7 BUS��ѹ����
    ADC->ADS = ADC_CHANNEL_7; //ָ��ģ������ͨ����
    ADC->ADM0 |= ADCS; //����ת������
    while((ADC->ADM0 & ADCS) != 0);  //�ȴ�AD�жϱ��Ϊ1
    ADSample_Info.vBus_AD = ADC->ADCR;  // ��ȡ12λA/Dת�����

    //ĸ�ߵ�ѹ���ݴ���		
    ADSample_Info.vBus_Hold = DFILTER(1,ADSample_Info.vBus_AD,ADSample_Info.vBus_Hold);
    ADSample_Info.vBus_AD_FIR = ADSample_Info.vBus_Hold >> 16;		
    
    //ANI10 PFC���뽻����ѹ����
    ADC->ADS = ADC_CHANNEL_10; //ָ��ģ������ͨ����
    ADC->ADM0 |= ADCS; //����ת������
    while((ADC->ADM0 & ADCS) != 0);  //�ȴ�AD�жϱ��Ϊ1
    ADSample_Info.PFC_AC_Vol_AD = ADC->ADCR ;  // ��ȡ12λA/Dת�����
	
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

    //PFC�����ѹ���ݴ���	
    ADSample_Info.PFC_AC_Vol_Hold = (ADSample_Info.PFC_AC_Vol_AD - AD_Correct_Vref.out_Val);//��ȥƫ�õ�ѹ
    ADSample_Info.PFC_AC_Vol_AD_FIR = (ADSample_Info.PFC_AC_Vol_AD_FIR * 1596 + ADSample_Info.PFC_AC_Vol_Hold * 2500)>> 12;	


    
    //ANI9 ��������������ѹ����
    ADC->ADS = ADC_CHANNEL_9; //ָ��ģ������ͨ����
    ADC->ADM0 |= ADCS; //����ת������
    while((ADC->ADM0 & ADCS) != 0);  //�ȴ�AD�жϱ��Ϊ1
    ADSample_Info.INV_AC_Vol_AD = ADC->ADCR ;  // ��ȡ12λA/Dת�����
	
    
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

    //�����ѹ���ݴ���	
    ADSample_Info.INV_AC_Vol_Hold = ADSample_Info.INV_AC_Vol_AD - AD_Correct_Vref.out_Val;//��ȥƫ�õ�ѹ   
    ADSample_Info.INV_AC_Vol_AD_FIR = (ADSample_Info.INV_AC_Vol_AD_FIR * 1596 + ADSample_Info.INV_AC_Vol_Hold * 2500)>> 12;	

    

    //����ѹ�ͺ������ѹ
    if( COM_Ctr_Info.INV_PFC_Mode_Select == PFC_MODE && Run_Deal.flag.bit.OpenDriver_Ok == E_TRUE)
    //if( COM_Ctr_Info.INV_PFC_Mode_Select == PFC_MODE )
    { 
        if(ad_ad_cnt<600)
            ad_ad_cnt++;
        if(ad_ad_cnt>=600)
            ADSample_Info.PFC_AC_Vol_AD_FIR = ADSample_Info.INV_AC_Vol_AD_FIR;        
    }
    else
    {
        ad_ad_cnt = 0;
    }    
    
}


/*************************************************
Description: COM_AuxPowerSample
Input      : 
Return     : 
Others     : ������Դ��ѹ����
*************************************************/
void COM_AuxPowerSample(void)
{	
    //ANI6 ��Դ��ѹ���� ��ȡANI6ͨ��ֵ
    ADC->ADS = ADC_CHANNEL_6; //ָ��ģ������ͨ����
    ADC->ADM0 |= ADCS; //����ת������
    while((ADC->ADM0 & ADCS) != 0);  //�ȴ�AD�жϱ��Ϊ1
    ADSample_Info.auxPower_AD = ADC->ADCR;  // ��ȡ12λA/Dת�����
    ADSample_Info.auxPower_AD_FIR = ADSample_Info.auxPower_AD;//( ADSample_Info.auxPower_AD_FIR * 96  + ( ADSample_Info.auxPower_AD ) * 4000 )>> 12;	

}


/*************************************************
Description: COM_TempNTC_Sample
Input      : 
Return     : 
Others     : NTC�¶Ȳ���
*************************************************/
void COM_TempNTC_Sample(void)
{
    //ANI5 ɢ�����¶Ȳ���
    ADC->ADS = ADC_CHANNEL_5; //ָ��ģ������ͨ����
    ADC->ADM0 |= ADCS; //����ת������
    while((ADC->ADM0 & ADCS) != 0);  //�ȴ�AD�жϱ��Ϊ1
    ADSample_Info.temp_NTC_AD = ADC->ADCR;  // ��ȡ12λA/Dת�����
    ADSample_Info.temp_NTC_AD_FIR = ADSample_Info.temp_NTC_AD;//( ADSample_Info.temp_NTC_AD_FIR * 96  + ( ADSample_Info.temp_NTC_AD ) * 4000 )>> 12;	
}


/*************************************************
Description: COM_Altern_Sample
Input      : 
Return     : 
Others     : ��ѯ�����¶Ⱥ͸�����Դ
*************************************************/
void COM_Altern_Sample(void)
{
    static int8_t Altern_Cnt = 0;

    if(Altern_Cnt == 0 )
    {
        COM_TempNTC_Sample();//�¶Ȳ���
        Altern_Cnt = 1;
    }
    else  if(Altern_Cnt == 1 )
    {
        COM_AuxPowerSample();//������Դ��ѹ����   
        Altern_Cnt = 2;
    }  
    else 
    {
        COM_Vref_Sample();//VREF��ѹ����   
        Altern_Cnt = 0;
    }       
}

/*************************************************
Description: RMS_PQ_Calc
Input      : 
Return     : ʹ�ö��巨(������)���������ѹ��Чֵ����ƽ������ͺ���ƽ��ֵ����󿪷�
Others     : û�н��п���������Ϊ�������������빦�ܵĻ����Ͼ�����Сʱ�俪��
*************************************************/
int16_t P_table[680] = {0};
int16_t INV_AC_Vol_table[680] = {0};
int16_t PFC_AC_Vol_table[680] = {0};
int16_t CurLoad_table[680] = {0};
int16_t CurInduc_table[680] = {0};
int32_t sum_PFC_AC_Vol,sum_INV_AC_Vol = 0;
void RMS_PQ_Calc(RMS_PQ_Var_t *rms_PQ)
{
    //Pֵ����
    rms_PQ->sum_P = rms_PQ->sum_P - P_table[rms_PQ->periodDot_Cnt];   
    P_table[rms_PQ->periodDot_Cnt] = ((int32_t)rms_PQ->INV_AC_Vol_Peak * rms_PQ->curInduc_Peak)>>12;
    rms_PQ->sum_P = ((rms_PQ->sum_P + P_table[rms_PQ->periodDot_Cnt]));
    
    if(rms_PQ->INV_AC_Vol_Peak < 0)     rms_PQ->INV_AC_Vol_Peak     = -rms_PQ->INV_AC_Vol_Peak;
    if(rms_PQ->PFC_AC_Vol_Peak < 0)     rms_PQ->PFC_AC_Vol_Peak     = -rms_PQ->PFC_AC_Vol_Peak;
    if(rms_PQ->curLoad_Peak < 0)    rms_PQ->curLoad_Peak    = -rms_PQ->curLoad_Peak;
    if(rms_PQ->curInduc_Peak < 0)   rms_PQ->curInduc_Peak   = -rms_PQ->curInduc_Peak;    
  
    //INV_RMS_VOLֵ����
    rms_PQ->sum_INV_AC_Vol = rms_PQ->sum_INV_AC_Vol - INV_AC_Vol_table[rms_PQ->periodDot_Cnt];   
    INV_AC_Vol_table[rms_PQ->periodDot_Cnt] = ((int32_t)rms_PQ->INV_AC_Vol_Peak * rms_PQ->INV_AC_Vol_Peak)>>12;
    rms_PQ->sum_INV_AC_Vol = ((rms_PQ->sum_INV_AC_Vol + INV_AC_Vol_table[rms_PQ->periodDot_Cnt]));
    rms_PQ->sum_INV_AC_Vol_Hold = rms_PQ->sum_INV_AC_Vol_Hold + INV_AC_Vol_table[rms_PQ->periodDot_Cnt];
    
    //PFC_RMS_VOLֵ����
    rms_PQ->sum_PFC_AC_Vol = rms_PQ->sum_PFC_AC_Vol - PFC_AC_Vol_table[rms_PQ->periodDot_Cnt];   
    PFC_AC_Vol_table[rms_PQ->periodDot_Cnt] = ((int32_t)rms_PQ->PFC_AC_Vol_Peak * rms_PQ->PFC_AC_Vol_Peak)>>12;
    rms_PQ->sum_PFC_AC_Vol = ((rms_PQ->sum_PFC_AC_Vol + PFC_AC_Vol_table[rms_PQ->periodDot_Cnt]));
    rms_PQ->sum_PFC_AC_Vol_Hold = rms_PQ->sum_PFC_AC_Vol_Hold + PFC_AC_Vol_table[rms_PQ->periodDot_Cnt];
    
    //RMS_LoadCurֵ����
    rms_PQ->sum_CurLoad = rms_PQ->sum_CurLoad - CurLoad_table[rms_PQ->periodDot_Cnt];   
    CurLoad_table[rms_PQ->periodDot_Cnt] = ((int32_t)rms_PQ->curLoad_Peak * rms_PQ->curLoad_Peak)>>6;
    rms_PQ->sum_CurLoad = ((rms_PQ->sum_CurLoad + CurLoad_table[rms_PQ->periodDot_Cnt]));     
  
    //RMS_InductorCurֵ����
    rms_PQ->sum_CurInduc = rms_PQ->sum_CurInduc - CurInduc_table[rms_PQ->periodDot_Cnt];   
    CurInduc_table[rms_PQ->periodDot_Cnt] = ((int32_t)rms_PQ->curInduc_Peak * rms_PQ->curInduc_Peak)>>11;
    rms_PQ->sum_CurInduc = ((rms_PQ->sum_CurInduc + CurInduc_table[rms_PQ->periodDot_Cnt]));
   
   
    //������Чֵ����ֵ
    rms_PQ->periodDot_Cnt++;
    if ( rms_PQ->periodDot_Cnt >= rms_PQ->periodDot_Val )
    {
        rms_PQ->periodDot_Cnt           = 0;	
        rms_PQ->sum_PFC_AC_Vol          = rms_PQ->sum_PFC_AC_Vol_Hold;
        rms_PQ->sum_INV_AC_Vol          = rms_PQ->sum_INV_AC_Vol_Hold;
        rms_PQ->sum_PFC_AC_Vol_Hold     = 0;
        rms_PQ->sum_INV_AC_Vol_Hold     = 0;
    }
    
}

/*************************************************
Description: RMS_PQ_Update
Input      : 
Return     : 
Others     : ��Чֵ���������ݴ���
*************************************************/
void RMS_PQ_Update(RMS_PQ_Var_t *rms_PQ)
{
    int32_t     out_P_temp = 0;
    uint32_t    out_INV_Vol_temp = 0;
    uint32_t    out_PFC_Vol_temp = 0;
    uint32_t    out_LoadCur_temp = 0;
    uint32_t    out_InductorCur_temp = 0;

    //INV_RMS_VOLֵ����
    out_INV_Vol_temp = (rms_PQ->sum_INV_AC_Vol * rms_PQ->dot_Reciprocal)>>15;//15+24 - 15 - 12 = 12
    rms_PQ->out_RMS_INV_AC_Vol_FIR = sqrt(out_INV_Vol_temp * (COM_AC_VOL_BASE * COM_AC_VOL_BASE)) ;  //6= 12/2
    rms_PQ->out_RMS_INV_AC_Vol_FIR = rms_PQ->out_RMS_INV_AC_Vol_FIR*COM_REAL_VACOUT_RMS_SCAL>>6;
    
    rms_PQ->out_RMS_INV_AC_Vol_Hold = DFILTER(7,rms_PQ->out_RMS_INV_AC_Vol_FIR,rms_PQ->out_RMS_INV_AC_Vol_Hold);

    rms_PQ->out_RMS_INV_AC_Vol = rms_PQ->out_RMS_INV_AC_Vol_Hold >> 16;  //���AC_VOL��ʵ��Чֵ*COM_REAL_VACOUT_RMS_SCAL

    
    //PFC_RMS_VOLֵ����
    out_PFC_Vol_temp = (rms_PQ->sum_PFC_AC_Vol * rms_PQ->dot_Reciprocal)>>15;
    rms_PQ->out_RMS_PFC_AC_Vol_FIR = sqrt(out_PFC_Vol_temp * (COM_AC_VOL_BASE * COM_AC_VOL_BASE)) ;        
    rms_PQ->out_RMS_PFC_AC_Vol_FIR = rms_PQ->out_RMS_PFC_AC_Vol_FIR * COM_REAL_VACIN_RMS_SCAL>>6;
    
    rms_PQ->out_RMS_PFC_AC_Vol_Hold = DFILTER(8,rms_PQ->out_RMS_PFC_AC_Vol_FIR,rms_PQ->out_RMS_PFC_AC_Vol_Hold);
    rms_PQ->out_RMS_PFC_AC_Vol = rms_PQ->out_RMS_PFC_AC_Vol_Hold >> 16;  //����AC_VOL��ʵ��Чֵ*COM_REAL_VACIN_RMS_SCAL 
    
    
    //RMS_LoadCurֵ����
    out_LoadCur_temp = (rms_PQ->sum_CurLoad * rms_PQ->dot_Reciprocal)>>15;  //15+24  - 15 -6 = 18
    rms_PQ->out_RMS_CurLoad_FIR = sqrt(out_LoadCur_temp * COM_CUR_LOAD_BASE * COM_CUR_LOAD_BASE);   // 9=18/2 
      
    rms_PQ->out_RMS_CurLoad_Hold = DFILTER(6,rms_PQ->out_RMS_CurLoad_FIR,rms_PQ->out_RMS_CurLoad_Hold);
    rms_PQ->out_RMS_CurLoad = (rms_PQ->out_RMS_CurLoad_Hold >> 16)*COM_REAL_ILOAD_RMS_SCAL>>9 ;  //���AC_LoadCur��Чֵ*COM_REAL_ILOAD_RMS_SCAL  
  
    //RMS_InductorCurֵ����
    out_InductorCur_temp = (rms_PQ->sum_CurInduc * rms_PQ->dot_Reciprocal)>>9>>1;  //15 +24- 9-10 -2 = 18
    rms_PQ->out_RMS_CurInduc_FIR = sqrt((out_InductorCur_temp) * (COM_CUR_INDUC_BASE * COM_CUR_INDUC_BASE)) ;   //9=18/2 

    rms_PQ->out_RMS_CurInduc_Hold = DFILTER(6,rms_PQ->out_RMS_CurInduc_FIR,rms_PQ->out_RMS_CurInduc_Hold);
    rms_PQ->out_RMS_CurInduc = (rms_PQ->out_RMS_CurInduc_Hold >> 16)*COM_REAL_IINDUC_RMS_SCAL>>9;  //���AC_LoadCur��Чֵ*COM_REAL_IINDUC_RMS_SCAL 
   
   
    //Pֵ����
    out_P_temp = rms_PQ->sum_P * rms_PQ->dot_Reciprocal>>15;
    rms_PQ->out_P_FIR =  out_P_temp * COM_AC_VOL_BASE * COM_CUR_INDUC_BASE >> 12 ;//�й�������ʵֵ
    
    rms_PQ->out_P_Hold = DFILTER(7,rms_PQ->out_P_FIR,rms_PQ->out_P_Hold);
    rms_PQ->out_P = rms_PQ->out_P_Hold >> 16;	    
    
    if(rms_PQ->out_P < 0)
    {
       rms_PQ->out_P = -rms_PQ->out_P; 
    }

    //Sֵ����
    rms_PQ->out_S_FIR =  rms_PQ->out_RMS_CurLoad * rms_PQ->out_RMS_INV_AC_Vol / (COM_REAL_ILOAD_RMS_SCAL *COM_REAL_VACOUT_RMS_SCAL );//���ڹ�����ʵֵ
    rms_PQ->out_S_Hold = DFILTER(4,rms_PQ->out_S_FIR,rms_PQ->out_S_Hold);
    rms_PQ->out_S = rms_PQ->out_S_Hold >> 16;    
    
    if(rms_PQ->out_S < 0)
    {
       rms_PQ->out_S = 0; 
    }    
}   
