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
#include "sys_hardware_init.h"
#include "clk.h"
#include "sci.h"
#include "rtc.h"
#include "elc.h"
#include "timc.h"
#include "DMA.h"
#include "gpio.h"
#include "userdefine.h"
/***************************************************************************/

/*************************************************
Description: SystemClock_Init
Input      : 
Return     : 
Others     : EPWM���� 64M
*************************************************/
void SystemClock_Init(void)
{
    //0x40:ѡ��X1��ģʽ
	CLK_Osc_Setting(OSC_OSCILLATOR, OSC_OSCILLATOR);
    //0x40:ѡ��X1��ģʽ�� 0x00:�ⲿ����Χ0 - 10MHZ��  ���ȶ�ʱ��ѡ��Ĭ��ֵ�� ���ȶ�ʱ�����ֵ�ж�
	CLK_MainOsc_Setting(OSC_OSCILLATOR, OSC_UNDER_10M);

    //-----------------------------------------------------------------------
    // Select PLL as system clock
    //-----------------------------------------------------------------------
    //0x80:ѡ��FMAX�ⲿ������ΪPLL��ʱ��Դ��0X20 PLL��Ƶѡ��16��  ��0X04  PLL��Ƶѡ��2��Ƶ
    CLK_PLL_Setting(PLL_SR_fMX,PLL_DIV_2,PLL_MUL_16);
    CLK_PLL_Start(); /* PLLON = 1 */
	CLK_Fclk_Select(MAINCLK_FPLL);//�����ڲ�����ʱ�Ӻ�PLLʱ�ӵ�ѡ��:1��ʾ����ΪPLLʱ��
	while((CGC->MCKC & CGC_MCKC_CKSTR_Msk) == 0)//CKSTR�����ڲ�����ʱ�Ӻ�PLLʱ�ӵ�ѡ���״̬:Ϊ1ʱ��ʾ��ǰʱ��״̬ѡ�����PLLʱ��
       __NOP();
    SystemCoreClock = 64000000; // 8M*16/2 =  64MHz     
    
//	// ���������
//	CGC->CMC  = _40_CGC_HISYS_OSC |_00_CGC_SYSOSC_UNDER10M; //0x40:ѡ��X1��ģʽ�� 0x00:�ⲿ����Χ0 - 10MHZ
//	CGC->OSTS = _06_CGC_OSCSTAB_SEL17; //���ȶ�ʱ��ѡ�� 2^17/fX us	
//	CGC->CSC  = _40_CGC_FSUB_STOP; //ʹ��X1��ģʽ����,XT1ֹͣ
//    
//    ////CGC->CKC = _10_CGC_MAINCLK_SELHISYS;//ע�⣺�˳������ã�����������//ѡ�����ϵͳʱ�ӣ�f MX ����Ϊ��ϵͳʱ�ӣ�f MAIN ����	
//	while(CGC->OSTC < _FE_CGC_OSCSTAB_STA17); // ���ȶ�ʱ�����ֵ�ж�
//	
//    //ע�⣺���ļ���_01_CGC_PLL_STOP��д����01��ʾʹ��PLL
//	CGC->PLLCR = 0x01|_80_CGC_PLLSR_fMX|_02_CGC_PLL_MUL_16_0|_04_CGC_PLL_DIV_2;  //0x80:ѡ��FMAX�ⲿ������ΪPLL��ʱ��Դ��0X01 ʹ��PLL������0X20 PLL��Ƶѡ��16��  ��0X04  PLL��Ƶѡ��2��Ƶ
//	CGC->MCKC = 0x01; /* CKSELR = 1, PDIV = 0 */  //�����ڲ�����ʱ�Ӻ�PLLʱ�ӵ�ѡ��:1��ʾ����ΪPLLʱ��
//	while((CGC->MCKC & CGC_MCKC_CKSTR_Msk) == 0);//CKSTR�����ڲ�����ʱ�Ӻ�PLLʱ�ӵ�ѡ���״̬:Ϊ1ʱ��ʾ��ǰʱ��״̬ѡ�����PLLʱ��
//    SystemCoreClock = 64000000; // 8M*16/2 =  64MHz
}


/*************************************************
Description: WDT_ConfigInit
Input      : 
Return     : 
Others     : ���Ź�����
*************************************************/
void WDT_ConfigInit(void)
{	
	
}


/*************************************************
Description: SysTick_ConfigInit
Input      : 
Return     : 
Others     : ϵͳ��ʱ������
*************************************************/
void SysTick_ConfigInit(void)
{
    uint32_t msCnt; 	// count value of 1ms
    
    //ע�⣺ʹ���ڲ�ʱ��ʱ�����ã�ʹ�þ�����������
    //SystemCoreClockUpdate();	//����ʱ��״̬����ȡϵͳʱ��
    
    msCnt = SystemCoreClock / 1000; 
    SysTick_Config(msCnt);//�δ�ʱ����ʼ��				
}


/*************************************************
Description: GPIO_Init
Input      : 
Return     : 
Others     : GPIO��ʼ������
*************************************************/
void GPIO_Init(void)
{			

    /*P70 LED�������� ����0����ʾ����*/ 
    PORT_Init(PORT7,PIN0,OUTPUT); // P70/led  ����Ϊ���ģʽ
    PORT_SetBit(PORT7,PIN0);// ����Ϊ�ߵ�ƽ/* P70 output High level */

    /*P30 ĸ�߹�ѹ�ź� To DCDC�� ����1����ʾ��ѹ*/    
    PORT_Init(PORT3,PIN0,OUTPUT); // P30 BUS��ѹ�ź� 
    PORT_ClrBit(PORT3,PIN0);// ����Ϊ�͵�ƽ/* P63 output low level */


    /*P20 INV_�̵���1�������ã���1��ΪON*/ 
    PORT_Init(PORT2,PIN0,OUTPUT); // P20/INV_�̵���1�����ź�,����Ϊ���ģʽ
    PORT_ClrBit(PORT2,PIN0);// ����Ϊ�͵�ƽ/* P20 output low level */
    
    /*P21 INV_�̵���3��������(ά��)����1��ΪON*/ 
    PORT_Init(PORT2,PIN1,OUTPUT); // P21/INV_�̵���3�����ź�,����Ϊ���ģʽ
    PORT_ClrBit(PORT2,PIN1);// ����Ϊ�͵�ƽ/* P21 output low level */    

    /*P130 PFC_�̵���2�������ã���1��ΪON*/ 
    PORT_Init(PORT13,PIN0,OUTPUT); // P130/INV_�̵���1�����ź�,����Ϊ���ģʽ
    PORT_ClrBit(PORT13,PIN0);// ����Ϊ�͵�ƽ/* P130 output low level */

    /*P75 ����INV�����ź� �������ã���1��ΪON*/ 
    PORT_Init(PORT7,PIN5,OUTPUT); // P75/����INV�����ź�,����Ϊ���ģʽ
    PORT_ClrBit(PORT7,PIN5);// ����Ϊ�͵�ƽ/* P75 output low level */
    
    //P31 ����DCDC�๤���ź�����
    PORT_Init(PORT3,PIN1,PULLUP_INPUT);   // P31����Ϊ��������˿�

    //P63 ����������
    PORT_Init(PORT6,PIN3,PULLUP_INPUT);   // P63����Ϊ��������˿�
    
    //P136 ������������ź����ã���0������
    PORT_Init(PORT13,PIN6,PULLUP_INPUT);   // P136����Ϊ��������˿�	

    /* �����Ƶ��ѡ��Ĭ����������(�͵�ƽ��Ч)base_freq=1->50hz/0->60hz */	
    //PORT_Init(PORT7,PIN1,PULLUP_INPUT);   // P71����Ϊ��������˿�  
}


/*************************************************
Description: TIMC_ConfigInit
Input      : 
Return     : 
Others     : EPWM���� - M��ʱ��
*************************************************/
void TIMC_ConfigInit(void)
{
    CGC->PER1 |= CGC_PER1_TMCEN_Msk;     /* enables input clock supply */
    TMC->TCCR2 &= (uint8_t)~_01_TMC_COUNTING_START;
    INTC_DisableIRQ(TMC_IRQn);/* disable INTTMC interrupt */
    INTC_ClearPendingIRQ(TMC_IRQn);/* clear INTTMC interrupt flag */
    /* select count clock */
    TMC->TCCR1  = TMC_CLOCK_FCLK32 << 5;
    /* software start and trigger enable, tm0 trigger */
    TMC->TCCR1 |=  _00_TMC_SOFTWARE_START | _08_TMC_SOFTWARE_ENABLE | _00_TMC_INTERRUPT_DISABLE;
    /* transfer counter value to buffer register, set 0000H to counter,  and then continue counting */
    TMC->TCCR2 = _06_TMC_COUNTING_TRANSFERRED_SET_0000 | _00_TMC_COUNTING_STOP;

    //TMC->TCCR2 |= _01_TMC_COUNTING_START;
    //TMC_Start();    
}
/*************************************************
Description: EPWM_ConfigInit
Input      : 
Return     : 
Others     : EPWM���� - M��ʱ��
*************************************************/
void EPWM_ConfigInit(void)
{
	/* 
	(1)����EPWMʱ��
	*/
    CGC->PER1 |= CGC_PER1_TMMEN_Msk;     /* enables input clock supply */
    CGC->PER1 |= 0x04U;

	/*
	(2)����EPWM����ģʽ
	*/
    TMM->TMSTR |= _04_TMM_TM0_COUNT_CONTINUES | _08_TMM_TM1_COUNT_CONTINUES;//�ں�TMGRA1��TMGRA0�Ĵ����Ƚ�ƥ��󻹼���������
    TMM->TMSTR &= (uint8_t)~(_02_TMM_TM1_COUNT_START | _01_TMM_TM0_COUNT_START);//TM0��TM1ֹͣ������
	
    /* the TMGRD0/TMGRC1/GMGRD1 is used as buffer register of TMGRB0/TMGRA1/TMGRB1 */
	//TMGRD0/TMGRC1/GMGRD1�Ĵ�����ΪTMGRB0/TMGRA1/TMGRB1�Ĵ����Ļ���Ĵ���  
    TMM->TMMR = _80_TMM_TMGRD1_BUFFER | _40_TMM_TMGRC1_BUFFER | _20_TMM_TMGRD0_BUFFER;
	
    /* the counter is free running */
    //TM0��TM1�����������ѡ��ʹ�ܣ�   clkΪ��ʱ������Դ��ѡ��
    #if 0
        TMM->TMCR0 =   _20_TMM_COUNTER_CLEAR_TMGRA | TMM_CLOCK_FCLK; //��λͬ��ģʽ�����á�001�����ں�TMGRA0�Ĵ����Ƚ�ƥ��ʱ���TM0�Ĵ�����  _00_TMM_COUNTER_CLEAR_DISABLE | clk;//����  
        TMM->TMCR1 = _00_TMM_COUNTER_CLEAR_DISABLE | TMM_CLOCK_FCLK;
    #else
        //TM0��TM1�����������ѡ��ʹ�ܣ��������У���   clkΪ��ʱ������Դ��ѡ��
        TMM->TMCR0 = _00_TMM_COUNTER_CLEAR_DISABLE | _00_TMM_INETNAL_CLOCK_FCLK_FHOCO;//����PWMģʽ�����á�000������ֹ������������У���
        TMM->TMCR1 = _00_TMM_COUNTER_CLEAR_DISABLE | _00_TMM_INETNAL_CLOCK_FCLK_FHOCO;  
    #endif    
    
	/*
	(3)����EPWM������������ݼ��ط�ʽ
	*/
	/* Set the initial level and active level: 
	   OLS1(����-COUNTER), OLS0(����-NORMAL): 0����ʼ�����H����ƽ����L����ƽ��Ч��1����ʼ�����L����ƽ����H����ƽ��Ч��
	   CMD1, CMD0 :  0b10:����PWMģʽ����TM1��������ʱ�������ݴӻ���Ĵ������͵�ͨ�üĴ�����*/  
    #if 1
    		TMM->TMFCR |= _02_TMM_TRANSFER_TM1_UNDERFLOW | _00_TMM_NORMAL_PHASE_LEVEl_HL | _00_TMM_COUNTER_PHASE_LEVEl_HL; //���ĶԳ�
            //TMM->TMFCR |= _01_TMM_TRANSFER_RESET_SYNCHRONOUS | _00_TMM_NORMAL_PHASE_LEVEl_HL | _00_TMM_COUNTER_PHASE_LEVEl_HL;	//��λͬ��PWMģʽ   ��ݲ�
    #else
    		TMM->TMFCR |= _02_TMM_TRANSFER_TM1_UNDERFLOW | _04_TMM_NORMAL_PHASE_LEVEl_LH | _08_TMM_COUNTER_PHASE_LEVEl_LH;
            //TMM->TMFCR |= _01_TMM_TRANSFER_RESET_SYNCHRONOUS | _04_TMM_NORMAL_PHASE_LEVEl_LH | _08_TMM_COUNTER_PHASE_LEVEl_LH;	//��λͬ��PWMģʽ   ��ݲ�
    #endif
    
        
	/*
	(4)����EPWM���ڡ�EPWM����
	*/
    TMM->TM0    = INV_DEADTIME;//����ʱ������
    TMM->TM1    = 0x0000;
    TMM->TMGRA0 = INV_PWM_PERIOD;   //�������Գ����ĵĶ���ֵ
    
    COM_Ctr_Info.EPWM_Init_Mode = 1;//��ʼ��ΪINV����Ƶ��
    
    TMM->TMGRC1 = 1;//PWM_PERIOD_HALF;//PWM2�����ռ�ձ�����   TMGRC1�Ĵ���ΪTMGRA1�Ĵ����Ļ���Ĵ���  
    TMM->TMGRA1 = 1;

    TMM->TMGRD0 = 1;//PWM_PERIOD_HALF;//PWM1�����ռ�ձ�����   TMGRD0�Ĵ���ΪTMGRB0�Ĵ����Ļ���Ĵ���  
    TMM->TMGRB0 = 1;

    TMM->TMGRD1 = 0;//PWM3�����ռ�ձ�����   TMGRD1�Ĵ���ΪTMGRB1�Ĵ����Ļ���Ĵ���  
    TMM->TMGRB1 = 0;		


	/*
	(5)����EPWM���ʹ��
	*/
    //��ʼ��ʱ���ȫ���ض�
    TMM->TMOER1 = _01_TMM_TMIOA0_OUTPUT_DISABLE | _02_TMM_TMIOB0_OUTPUT_DISABLE | _04_TMM_TMIOC0_OUTPUT_DISABLE | _08_TMM_TMIOD0_OUTPUT_DISABLE |
                  _10_TMM_TMIOA1_OUTPUT_DISABLE | _20_TMM_TMIOB1_OUTPUT_DISABLE | _40_TMM_TMIOC1_OUTPUT_DISABLE | _80_TMM_TMIOD1_OUTPUT_DISABLE;


	/*
	(6)���ñȽ�ƥ���ж�  ʹ��TMM0��TMM1�ж�
	*/
    /* Clear the interrupt flag register */
    TMM->TMSR0 =  0;
    TMM->TMSR1 =  0;		
    /* Set the interrupt enable register */
    TMM->TMIER0 = _00_TMM_OVIE_DISABLE | _00_TMM_IMID_DISABLE | _00_TMM_IMIC_DISABLE | _00_TMM_IMIB_DISABLE | _01_TMM_IMIA_ENABLE;
    TMM->TMIER1 = _10_TMM_OVIE_ENABLE | _00_TMM_IMID_DISABLE | _00_TMM_IMIC_DISABLE | _00_TMM_IMIB_DISABLE | _00_TMM_IMIA_DISABLE;

    //NVIC_SetPriority(TMM0_IRQn, 0); /* Set higher priority to execute slave ISR firstly */
    //INTC_EnableIRQ(TMM0_IRQn);/* enable INTTMM0 interrupt */
    //   NVIC_SetPriority(TMM1_IRQn, 0); /* ����ռ�ձ�   ���ȼ���  Set higher priority to execute slave ISR firstly */	
    //   INTC_EnableIRQ(TMM1_IRQn);/* enable INTTMM0 interrupt */
            
            
	/*
	(7)����EPWMɲ��
	*/
    //ǿ�ƽ�ֹ�������  ����ԴΪINTP0 ����
    TMM->OPCTL0 =  _00_TMM_PWMOP_HAZARD_DIASBLE | _20_TMM_PWMOP_RISING_RELEASE | _10_TMM_PWMOP_INTP_CUTOFF | _04_TMM_PWMOP_SOFT_CONDITION_RELEASE |_00_TMM_PWMOP_HARDWARE_RELEAS ;             
    TMM->OPDF0  =  _80_TMM_PWMOP_TMIODm_LOW | _08_TMM_PWMOP_TMIOBm_LOW ; //ǿ�����Ϊ�͵�ƽ   B0��D0���Ž�ֹǿ�����

    TMM->OPDF1  =  _80_TMM_PWMOP_TMIODm_LOW | _20_TMM_PWMOP_TMIOCm_LOW | _08_TMM_PWMOP_TMIOBm_LOW | _02_TMM_PWMOP_TMIOAm_LOW ; 		//ǿ�����Ϊ�͵�ƽ		                  
    TMM->OPEDGE =  _01_TMM_PWMOP_RELEASE_TMIOC0_F;//�� TMIOC0 ���½��ؽ��                       


	/*
	(8)����IO�����
	*/	
    /* Set TMIOC0 pin */
    TMOC0_PORT_SETTING();

    /* Set PWM1 TMIOB0 U+/������1�¹�PWM P14 pin */
    TMOB0_PORT_SETTING(); 
    /* Set PWM1 TMIOD0 U-/������1�Ϲ�PWM P15 pin */
    TMOD0_PORT_SETTING();
		
    /* Set PWM2 TMIOA1 V+/������2�¹�PWM P12 pin */
    TMOA1_PORT_SETTING();
    /* Set PWM2 TMIOC1 V-/������2�Ϲ�PWM P13 pin */
    TMOC1_PORT_SETTING();
		
    /* Set TMIOB1/W+ pin */
//    TMOB1_PORT_SETTING();
    /* Set TMIOD1/W- pin */
//    TMOD1_PORT_SETTING();
                      
	/*
	(9)��ӳ��,�ͻ�����ʵ��Ӧ������ 
	*/	

	
	/*
	(10)��zero�㡢���ڵ����ռ�ձ�
	*/	

	
	/*
	(11)����EPWM
	*/	
    TMM->TMSTR |= _02_TMM_TM1_COUNT_START | _01_TMM_TM0_COUNT_START;//������ʱ��	


}


/*************************************************
Description: INV_PWM_Enable
Input      : 
Return     : 
Others     : ����PWM���
*************************************************/
void INV_PWM_Enable(void)
{
    //���¹ܣ�Ԥ���
    PORT->P1    |= (1 << 3);    /* P13 output low level */ \
    PORT->P1    |= (1 << 5);    /* P15 output low level */ \
    PORT->P1    &= ~(1 << 2);    /* P13 output low level */ \
    PORT->P1    &= ~(1 << 4);    /* P15 output low level */ \
    User_DelayTime_us(500);    
    PORT->P1    &= ~(15 << 2);  // PWM���õ͵�ƽ
    
    

    TMM->TMGRA0 = INV_PWM_PERIOD;   //�������Գ����ĵĶ���ֵ
    
    COM_Ctr_Info.EPWM_Init_Mode = 1;//��ʼ��ΪINV����Ƶ�� 
    
    TMM->TMGRC1 = 1;//PWM2�����ռ�ձ�����   TMGRC1�Ĵ���ΪTMGRA1�Ĵ����Ļ���Ĵ���  
    TMM->TMGRA1 = 1;

    TMM->TMGRD0 = 1;//PWM1�����ռ�ձ�����   TMGRD0�Ĵ���ΪTMGRB0�Ĵ����Ļ���Ĵ���  
    TMM->TMGRB0 = 1;

    TMM->TMGRD1 = 0;//PWM3�����ռ�ձ�����   TMGRD1�Ĵ���ΪTMGRB1�Ĵ����Ļ���Ĵ���  
    TMM->TMGRB1 = 0;
    
    
}

/*************************************************
Description: PFC_PWM_Enable
Input      : 
Return     : 
Others     : ����PWM���
*************************************************/
int PFC_Freq_Temp = 0;
void PFC_PWM_Enable(void)
{

    //���µ���  24000Hz/19200Hz = 1.25; 1.25*8 = 10;
    if(COM_Ctr_Info.EPWM_Init_Mode == 1)//��INV����Ƶ���²�������ѹƵ�ʣ���������
    {       
        INV_Ctrl_Info.periodDot_Val         = PFC_PWM_FREQ*INV_Ctrl_Info.periodDot_Val/INV_PWM_FREQ;
    }
//    TMM->TM0    = PFC_DEADTIME;//����ʱ������
//    TMM->TM1    = 0x0000;
    
    TMM->TMGRA0 = PFC_PWM_PERIOD;   //�������Գ����ĵĶ���ֵ
    COM_Ctr_Info.EPWM_Init_Mode = 2;//��ʼ��ΪINV����Ƶ��
    
    TMM->TMGRC1 = 1;//PWM2�����ռ�ձ�����   TMGRC1�Ĵ���ΪTMGRA1�Ĵ����Ļ���Ĵ���  
    TMM->TMGRA1 = 1;

    TMM->TMGRD0 = 1;//PWM1�����ռ�ձ�����   TMGRD0�Ĵ���ΪTMGRB0�Ĵ����Ļ���Ĵ���  
    TMM->TMGRB0 = 1;

    TMM->TMGRD1 = 0;//PWM3�����ռ�ձ�����   TMGRD1�Ĵ���ΪTMGRB1�Ĵ����Ļ���Ĵ���  
    TMM->TMGRB1 = 0;
    
}


/*************************************************
Description: COM_PWM_Disable
Input      : 
Return     : 
Others     : �ر�PWM���
*************************************************/
void COM_PWM_Disable(void)
{
    
    
    TMM->TMOER1 = _01_TMM_TMIOA0_OUTPUT_DISABLE | _02_TMM_TMIOB0_OUTPUT_DISABLE | _04_TMM_TMIOC0_OUTPUT_DISABLE | _08_TMM_TMIOD0_OUTPUT_DISABLE |
                  _10_TMM_TMIOA1_OUTPUT_DISABLE | _20_TMM_TMIOB1_OUTPUT_DISABLE | _40_TMM_TMIOC1_OUTPUT_DISABLE | _80_TMM_TMIOD1_OUTPUT_DISABLE;

    PORT->P1    &= ~(15 << 2);  // PWM���õ͵�ƽ
    
		
    // PFC_RY2_DISABLE;
    //INV_RY1_DISABLE;
    
}


/*************************************************
Description: ADC_ConfigInit
Input      : 
Return     : 
Others     : ����ADC����
*************************************************/
void ADC_ConfigInit(void)
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
    ADC->ADM0 = _28_AD_CONVERSION_CLOCK_1 | _00_AD_COMPARATOR_DISABLE;    //Fclk /1  ת��ʱ��Ƶ�� ;ת��ʱ��Ϊ45/Fclk
//_28_AD_CONVERSION_CLOCK_1
//_20_AD_CONVERSION_CLOCK_2
    /* AD conversion mode setting */
    ADC->ADM1 = _00_AD_HISPEED;  //���ٱ任ģʽ
		
    //����ʱ��������Ϊ13.5 * Fclk
    ADC->ADNSMP = 0x10;//Ӱ���е���������Ӱ��������������;   //����ʱ��������Ϊ13.5 * Fclk
		
    //ADת��ͨ��ģʽ����		
    #if 0     
        ADC->ADM1 |= _00_AD_OPERMODE_SELECT;  //ADת��ͨ������Ϊѡ��ģʽ
    #else     
        ADC->ADM1 |= _80_AD_OPERMODE_SCAN; //ADת��ͨ������Ϊɨ��ģʽ
    #endif
    
    // ADת��ģʽ����
    #if 0
        ADC->ADM1 |= _00_AD_CONVMODE_SEQUENTIAL; // ADת��ģʽ����Ϊ����ת��ģʽ
    #else 
        ADC->ADM1 |= _08_AD_CONVMODE_ONESHOT;  // ADת��ģʽ����Ϊ����ת��ģʽ
    #endif

    // AD�ο���ѹ����
    /* AD reference voltage setting */
    ADC->ADM2 = _00_AD_POSITIVE_VDD | _00_AD_NEGATIVE_VSS | _00_AD_AREA_MODE_1 ;

//    ADC->ADM2 = _40_AD_POSITIVE_AVREFP | _20_AD_NEGATIVE_AVREFM | _00_AD_AREA_MODE_1 ;

//    ADC->ADM2 = _40_AD_POSITIVE_AVREFP | _00_AD_NEGATIVE_VSS | _00_AD_AREA_MODE_1 ;


    // AD�ο���ѹ����
    /* AD trigger selection */
//    ADC->ADTRG = _00_AD_TRIGGER_SOFTWARE;   //�������ģʽ

    ADC->ADTRG = _80_AD_TRIGGER_HARDWARE_NOWAIT | //Ӳ���޵ȴ�����ģʽ
                            _01_AD_TRIGGER_EVENTC;					//Ӳ�������źŵ�ѡ��ELCѡ����¼��ź�

//    ADC->ADTRG = _C0_AD_TRIGGER_HARDWARE_WAIT;//Ӳ���ȴ�����ģʽ


    // AD������ֵ����	
    /* AD comversion result comprision upper limit setting */
    ADC->ADUL = _FF_AD_ADUL_VALUE;   //�Ƚ�AD�ɼ���12λ���ݵĸ߰�λ
    /* AD comversion result comprision lower limit setting */
    ADC->ADLL = _00_AD_ADLL_VALUE;


    //���ó�Ӳ�� TMM����ʱ��������
    ELC_Start(_0C_ELC_EVENT_TMUDF1, _01_ELC_EVENT_LINK_AD);		//TMM1�����紥��ADC������
    //ELC_Start(_08_ELC_EVENT_TMIMFA0, _01_ELC_EVENT_LINK_AD);		//TMM0�����紥��ADC������

	/*
	(4)����ADC�ж�
	*/	
	  //����жϱ�־
    INTC_ClearPendingIRQ(ADC_IRQn);     /* clear INTAD interrupt flag */
    NVIC_ClearPendingIRQ(ADC_IRQn);     /* clear INTAD interrupt flag */ 
    #if 0
        INTC_EnableIRQ(ADC_IRQn);       /* enable INTAD interrupt */
    #else
        INTC_DisableIRQ(ADC_IRQn);      /* disable INTAD interrupt */     
    #endif
    
	/*
	(5)�������ȼ�
	*/	
//    NVIC_SetPriority(ADC_IRQn, 2); /* Set higher priority to execute slave ISR firstly */

	/*
	(6)����ADC
	*/		

    ADC->ADM0 |= ADCE;          // ʹ��A/D��ѹ�Ƚ��������С�
    ADC->ADS = ADC_CHANNEL_7;   //ָ��ģ������ͨ����
//    ADC->ADM0 |= ADCS;          //����ת������

}


/*************************************************
Description: ACMP0_ConfigInit
Input      : 
Return     : 
Others     : ACMP0���ã����������ѹƵ�ʼ��
*************************************************/
void ACMP0_ConfigInit(void)
{
	
	/*
	(1)���ñȽ���������ʱ��Դ
	 */
    CGC->PER1 |= CGC_PER1_PGACMPEN_Msk;    /* enables input clock supply */
 
    
	/*
	 (2)����ACMP0 ��׼��ѹֵ 
	*/	
    CMP_VREF_Set_Value(CMP_CHANNEL_0, 128);//�������û�׼��ѹֵ   VDD��5.0V�� / 256 * 128 = 2.5V 
    //CMP_VREF_Init     (CMP_CHANNEL_0, CMP_VDD_VSS_REFERENCE_VOLTAGE);//�Ƚ������û�׼��ѹ���ƼĴ�������(CVRCTL)


     CMP->CVRCTL |= _00_VDD_INTERNAL_REFERVOL | _00_VSS_INTERNAL_REFERVOL;//�Ƚ������û�׼��ѹ���ƼĴ�������(CVRCTL)
     CMP->CVRCTL |= _02_COMP0_REFVOLTAGE_ENABLE;//�������û�׼��ѹ0������ 

	
	/*
	(3)����ACMP0 ����˺ͷ���˹���ģʽ
	*/	
    //�Ƚ���0�������ź�ѡ����ƼĴ�����CMPSEL0��:
    CMP_Input_Select  (CMP_CHANNEL_0, CMP_PSIDE_VCIN0,CMP_NSIDE_IVREF);//�Ƚ���0������ѡ���ⲿ��������(VCIN0����)������ѡ��ѡ�����û�׼��ѹVREF0

    
	/*
	(4)�����˲����
	*/
    //�Ƚ����˲����ƼĴ�����COMPFIR��
    //CMP_Filter_Setting(CMP_CHANNEL_0, CMP_FILTER_FCLK_8, CMP_EDGE_RISING);//�Ƚ���0���˲�Ƶ��ѡ��Fclk/8��˫���ؼ������ж�����
    CMP->COMPFIR |= CMP_EDGE_RISING << 2 | CMP_FILTER_FCLK_8 << 0;

	/*
	(5)�Ƚ����ж�
	*/
//    #if 0
//        CMP->COMPOCR =  _01_COMP0_INTERRPUT_ENABLE;		 //����Ƚ���0���ж�����	
//        INTC_EnableIRQ(CMP0_IRQn);          /* enable INTCMP0 interrupt */    
//        NVIC_SetPriority(CMP0_IRQn, 3); /* Set higher priority to execute slave ISR firstly */
//    #else
//        INTC_ClearPendingIRQ(CMP0_IRQn);    /* clear INTCMP0 interrupt flag */       
//    #endif


	/*
	(6)���ó���
	*/	 	 
//	*( uint8_t*)(0x4004384E) = 0x13;//����
	
    
	/*
	(7)����ACMP IO��
	*/
    /* Set VCIN0(P22) pin   �Ƚ����������������*/				
    PORT->PMC2  |= (1 << 2);  
          

	/*
	(8)����ACMP �������
	*/
    CMP->COMPOCR &= 0xfb;//���Ƚ���0����ʹ��λ����
    #if 1
        CMP->COMPOCR |=  _00_COMP0_OUTPUT_NORMAL_TO_VCOUT0;		    //VCOUT0���бȽ���0�����
    #else
        CMP->COMPOCR |=  _04_COMP0_OUTPUT_INVERTED_TO_VCOUT0;		 //VCOUT0���бȽ���0�ķ������   
    #endif    
	/*
	(9)����ACMP0
	*/
    CMP->COMPMDR |= _01_COMP0_OPERATION_ENABLED;//ʹ�ܱȽ���0������
}


/*************************************************
Description: ACMP1_ConfigInit
Input      : 
Return     : 
Others     : 
*************************************************/
void ACMP1_ConfigInit(void)
{
     
	/*
	(1)���ñȽ���������ʱ��Դ
	 */
    CGC->PER1 |= CGC_PER1_PGACMPEN_Msk;    /* enables input clock supply */
    
    
	/*
	 (2)����ACMP1 ��׼��ѹֵ 
	*/	
    CMP_VREF_Set_Value(CMP_CHANNEL_1, 128);//�������û�׼��ѹֵ   VDD��5.0V�� / 256 * 128 = 2.5V 
    //CMP_VREF_Init     (CMP_CHANNEL_1, CMP_VDD_VSS_REFERENCE_VOLTAGE);//�Ƚ������û�׼��ѹ���ƼĴ�������(CVRCTL)
    
     CMP->CVRCTL |= _00_VDD_INTERNAL_REFERVOL | _00_VSS_INTERNAL_REFERVOL;//�Ƚ������û�׼��ѹ���ƼĴ�������(CVRCTL)
     CMP->CVRCTL |= _20_COMP1_REFVOLTAGE_ENABLE;//�������û�׼��ѹ1������

         	
	/*
	(3)����ACMP1 ����˺ͷ���˹���ģʽ
	*/	
    //�Ƚ���1�������ź�ѡ����ƼĴ�����CMPSEL1��:
    CMP_Input_Select  (CMP_CHANNEL_1, CMP_PSIDE_VCIN10,CMP_NSIDE_IVREF);//�Ƚ���1������ѡ���ⲿ��������(VCIN10����)������ѡ��ѡ�����û�׼��ѹVREF1
    
    
	/*
	(4)�����˲����
	*/
    //�Ƚ����˲����ƼĴ�����COMPFIR��
    //CMP_Filter_Setting(CMP_CHANNEL_1, CMP_FILTER_FCLK_8, CMP_EDGE_RISING);//�Ƚ���1���˲�Ƶ��ѡ��Fclk/8��˫���ؼ������ж�����
    CMP->COMPFIR |= CMP_EDGE_RISING << 6 | CMP_FILTER_FCLK_32 << 4;    
    
	/*
	(5)�Ƚ����ж�
	*/
//    #if 1
//        CMP->COMPOCR =  _10_COMP1_INTERRPUT_ENABLE;		 //����Ƚ���1���ж�����	
//        INTC_EnableIRQ(CMP1_IRQn);          /* enable INTCMP1 interrupt */    
//        NVIC_SetPriority(CMP1_IRQn, 0); /* Set higher priority to execute slave ISR firstly */
//    #else
//        INTC_ClearPendingIRQ(CMP1_IRQn);    /* clear INTCMP0 interrupt flag */       
//    #endif


	/*
	(6)���ó���
	*/	 	 
//	*( uint8_t*)(0x4004384E) = 0x13;//����
	
    
	/*
	(7)����ACMP IO��
	*/
    /* Set VCIN10(P00) pin   �Ƚ����������������*/				
    PORT->PMC0  |= (1 << 0);     

	/*
	(8)����ACMP �������
	*/
    CMP->COMPOCR &= 0xbf;//���Ƚ���1����ʹ��λ����
    #if 1
        CMP->COMPOCR |=  _00_COMP1_OUTPUT_NORMAL_TO_VCOUT1;		    //VCOUT1���бȽ���0�����
    #else
        CMP->COMPOCR |=  _40_COMP1_OUTPUT_INVERTED_TO_VCOUT1;		 //VCOUT1���бȽ���0�ķ������   
    #endif   
    
	/*
	(9)����ACMP1
	*/
    CMP->COMPMDR |= _10_COMP1_OPERATION_ENABLED;//ʹ�ܱȽ���1������
}




/*************************************************
Description: UART0_ConfigInit
Input      : 
Return     : 
Others     : UART0���ã������ʣ�1M
*************************************************/
void UART0_ConfigInit(void)
{
	
	/*
	(1)����UARTxʱ��
	*/
    CGC->PER0 |= CGC_PER0_SCI0EN_Msk;
    SCI0->SPS0 = (0 << SCI0_SPS0_PRS00_Pos) | (3 << SCI0_SPS0_PRS00_Pos);

    
	/*
	(2)����UARTxģʽ
	*/        
    /* transmission channel */
    SCI0->SMR00 = _0020_SMRMN_DEFAULT_VALUE | _0000_SCI_CLOCK_SELECT_CK00 | _0000_SCI_CLOCK_MODE_CKS |
                  _0002_SCI_MODE_UART | _0000_SCI_TRANSFER_END;
    SCI0->SCR00 = _0004_SCRMN_DEFAULT_VALUE | _8000_SCI_TRANSMISSION | _0000_SCI_TIMING_1 | _0000_SCI_INTSRE_MASK |
                  _0000_SCI_PARITY_NONE | _0080_SCI_LSB | _0010_SCI_STOP_1 | _0003_SCI_LENGTH_8;
    SCI0->SDR00 = _CE00_SCI_BAUDRATE_DIVISOR;
    /* reception channel */
    MISC->NFEN0 |= _01_SCI_RXD0_FILTER_ON;
    SCI0->SIR01 = _0004_SCI_SIRMN_FECTMN | _0002_SCI_SIRMN_PECTMN | _0001_SCI_SIRMN_OVCTMN;
    SCI0->SMR01 = _0020_SMRMN_DEFAULT_VALUE | _0000_SCI_CLOCK_SELECT_CK00 | _0000_SCI_CLOCK_MODE_CKS |
                  _0100_SCI_TRIGGER_RXD | _0000_SCI_EDGE_FALL | _0002_SCI_MODE_UART | _0000_SCI_TRANSFER_END;
    SCI0->SCR01 = _0004_SCRMN_DEFAULT_VALUE | _4000_SCI_RECEPTION | _0000_SCI_TIMING_1 | _0000_SCI_INTSRE_MASK |
                  _0000_SCI_PARITY_NONE | _0080_SCI_LSB | _0010_SCI_STOP_1 | _0003_SCI_LENGTH_8;
    SCI0->SDR01 = _CE00_SCI_BAUDRATE_DIVISOR;
    

	/*
	(3)UARTx��������
	*/	 
    /* Set TxD0 pin */
    PORT->PIOR0 &= ~(1 << 1);    /* allocate TXD0 to P51 */ 
    PORT->P5    |=  (1 << 1);    /* P51 output high level */ 
    PORT->PM5   &= ~(1 << 1);    /* P51 is used as TXD0 output */ 
    PORT->POM5  &= ~(1 << 1);    /* P51 is push-pull output mode */ 
    /* Set RxD0 pin */
    PORT->PIOR0 &= ~(1 << 1);    /* allocate RXD0 to P50 */ 
    PORT->PM5   |=  (1 << 0);    /* P50 is used as RXD0 input */ 


	/*
	(4)UARTx����������
	*/
    /* UART0 Start, Setting baud rate */
    //�����ʼ��㣺64M/(2^5)/((1)*2) = 2M/2 = 1M ������    
    SCI0->ST0 = _0002_SCI_CH1_STOP_TRG_ON | _0001_SCI_CH0_STOP_TRG_ON;
    SCI0->SPS0 &= ~SCI0_SPS0_PRS00_Msk;
    SCI0->SPS0 |=  _0000_SCI_CK01_fCLK_0 | 5;   //ע�� UART0 �� UART1 ���� SCI0->SPS0 �Ĵ���
    SCI0->SDR00 = 0 << 9;
    SCI0->SDR01 = 0 << 9;
//    SCI0->SDR00 = 103 << 9;
//    SCI0->SDR01 = 103 << 9;

	/*
	(5)����UARTx���
	*/	    
    /* output enable */
    SCI0->SO0 |= _0001_SCI_CH0_DATA_OUTPUT_1;
    SCI0->SOL0 &= (uint16_t)~_0001_SCI_CHANNEL0_INVERTED;
    SCI0->SOE0 |= _0001_SCI_CH0_OUTPUT_ENABLE;
    SCI0->SS0 |= _0002_SCI_CH1_START_TRG_ON | _0001_SCI_CH0_START_TRG_ON;
 
	/*
	(6)�����ж�
	*/    
    INTC_ClearPendingIRQ(ST0_IRQn); /* clear INTST0 interrupt flag */
    INTC_ClearPendingIRQ(SR0_IRQn); /* clear INTSR0 interrupt flag */
    NVIC_ClearPendingIRQ(ST0_IRQn); /* clear INTST0 interrupt flag */
    NVIC_ClearPendingIRQ(SR0_IRQn); /* clear INTSR0 interrupt flag */
    #if 0
        INTC_EnableIRQ(ST0_IRQn);       /* enable INTST0 interrupt */
        INTC_EnableIRQ(SR0_IRQn);       /* enable INTSR0 interrupt */
    #else
        INTC_DisableIRQ(ST0_IRQn);      /* disable INTST0 interrupt */
        INTC_DisableIRQ(SR0_IRQn);      /* disable INTSR0 interrupt */
    #endif
}


/*************************************************
Description: UART1_ConfigInit
Input      : 
Return     : 
Others     : UART1���ã��������ݷ��� ��������9600��
*************************************************/
void UART1_ConfigInit(void)
{
	/*
	(1)����UARTxʱ��
	*/	    
    CGC->PER0 |= CGC_PER0_SCI0EN_Msk;
    //��STmn��д��1�����ͽ�����ͨ������״̬�Ĵ���m��SEm���Ķ�Ӧλ��SEmn���塰0��������ֹͣ״̬����
    SCI0->ST0 |= _0008_SCI_CH3_STOP_TRG_ON | _0004_SCI_CH2_STOP_TRG_ON;


	/*
	(2)����UARTxģʽ
	*/    
    /* transmission channel */
    SCI0->SMR02 = _0020_SMRMN_DEFAULT_VALUE | _0000_SCI_CLOCK_SELECT_CK00 | _0000_SCI_CLOCK_MODE_CKS |
                  _0002_SCI_MODE_UART | _0000_SCI_TRANSFER_END;
    SCI0->SCR02 = _0004_SCRMN_DEFAULT_VALUE | _8000_SCI_TRANSMISSION | _0000_SCI_TIMING_1 | _0000_SCI_INTSRE_MASK |
                  _0000_SCI_PARITY_NONE | _0080_SCI_LSB | _0010_SCI_STOP_1 | _0003_SCI_LENGTH_8;
    SCI0->SDR02 = _CE00_SCI_BAUDRATE_DIVISOR;
    
    /* reception channel */
    MISC->NFEN0 |= _04_SCI_RXD1_FILTER_ON;
    SCI0->SIR03 = _0004_SCI_SIRMN_FECTMN | _0002_SCI_SIRMN_PECTMN | _0001_SCI_SIRMN_OVCTMN;
    SCI0->SMR03 = _0020_SMRMN_DEFAULT_VALUE | _0000_SCI_CLOCK_SELECT_CK00 | _0000_SCI_CLOCK_MODE_CKS |
                  _0100_SCI_TRIGGER_RXD | _0000_SCI_EDGE_FALL | _0002_SCI_MODE_UART | _0000_SCI_TRANSFER_END;
    SCI0->SCR03 = _0004_SCRMN_DEFAULT_VALUE | _4000_SCI_RECEPTION | _0000_SCI_TIMING_1 | _0000_SCI_INTSRE_MASK |
                  _0000_SCI_PARITY_NONE | _0080_SCI_LSB | _0010_SCI_STOP_1 | _0003_SCI_LENGTH_8;									
    SCI0->SDR03 = _CE00_SCI_BAUDRATE_DIVISOR;
    
    
	/*
	(3)UARTx��������
	*/	    
    /* Set TxD1 pin  P72 �ض����TXD1*/
    PORT->PIOR0 |=  (1 << 5);    /* P72 �ض����TXD1ģʽ */ \
    PORT->P7    |=  (1 << 2);    /* P72 output high level */ 
    PORT->PM7   &= ~(1 << 2);    /* P72 is used as TXD1 output */ 
    PORT->POM7  &= ~(1 << 2);    /* P72 is push-pull output mode */ 
    
    /* Set RxD1 pin   P73 �ض����RXD1*/
    PORT->PIOR0 |=  (1 << 5);    /* P73 �ض����RXD1ģʽ */ \
    PORT->PM7   |=  (1 << 3);    /* P73 is used as RXD1 input */ 

	/*
	(4)UARTx����������
	*/	
    /* UART1 Start, Setting baud rate */
    //�����ʼ��㣺64M/(2^5)/((103+1)*2) = 2M/208 = 9615 �� 9600 ������
    SCI0->SPS0 |= _0000_SCI_CK01_fCLK_0 | 5;     //ע�� UART0 �� UART1 ���� SCI0->SPS0 �Ĵ���
    SCI0->SDR02 = 51 << 9;
    SCI0->SDR03 = 51 << 9;


	/*
	(5)����UARTx���
	*/	    
    /* output enable */
    SCI0->SO0 |= _0004_SCI_CH2_DATA_OUTPUT_1;
    SCI0->SOL0 &= (uint16_t)~_0004_SCI_CHANNEL2_INVERTED;
    SCI0->SOE0 |= _0004_SCI_CH2_OUTPUT_ENABLE;//�趨�������ֹͣ��ͨ���Ĵ���ͨ�ŵ������ ����   
    SCI0->SS0 |= _0008_SCI_CH3_START_TRG_ON | _0004_SCI_CH2_START_TRG_ON;//�����ͨ����ͨ��/��ʼ�����Ĵ����Ĵ���
    
//	/*
//	(6)�����ж�
//	*/    
//    INTC_ClearPendingIRQ(ST1_IRQn); /* clear INTST1 interrupt flag */
//    INTC_ClearPendingIRQ(SR1_IRQn); /* clear INTSR1 interrupt flag */
//    NVIC_ClearPendingIRQ(ST1_IRQn); /* clear INTST1 interrupt flag */
//    NVIC_ClearPendingIRQ(SR1_IRQn); /* clear INTSR1 interrupt flag */
//    #if 0
//        INTC_EnableIRQ(ST1_IRQn);       /* enable INTST1 interrupt */
//        INTC_EnableIRQ(SR1_IRQn);       /* enable INTSR1 interrupt */
//    #else
//        INTC_DisableIRQ(ST1_IRQn);      /* disable INTST1 interrupt */
//        INTC_DisableIRQ(SR1_IRQn);      /* disable INTSR1 interrupt */
//    #endif
}


/*************************************************
Description: Sys_IE_Init
Input      : 
Return     : 
Others     : �ж�ʹ�ܳ�ʼ��
*************************************************/
void Sys_IE_Init(void)
{

	/* Enable TMM0_INT (0�����ȼ����; 3:���ȼ����)*/
    INTC_ClearPendingIRQ(TMM0_IRQn);     /* clear INTTMM0 interrupt flag */
    NVIC_ClearPendingIRQ(TMM0_IRQn);     /* clear INTTMM0 interrupt flag */ 
    #if 0
        INTC_EnableIRQ(TMM0_IRQn);       /* enable INTTMM0 interrupt */
        NVIC_SetPriority(TMM0_IRQn, 2); /* Set higher priority to execute slave ISR firstly */
    #else
        INTC_DisableIRQ(TMM0_IRQn);      /* disable INTTMM0 interrupt */     
    #endif


	/* Enable TMM1_INT (0�����ȼ����; 3:���ȼ����)*/
    INTC_ClearPendingIRQ(TMM1_IRQn);     /* clear INTTMM1 interrupt flag */
    NVIC_ClearPendingIRQ(TMM1_IRQn);     /* clear INTTMM1 interrupt flag */ 
    #if 1
        INTC_EnableIRQ(TMM1_IRQn);       /* enable INTTMM1 interrupt */
        NVIC_SetPriority(TMM1_IRQn, 2); /* Set higher priority to execute slave ISR firstly */
    #else
        INTC_DisableIRQ(TMM1_IRQn);      /* disable INTTMM1 interrupt */     
    #endif
    

	/* Enable ADC_INT (0�����ȼ����; 3:���ȼ����)*/	
    INTC_ClearPendingIRQ(ADC_IRQn);     /* clear INTAD interrupt flag */
    NVIC_ClearPendingIRQ(ADC_IRQn);     /* clear INTAD interrupt flag */ 
    #if 1
        INTC_EnableIRQ(ADC_IRQn);       /* enable INTAD interrupt */
        NVIC_SetPriority(ADC_IRQn, 1); /* Set higher priority to execute slave ISR firstly */
    #else
        INTC_DisableIRQ(ADC_IRQn);      /* disable INTAD interrupt */     
    #endif


	/* Enable ACMP0_INT (0�����ȼ����; 3:���ȼ����)*/
    INTC_ClearPendingIRQ(CMP0_IRQn);     /* clear INTCMP0 interrupt flag */
    NVIC_ClearPendingIRQ(CMP0_IRQn);     /* clear INTCMP0 interrupt flag */ 
    #if 0
        CMP->COMPOCR =  _01_COMP0_INTERRPUT_ENABLE;		 //����Ƚ���0���ж�����	
        INTC_EnableIRQ(CMP0_IRQn);      /* enable INTCMP0 interrupt */    
        NVIC_SetPriority(CMP0_IRQn, 0); /* Set higher priority to execute slave ISR firstly */
    #else
        INTC_ClearPendingIRQ(CMP0_IRQn);    /* clear INTCMP0 interrupt flag */       
    #endif
	
	
	/* Enable ACMP1_INT (0�����ȼ����; 3:���ȼ����)*/
    INTC_ClearPendingIRQ(CMP1_IRQn);     /* clear INTCMP1 interrupt flag */
    NVIC_ClearPendingIRQ(CMP1_IRQn);     /* clear INTCMP1 interrupt flag */     
    #if 0
        CMP->COMPOCR =  _10_COMP1_INTERRPUT_ENABLE;		 //����Ƚ���1���ж�����	
        NVIC_SetPriority(CMP1_IRQn, 0); /* Set higher priority to execute slave ISR firstly */
        INTC_EnableIRQ(CMP1_IRQn);          /* enable INTCMP1 interrupt */    
    #else
        INTC_ClearPendingIRQ(CMP1_IRQn);    /* clear INTCMP1 interrupt flag */       
    #endif


	/* Enable UART0_INT (0�����ȼ����; 3:���ȼ����)*/
    INTC_ClearPendingIRQ(ST0_IRQn); /* clear INTST0 interrupt flag */
    INTC_ClearPendingIRQ(SR0_IRQn); /* clear INTSR0 interrupt flag */
    NVIC_ClearPendingIRQ(ST0_IRQn); /* clear INTST0 interrupt flag */
    NVIC_ClearPendingIRQ(SR0_IRQn); /* clear INTSR0 interrupt flag */
    #if 0
        INTC_EnableIRQ(ST0_IRQn);       /* enable INTST0 interrupt */
        INTC_EnableIRQ(SR0_IRQn);       /* enable INTSR0 interrupt */
        NVIC_SetPriority(SR0_IRQn, 2); /* Set higher priority to execute slave ISR firstly */        
    #else
        INTC_DisableIRQ(ST0_IRQn);      /* disable INTST0 interrupt */
        INTC_DisableIRQ(SR0_IRQn);      /* disable INTSR0 interrupt */
    #endif
    
    
	/* Enable UART1_INT (0�����ȼ����; 3:���ȼ����)*/
    INTC_ClearPendingIRQ(ST1_IRQn); /* clear INTST1 interrupt flag */
    INTC_ClearPendingIRQ(SR1_IRQn); /* clear INTSR1 interrupt flag */
    NVIC_ClearPendingIRQ(ST1_IRQn); /* clear INTST1 interrupt flag */
    NVIC_ClearPendingIRQ(SR1_IRQn); /* clear INTSR1 interrupt flag */
    #if 0
        //INTC_EnableIRQ(ST1_IRQn);       /* enable INTST1 interrupt */
        INTC_EnableIRQ(SR1_IRQn);       /* enable INTSR1 interrupt */   
        NVIC_SetPriority(SR1_IRQn, 2); /* Set higher priority to execute slave ISR firstly */
    #else
        INTC_DisableIRQ(ST1_IRQn);      /* disable INTST1 interrupt */
        INTC_DisableIRQ(SR1_IRQn);      /* disable INTSR1 interrupt */
    #endif    
    
}

  
/*************************************************
Description: Sys_HardConfigInit
Input      : 
Return     : 
Others     : ϵͳӲ�����ó�ʼ��
*************************************************/
extern  uint8_t u8_SendDataDebug[9] ,txBuf[24];

void Sys_HardConfigInit(void)
{
    
    
    //ϵͳʱ�ӳ�ʼ�� 	
    SystemClock_Init();
    
    // IO�˿ڳ�ʼ������
    GPIO_Init();
    
    //�δ�ʱ����ʼ��
    SysTick_ConfigInit();	
   
    // DMA��ʼ������UART0_TXD
    DMA_Start(DMA_VECTOR_ST0, 0, DMA_MODE_NORMAL, DMA_SIZE_BYTE, 24, (void *)&txBuf[1], (void *)&SCI0->TXD0);

    // DMA��ʼ������UART1_TXD
    DMA_Start(DMA_VECTOR_ST1, 2, DMA_MODE_NORMAL, DMA_SIZE_BYTE, UART1_SEND_NUM, (void *)&UART1_Info.TXD_B[1], (void *)&SCI0->TXD1);
    
    // DMA��ʼ������UART1_RXD
    DMA_Start(DMA_VECTOR_SR1, 1, DMA_MODE_NORMAL, DMA_SIZE_BYTE, UART1_RECE_NUM, (void *)&SCI0->RXD1, (void *)&UART1_Info.RXD_B[0]);
    
    // DMA��ʼ������ADC
	DMA_Start(DMA_VECTOR_ADC, 5, DMA_MODE_NORMAL, DMA_SIZE_HALF, 4, (void *)&ADC->ADCR, (void *)&ADSampleDMA_Info.u16Buff[0]);


    //����У׼��ʼ��
//    AD_CorrectInit(); 
       
    //UART0��ʼ�� 
    UART0_ConfigInit();
    
    //UART1��ʼ�� 
    UART1_ConfigInit();
     
    //EPWM��ʼ��  
    ADC_ConfigInit();  
    
    CMP->COMPFIR = 0 ;
    
    //�Ƚ���1��ʼ��    
    ACMP1_ConfigInit();
    
    //�Ƚ���0��ʼ��    
    ACMP0_ConfigInit();

    TIMC_ConfigInit();
    
    //EPWM��ʼ��    
    EPWM_ConfigInit();
          
    //�ж�ʹ�ܳ�ʼ��
    Sys_IE_Init();       
      
    
    INV_RY1_DISABLE;
    //PFC_RY2_DISABLE; 
    PFC_RY2_ENABLE;//�ϵ�Ĭ�Ͽ���״̬
    INV_RY3_DISABLE;        
    
    INV_START_DISABLE;  
    
    User_DelayTime_ms(20);
    ADC->ADM0 |= ADCS;          //����ת������
  
}




