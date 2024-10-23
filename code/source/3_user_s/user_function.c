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
Description: User_DelayTime_ms
Input      :
Return     :
Others     : ms��ʱ����
*************************************************/
void User_DelayTime_ms(uint32_t delay)
{
    uint32_t i, j;

    for (i = 0; i < delay; i++)
    {
        for (j = 0; j < 10000; j++) // 1ms,64M
        {
            ;
        }
    }
}

/*************************************************
Description: User_DelayTime_us
Input      :
Return     :
Others     : us��ʱ����
*************************************************/
void User_DelayTime_us(uint32_t delay)
{
    uint32_t i, j;

    for (i = 0; i < delay; i++)
    {
        for (j = 0; j < 12; j++) // 1ms,64M
        {
            ;
        }
    }
}

/*************************************************
Description: COM_CloseDrive
Input      :
Return     :
Others     : ����ʱ�ر�����
*************************************************/
void COM_CloseDrive(void)
{
    // ĸ�߹�ѹ����������ʱ����ѹ�෢�͹����ź�
    if (COM_AD_Data_Info.vBus_Val_Fir >= COM_VBUS_OVP2_VAL || System_ProtectFlag_Info.all != 0)
    {
        VBUS_OVER_ENABLE;
        if (System_ProtectFlag_Info.all == 0)
            System_ProtectFlag_Info.bit.vBus_OVP = 1;
    }

    //    if (( INV_Ctrl_Info.periodDot_Cnt <=   5 ) ||\
//        ( INV_Ctrl_Info.periodDot_Cnt >= ( INV_Ctrl_Info.periodDot_Val - 5 )) ||\
//        ( INV_Ctrl_Info.periodDot_Cnt >= ( (INV_Ctrl_Info.periodDot_Val >>1) -5 ) && INV_Ctrl_Info.periodDot_Cnt <= ( (INV_Ctrl_Info.periodDot_Val >>1) +5 )) )  //�ڽӽ�������ʱ��ر�����
    {
        if (E_FALSE != System_ProtectFlag_Info.all)
        {
#if OPERATING_MODE == NORMAL_MODE

            COM_PWM_Disable(); // �ر�PWM���

            INV_RY1_DISABLE; // ������������
            INV_RY3_DISABLE;

            State_Context.flag.bit.fault_Occur = E_TRUE;
#endif
        }
        else
        {
            State_Context.flag.bit.fault_Occur = E_FALSE;
        }
    }
}

/*************************************************
Description: User_LED_Deal
Input      :
Return     :
Others     : LEDָʾ���߼�
*************************************************/
void User_LED_ms(LED_Ctr_Var_t *LED_Info)
{
    if (1 == LED_Info->flag_1ms)
    {
        LED_Info->period_Cnt++;
    }

    if (LED_Info->cycle_Cnt < LED_Info->cycle_Val || LED_Info->cycle_Val == 0)
    {
        if (LED_Info->period_Cnt < LED_Info->on_Val)
        {
            LED_GREEN_ON;
        }
        else if (LED_Info->period_Cnt < LED_Info->period_Val)
        {
            LED_GREEN_OFF;
        }
        else
        {
            LED_Info->period_Cnt = 0;
            if (LED_Info->cycle_Val > 0)
            {
                LED_Info->cycle_Cnt++;
            }
        }
    }
    else if (LED_Info->cycle_Cnt >= LED_Info->cycle_Val)
    {
        if (LED_Info->period_Cnt < (LED_Info->period_Val << 2))
        {
            LED_GREEN_OFF;
        }
        else
        {
            LED_Info->period_Cnt = 0;
            LED_Info->cycle_Cnt = 0;
        }
    }
}

/*************************************************
Description: User_LED_Deal
Input      :
Return     :
Others     : LEDָʾ���߼�
*************************************************/
void User_LED_Deal(void)
{
    LED_Ctr_Info.flag_1ms = SysClockBase_ms.LED_1ms;

    /*---------------------------��ʼ׼��״̬�̵���˸-------------------------------------------*/
    if (System_ProtectFlag_Info.all == 0 &&
        State_Context.state_Value <= COM_READY_STATE)
    {
        LED_Ctr_Info.on_Val = 250;
        LED_Ctr_Info.period_Val = 1500;
        User_LED_ms(&LED_Ctr_Info);
    }
    /*----------------------------����״̬�̵Ƴ���---------------------------------------------*/
    else if (COM_RUN_STATE == State_Context.state_Value)
    {
        LED_Ctr_Info.on_Val = 1000;
        LED_Ctr_Info.period_Val = 1500;
        User_LED_ms(&LED_Ctr_Info);
    }
    /*-------------------------------����״̬�̵��� 1HZ-------------------------------------*/
    else
    {
        LED_Ctr_Info.on_Val = 200;
        LED_Ctr_Info.period_Val = 400;
        switch (System_ProtectFlag_Info.all)
        {
        case 1: // ����  LED��˸2��
            if (LED_Ctr_Info.cycle_Cnt == 0)
                LED_Ctr_Info.cycle_Val = 2;
            User_LED_ms(&LED_Ctr_Info);
            break;
        case 2: // ĸ�߹�Ƿѹ  LED��˸3��
        case 4:
            if (LED_Ctr_Info.cycle_Cnt == 0)
                LED_Ctr_Info.cycle_Val = 3;
            User_LED_ms(&LED_Ctr_Info);
            break;
        case 8: // �����ѹ��Ƿѹ  LED��˸4��
        case 16:
            if (LED_Ctr_Info.cycle_Cnt == 0)
                LED_Ctr_Info.cycle_Val = 4;
            User_LED_ms(&LED_Ctr_Info);
            break;
        case 32: // �����ѹ��·  LED��˸5��
            if (LED_Ctr_Info.cycle_Cnt == 0)
                LED_Ctr_Info.cycle_Val = 5;
            User_LED_ms(&LED_Ctr_Info);
            break;
        case 64: // ������Դ��Ƿѹ  LED��˸6��
        case 128:
            if (LED_Ctr_Info.cycle_Cnt == 0)
                LED_Ctr_Info.cycle_Val = 6;
            User_LED_ms(&LED_Ctr_Info);
            break;
        case 256: // ����  LED��˸7��
            if (LED_Ctr_Info.cycle_Cnt == 0)
                LED_Ctr_Info.cycle_Val = 7;
            User_LED_ms(&LED_Ctr_Info);
            break;
        case 512:  //
        case 1024: // ����  LED��˸8��
            if (LED_Ctr_Info.cycle_Cnt == 0)
                LED_Ctr_Info.cycle_Val = 8;
            User_LED_ms(&LED_Ctr_Info);
            break;
        case 2048: // ��ʼ��ʧ��  LED��˸9��
            if (LED_Ctr_Info.cycle_Cnt == 0)
                LED_Ctr_Info.cycle_Val = 9;
            User_LED_ms(&LED_Ctr_Info);
            break;
        case 4096: // DCDC�����  LED��˸10��
            if (LED_Ctr_Info.cycle_Cnt == 0)
                LED_Ctr_Info.cycle_Val = 10;
            User_LED_ms(&LED_Ctr_Info);
            break;
        case 8192: // ͨ�Ź���  LED��˸11��
            if (LED_Ctr_Info.cycle_Cnt == 0)
                LED_Ctr_Info.cycle_Val = 11;
            User_LED_ms(&LED_Ctr_Info);
            break;
        case 16384: // VREF�쳣  LED��˸12��
            if (LED_Ctr_Info.cycle_Cnt == 0)
                LED_Ctr_Info.cycle_Val = 12;
            User_LED_ms(&LED_Ctr_Info);
            break;
        default: //  �̵���13��
            if (LED_Ctr_Info.cycle_Cnt == 0)
                LED_Ctr_Info.cycle_Val = 13;
            User_LED_ms(&LED_Ctr_Info);
            break;
        }
    }
    SysClockBase_ms.LED_1ms = 0;
    LED_Ctr_Info.flag_1ms = 0;
}

/*************************************************
Description: Key_Calc
Input      :
Return     :
Others     : �������ܳ����߼�����
*************************************************/
void Key_Calc(Key_Var_t *key)
{
    /*------------------------------------------------------------------------------------*/
    if (key->in == key->ref_Active_Level) // �����ƽ���趨��Ч��ƽһ��
    {
        if (key->release_Cnt > 0)
        {
            key->release_Cnt--;
        }
        if (key->close_Cnt < key->c_Delay_Val) // �����պϼ����Ƚ�
        {
            key->close_Cnt++;
        }
        else
        {
            key->close_Cnt = 0;
            key->out_State = E_KEY_CLOSE; // ��ʾ��������
        }
    }
    else // �����ƽ���趨��Ч��ƽ��һ��
    {
        if (key->close_Cnt > 0)
        {
            key->close_Cnt--;
        }
        if (key->release_Cnt < key->r_Delay_Val) // �����ɿ������Ƚ�
        {
            key->release_Cnt++;
        }
        else
        {
            key->release_Cnt = 0;
            key->out_State = E_KEY_RELEASE; // ��ʾ�����ɿ�
        }
    }
}

/*************************************************
Description: User_Key_Deal
Input      :
Return     :
Others     : ����״̬����
*************************************************/
uint8_t u8_Key_SW_InvOn_Flag = 0;
uint16_t u16_InvOn_Start_count, u16Inv_RY3_count = 0;
uint16_t u16INV_OFF_Flag = 0;
void User_Key_Deal(void)
{
    if (SysClockBase_ms.sys_1ms == 1)
    {
        // ���ػ���ť
        Key_SW_InvOn.in = INV_ON_SWITCH;
        Key_SW_InvOn.Calc(&Key_SW_InvOn);

        if (u16_InvOn_Start_count <= COM_INV_ON_DELAY_TIME)
        {
            u16_InvOn_Start_count++;
        }

        if (u16_InvOn_Start_count >= COM_INV_ON_DELAY_TIME) // ������ʱʱ��
        {
            // ��䰴ť�ź��߼�����
            if (Key_SW_InvOn.out_State == E_KEY_CLOSE && COM_Ctr_Info.INV_Enable_Flag == 0 && u8_Key_SW_InvOn_Flag == 0)
            {
                u8_Key_SW_InvOn_Flag = 1;         // ��0����ʾ�����ɿ�����1����ʾ����δ�ɿ�
                COM_Ctr_Info.INV_Enable_Flag = 1; // �������
                u16INV_OFF_Flag = 0;
            }
            else if (Key_SW_InvOn.out_State == E_KEY_CLOSE && COM_Ctr_Info.INV_Enable_Flag == 1 && u8_Key_SW_InvOn_Flag == 0)
            {
                u8_Key_SW_InvOn_Flag = 1; // ��0����ʾ�����ɿ�����1����ʾ����δ�ɿ�
                // COM_Ctr_Info.INV_Enable_Flag = 0;//�������
                u16INV_OFF_Flag = 1;
            }
        }

        // INV���ذ�ť�ɿ���־λ
        if (Key_SW_InvOn.out_State == E_KEY_RELEASE)
        {
            u8_Key_SW_InvOn_Flag = 0; // ��0����ʾ�����ɿ�����1����ʾ����δ�ɿ�
        }
    }

    // ��INVģʽʱ����������ʱֱ�ӷ��
    // ��PFCģʽʱ����������ʱ�����
    if (u16INV_OFF_Flag == 1)
    {
        if (ADSample_Info.INV_AC_Vol_AD_FIR <= 100 && ADSample_Info.INV_AC_Vol_AD_FIR >= -100)
        {
            if (COM_Ctr_Info.INV_PFC_Mode_Select == INV_MODE) // INVģʽ���ŵ�
            {
                if (OPERATING_MODE == DEBUG_MODE)
                    COM_Ctr_Info.PWM_Enable = 0;

                COM_Ctr_Info.INV_Enable_Flag = 0; // �������
                COM_PWM_Disable();                // ��INVģʽʱ�ر�PWM��
            }
            else
                COM_Ctr_Info.INV_Enable_Flag = 0; // �������
        }
    }

    // ���������̵���
    if (COM_Ctr_Info.INV_Enable_Flag == 1 && System_ProtectFlag_Info.all == 0)
    {
        // if (INV_RY1_STATE == 0)
        // {
        //     u16Inv_RY3_count = 0;
        //     INV_RY1_ENABLE; // ������������
        //     INV_RY3_ENABLE; // ������������
        // }
        // else
        // {
        //     if (SysClockBase_ms.sys_1ms == 1)
        //         u16Inv_RY3_count++;
        //     if (u16Inv_RY3_count >= 20)
        //     {
        //         u16Inv_RY3_count = 20;
        //         // INV_RY1_DISABLE;
        //     }
        // }
    }
    else
    {
        COM_Ctr_Info.INV_Enable_Flag = 0;
        // INV_RY1_DISABLE;//������������
        // INV_RY3_DISABLE;
    }
}

/*************************************************
Description: User_RMS_Deal
Input      :
Return     :
Others     : �����ź�rms����
*************************************************/
void User_RMS_Deal(void)
{
    // ���ʡ���ѹ������Чֵ����
    RMS_PQ_Update(&RMS_PQ_Info);

    INV_Ctrl_Info.active_Power = RMS_PQ_Info.out_P; // �й�����
    PFC_Ctrl_Info.active_Power = RMS_PQ_Info.out_P;
    PFC_Ctrl_Info.RMS_InductorCur = RMS_PQ_Info.out_RMS_CurInduc; // ��е�����Чֵ

    //    PFC_Ctrl_Info.RMS_Vol           = RMS_PQ_Info.out_RMS_PFC_AC_Vol;//
    PFC_Ctrl_Info.RMS_Vol = (RMS_PQ_Info.out_RMS_PFC_AC_Vol << 2) / COM_REAL_VACIN_RMS_SCAL; //
}

/*************************************************
Description: User_Divider
Input      :
Return     :
Others     : Ӳ��������
*************************************************/
int32_t User_Divider(int32_t dividend, int32_t divisor)
{
    uint32_t quotient;

    DIV->DIVIDEND = dividend;
    DIV->DIVISOR = divisor;
    while (DIV->STATUS & (1 << 8))
        ;
    quotient = DIV->QUOTIENT;
    return quotient;
}

/*************************************************
Description: COM_UART1_Deal
Input      :
Return     :
Others     : ����1���ݴ���
*************************************************/
uint8_t *UART1_RXD_B_Temp;
void COM_UART1_Deal(void)
{
    /*------------------------ͨ�����ݽ���-----------------------------------------------------*/
    /*---------------------8λ����ת��Ϊ16λ��ʽ------------------------------------------*/
    if (DMAVEC->CTRL[1].DMACT == 0)
    {
        for (int i = 0; i <= UART1_SEND_NUM; i++)
        {
            if (UART1_Info.RXD_B[i] == 0XAA && UART1_Info.RXD_B[i + 1] == 0X55)
            {
                UART1_RXD_B_Temp = &(UART1_Info.RXD_B[i]);
                break;
            }
        }

        // CRCУ�����ô���
        for (int i = 0; i < (UART1_SEND_NUM - 2); i++)
        {
            UART1_Info.RXD_CRC_Calc = crc16_modbus(&UART1_RXD_B_Temp[i], 1, UART1_Info.RXD_CRC_Init);
            UART1_Info.RXD_CRC_Init = UART1_Info.RXD_CRC_Calc;
        }
        UART1_Info.RXD_CRC_Init = 0xffff; // ��ʼ��CRC�Ĵ�����ʼֵ

        // ���յ�CRCУ���봦��
        UART1_Info.RXD_CRC_CkCd = (UART1_RXD_B_Temp[UART1_SEND_NUM - 2]) +
                                  (UART1_RXD_B_Temp[UART1_SEND_NUM - 1] << 8);

        // �����CRCУ��������յ�CRCУ�������ȽϺ�֡ͷ�ж�(0X55AA)
        if (UART1_Info.RXD_CRC_Calc == UART1_Info.RXD_CRC_CkCd &&
            UART1_RXD_B_Temp[0] == 0XAA && UART1_RXD_B_Temp[1] == 0X55)
        {
            for (int i = 0; i < (UART1_SEND_NUM >> 1); i++)
            {
                UART1_Info.RXD_W[i] = UART1_RXD_B_Temp[2 * i] + (UART1_RXD_B_Temp[2 * i + 1] << 8);
            }

            /*------------------------------------------------------------------------------------*/
            /*---------------------UART1ͨ������λ�߼�--------------------------------------------*/
            if (UART1_Info.heart_Status == 0)
            {
                UART1_Info.heart_Status = 1;
            }
            else
            {
                UART1_Info.heart_Status = 0;
            }
        }

        DMAVEC->CTRL[1].DMACT = UART1_RECE_NUM;
        DMAVEC->CTRL[1].DMDAR = (uint32_t)(void *)&UART1_Info.RXD_B[0];
        DMA->DMAEN1 |= 1 << 5; // uart1_TXD                                 //ʹ�ܴ���(UART1)
    }

    if (SysClockBase_ms.sys_1ms == 1)
    {
        /*------------------------------------------------------------------------------------*/
        /*---------------------UART1ͨ���������----------------------------------------------*/
        COM_Uartx_HeartCheck(&UART1_Info);
    }

    /*------------------------------------------------------------------------------------*/
    /*---------------------UART1ͨ���쳣������--------------------------------------------*/
    if (UART1_Info.heart_OK_Status == 0) ////UART2��������ͨ���쳣ʱ RXD0_Status_Heart_OK����״̬Ϊ 0
    {
        if (System_ProtectFlag_Info.all == 0 && INV_Ctrl_Info.mode_Operate == NORMAL_MODE)
        {
            System_ProtectFlag_Info.all |= E_COMMUNICA_ERR;
        }
        UART1_Info.RXD_W[0] = 0; // 2�ֽ�֡ͷ���ݣ�
        UART1_Info.RXD_W[1] = 0; // ǰ������ģʽ��1-�ŵ磻2-���
        UART1_Info.RXD_W[2] = 0; // ǰ�����ϴ���
        UART1_Info.RXD_W[3] = 0; // ǰ������PFCĸ�ߵ�ѹREF
        UART1_Info.RXD_W[4] = 0; // ǰ�����״̬��1-�����ɣ�0-��Ҫ���
        UART1_Info.RXD_W[5] = 0;
        UART1_Info.RXD_W[6] = 0; // ǰ��׼��OK�źţ�1-��Դ׼��OK��0-׼�� NOK
    }

    UARTx_DC_Info.err_Code = UART1_Info.RXD_W[2];        // ǰ�����ϴ���
    UARTx_DC_Info.mode_State = UART1_Info.RXD_W[1];      // ǰ������ģʽ��1-�ŵ磻2-���
    UARTx_DC_Info.ready_State = UART1_Info.RXD_W[6];     // ǰ��׼��OK�źţ�1-��Դ׼��OK��0-׼�� NOK
    UARTx_DC_Info.vBus_SetVal = UART1_Info.RXD_W[3];     // ǰ������PFCĸ�ߵ�ѹREF
    UARTx_DC_Info.CHG_FinishState = UART1_Info.RXD_W[4]; // ǰ�����״̬��1-�����ɣ�0-��Ҫ���

    // ����ͨ�ţ�����������DCDC��
    if (UART1_Info.TXD_Period_Cnt >= UART1_Info.TXD_Period_Val)
    {
        INV_DataSend();
        UART1_Info.TXD_Period_Cnt = 0;
    }
}

/*************************************************
Description: COM_UPS_Deal
Input      :
Return     :
Others     : �е��Ƿ�OK�߼�����
*************************************************/
int16_t i16PFC_AC_Vol_AD_temp = 0;
void COM_UPS_Deal(void)
{
    i16PFC_AC_Vol_AD_temp = ADSample_Info.PFC_AC_Vol_AD_FIR;
    if (ADSample_Info.PFC_AC_Vol_AD_FIR < 0)
        i16PFC_AC_Vol_AD_temp = -ADSample_Info.PFC_AC_Vol_AD_FIR;

    if (i16PFC_AC_Vol_AD_temp <= PFC_V_ACIN_P_DN_NOK_4)
    {
        UPS_Ctr_Info.V_ACIN_NOK_Cnt++;
    }
    else
    {
        UPS_Ctr_Info.V_ACIN_NOK_Cnt = 0;
    }
    if ((i16PFC_AC_Vol_AD_temp <= PFC_V_ACIN_P_DN_NOK_8))
    {
        UPS_Ctr_Info.V_ACIN_NOK_Cnt_P++;
    }
    else
    {
        UPS_Ctr_Info.V_ACIN_NOK_Cnt_P = 0;
    }

    if ((i16PFC_AC_Vol_AD_temp <= PFC_V_ACIN_P_DN_NOK_16))
    {
        UPS_Ctr_Info.V_ACIN_NOK_Cnt_N++;
    }
    else
    {
        UPS_Ctr_Info.V_ACIN_NOK_Cnt_N = 0;
    }

    if (COM_AD_Data_Info.VACIN_RMS_Val_Fir > PFC_START_CHECK_AC_VOL_UP_BACK ||
        COM_AD_Data_Info.VACIN_RMS_Val_Fir < PFC_START_CHECK_AC_VOL_DN_BACK ||
        UPS_Ctr_Info.V_ACIN_NOK_Cnt_P >= (PFC_Ctrl_Info.periodDot_Val >> 2) ||
        UPS_Ctr_Info.V_ACIN_NOK_Cnt_N >= (PFC_Ctrl_Info.periodDot_Val >> 3) ||
        UPS_Ctr_Info.V_ACIN_NOK_Cnt >= (PFC_Ctrl_Info.periodDot_Val >> 1))
    {

        UPS_Ctr_Info.V_ACIN_OK = 0; // �е����OK��־
        UPS_Ctr_Info.V_ACIN_NOK = 1;
        UPS_Ctr_Info.delta_T = 0;          //
        UPS_Ctr_Info.lock_Phase_OK = 0;    // ����OK
        UPS_Ctr_Info.V_ACIN_NOK_Cnt_P = 0; // �������е����NOK����
        UPS_Ctr_Info.V_ACIN_NOK_Cnt_N = 0; // �������е����NOK����
        UPS_Ctr_Info.V_ACIN_NOK_Cnt = 0;
    }
}

/*************************************************
Description: COM_CHG_INV_Select
Input      :
Return     :
Others     : �����߷ŵ繦��ѡ��
*************************************************/
void COM_CHG_INV_Select(void)
{
    if (SysClockBase_ms.sys_Mode_1ms == 1)
    {
        if (System_ProtectFlag_Info.all == 0 &&
            State_Context.state_Value <= COM_RUN_STATE &&
            COM_AD_Data_Info.VACIN_Freq_Val_Fir > PFC_START_CHECK_FREQ_DN &&
            COM_AD_Data_Info.VACIN_Freq_Val_Fir < PFC_START_CHECK_FREQ_UP &&
            COM_AD_Data_Info.VACIN_RMS_Val_Fir >= PFC_START_CHECK_AC_VOL_DN_BACK) // �����ѹ��Χ��ʱ�����ÿ��㷨
        {
            COM_Ctr_Info.PFC_FREQ_NOKCnt = 0;
            if (COM_Ctr_Info.PFC_FREQ_Cnt < COM_Ctr_Info.PFC_FREQ_TimeVal)
            {
                COM_Ctr_Info.PFC_FREQ_Cnt++;
            }
            if (COM_Ctr_Info.PFC_FREQ_Cnt >= COM_Ctr_Info.PFC_FREQ_TimeVal)
            {
                // ע�⣺�жϵ�����Ӳ�����������治���ٵ���Ӳ����������(User_Divider)
                INV_Ctrl_Info.PWM_Freq_Init = 2000000L * INV_Ctrl_Info.periodDot_Val / INV_Ctrl_Info.PWM_Freq_Init_Temp;
                INV_Ctrl_Info.PWM_Period_Init = 64000000L / (INV_Ctrl_Info.PWM_Freq_Init * 2) - 2; // ������ʽ�����ǲ�
                INV_Ctrl_Info.V_ACIN_Freq_Init = INV_Ctrl_Info.PWM_Freq_Init;                      //(User_Divider(INV_Ctrl_Info.PWM_Freq_Init, INV_Ctrl_Info.periodDot_Val));
                PFC_Ctrl_Info.periodDot_Val = PFC_Ctrl_Info.AC_Vol_Freq;
                COM_Ctr_Info.PFC_FREQ_Cnt = 0;
                COM_Ctr_Info.PFC_FREQ_State = 1;
            }
        }
        if (COM_AD_Data_Info.VACIN_Freq_Val_Fir < PFC_START_CHECK_FREQ_DN ||
            COM_AD_Data_Info.VACIN_Freq_Val_Fir > PFC_START_CHECK_FREQ_UP)
        {
            COM_Ctr_Info.PFC_FREQ_Cnt = 0;
            if (COM_Ctr_Info.PFC_FREQ_NOKCnt < COM_Ctr_Info.PFC_FREQ_TimeVal)
            {
                COM_Ctr_Info.PFC_FREQ_NOKCnt++;
            }
            if (COM_Ctr_Info.PFC_FREQ_NOKCnt >= 45) // COM_Ctr_Info.PFC_FREQ_TimeVal)
            {
                INV_Ctrl_Info.PWM_Freq_Init = INV_PWM_FREQ;
                INV_Ctrl_Info.PWM_Period_Init = INV_PWM_PERIOD;
                COM_Ctr_Info.PFC_FREQ_State = 0;
                COM_Ctr_Info.PFC_FREQ_NOKCnt = 0;
            }
        }

        // �е�����жϣ��趨����ģʽΪPFC״̬
        if (System_ProtectFlag_Info.all == 0 &&
            State_Context.state_Value <= COM_RUN_STATE &&
            COM_AD_Data_Info.VACIN_RMS_Val_Fir > PFC_START_CHECK_AC_VOL_DN &&
            COM_AD_Data_Info.VACIN_RMS_Val_Fir < PFC_START_CHECK_AC_VOL_UP &&
            COM_Ctr_Info.PFC_FREQ_State == 1 &&
            (COM_Ctr_Info.INV_PFC_Mode_Select == 1 || COM_Ctr_Info.INV_PFC_Mode_Select == 0) &&
            COM_AD_Data_Info.VACIN_Freq_Val_Fir > PFC_START_CHECK_FREQ_DN &&
            COM_AD_Data_Info.VACIN_Freq_Val_Fir < PFC_START_CHECK_FREQ_UP) // �����ѹ��Χ��ʱ�����ÿ��㷨
        {
            if (COM_Ctr_Info.PFC_AC_Vol_OK_Cnt < COM_Ctr_Info.PFC_AC_Vol_OK_TimeVal)
            {
                COM_Ctr_Info.PFC_AC_Vol_OK_Cnt++;
            }

            if (COM_Ctr_Info.PFC_AC_Vol_OK_Cnt >= COM_Ctr_Info.PFC_AC_Vol_OK_TimeVal)
            {
                UPS_Ctr_Info.V_ACIN_OK = 1; // �е�OK
                UPS_Ctr_Info.V_ACIN_NOK = 0;
            }

            if (COM_Ctr_Info.PFC_AC_Vol_OK_Cnt >= COM_Ctr_Info.PFC_AC_Vol_OK_TimeVal &&
                UPS_Ctr_Info.V_ACIN_OK == 1 &&
                (UPS_Ctr_Info.lock_Phase_OK == 1 || COM_Ctr_Info.INV_Enable_Flag == 0)) // ������ɻ���δ�������
            {
                INV_START_DISABLE;

                // ���½����ʼ���׶�
                COM_PWM_Disable(); // �ر�PWM������̵���

                // PFC_RY2_ENABLE;//UPS�̵�������     PASS
                INV_RY1_ENABLE; // ������������
                INV_RY3_ENABLE; // ������������

                StartCheck_Flag_Info.bit.AC_Vol_High_OK = E_FALSE;
                StartCheck_Flag_Info.bit.AC_Vol_Low_OK = E_FALSE;
                StartCheck_Flag_Info.bit.FREQ_High_OK = E_FALSE;
                StartCheck_Flag_Info.bit.FREQ_Low_OK = E_FALSE;

                Initial_Deal.flag.bit.Initial_Ok = E_FALSE;
                Ready_Deal.flag.bit.Ready_Ok = E_FALSE;
                Run_Deal.flag.bit.OpenDriver_Ok = E_FALSE;

                State_Context.flag.bit.initial_Ok = E_FALSE;
                State_Context.flag.bit.ready_Ok = E_FALSE;
                State_Context.state_Value = COM_WAITING_STATE;       // COM_READY_STATE;
                PFC_Ctrl_Info.vBus_Ref_SS = PFC_VBUS_REF_INIT_SS;    // ��ѹ���������ο�ֵ����
                PFC_Ref_Info.u32SS_vBus_Hold = PFC_VBUS_REF_INIT_SS; // PFC��ѹ���������ο�ֵ����
                COM_Ctr_Info.PFC_AC_Vol_OK_Cnt = 0;
                COM_Ctr_Info.INV_PFC_Mode_Select = 2; // ����е���������ģʽΪPFCģʽ
            }
        }
        else
        {
            COM_Ctr_Info.PFC_AC_Vol_OK_Cnt = 0;
        }

        // ��乤��ģʽ����
        if (System_ProtectFlag_Info.all == 0 &&
            State_Context.state_Value <= COM_RUN_STATE &&
            COM_Ctr_Info.INV_Enable_Flag == 1 &&
            (COM_AD_Data_Info.VACIN_RMS_Val_Fir > PFC_START_CHECK_AC_VOL_UP_BACK ||
             COM_AD_Data_Info.VACIN_RMS_Val_Fir < PFC_START_CHECK_AC_VOL_DN_BACK ||
             COM_Ctr_Info.PFC_FREQ_State == 0 ||
             UPS_Ctr_Info.V_ACIN_NOK == 1) &&
            (COM_Ctr_Info.INV_PFC_Mode_Select == 0 || COM_Ctr_Info.INV_PFC_Mode_Select == 2)) // �����ѹ��Χ��ʱ�����ÿ��㷨
        {
            if (COM_Ctr_Info.PFC_AC_Vol_NOK_Cnt < COM_Ctr_Info.PFC_AC_Vol_NOK_TimeVal)
                COM_Ctr_Info.PFC_AC_Vol_NOK_Cnt++;
            if (COM_Ctr_Info.PFC_AC_Vol_NOK_Cnt >= COM_Ctr_Info.PFC_AC_Vol_NOK_TimeVal || UPS_Ctr_Info.V_ACIN_NOK == 1)
            {
                INV_START_DISABLE;

                COM_Ctr_Info.PFC_FREQ_State = 0;

                // ���½����ʼ���׶�
                TMM->TMOER1 = _01_TMM_TMIOA0_OUTPUT_DISABLE | _02_TMM_TMIOB0_OUTPUT_DISABLE | _04_TMM_TMIOC0_OUTPUT_DISABLE | _08_TMM_TMIOD0_OUTPUT_DISABLE |
                              _10_TMM_TMIOA1_OUTPUT_DISABLE | _20_TMM_TMIOB1_OUTPUT_DISABLE | _40_TMM_TMIOC1_OUTPUT_DISABLE | _80_TMM_TMIOD1_OUTPUT_DISABLE;
                PORT->P1 &= ~(3 << 4); // ��Ƶ������

                // PFC_RY2_DISABLE;

                StartCheck_Flag_Info.bit.vBus_High_OK = E_FALSE;
                StartCheck_Flag_Info.bit.vBus_Low_OK = E_FALSE;

                //                    Initial_Deal.flag.bit.Initial_Ok        = E_FALSE;
                Ready_Deal.flag.bit.Ready_Ok = E_FALSE;
                Run_Deal.flag.bit.OpenDriver_Ok = E_FALSE;

                //                    State_Context.flag.bit.initial_Ok       = E_FALSE;
                State_Context.flag.bit.ready_Ok = E_FALSE;
                State_Context.state_Value = COM_WAITING_STATE;

                COM_Ctr_Info.PFC_AC_Vol_NOK_Cnt = 0;
                COM_Ctr_Info.INV_PFC_Mode_Select = 1; // ���ģʽ

                // �����״ֵ̬�ж�
                if ((COM_AD_Data_Info.auxPower_Val_Fir > COM_AUX_POWER_LVP_VAL &&
                     COM_AD_Data_Info.auxPower_Val_Fir < COM_AUX_POWER_OVP_VAL)) // �ж���乤����Դ�ڿ�����ⷶΧ��
                {
                    INV_START_ENABLE; // �������׼�����ź�
                }
            }
        }
        else
        {
            COM_Ctr_Info.PFC_AC_Vol_NOK_Cnt = 0;
        }

        // �趨����ģʽΪ����״̬
        if (System_ProtectFlag_Info.all == 0 &&
            State_Context.state_Value <= COM_RUN_STATE &&
            COM_Ctr_Info.INV_Enable_Flag == 0 &&
            (COM_AD_Data_Info.VACIN_RMS_Val_Fir > PFC_START_CHECK_AC_VOL_UP_BACK ||
             COM_AD_Data_Info.VACIN_RMS_Val_Fir < PFC_START_CHECK_AC_VOL_DN_BACK ||
             COM_Ctr_Info.PFC_FREQ_State == 0 ||
             UPS_Ctr_Info.V_ACIN_NOK == 1) &&
            (COM_Ctr_Info.INV_PFC_Mode_Select == 1 || COM_Ctr_Info.INV_PFC_Mode_Select == 2)) // �����ѹ��Χ��ʱ�����ÿ��㷨
        {
            if (COM_Ctr_Info.NO_Mode_OK_Cnt < COM_Ctr_Info.NO_Mode_OK_TimeVal)
            {
                COM_Ctr_Info.NO_Mode_OK_Cnt++;
            }

            if (COM_Ctr_Info.NO_Mode_OK_Cnt >= COM_Ctr_Info.NO_Mode_OK_TimeVal) //
            {
                // ���½����ʼ���׶�
                COM_PWM_Disable(); // �ر�PWM������̵���

                StartCheck_Flag_Info.bit.AC_Vol_High_OK = E_FALSE;
                StartCheck_Flag_Info.bit.AC_Vol_Low_OK = E_FALSE;
                StartCheck_Flag_Info.bit.FREQ_High_OK = E_FALSE;
                StartCheck_Flag_Info.bit.FREQ_Low_OK = E_FALSE;

                Initial_Deal.flag.bit.Initial_Ok = E_FALSE;
                Ready_Deal.flag.bit.Ready_Ok = E_FALSE;
                Run_Deal.flag.bit.OpenDriver_Ok = E_FALSE;

                State_Context.flag.bit.initial_Ok = E_FALSE;
                State_Context.flag.bit.ready_Ok = E_FALSE;
                State_Context.state_Value = COM_WAITING_STATE; // COM_READY_STATE;

                COM_Ctr_Info.NO_Mode_OK_Cnt = 0;
                COM_Ctr_Info.INV_PFC_Mode_Select = 0; // ��������ģʽΪ����ģʽ
                INV_START_DISABLE;
            }
        }
        else
        {
            COM_Ctr_Info.NO_Mode_OK_Cnt = 0;
        }
    }
    SysClockBase_ms.sys_Mode_1ms = 0;

    // ����Ƶ�ʱ仯ʵʱ���¹�Ƶ���ڼ���ֵ
    if (COM_AD_Data_Info.VACIN_Freq_Val_Fir >= PFC_START_CHECK_FREQ_DN &&
        COM_AD_Data_Info.VACIN_Freq_Val_Fir <= PFC_START_CHECK_FREQ_UP &&
        COM_Ctr_Info.INV_PFC_Mode_Select == PFC_MODE &&
        COM_RUN_STATE == State_Context.state_Value &&
        Run_Deal.flag.bit.OpenDriver_Ok == E_TRUE)
    {
        if (SysClockBase_ms.sys_1ms == 1)
        {
            if (PFC_Ctrl_Info.PFC_FreqOK_Cnt <= 1000)
                PFC_Ctrl_Info.PFC_FreqOK_Cnt++;
        }
        if (PFC_Ctrl_Info.PFC_FreqOK_Cnt >= 1000 && INV_Ctrl_Info.periodDot_Cnt == 0)
        {
            INV_Ctrl_Info.periodDot_Val = PFC_Ctrl_Info.AC_Vol_Freq;
        }
    }

    // ���ڳ��ģʽ��PFC_RY2_DISABLE
    if (COM_Ctr_Info.INV_PFC_Mode_Select != PFC_MODE)
    {
        // PFC_RY2_DISABLE;
    }
}

/*************************************************
Description: COM_AD_Data_Deal
Input      :
Return     :
Others     : AD������ʵ���ݴ���
*************************************************/
void COM_AD_Data_Deal(void)
{
    // AD������ʵ���ݴ���,����ʵֵ�����ϷŴ��趨����
    COM_AD_Data_Info.vRef_Val = ADSample_Info.ref_AD_Fir;             // ��׼VREF
    COM_AD_Data_Info.temp_NTC_Val = ADSample_Info.temp_NTC_AD_FIR;    // �¶�
    COM_AD_Data_Info.VACIN_RMS_Val = RMS_PQ_Info.out_RMS_PFC_AC_Vol;  // �е������ѹ��Чֵ
    COM_AD_Data_Info.VACOUT_RMS_Val = RMS_PQ_Info.out_RMS_INV_AC_Vol; // ��������ѹ��Чֵ
    COM_AD_Data_Info.iLoad_RMS_Val = RMS_PQ_Info.out_RMS_CurLoad;     // ���ص�����Чֵ
    COM_AD_Data_Info.iInduc_RMS_Val = RMS_PQ_Info.out_RMS_CurInduc;   // ��е�����Чֵ

    COM_AD_Data_Info.vBus_Val = ADSample_Info.vBus_AD_FIR * COM_REAL_VBUS_SCAL * COM_VBUS_BASE >> 12;                                 // ĸ�ߵ�ѹ
    COM_AD_Data_Info.auxPower_Val = ADSample_Info.auxPower_AD_FIR * COM_REAL_AUXPOWER_SCAL * COM_AUX_POWER_BASE >> 12;                // ������Դ��ѹ
    COM_AD_Data_Info.VACIN_Freq_Val = COM_MCU_CLK * COM_REAL_VACIN_FREQ_SCAL / (2 * (TMM->TMGRA0 + 2)) / PFC_Ctrl_Info.AC_Vol_Freq;   // �е������ѹƵ�ʣ�Hz��
    COM_AD_Data_Info.VACOUT_Freq_Val = COM_MCU_CLK * COM_REAL_VACOUT_FREQ_SCAL / (2 * (TMM->TMGRA0 + 2)) / INV_Ctrl_Info.AC_Vol_Freq; // ��������ѹƵ�ʣ�Hz��

    if (COM_AD_Data_Info.VACIN_Freq_Val > 700)
        COM_AD_Data_Info.VACIN_Freq_Val = 700;
    if (COM_AD_Data_Info.VACOUT_Freq_Val > 700)
        COM_AD_Data_Info.VACOUT_Freq_Val = 700;

    COM_AD_Data_Info.vBus_Hold = DFILTER_N(4, COM_AD_Data_Info.vBus_Val, COM_AD_Data_Info.vBus_Hold);
    COM_AD_Data_Info.vBus_Val_Fir = COM_AD_Data_Info.vBus_Hold >> 16; // ĸ�ߵ�ѹ

    COM_AD_Data_Info.vRef_Val_Fir = COM_AD_Data_Info.vRef_Val;         // VRef�ο�
    COM_AD_Data_Info.temp_NTC_Val_Fir = COM_AD_Data_Info.temp_NTC_Val; // �����¶�
    COM_AD_Data_Info.auxPower_Val_Fir = COM_AD_Data_Info.auxPower_Val; // ������Դ��ѹ

    COM_AD_Data_Info.VACIN_Freq_Val_Fir = COM_AD_Data_Info.VACIN_Freq_Val;   // �е������ѹƵ��
    COM_AD_Data_Info.VACOUT_Freq_Val_Fir = COM_AD_Data_Info.VACOUT_Freq_Val; // ��������ѹƵ��
    COM_AD_Data_Info.VACIN_RMS_Val_Fir = COM_AD_Data_Info.VACIN_RMS_Val;     // �е������ѹ��Чֵ
    COM_AD_Data_Info.VACOUT_RMS_Val_Fir = COM_AD_Data_Info.VACOUT_RMS_Val;   // ��������ѹ��Чֵ
    COM_AD_Data_Info.iLoad_RMS_Val_Fir = COM_AD_Data_Info.iLoad_RMS_Val;     // ���ص�����Чֵ
    COM_AD_Data_Info.iInduc_RMS_Val_Fir = COM_AD_Data_Info.iInduc_RMS_Val;   // ��е�����Чֵ

    COM_AD_Data_Info.VACIN_BypassPower = RMS_PQ_Info.out_S;    // �е�������·���ʣ�VA�����ص�����Чֵ*��ѹ��Чֵ
    COM_AD_Data_Info.VACIN_PFC_Power = RMS_PQ_Info.out_P;      // �е�����PFC���ʣ�W����е���*�����ѹ
    COM_AD_Data_Info.VACOUT_ApparentPower = RMS_PQ_Info.out_S; // ���������ڹ��ʣ�VA�����ص�����Чֵ*��ѹ��Чֵ
    COM_AD_Data_Info.VACOUT_ActivePower = RMS_PQ_Info.out_P;   // �������й����ʣ�W����е���*�����ѹ
}

/*************************************************
Description: COM_Function
Input      :
Return     :
Others     : �û�����ִ��
*************************************************/
uint8_t u8_System1msbit_cnt = 0;
void COM_Function(void)
{
    // ��ȡ״̬����״̬
    State_Context.ContextPtr(&State_Context);

    // ִ��״̬����Ӧ������
    State_Context_Task();

    // error�ر�����
    COM_CloseDrive();

    // �����ź�RMS����
    User_RMS_Deal();

    // LED״ָ̬ʾ
    User_LED_Deal();

    // KEY״̬����
    User_Key_Deal();

    // ����1���ݴ���
    COM_UART1_Deal();

    // �����߷ŵ繦��ѡ��
    COM_CHG_INV_Select();

    // AD������ʵ���ݴ���
    COM_AD_Data_Deal();

    if (SysClockBase_ms.sys_1ms == 1)
    {
        u8_System1msbit_cnt++; // 1msʱ��������Main�������ڵ����ڼ���
    }
    if (u8_System1msbit_cnt >= 2)
    {
        SysClockBase_ms.sys_1ms = 0; // 1ms ʱ�ӱ������
        u8_System1msbit_cnt = 0;     // 1msʱ��������Main�������ڵ����ڼ���ֵ����
    }
}
/*------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------*/
