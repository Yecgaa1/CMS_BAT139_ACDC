#include "change.h"
#include "stdio.h"
#include "string.h"
// ��ŵ���Ʊ�־λ
int isAllowCHG = 0; // 0���������磬1��������

int32_t Get_PLL_Sin(PLL_Ctrl_Var_t *PLL_Info)
{
    return (PLL_Info->i32SinTheta) >> 3;
}

char txChar[24] = {0};
void Function_TxSendDebug_INT(int32_t data)
{
    sprintf((char *)txChar, "\r\n%d\r\n", data);

    DMAVEC->CTRL[0].DMSAR = (uint32_t)(txChar + 1);
    DMAVEC->CTRL[0].DMACT = strlen(txChar) - 1; // ����8������
    DMA->DMAEN1 |= 1 << 4;                      // uart1                                  //ʹ�ܴ���(UART1)
    SCI0->TXD0 = (uint8_t)txChar[0];
}
void DebugUse(void)
{
    // ������
    INV_Ctrl_Info.PWM_Duty=300;

    // ռ�ձȡ����ڸ���
    TMM->TMGRA0 = INV_Ctrl_Info.PWM_Period; // Set pwm period
    TMM->TMGRD0 = INV_Ctrl_Info.PWM_Duty;   // ����Ϊ�������
    // TMM->TMGRD0 = INV_Ctrl_Info.PWM_DutyB;//PWM2�����ռ�ձ�����   TMGRC1�Ĵ���ΪTMGRA1�Ĵ����Ļ���Ĵ���
    TMM->TMGRC1 = INV_Ctrl_Info.PWM_Duty; // PWM2�����ռ�ձ�����   TMGRC1�Ĵ���ΪTMGRA1�Ĵ����Ļ���Ĵ���

    if (COM_Ctr_Info.PWM_Enable == 0)
    {
        // ��INV״̬ʱPWM��
        TMM->TMOER1 = _01_TMM_TMIOA0_OUTPUT_DISABLE | _00_TMM_TMIOB0_OUTPUT_ENABLE | _00_TMM_TMIOC0_OUTPUT_ENABLE | _00_TMM_TMIOD0_OUTPUT_ENABLE |
                      _00_TMM_TMIOA1_OUTPUT_ENABLE | _20_TMM_TMIOB1_OUTPUT_DISABLE | _00_TMM_TMIOC1_OUTPUT_ENABLE | _80_TMM_TMIOD1_OUTPUT_DISABLE;

        INV_Ctrl_Info.periodDot_Cnt = 0;
        COM_Ctr_Info.PWM_Enable = 1;
    }
}
