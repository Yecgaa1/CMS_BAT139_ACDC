#include "change.h"
#include "stdio.h"
#include "string.h"
// ��ŵ���Ʊ�־λ
int isAllowCHG = 0; // 0���������磬1��������
volatile int32_t Duty_Out1 = 0;
int32_t save[1024] = {0};
volatile uint32_t save_cnt = 0;
volatile uint32_t isONPWM=1;
int32_t Get_PLL_Sin(PLL_Ctrl_Var_t *PLL_Info)
{
    return (PLL_Info->i32SinTheta) >> 3;
}
int32_t Get_PLL_Cos(PLL_Ctrl_Var_t *PLL_Info)
{
    return (PLL_Info->i32CosTheta) >> 3;
}
char txChar[24] = {0};
char txCharL[40] = {0};
void Function_TxSendDebug_INT(int32_t data)
{
    sprintf((char *)txChar, "%d\r\n", data);

    DMAVEC->CTRL[0].DMSAR = (uint32_t)(txChar + 1);
    DMAVEC->CTRL[0].DMACT = strlen(txChar) - 1; // ����8������
    DMA->DMAEN1 |= 1 << 4;                      // uart1                                  //ʹ�ܴ���(UART1)
    SCI0->TXD0 = (uint8_t)txChar[0];
}
void Function_TxSendDebug_Float(float data)
{
    sprintf((char *)txChar, "%.4f\n", data);

    DMAVEC->CTRL[0].DMSAR = (uint32_t)(txChar + 1);
    DMAVEC->CTRL[0].DMACT = strlen(txChar) - 1; // ����8������
    DMA->DMAEN1 |= 1 << 4;                      // uart1                                  //ʹ�ܴ���(UART1)
    SCI0->TXD0 = (uint8_t)txChar[0];
}
void Function_TxSendDebug_Two_Float(float data1, float data2)
{
    sprintf((char *)txChar, "%.2f,%.2f\n", data1, data2);

    DMAVEC->CTRL[0].DMSAR = (uint32_t)(txChar + 1);
    DMAVEC->CTRL[0].DMACT = strlen(txChar) - 1; // ����8������
    DMA->DMAEN1 |= 1 << 4;                      // uart1                                  //ʹ�ܴ���(UART1)
    SCI0->TXD0 = (uint8_t)txChar[0];
}
void Function_TxSendDebug_Four_Float(int32_t data1, int32_t data2, int32_t data3, int32_t data4)
{
    sprintf((char *)txCharL, "%d\n%d\n%d\n%d\n", data1, data2, data3, data4);

    DMAVEC->CTRL[0].DMSAR = (uint32_t)(txCharL + 1);
    DMAVEC->CTRL[0].DMACT = strlen(txCharL) - 1; // ����8������
    DMA->DMAEN1 |= 1 << 4;                      // uart1                                  //ʹ�ܴ���(UART1)
    SCI0->TXD0 = (uint8_t)txCharL[0];
}

void DebugUse(void)
{
    while (1)
    {
        /* code */
        WDT->WDTE = 0xACU;

        Duty_Out1 = (int32_t)(4096 * Sine_Table_50Hz[INV_Ctrl_Info.periodDot_Cnt]) >> 12;
        INV_Ctrl_Info.PWM_Duty = (int32_t)((Duty_Out1 * (INV_Ctrl_Info.PWM_Period)) >> 12);  // -period��-period�Ĺ�������
        INV_Ctrl_Info.PWM_DutyB = (-INV_Ctrl_Info.PWM_Duty);                                 // �ĸ�
        INV_Ctrl_Info.PWM_Duty = (INV_Ctrl_Info.PWM_Duty + INV_Ctrl_Info.PWM_Period) >> 1;   // 0��period����
        INV_Ctrl_Info.PWM_DutyB = (INV_Ctrl_Info.PWM_DutyB + INV_Ctrl_Info.PWM_Period) >> 1; // 0��period����

        // ռ�ձȡ����ڸ���
        TMM->TMGRA0 = INV_Ctrl_Info.PWM_Period; // Set pwm period
        TMM->TMGRD0 = INV_Ctrl_Info.PWM_DutyB;  // ����Ϊ�������
                                                // TMM->TMGRD0 = INV_Ctrl_Info.PWM_DutyB;//PWM2�����ռ�ձ�����   TMGRC1�Ĵ���ΪTMGRA1�Ĵ����Ļ���Ĵ���
        TMM->TMGRC1 = INV_Ctrl_Info.PWM_Duty;   // PWM2�����ռ�ձ�����   TMGRC1�Ĵ���ΪTMGRA1�Ĵ����Ļ���Ĵ���

        if (COM_Ctr_Info.PWM_Enable == 0)
        {
            // ��INV״̬ʱPWM��
            TMM->TMOER1 = _01_TMM_TMIOA0_OUTPUT_DISABLE | _00_TMM_TMIOB0_OUTPUT_ENABLE | _00_TMM_TMIOC0_OUTPUT_ENABLE | _00_TMM_TMIOD0_OUTPUT_ENABLE |
                          _00_TMM_TMIOA1_OUTPUT_ENABLE | _20_TMM_TMIOB1_OUTPUT_DISABLE | _00_TMM_TMIOC1_OUTPUT_ENABLE | _80_TMM_TMIOD1_OUTPUT_DISABLE;

            INV_Ctrl_Info.periodDot_Cnt = 0;
            COM_Ctr_Info.PWM_Enable = 1;
            TMM->TM0 = 0;
            TMM->TM1 = 0;
        }
    }
}
