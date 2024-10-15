#include "change.h"
#include "stdio.h"
#include "string.h"
// 充放电控制标志位
int isAllowCHG = 1; // 0即不允许充电，1即允许充电
volatile int32_t Duty_Out1 = 0;
int32_t save[512] = {0};
int32_t save2[512] = {0};
uint16_t PWM_CNT=0;
volatile uint32_t save_cnt = 0;
volatile uint32_t isONPWM = 1;
extern const int16_t Sin_Cos_Table[256];
int32_t Get_PLL_Sin_WithARG(PLL_Ctrl_Var_t *PLL_Info, float ARG)
{
    int offset = (int)(ARG * 1024 / 360 + 0.5); // 四舍五入
    // 原始的角度计算
     uint16_t u16Index = (uint16_t)(PLL_Info->i32Theta + 32768) >> 6;

    // 调整索引以 offset 度对应的偏移量
    u16Index = (u16Index + offset) & 0x03FF; // 确保索引在 0~1023 范围内
    int16_t sine_value;
    // 后续计算保持不变
    switch (u16Index & 0x0300)
    {
    /* 0~90度 */
    case 0x0200:
        sine_value = Sin_Cos_Table[(uint8_t)(u16Index)];
        break;

    /* 90~180度 */
    case 0x0300:
        sine_value = Sin_Cos_Table[(uint8_t)(0xFF - (uint8_t)(u16Index))];
        break;

    /* 180~270度 */
    case 0x0000:
        sine_value = -Sin_Cos_Table[(uint8_t)(u16Index)];
        break;

    /* 270~360度 */
    case 0x0100:
        sine_value = -Sin_Cos_Table[(uint8_t)(0xFF - (uint8_t)(u16Index))];
        break;

    default:
        sine_value = 0;
        break;
    }
    return sine_value >> 3;
}
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
    DMAVEC->CTRL[0].DMACT = strlen(txChar) - 1; // 传输8个数据
    DMA->DMAEN1 |= 1 << 4;                      // uart1                                  //使能传输(UART1)
    SCI0->TXD0 = (uint8_t)txChar[0];
}

void Function_TxSendDebug_TWO_INT(int32_t data1, int32_t data2)
{
    sprintf((char *)txCharL, "%d,%d\r\n", data1, data2);

    DMAVEC->CTRL[0].DMSAR = (uint32_t)(txCharL + 1);
    DMAVEC->CTRL[0].DMACT = strlen(txCharL) - 1; // 传输8个数据
    DMA->DMAEN1 |= 1 << 4;                       // uart1                                  //使能传输(UART1)
    SCI0->TXD0 = (uint8_t)txCharL[0];
}

void Function_TxSendDebug_Float(float data)
{
    sprintf((char *)txChar, "%.4f\n", data);

    DMAVEC->CTRL[0].DMSAR = (uint32_t)(txChar + 1);
    DMAVEC->CTRL[0].DMACT = strlen(txChar) - 1; // 传输8个数据
    DMA->DMAEN1 |= 1 << 4;                      // uart1                                  //使能传输(UART1)
    SCI0->TXD0 = (uint8_t)txChar[0];
}
void Function_TxSendDebug_Two_Float(float data1, float data2)
{
    sprintf((char *)txChar, "%.2f,%.2f\n", data1, data2);

    DMAVEC->CTRL[0].DMSAR = (uint32_t)(txChar + 1);
    DMAVEC->CTRL[0].DMACT = strlen(txChar) - 1; // 传输8个数据
    DMA->DMAEN1 |= 1 << 4;                      // uart1                                  //使能传输(UART1)
    SCI0->TXD0 = (uint8_t)txChar[0];
}
void Function_TxSendDebug_Four_Float(int32_t data1, int32_t data2, int32_t data3, int32_t data4)
{
    sprintf((char *)txCharL, "%d,%d,%d,%d\r\n", data1, data2, data3, data4);

    DMAVEC->CTRL[0].DMSAR = (uint32_t)(txCharL + 1);
    DMAVEC->CTRL[0].DMACT = strlen(txCharL) - 1; // 传输8个数据
    DMA->DMAEN1 |= 1 << 4;                       // uart1                                  //使能传输(UART1)
    SCI0->TXD0 = (uint8_t)txCharL[0];
}

void DebugUse(void)
{
    while (1)
    {
        /* code */
        WDT->WDTE = 0xACU;

        Duty_Out1 = (int32_t)(4096 * Sine_Table_50Hz[INV_Ctrl_Info.periodDot_Cnt]) >> 12;
        INV_Ctrl_Info.PWM_Duty = (int32_t)((Duty_Out1 * (INV_Ctrl_Info.PWM_Period)) >> 12);  // -period到-period的过零正弦
        INV_Ctrl_Info.PWM_DutyB = (-INV_Ctrl_Info.PWM_Duty);                                 // 改负
        INV_Ctrl_Info.PWM_Duty = (INV_Ctrl_Info.PWM_Duty + INV_Ctrl_Info.PWM_Period) >> 1;   // 0到period正弦
        INV_Ctrl_Info.PWM_DutyB = (INV_Ctrl_Info.PWM_DutyB + INV_Ctrl_Info.PWM_Period) >> 1; // 0到period正弦

        // 占空比、周期更新
        TMM->TMGRA0 = INV_Ctrl_Info.PWM_Period; // Set pwm period
        TMM->TMGRD0 = INV_Ctrl_Info.PWM_DutyB;  // 控制为互补输出
                                                // TMM->TMGRD0 = INV_Ctrl_Info.PWM_DutyB;//PWM2的输出占空比设置   TMGRC1寄存器为TMGRA1寄存器的缓冲寄存器
        TMM->TMGRC1 = INV_Ctrl_Info.PWM_Duty;   // PWM2的输出占空比设置   TMGRC1寄存器为TMGRA1寄存器的缓冲寄存器

        if (COM_Ctr_Info.PWM_Enable == 0 &&0)
        {
            // 打开INV状态时PWM口
            TMM->TMOER1 = _01_TMM_TMIOA0_OUTPUT_DISABLE | _00_TMM_TMIOB0_OUTPUT_ENABLE | _00_TMM_TMIOC0_OUTPUT_ENABLE | _00_TMM_TMIOD0_OUTPUT_ENABLE |
                          _00_TMM_TMIOA1_OUTPUT_ENABLE | _20_TMM_TMIOB1_OUTPUT_DISABLE | _00_TMM_TMIOC1_OUTPUT_ENABLE | _80_TMM_TMIOD1_OUTPUT_DISABLE;

            INV_Ctrl_Info.periodDot_Cnt = 0;
            COM_Ctr_Info.PWM_Enable = 1;
            TMM->TM0 = 0;
            TMM->TM1 = 0;
        }
    }
}
