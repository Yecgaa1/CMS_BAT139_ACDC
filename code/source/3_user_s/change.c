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
