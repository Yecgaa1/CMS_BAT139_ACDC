#include "stdint.h"
#include "sys_mcu_header.h"
#include "PLL_Ctrl.h"
extern int isAllowCHG;
int32_t Get_PLL_Sin(PLL_Ctrl_Var_t *PLL_Info);
void Function_TxSendDebug_INT(int32_t data);
void Function_TxSendDebug_Two_Float(float data1, float data2);
void Function_TxSendDebug_Four_Float(int32_t data1, int32_t data2, int32_t data3, int32_t data4);
void Function_TxSendDebug_Float(float data);
void DebugUse(void);
#define BWRY_State 0
#define Enable_RY_CurPID (600 * COM_REAL_VBUS_SCAL)
#define Enable_PWM (0 * COM_REAL_VBUS_SCAL)
int32_t Get_PLL_Cos(PLL_Ctrl_Var_t *PLL_Info);
extern int32_t save[1024];
extern volatile uint32_t save_cnt,isONPWM;
