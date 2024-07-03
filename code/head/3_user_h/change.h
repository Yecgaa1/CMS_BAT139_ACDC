#include "stdint.h"
#include "sys_mcu_header.h"
#include "PLL_Ctrl.h"
extern int isAllowCHG;
int32_t Get_PLL_Sin(PLL_Ctrl_Var_t *PLL_Info);
void Function_TxSendDebug_INT(int32_t data);
void DebugUse(void);
#define BWRY_State 1
#define Enable_RY_CurPID (410 * COM_REAL_VBUS_SCAL)
#define Enable_PWM (440 * COM_REAL_VBUS_SCAL)
