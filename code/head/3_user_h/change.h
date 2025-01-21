#include "stdint.h"
#include "sys_mcu_header.h"
#include "PLL_Ctrl.h"
extern int isAllowCHG;
extern uint8_t is220V;

void INV_Ctrl_220V(void);
int32_t Get_PLL_Sin(PLL_Ctrl_Var_t *PLL_Info);
int32_t Get_PLL_Sin_WithARG(PLL_Ctrl_Var_t *PLL_Info, float ARG);
void Function_TxSendDebug_INT(int32_t data);
void Function_TxSendDebug_TWO_INT(int32_t data1, int32_t data2);
void Function_TxSendDebug_Four_INT(int32_t data1, int32_t data2, int32_t data3, int32_t data4);

void Function_TxSendDebug_Float(float data);
void Function_TxSendDebug_Two_Float(float data1, float data2);

void DebugUse(void);
#define BWRY_State 0
#define Enable_RY_CurPID (600 * COM_REAL_VBUS_SCAL)
#define Enable_PWM (0 * COM_REAL_VBUS_SCAL)
#define PWMDelay (0) // ms，不得大于2000，可以是0
#define PWMDelayCNT (PWMDelay * 24)

#define ENABLE_IN_UP 230 * COM_REAL_VACIN_RMS_SCAL   // 开始充电230是V，真实值
#define ENABLE_OUT_DN 198 * COM_REAL_VACIN_RMS_SCAL  // 开始放电198是V，真实值
#define DISABLE_IN_UP 220 * COM_REAL_VACIN_RMS_SCAL  // 停止充电220是V，真实值
#define DISABLE_OUT_DN 213 * COM_REAL_VACIN_RMS_SCAL // 停止放电203是V，真实值

#define I3_OUT 208 * COM_REAL_VACIN_RMS_SCAL
#define I4_OUT 203 * COM_REAL_VACIN_RMS_SCAL
#define I5_OUT 198 * COM_REAL_VACIN_RMS_SCAL
#define I6_OUT 193 * COM_REAL_VACIN_RMS_SCAL

int32_t Get_PLL_Cos(PLL_Ctrl_Var_t *PLL_Info);
extern int32_t save[512], save2[512];
extern volatile uint32_t save_cnt, isONPWM, send_cnt;

extern uint16_t PWM_CNT;

extern uint16_t NOK_CNT;
extern uint32_t periodDot_Val;
extern uint16_t why;

extern uint32_t Workms, Works, WorkMin;
