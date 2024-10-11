/*****************************************************************************
Copyright (C) 2021 China Micro Semiconductor Limited Company. All Rights Reserved.
@Description:
@Author     :
@Version    : 0.0.1
@History    :
@Attention  ：
*****************************************************************************/

/****************************************************************************/
/*----include files---------------------------------------------------------*/

#include "INV_ctrl.h"
#include "sys_mcu_header.h"
#include "sys_define_param.h"
#include "sys_state_machine.h"
#include "user_function.h"
#include "change.h"
/***************************************************************************/

/*************************************************
Description: INV_Lock_Phase
Input      :
Return     :
Others     : 接入市电锁相处理
*************************************************/
extern uint16_t u16_INV_Freq_Cnt, PFC_Freq_Time_Cnt;
extern uint8_t PFC_flag, INV_flag;

void INV_Lock_Phase(void)
{
    PLL_Ctrl_Info_V_ACIN.i32Theta_delta = PLL_Ctrl_Info_V_ACIN.i32Theta - PLL_Ctrl_Info_V_ACOUT.i32Theta;
    // 输出电压与市电锁相逻辑处理
    if (UPS_Ctr_Info.V_ACIN_OK == 1 && UPS_Ctr_Info.lock_Phase_OK == 0)
    {
        if (PLL_Ctrl_Info_V_ACIN.i32Theta_delta < 32767 && PLL_Ctrl_Info_V_ACIN.i32Theta_delta >= 0)
        {
            UPS_Ctr_Info.delta_P = PLL_Ctrl_Info_V_ACIN.i32Theta_delta;
            UPS_Ctr_Info.V_ACIN_HYS_ADV = 2; // 输出电压超前;市电波形滞后
        }
        else if (PLL_Ctrl_Info_V_ACIN.i32Theta_delta >= 32767 && PLL_Ctrl_Info_V_ACIN.i32Theta_delta <= 65535)
        {
            UPS_Ctr_Info.delta_P = (65535 - PLL_Ctrl_Info_V_ACIN.i32Theta_delta);
            UPS_Ctr_Info.V_ACIN_HYS_ADV = 1; // 输出电压滞后;市电波形超前
        }
        else if (PLL_Ctrl_Info_V_ACIN.i32Theta_delta <= -32767 && PLL_Ctrl_Info_V_ACIN.i32Theta_delta >= -65535)
        {
            UPS_Ctr_Info.delta_P = (-PLL_Ctrl_Info_V_ACIN.i32Theta_delta);
            UPS_Ctr_Info.V_ACIN_HYS_ADV = 2; // 输出电压超前;市电波形滞后
        }
        else if (PLL_Ctrl_Info_V_ACIN.i32Theta_delta > -32767 && PLL_Ctrl_Info_V_ACIN.i32Theta_delta <= 0)
        {
            UPS_Ctr_Info.delta_P = 65535 + PLL_Ctrl_Info_V_ACIN.i32Theta_delta;
            UPS_Ctr_Info.V_ACIN_HYS_ADV = 1; // 输出电压滞后;市电波形超前
        }

        UPS_Ctr_Info.delta_P_Fir = UPS_Ctr_Info.delta_P << 3;
        if ((UPS_Ctr_Info.delta_T * (UPS_Ctr_Info.delta_T + 1)) < UPS_Ctr_Info.delta_P_Fir)
        {
            UPS_Ctr_Info.delta_T = UPS_Ctr_Info.delta_T + 1;
            UPS_Ctr_Info.delta_P_Fir = UPS_Ctr_Info.delta_P_Fir - UPS_Ctr_Info.delta_T;
        }
        else
        {
            UPS_Ctr_Info.delta_T = UPS_Ctr_Info.delta_T - 1;
            UPS_Ctr_Info.delta_P_Fir = UPS_Ctr_Info.delta_P_Fir - UPS_Ctr_Info.delta_T;
        }

        // 输出电压超前;市电波形滞后
        if (UPS_Ctr_Info.V_ACIN_HYS_ADV == 1)
        {

            //                UPS_Ctr_Info.PWM_Period = User_Divider(64000000, (INV_Ctrl_Info.PWM_Freq_Init-UPS_Ctr_Info.delta_T)*2);//计数方式：锯齿波
            //                UPS_Ctr_Info.PWM_Period = User_Divider(64000000, (INV_Ctrl_Info.PWM_Freq_Init-(UPS_Ctr_Info.delta_T))*2);//计数方式：三角波

            UPS_Ctr_Info.PWM_Period = INV_Ctrl_Info.PWM_Period_Init + (UPS_Ctr_Info.delta_T >> 5); // 计数方式：三角波
        }
        // 输出电压滞后;市电波形超前
        else // if(UPS_Ctr_Info.V_ACIN_HYS_ADV == 2)
        {
            //                UPS_Ctr_Info.PWM_Period = User_Divider(64000000, (INV_Ctrl_Info.PWM_Freq_Init+UPS_Ctr_Info.delta_T));//计数方式：锯齿波
            //                UPS_Ctr_Info.PWM_Period = User_Divider(64000000, (INV_Ctrl_Info.PWM_Freq_Init+(UPS_Ctr_Info.delta_T))*2);//计数方式：三角波

            UPS_Ctr_Info.PWM_Period = INV_Ctrl_Info.PWM_Period_Init - (UPS_Ctr_Info.delta_T >> 5); // 计数方式：三角波
        }

        // 锁相完成判断
        if ((PLL_Ctrl_Info_V_ACIN.i32Theta_delta < 100 && PLL_Ctrl_Info_V_ACIN.i32Theta_delta > -100) ||
            (PLL_Ctrl_Info_V_ACIN.i32Theta_delta < -65435 && PLL_Ctrl_Info_V_ACIN.i32Theta_delta > 65435))
        {
            UPS_Ctr_Info.lock_Phase_OK_Cnt++;
            if (UPS_Ctr_Info.lock_Phase_OK_Cnt >= 2 &&
                INV_Ctrl_Info.periodDot_Cnt == (INV_Ctrl_Info.periodDot_Val - 5)) //(INV_Ctrl_Info.periodDot_Val*3>>2))
            {
                UPS_Ctr_Info.lock_Phase_OK = 1;
            }
        }
        else
        {
            UPS_Ctr_Info.lock_Phase_OK_Cnt = 0;
        }

        INV_Ctrl_Info.PWM_Period = UPS_Ctr_Info.PWM_Period;
    }
    else
    {
        INV_Ctrl_Info.PWM_Period = INV_Ctrl_Info.PWM_Period_Init; // 锁相时输出频率切换为输入频率
    }

    // 限制工作周期
    if (INV_Ctrl_Info.PWM_Period < INV_Ctrl_Info.PWM_Period_67Hz)
        INV_Ctrl_Info.PWM_Period = INV_Ctrl_Info.PWM_Period_67Hz;
    if (INV_Ctrl_Info.PWM_Period > INV_Ctrl_Info.PWM_Period_43Hz)
        INV_Ctrl_Info.PWM_Period = INV_Ctrl_Info.PWM_Period_43Hz;
}

/*************************************************
Description: INV_Ctrl
Input      :
Return     :
Others     : 正弦逆变电源控制
*************************************************/
extern int16_t RepeatOut[600];
extern int16_t RepeatErr[600];
void INV_Ctrl(void)
{

    // 限流环上下限补偿
    if (INV_Ctrl_Info.active_Power < INV_Ctrl_Info.curLoop_active_Power_Val)
    {
        INV_Ctrl_Info.curLoop_Up = INV_Ctrl_Info.curLoop_UpDef_Init + (INV_Ctrl_Info.active_Power * 4 >> 4);
        INV_Ctrl_Info.curLoop_Dn = INV_Ctrl_Info.curLoop_DnDef_Init - (INV_Ctrl_Info.active_Power * 4 >> 4);
        INV_PID_Vol.err_Integral_Max = INV_Ctrl_Info.curLoop_Up * INV_Ctrl_Info.VolErr_Integral_Fact;
        INV_PID_Vol.err_Integral_Min = INV_Ctrl_Info.curLoop_Dn * INV_Ctrl_Info.VolErr_Integral_Fact;
    }
    else if (INV_Ctrl_Info.curLoop_Up < INV_Ctrl_Info.curLoop_UpDef && INV_Ctrl_Info.periodDot_Cnt == 0)
    {
        INV_Ctrl_Info.curLoop_Up += INV_Ctrl_Info.curLoop_Step_Val;
        INV_Ctrl_Info.curLoop_Dn = -INV_Ctrl_Info.curLoop_Up;

        INV_PID_Vol.err_Integral_Max = INV_Ctrl_Info.curLoop_Up * INV_Ctrl_Info.VolErr_Integral_Fact;
        INV_PID_Vol.err_Integral_Min = INV_Ctrl_Info.curLoop_Dn * INV_Ctrl_Info.VolErr_Integral_Fact;
    }

    int32_t Duty_Out = 0;

    /***************************************************************************/
    /*-------------------电压控制环路缓启动处理--------------------------------*/
    if (INV_Ctrl_Info.mode_AC_Software_Ctrl == INV_SOFTWARE_ENABLE) // 使能缓启动
    {
        if (INV_Ctrl_Info.AC_Vol_AMP_Target < INV_Ctrl_Info.AC_Vol_AMP_Target_Ref)
        {
            INV_Ctrl_Info.SS_AMP_Target_Hold += INV_Ctrl_Info.SS_Step_Value;
            INV_Ctrl_Info.AC_Vol_AMP_Target = INV_Ctrl_Info.SS_AMP_Target_Hold >> INV_Ctrl_Info.SS_Shift_Value;
        }
    }
    else
    {
        INV_Ctrl_Info.AC_Vol_AMP_Target = INV_Ctrl_Info.AC_Vol_AMP_Target_Ref;
    }

    if (INV_Ctrl_Info.AC_Vol_AMP_Target > INV_Ctrl_Info.AC_Vol_AMP_Target_Ref)
    {
        INV_Ctrl_Info.AC_Vol_AMP_Target = INV_Ctrl_Info.AC_Vol_AMP_Target_Ref;
    }
    /***************************************************************************/
    /*-------------------开环、闭环工作选择------------------------------------*/
    if (INV_Ctrl_Info.mode_Operate == DEBUG_MODE) // 开环工作
    {
        if (INV_Ctrl_Info.mode_AC_Freq_Select == INV_AC_VOL_FREQ_50HZ)
        {
            Duty_Out = (INV_Ctrl_Info.AC_Vol_AMP_Target * Sine_Table_50Hz[INV_Ctrl_Info.periodDot_Cnt] +
                        INV_Ctrl_Info.AC_Vol_AMP_Target * INV_PID_DCIM.out) >>
                       12;
        }
        else
        {
            Duty_Out = (INV_Ctrl_Info.AC_Vol_AMP_Target * Sine_Table_60Hz[INV_Ctrl_Info.periodDot_Cnt] +
                        INV_Ctrl_Info.AC_Vol_AMP_Target * INV_PID_DCIM.out) >>
                       12;
        }
    }
    else // 闭环工作
    {
        /***************************************************************************/
        /*-------------------PQ下垂控制--------------------------------------------*/
        if (INV_Ctrl_Info.mode_PQ_Droop_Ctrl == INV_PQ_DROOP_ENABLE) //
        {
            INV_PQ_Droop_Info.P_In = INV_Ctrl_Info.active_Power;   // 有功功率
            INV_PQ_Droop_Info.Q_In = INV_Ctrl_Info.reavtive_Power; // 无功功率
            INV_PQ_Droop_Ctrl(&INV_PQ_Droop_Info);

            // PQ下垂频率计算
            // OMIGA(W) = 2*PI*f_base;  f_base = PWM_FREQ/SineTab_PeriodPoint;   PERIOD = (COM_MCU_CLK  / (INV_PWM_FREQ * 2)) + INV_DEADTIME - 2)
            INV_Ctrl_Info.PWM_Period = User_Divider(PERIOD_COEFF, (SPWMWAVE_DOT_50Hz * OMIGA_REF - INV_PQ_Droop_Info.omiga_Out)) + INV_DEADTIME - 2;
            // PQ下垂时降电压参考
            INV_Ctrl_Info.AC_Vol_AMP_Target = INV_Ctrl_Info.AC_Vol_AMP_Target - INV_PQ_Droop_Info.AMP_Out;
        }

        /***************************************************************************/
        /*-------------------功率环控制--------------------------------------------*/
        if (INV_Ctrl_Info.mode_Power_Ctrl == INV_POWER_ENABLE) // 启用限功率控制
        {
            INV_PID_Power.fdb = INV_Ctrl_Info.active_Power;
            INV_PID_Power.Calc(&INV_PID_Power);
            if (INV_PID_Power.out > 0) // 输出小于0开始进行限功率调节
            {
                INV_PID_Power.out = 0;
            }

            // 限功率运行时降电压参考
            INV_Ctrl_Info.AC_Vol_AMP_Target = INV_Ctrl_Info.AC_Vol_AMP_Target + INV_PID_Power.out;
        }

        /***************************************************************************/
        /*-------------------电压环参考给定----------------------------------------*/
        // 计算电压参考
        if (INV_Ctrl_Info.mode_AC_Freq_Select == INV_AC_VOL_FREQ_50HZ)
        {

            // 用自己的正弦表
            // INV_PID_Vol.ref = (INV_Ctrl_Info.AC_Vol_AMP_Target * Sine_Table_50Hz[INV_Ctrl_Info.periodDot_Cnt] +
            //                    INV_Ctrl_Info.AC_Vol_AMP_Target * INV_PID_DCIM.out) >>
            //                   12;

            // 市电正常就用市电的thete角
            INV_PID_Vol.ref = (INV_Ctrl_Info.AC_Vol_AMP_Target * (-(Get_PLL_Sin(&PLL_Ctrl_Info_V_ACIN))) +
                               INV_Ctrl_Info.AC_Vol_AMP_Target * INV_PID_DCIM.out) >>
                              12;
        }
        else
        {
            INV_PID_Vol.ref = (INV_Ctrl_Info.AC_Vol_AMP_Target * Sine_Table_60Hz[INV_Ctrl_Info.periodDot_Cnt] +
                               INV_Ctrl_Info.AC_Vol_AMP_Target * INV_PID_DCIM.out) >>
                              12;
        }

        // 加入虚拟阻抗降低参考电压
        INV_PID_Vol.ref = INV_PID_Vol.ref - ((INV_Ctrl_Info.virtual_Res_Coeff * INV_Ctrl_Info.curInduc_Peak) >> 12);

        /***************************************************************************/
        /*-------------------重复控制----------------------------------------------*/
        if (INV_Ctrl_Info.mode_Repeat_Ctrl == INV_REPEAT_ENABLE) // 启用重复控制
        {
            //            INV_Repeat_Info.ref = INV_PID_Vol.ref;
            //            INV_Repeat_Info.fdb = INV_Ctrl_Info.AC_Vol_Peak;
            //            INV_Repeat_Info.periodDot_Cnt = INV_Ctrl_Info.periodDot_Cnt;
            //            INV_Repeat_Ctrl(&INV_Repeat_Info);

            INV_Repeat_Info.err = (INV_PID_Vol.ref - INV_Ctrl_Info.AC_Vol_Peak);
            // 误差值限制
            if (INV_Repeat_Info.err > INV_Repeat_Info.err_Max)
                INV_Repeat_Info.err = INV_Repeat_Info.err_Max;
            if (INV_Repeat_Info.err < INV_Repeat_Info.err_Min)
                INV_Repeat_Info.err = INV_Repeat_Info.err_Min;

            if (INV_Ctrl_Info.periodDot_Cnt < (INV_Repeat_Info.periodDot_Val - INV_Repeat_Info.lag_Point))
            {
                INV_Repeat_Info.out = (INV_Repeat_Info.Qr_Coeff * RepeatOut[INV_Ctrl_Info.periodDot_Cnt] +
                                       INV_Repeat_Info.Kr_Coeff * RepeatErr[INV_Repeat_Info.lag_Point + INV_Ctrl_Info.periodDot_Cnt]) >>
                                      12;
            }
            else
            {
                INV_Repeat_Info.out = (INV_Repeat_Info.Qr_Coeff * RepeatOut[INV_Ctrl_Info.periodDot_Cnt] +
                                       INV_Repeat_Info.Kr_Coeff * RepeatErr[INV_Ctrl_Info.periodDot_Cnt -
                                                                            INV_Repeat_Info.periodDot_Val + INV_Repeat_Info.lag_Point]) >>
                                      12;
            }
            RepeatOut[INV_Ctrl_Info.periodDot_Cnt] = INV_Repeat_Info.out;
            RepeatErr[INV_Ctrl_Info.periodDot_Cnt] = INV_Repeat_Info.err;

            // 嵌入式重复控制输出误差值
            INV_Repeat_Info.out += INV_Repeat_Info.err;

            // 输出值限制
            if (INV_Repeat_Info.out > INV_Repeat_Info.out_Max)
                INV_Repeat_Info.out = INV_Repeat_Info.out_Max;
            if (INV_Repeat_Info.out < INV_Repeat_Info.out_Min)
                INV_Repeat_Info.out = INV_Repeat_Info.out_Min;

            INV_PID_Vol.ref = (INV_PID_Vol.ref + INV_Repeat_Info.out);
        }

        /***************************************************************************/
        /*-------------------电压环控制--------------------------------------------*/
        //        INV_PID_Vol.fdb       = INV_Ctrl_Info.AC_Vol_Peak;
        //        INV_PID_Vol.Calc(&INV_PID_Vol);

        INV_PID_Vol.err = INV_PID_Vol.ref - INV_Ctrl_Info.AC_Vol_Peak;
        //        //误差限制
        //        if ( INV_PID_Vol.err > INV_PID_Vol.err_Max ) INV_PID_Vol.err = INV_PID_Vol.err_Max;
        //        if ( INV_PID_Vol.err < INV_PID_Vol.err_Min ) INV_PID_Vol.err = INV_PID_Vol.err_Min;

        // 积分误差累计
        INV_PID_Vol.err_Integral += INV_PID_Vol.err;
        // 积分限幅
        if (INV_PID_Vol.err_Integral >= INV_PID_Vol.err_Integral_Max)
        {
            INV_PID_Vol.err_Integral = INV_PID_Vol.err_Integral_Max;
        }
        else if (INV_PID_Vol.err_Integral < INV_PID_Vol.err_Integral_Min)
        {
            INV_PID_Vol.err_Integral = INV_PID_Vol.err_Integral_Min;
        }

        INV_PID_Vol.up = (INV_PID_Vol.kp * INV_PID_Vol.err) >> 15;          // 比例部分作用
        INV_PID_Vol.ui = (INV_PID_Vol.ki * INV_PID_Vol.err_Integral) >> 15; // 积分部分作用
        /*------------------------------------------------------------------------------------*/
        INV_PID_Vol.out = (INV_PID_Vol.up + INV_PID_Vol.ui);

        /*----------------------------------PID输出限幅--------------------------------------------*/
        if (INV_PID_Vol.out > INV_PID_Vol.out_Max)
            INV_PID_Vol.out = INV_PID_Vol.out_Max;
        if (INV_PID_Vol.out < INV_PID_Vol.out_Min)
            INV_PID_Vol.out = INV_PID_Vol.out_Min;

        Duty_Out = INV_PID_Vol.out; // 电压单环输出

        /***************************************************************************/
        /*-------------------电流环控制--------------------------------------------*/
        if (INV_Ctrl_Info.mode_PID_Loop == INV_DOUBLE_LOOP) // 电压电流双环输出
        {

            // if (COM_AD_Data_Info.vBus_Val_Fir < Enable_RY_CurPID)
            if (1)
            {
                // 410V母线电压前,使用电压环
                // INV_PID_Cur.ref = INV_PID_Vol.out;
                INV_PID_Cur.ref = ((int32_t)(-4.0 * Get_PLL_Sin_WithARG(&PLL_Ctrl_Info_V_ACIN,45.0) / 100));
            }
            else
            {
                // 410V母线电压后,使用电流环
                INV_PID_Cur.ref = ((int32_t)(-1.0 * Get_PLL_Sin(&PLL_Ctrl_Info_V_ACIN) / COM_CUR_INDUC_BASE) * 3 >> 2);
                // INV_PID_Cur.ref = INV_PID_Vol.out;
                if (BWRY_State)
                {
                    // INV_RY1_ENABLE; // 开启逆变器输出
                    // INV_RY3_ENABLE; // 开启逆变器输出
                }
            }

            // 使用电压环控制电流环

            // 电流环控制

            //  INV_PID_Cur.ref = 0;
            //              INV_PID_Cur.fdb         = INV_Ctrl_Info.curInduc_Peak;
            //   限电流
            if (INV_PID_Cur.ref >= INV_Ctrl_Info.curLoop_Up)
                INV_PID_Cur.ref = INV_Ctrl_Info.curLoop_Up;

            if (INV_PID_Cur.ref <= INV_Ctrl_Info.curLoop_Dn)
                INV_PID_Cur.ref = INV_Ctrl_Info.curLoop_Dn;

            //            INV_PID_Cur.Calc(&INV_PID_Cur);
            // INV_PID_Cur.err = INV_PID_Cur.ref - INV_Ctrl_Info.curInduc_Peak;
            INV_PID_Cur.err = INV_PID_Cur.ref - ADSample_Info.curLoad_AD_FIR;
            //        //误差限制
            //        if ( INV_PID_Cur.err > INV_PID_Cur.err_Max ) INV_PID_Cur.err = INV_PID_Cur.err_Max;
            //        if ( INV_PID_Cur.err < INV_PID_Cur.err_Min ) INV_PID_Cur.err = INV_PID_Cur.err_Min;

            // 积分误差累计
            INV_PID_Cur.err_Integral += INV_PID_Cur.err;
            // 积分限幅
            if (INV_PID_Cur.err_Integral >= INV_PID_Cur.err_Integral_Max)
            {
                INV_PID_Cur.err_Integral = INV_PID_Cur.err_Integral_Max;
            }
            if (INV_PID_Cur.err_Integral < INV_PID_Cur.err_Integral_Min)
            {
                INV_PID_Cur.err_Integral = INV_PID_Cur.err_Integral_Min;
            }

            INV_PID_Cur.up = (INV_PID_Cur.kp * INV_PID_Cur.err) >> 15;          // 比例部分作用
            INV_PID_Cur.ui = (INV_PID_Cur.ki * INV_PID_Cur.err_Integral) >> 15; // 积分部分作用
            /*------------------------------------------------------------------------------------*/
            INV_PID_Cur.out = (INV_PID_Cur.up + INV_PID_Cur.ui);

            /*----------------------------------PID输出限幅--------------------------------------------*/
            if (INV_PID_Cur.out > INV_PID_Cur.out_Max)
                INV_PID_Cur.out = INV_PID_Cur.out_Max;
            if (INV_PID_Cur.out < INV_PID_Cur.out_Min)
                INV_PID_Cur.out = INV_PID_Cur.out_Min;

            Duty_Out = INV_PID_Cur.out;
        }
    }

    // 直流分量校正
    if (INV_Ctrl_Info.periodDot_Cnt == 0)
    {
        INV_Ctrl_Info.DCIM_Val = INV_Ctrl_Info.DCIM_Sum >> 7;
        INV_Ctrl_Info.DCIM_Val_Fir = (INV_Ctrl_Info.DCIM_Val_Fir * 1593 >> 12) + (INV_Ctrl_Info.DCIM_Val * 2503 >> 12);
        INV_Ctrl_Info.DCIM_Sum = 0;

        INV_PID_DCIM.err = -INV_Ctrl_Info.DCIM_Val_Fir;
        // 积分误差累计
        INV_PID_DCIM.err_Integral += INV_PID_DCIM.err;
        // 积分限幅
        if (INV_PID_DCIM.err_Integral >= INV_PID_DCIM.err_Integral_Max)
        {
            INV_PID_DCIM.err_Integral = INV_PID_DCIM.err_Integral_Max;
        }
        if (INV_PID_DCIM.err_Integral < INV_PID_DCIM.err_Integral_Min)
        {
            INV_PID_DCIM.err_Integral = INV_PID_DCIM.err_Integral_Min;
        }

        INV_PID_DCIM.up = (INV_PID_DCIM.kp * INV_PID_DCIM.err) >> 15;          // 比例部分作用
        INV_PID_DCIM.ui = (INV_PID_DCIM.ki * INV_PID_DCIM.err_Integral) >> 15; // 积分部分作用
        /*------------------------------------------------------------------------------------*/
        INV_PID_DCIM.out = (INV_PID_DCIM.up + INV_PID_DCIM.ui);

        /*----------------------------------PID输出限幅--------------------------------------------*/
        if (INV_PID_DCIM.out > INV_PID_DCIM.out_Max)
            INV_PID_DCIM.out = INV_PID_DCIM.out_Max;
        if (INV_PID_DCIM.out < INV_PID_DCIM.out_Min)
            INV_PID_DCIM.out = INV_PID_DCIM.out_Min;

        //        INV_PID_DCIM.fdb = INV_Ctrl_Info.DCIM_Val_Fir;
        //        INV_PID_DCIM.Calc(&INV_PID_DCIM);
    }
    // 两半周乘积和做差处理
    INV_Ctrl_Info.DCIM_Sum += (INV_Ctrl_Info.vBus * Duty_Out >> 12) + 0;

    /***************************************************************************/
    /*-------------------占空比更新--------------------------------------------*/
    // INV_Ctrl_Info.PWM_Duty = (int32_t)((Duty_Out * (INV_Ctrl_Info.PWM_Period)) >> 12); // 更新占空比
    // INV_Ctrl_Info.PWM_DutyB = INV_Ctrl_Info.PWM_Duty_Dn;
    // if (INV_Ctrl_Info.PWM_Duty < 0)
    // {
    //     INV_Ctrl_Info.PWM_Duty = INV_Ctrl_Info.PWM_Period + INV_Ctrl_Info.PWM_Duty; // 更新占空比
    //     INV_Ctrl_Info.PWM_DutyB = INV_Ctrl_Info.PWM_Period - INV_Ctrl_Info.PWM_Duty_Dn;
    // }

    INV_Ctrl_Info.PWM_Duty = (int32_t)((Duty_Out * (INV_Ctrl_Info.PWM_Period)) >> 12);   // -period到-period的过零正弦
    INV_Ctrl_Info.PWM_DutyB = (-INV_Ctrl_Info.PWM_Duty);                                 // 改负
    INV_Ctrl_Info.PWM_Duty = (INV_Ctrl_Info.PWM_Duty + INV_Ctrl_Info.PWM_Period) >> 1;   // 0到period正弦
    INV_Ctrl_Info.PWM_DutyB = (INV_Ctrl_Info.PWM_DutyB + INV_Ctrl_Info.PWM_Period) >> 1; // 0到period正弦

    // 限制最大占空比
    if (INV_Ctrl_Info.PWM_Duty > (INV_Ctrl_Info.PWM_Period - INV_Ctrl_Info.PWM_Duty_Dn))
    {
        INV_Ctrl_Info.PWM_Duty = INV_Ctrl_Info.PWM_Period - INV_Ctrl_Info.PWM_Duty_Dn; // 更新占空比
    }
    // 限制最小占空比
    if (INV_Ctrl_Info.PWM_Duty < (INV_Ctrl_Info.PWM_Duty_Dn * 59)) // 补偿59，输出电压过零更平滑
    {
        INV_Ctrl_Info.PWM_Duty = INV_Ctrl_Info.PWM_Duty_Dn * 59; // 更新占空比
    }

    // 为PWM_DutyB添加相同的保护代码
    //  限制最大占空比
    if (INV_Ctrl_Info.PWM_DutyB > (INV_Ctrl_Info.PWM_Period - INV_Ctrl_Info.PWM_Duty_Dn))
    {
        INV_Ctrl_Info.PWM_DutyB = INV_Ctrl_Info.PWM_Period - INV_Ctrl_Info.PWM_Duty_Dn; // 更新占空比
    }
    // 限制最小占空比
    if (INV_Ctrl_Info.PWM_DutyB < (INV_Ctrl_Info.PWM_Duty_Dn * 59)) // 补偿59，输出电压过零更平滑
    {
        INV_Ctrl_Info.PWM_DutyB = INV_Ctrl_Info.PWM_Duty_Dn * 59; // 更新占空比
    }
}
