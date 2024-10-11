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
Others     : �����е����ദ��
*************************************************/
extern uint16_t u16_INV_Freq_Cnt, PFC_Freq_Time_Cnt;
extern uint8_t PFC_flag, INV_flag;

void INV_Lock_Phase(void)
{
    PLL_Ctrl_Info_V_ACIN.i32Theta_delta = PLL_Ctrl_Info_V_ACIN.i32Theta - PLL_Ctrl_Info_V_ACOUT.i32Theta;
    // �����ѹ���е������߼�����
    if (UPS_Ctr_Info.V_ACIN_OK == 1 && UPS_Ctr_Info.lock_Phase_OK == 0)
    {
        if (PLL_Ctrl_Info_V_ACIN.i32Theta_delta < 32767 && PLL_Ctrl_Info_V_ACIN.i32Theta_delta >= 0)
        {
            UPS_Ctr_Info.delta_P = PLL_Ctrl_Info_V_ACIN.i32Theta_delta;
            UPS_Ctr_Info.V_ACIN_HYS_ADV = 2; // �����ѹ��ǰ;�е粨���ͺ�
        }
        else if (PLL_Ctrl_Info_V_ACIN.i32Theta_delta >= 32767 && PLL_Ctrl_Info_V_ACIN.i32Theta_delta <= 65535)
        {
            UPS_Ctr_Info.delta_P = (65535 - PLL_Ctrl_Info_V_ACIN.i32Theta_delta);
            UPS_Ctr_Info.V_ACIN_HYS_ADV = 1; // �����ѹ�ͺ�;�е粨�γ�ǰ
        }
        else if (PLL_Ctrl_Info_V_ACIN.i32Theta_delta <= -32767 && PLL_Ctrl_Info_V_ACIN.i32Theta_delta >= -65535)
        {
            UPS_Ctr_Info.delta_P = (-PLL_Ctrl_Info_V_ACIN.i32Theta_delta);
            UPS_Ctr_Info.V_ACIN_HYS_ADV = 2; // �����ѹ��ǰ;�е粨���ͺ�
        }
        else if (PLL_Ctrl_Info_V_ACIN.i32Theta_delta > -32767 && PLL_Ctrl_Info_V_ACIN.i32Theta_delta <= 0)
        {
            UPS_Ctr_Info.delta_P = 65535 + PLL_Ctrl_Info_V_ACIN.i32Theta_delta;
            UPS_Ctr_Info.V_ACIN_HYS_ADV = 1; // �����ѹ�ͺ�;�е粨�γ�ǰ
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

        // �����ѹ��ǰ;�е粨���ͺ�
        if (UPS_Ctr_Info.V_ACIN_HYS_ADV == 1)
        {

            //                UPS_Ctr_Info.PWM_Period = User_Divider(64000000, (INV_Ctrl_Info.PWM_Freq_Init-UPS_Ctr_Info.delta_T)*2);//������ʽ����ݲ�
            //                UPS_Ctr_Info.PWM_Period = User_Divider(64000000, (INV_Ctrl_Info.PWM_Freq_Init-(UPS_Ctr_Info.delta_T))*2);//������ʽ�����ǲ�

            UPS_Ctr_Info.PWM_Period = INV_Ctrl_Info.PWM_Period_Init + (UPS_Ctr_Info.delta_T >> 5); // ������ʽ�����ǲ�
        }
        // �����ѹ�ͺ�;�е粨�γ�ǰ
        else // if(UPS_Ctr_Info.V_ACIN_HYS_ADV == 2)
        {
            //                UPS_Ctr_Info.PWM_Period = User_Divider(64000000, (INV_Ctrl_Info.PWM_Freq_Init+UPS_Ctr_Info.delta_T));//������ʽ����ݲ�
            //                UPS_Ctr_Info.PWM_Period = User_Divider(64000000, (INV_Ctrl_Info.PWM_Freq_Init+(UPS_Ctr_Info.delta_T))*2);//������ʽ�����ǲ�

            UPS_Ctr_Info.PWM_Period = INV_Ctrl_Info.PWM_Period_Init - (UPS_Ctr_Info.delta_T >> 5); // ������ʽ�����ǲ�
        }

        // ��������ж�
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
        INV_Ctrl_Info.PWM_Period = INV_Ctrl_Info.PWM_Period_Init; // ����ʱ���Ƶ���л�Ϊ����Ƶ��
    }

    // ���ƹ�������
    if (INV_Ctrl_Info.PWM_Period < INV_Ctrl_Info.PWM_Period_67Hz)
        INV_Ctrl_Info.PWM_Period = INV_Ctrl_Info.PWM_Period_67Hz;
    if (INV_Ctrl_Info.PWM_Period > INV_Ctrl_Info.PWM_Period_43Hz)
        INV_Ctrl_Info.PWM_Period = INV_Ctrl_Info.PWM_Period_43Hz;
}

/*************************************************
Description: INV_Ctrl
Input      :
Return     :
Others     : ��������Դ����
*************************************************/
extern int16_t RepeatOut[600];
extern int16_t RepeatErr[600];
void INV_Ctrl(void)
{

    // �����������޲���
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
    /*-------------------��ѹ���ƻ�·����������--------------------------------*/
    if (INV_Ctrl_Info.mode_AC_Software_Ctrl == INV_SOFTWARE_ENABLE) // ʹ�ܻ�����
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
    /*-------------------�������ջ�����ѡ��------------------------------------*/
    if (INV_Ctrl_Info.mode_Operate == DEBUG_MODE) // ��������
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
    else // �ջ�����
    {
        /***************************************************************************/
        /*-------------------PQ�´�����--------------------------------------------*/
        if (INV_Ctrl_Info.mode_PQ_Droop_Ctrl == INV_PQ_DROOP_ENABLE) //
        {
            INV_PQ_Droop_Info.P_In = INV_Ctrl_Info.active_Power;   // �й�����
            INV_PQ_Droop_Info.Q_In = INV_Ctrl_Info.reavtive_Power; // �޹�����
            INV_PQ_Droop_Ctrl(&INV_PQ_Droop_Info);

            // PQ�´�Ƶ�ʼ���
            // OMIGA(W) = 2*PI*f_base;  f_base = PWM_FREQ/SineTab_PeriodPoint;   PERIOD = (COM_MCU_CLK  / (INV_PWM_FREQ * 2)) + INV_DEADTIME - 2)
            INV_Ctrl_Info.PWM_Period = User_Divider(PERIOD_COEFF, (SPWMWAVE_DOT_50Hz * OMIGA_REF - INV_PQ_Droop_Info.omiga_Out)) + INV_DEADTIME - 2;
            // PQ�´�ʱ����ѹ�ο�
            INV_Ctrl_Info.AC_Vol_AMP_Target = INV_Ctrl_Info.AC_Vol_AMP_Target - INV_PQ_Droop_Info.AMP_Out;
        }

        /***************************************************************************/
        /*-------------------���ʻ�����--------------------------------------------*/
        if (INV_Ctrl_Info.mode_Power_Ctrl == INV_POWER_ENABLE) // �����޹��ʿ���
        {
            INV_PID_Power.fdb = INV_Ctrl_Info.active_Power;
            INV_PID_Power.Calc(&INV_PID_Power);
            if (INV_PID_Power.out > 0) // ���С��0��ʼ�����޹��ʵ���
            {
                INV_PID_Power.out = 0;
            }

            // �޹�������ʱ����ѹ�ο�
            INV_Ctrl_Info.AC_Vol_AMP_Target = INV_Ctrl_Info.AC_Vol_AMP_Target + INV_PID_Power.out;
        }

        /***************************************************************************/
        /*-------------------��ѹ���ο�����----------------------------------------*/
        // �����ѹ�ο�
        if (INV_Ctrl_Info.mode_AC_Freq_Select == INV_AC_VOL_FREQ_50HZ)
        {

            // ���Լ������ұ�
            // INV_PID_Vol.ref = (INV_Ctrl_Info.AC_Vol_AMP_Target * Sine_Table_50Hz[INV_Ctrl_Info.periodDot_Cnt] +
            //                    INV_Ctrl_Info.AC_Vol_AMP_Target * INV_PID_DCIM.out) >>
            //                   12;

            // �е����������е��thete��
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

        // ���������迹���Ͳο���ѹ
        INV_PID_Vol.ref = INV_PID_Vol.ref - ((INV_Ctrl_Info.virtual_Res_Coeff * INV_Ctrl_Info.curInduc_Peak) >> 12);

        /***************************************************************************/
        /*-------------------�ظ�����----------------------------------------------*/
        if (INV_Ctrl_Info.mode_Repeat_Ctrl == INV_REPEAT_ENABLE) // �����ظ�����
        {
            //            INV_Repeat_Info.ref = INV_PID_Vol.ref;
            //            INV_Repeat_Info.fdb = INV_Ctrl_Info.AC_Vol_Peak;
            //            INV_Repeat_Info.periodDot_Cnt = INV_Ctrl_Info.periodDot_Cnt;
            //            INV_Repeat_Ctrl(&INV_Repeat_Info);

            INV_Repeat_Info.err = (INV_PID_Vol.ref - INV_Ctrl_Info.AC_Vol_Peak);
            // ���ֵ����
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

            // Ƕ��ʽ�ظ�����������ֵ
            INV_Repeat_Info.out += INV_Repeat_Info.err;

            // ���ֵ����
            if (INV_Repeat_Info.out > INV_Repeat_Info.out_Max)
                INV_Repeat_Info.out = INV_Repeat_Info.out_Max;
            if (INV_Repeat_Info.out < INV_Repeat_Info.out_Min)
                INV_Repeat_Info.out = INV_Repeat_Info.out_Min;

            INV_PID_Vol.ref = (INV_PID_Vol.ref + INV_Repeat_Info.out);
        }

        /***************************************************************************/
        /*-------------------��ѹ������--------------------------------------------*/
        //        INV_PID_Vol.fdb       = INV_Ctrl_Info.AC_Vol_Peak;
        //        INV_PID_Vol.Calc(&INV_PID_Vol);

        INV_PID_Vol.err = INV_PID_Vol.ref - INV_Ctrl_Info.AC_Vol_Peak;
        //        //�������
        //        if ( INV_PID_Vol.err > INV_PID_Vol.err_Max ) INV_PID_Vol.err = INV_PID_Vol.err_Max;
        //        if ( INV_PID_Vol.err < INV_PID_Vol.err_Min ) INV_PID_Vol.err = INV_PID_Vol.err_Min;

        // ��������ۼ�
        INV_PID_Vol.err_Integral += INV_PID_Vol.err;
        // �����޷�
        if (INV_PID_Vol.err_Integral >= INV_PID_Vol.err_Integral_Max)
        {
            INV_PID_Vol.err_Integral = INV_PID_Vol.err_Integral_Max;
        }
        else if (INV_PID_Vol.err_Integral < INV_PID_Vol.err_Integral_Min)
        {
            INV_PID_Vol.err_Integral = INV_PID_Vol.err_Integral_Min;
        }

        INV_PID_Vol.up = (INV_PID_Vol.kp * INV_PID_Vol.err) >> 15;          // ������������
        INV_PID_Vol.ui = (INV_PID_Vol.ki * INV_PID_Vol.err_Integral) >> 15; // ���ֲ�������
        /*------------------------------------------------------------------------------------*/
        INV_PID_Vol.out = (INV_PID_Vol.up + INV_PID_Vol.ui);

        /*----------------------------------PID����޷�--------------------------------------------*/
        if (INV_PID_Vol.out > INV_PID_Vol.out_Max)
            INV_PID_Vol.out = INV_PID_Vol.out_Max;
        if (INV_PID_Vol.out < INV_PID_Vol.out_Min)
            INV_PID_Vol.out = INV_PID_Vol.out_Min;

        Duty_Out = INV_PID_Vol.out; // ��ѹ�������

        /***************************************************************************/
        /*-------------------����������--------------------------------------------*/
        if (INV_Ctrl_Info.mode_PID_Loop == INV_DOUBLE_LOOP) // ��ѹ����˫�����
        {

            // if (COM_AD_Data_Info.vBus_Val_Fir < Enable_RY_CurPID)
            if (1)
            {
                // 410Vĸ�ߵ�ѹǰ,ʹ�õ�ѹ��
                // INV_PID_Cur.ref = INV_PID_Vol.out;
                INV_PID_Cur.ref = ((int32_t)(-4.0 * Get_PLL_Sin_WithARG(&PLL_Ctrl_Info_V_ACIN,45.0) / 100));
            }
            else
            {
                // 410Vĸ�ߵ�ѹ��,ʹ�õ�����
                INV_PID_Cur.ref = ((int32_t)(-1.0 * Get_PLL_Sin(&PLL_Ctrl_Info_V_ACIN) / COM_CUR_INDUC_BASE) * 3 >> 2);
                // INV_PID_Cur.ref = INV_PID_Vol.out;
                if (BWRY_State)
                {
                    // INV_RY1_ENABLE; // ������������
                    // INV_RY3_ENABLE; // ������������
                }
            }

            // ʹ�õ�ѹ�����Ƶ�����

            // ����������

            //  INV_PID_Cur.ref = 0;
            //              INV_PID_Cur.fdb         = INV_Ctrl_Info.curInduc_Peak;
            //   �޵���
            if (INV_PID_Cur.ref >= INV_Ctrl_Info.curLoop_Up)
                INV_PID_Cur.ref = INV_Ctrl_Info.curLoop_Up;

            if (INV_PID_Cur.ref <= INV_Ctrl_Info.curLoop_Dn)
                INV_PID_Cur.ref = INV_Ctrl_Info.curLoop_Dn;

            //            INV_PID_Cur.Calc(&INV_PID_Cur);
            // INV_PID_Cur.err = INV_PID_Cur.ref - INV_Ctrl_Info.curInduc_Peak;
            INV_PID_Cur.err = INV_PID_Cur.ref - ADSample_Info.curLoad_AD_FIR;
            //        //�������
            //        if ( INV_PID_Cur.err > INV_PID_Cur.err_Max ) INV_PID_Cur.err = INV_PID_Cur.err_Max;
            //        if ( INV_PID_Cur.err < INV_PID_Cur.err_Min ) INV_PID_Cur.err = INV_PID_Cur.err_Min;

            // ��������ۼ�
            INV_PID_Cur.err_Integral += INV_PID_Cur.err;
            // �����޷�
            if (INV_PID_Cur.err_Integral >= INV_PID_Cur.err_Integral_Max)
            {
                INV_PID_Cur.err_Integral = INV_PID_Cur.err_Integral_Max;
            }
            if (INV_PID_Cur.err_Integral < INV_PID_Cur.err_Integral_Min)
            {
                INV_PID_Cur.err_Integral = INV_PID_Cur.err_Integral_Min;
            }

            INV_PID_Cur.up = (INV_PID_Cur.kp * INV_PID_Cur.err) >> 15;          // ������������
            INV_PID_Cur.ui = (INV_PID_Cur.ki * INV_PID_Cur.err_Integral) >> 15; // ���ֲ�������
            /*------------------------------------------------------------------------------------*/
            INV_PID_Cur.out = (INV_PID_Cur.up + INV_PID_Cur.ui);

            /*----------------------------------PID����޷�--------------------------------------------*/
            if (INV_PID_Cur.out > INV_PID_Cur.out_Max)
                INV_PID_Cur.out = INV_PID_Cur.out_Max;
            if (INV_PID_Cur.out < INV_PID_Cur.out_Min)
                INV_PID_Cur.out = INV_PID_Cur.out_Min;

            Duty_Out = INV_PID_Cur.out;
        }
    }

    // ֱ������У��
    if (INV_Ctrl_Info.periodDot_Cnt == 0)
    {
        INV_Ctrl_Info.DCIM_Val = INV_Ctrl_Info.DCIM_Sum >> 7;
        INV_Ctrl_Info.DCIM_Val_Fir = (INV_Ctrl_Info.DCIM_Val_Fir * 1593 >> 12) + (INV_Ctrl_Info.DCIM_Val * 2503 >> 12);
        INV_Ctrl_Info.DCIM_Sum = 0;

        INV_PID_DCIM.err = -INV_Ctrl_Info.DCIM_Val_Fir;
        // ��������ۼ�
        INV_PID_DCIM.err_Integral += INV_PID_DCIM.err;
        // �����޷�
        if (INV_PID_DCIM.err_Integral >= INV_PID_DCIM.err_Integral_Max)
        {
            INV_PID_DCIM.err_Integral = INV_PID_DCIM.err_Integral_Max;
        }
        if (INV_PID_DCIM.err_Integral < INV_PID_DCIM.err_Integral_Min)
        {
            INV_PID_DCIM.err_Integral = INV_PID_DCIM.err_Integral_Min;
        }

        INV_PID_DCIM.up = (INV_PID_DCIM.kp * INV_PID_DCIM.err) >> 15;          // ������������
        INV_PID_DCIM.ui = (INV_PID_DCIM.ki * INV_PID_DCIM.err_Integral) >> 15; // ���ֲ�������
        /*------------------------------------------------------------------------------------*/
        INV_PID_DCIM.out = (INV_PID_DCIM.up + INV_PID_DCIM.ui);

        /*----------------------------------PID����޷�--------------------------------------------*/
        if (INV_PID_DCIM.out > INV_PID_DCIM.out_Max)
            INV_PID_DCIM.out = INV_PID_DCIM.out_Max;
        if (INV_PID_DCIM.out < INV_PID_DCIM.out_Min)
            INV_PID_DCIM.out = INV_PID_DCIM.out_Min;

        //        INV_PID_DCIM.fdb = INV_Ctrl_Info.DCIM_Val_Fir;
        //        INV_PID_DCIM.Calc(&INV_PID_DCIM);
    }
    // �����ܳ˻��������
    INV_Ctrl_Info.DCIM_Sum += (INV_Ctrl_Info.vBus * Duty_Out >> 12) + 0;

    /***************************************************************************/
    /*-------------------ռ�ձȸ���--------------------------------------------*/
    // INV_Ctrl_Info.PWM_Duty = (int32_t)((Duty_Out * (INV_Ctrl_Info.PWM_Period)) >> 12); // ����ռ�ձ�
    // INV_Ctrl_Info.PWM_DutyB = INV_Ctrl_Info.PWM_Duty_Dn;
    // if (INV_Ctrl_Info.PWM_Duty < 0)
    // {
    //     INV_Ctrl_Info.PWM_Duty = INV_Ctrl_Info.PWM_Period + INV_Ctrl_Info.PWM_Duty; // ����ռ�ձ�
    //     INV_Ctrl_Info.PWM_DutyB = INV_Ctrl_Info.PWM_Period - INV_Ctrl_Info.PWM_Duty_Dn;
    // }

    INV_Ctrl_Info.PWM_Duty = (int32_t)((Duty_Out * (INV_Ctrl_Info.PWM_Period)) >> 12);   // -period��-period�Ĺ�������
    INV_Ctrl_Info.PWM_DutyB = (-INV_Ctrl_Info.PWM_Duty);                                 // �ĸ�
    INV_Ctrl_Info.PWM_Duty = (INV_Ctrl_Info.PWM_Duty + INV_Ctrl_Info.PWM_Period) >> 1;   // 0��period����
    INV_Ctrl_Info.PWM_DutyB = (INV_Ctrl_Info.PWM_DutyB + INV_Ctrl_Info.PWM_Period) >> 1; // 0��period����

    // �������ռ�ձ�
    if (INV_Ctrl_Info.PWM_Duty > (INV_Ctrl_Info.PWM_Period - INV_Ctrl_Info.PWM_Duty_Dn))
    {
        INV_Ctrl_Info.PWM_Duty = INV_Ctrl_Info.PWM_Period - INV_Ctrl_Info.PWM_Duty_Dn; // ����ռ�ձ�
    }
    // ������Сռ�ձ�
    if (INV_Ctrl_Info.PWM_Duty < (INV_Ctrl_Info.PWM_Duty_Dn * 59)) // ����59�������ѹ�����ƽ��
    {
        INV_Ctrl_Info.PWM_Duty = INV_Ctrl_Info.PWM_Duty_Dn * 59; // ����ռ�ձ�
    }

    // ΪPWM_DutyB�����ͬ�ı�������
    //  �������ռ�ձ�
    if (INV_Ctrl_Info.PWM_DutyB > (INV_Ctrl_Info.PWM_Period - INV_Ctrl_Info.PWM_Duty_Dn))
    {
        INV_Ctrl_Info.PWM_DutyB = INV_Ctrl_Info.PWM_Period - INV_Ctrl_Info.PWM_Duty_Dn; // ����ռ�ձ�
    }
    // ������Сռ�ձ�
    if (INV_Ctrl_Info.PWM_DutyB < (INV_Ctrl_Info.PWM_Duty_Dn * 59)) // ����59�������ѹ�����ƽ��
    {
        INV_Ctrl_Info.PWM_DutyB = INV_Ctrl_Info.PWM_Duty_Dn * 59; // ����ռ�ձ�
    }
}
