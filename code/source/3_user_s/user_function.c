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
#include "sys_state_machine.h"
#include "sys_mcu_header.h"
/***************************************************************************/

/*************************************************
Description: User_DelayTime_ms
Input      :
Return     :
Others     : ms延时函数
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
Others     : us延时函数
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
Others     : 保护时关闭驱动
*************************************************/
void COM_CloseDrive(void)
{
    // 母线过压或者逆变故障时给升压侧发送故障信号
    if (COM_AD_Data_Info.vBus_Val_Fir >= COM_VBUS_OVP2_VAL || System_ProtectFlag_Info.all != 0)
    {
        VBUS_OVER_ENABLE;
        if (System_ProtectFlag_Info.all == 0)
            System_ProtectFlag_Info.bit.vBus_OVP = 1;
    }

    //    if (( INV_Ctrl_Info.periodDot_Cnt <=   5 ) ||\
//        ( INV_Ctrl_Info.periodDot_Cnt >= ( INV_Ctrl_Info.periodDot_Val - 5 )) ||\
//        ( INV_Ctrl_Info.periodDot_Cnt >= ( (INV_Ctrl_Info.periodDot_Val >>1) -5 ) && INV_Ctrl_Info.periodDot_Cnt <= ( (INV_Ctrl_Info.periodDot_Val >>1) +5 )) )  //在接近过零点的时候关闭驱动
    {
        if (E_FALSE != System_ProtectFlag_Info.all)
        {
#if OPERATING_MODE == NORMAL_MODE

            COM_PWM_Disable(); // 关闭PWM输出

            INV_RY1_DISABLE; // 开启逆变器输出
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
Others     : LED指示灯逻辑
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
Others     : LED指示灯逻辑
*************************************************/
void User_LED_Deal(void)
{
    LED_Ctr_Info.flag_1ms = SysClockBase_ms.LED_1ms;

    /*---------------------------初始准备状态绿灯闪烁-------------------------------------------*/
    if (System_ProtectFlag_Info.all == 0 &&
        State_Context.state_Value <= COM_READY_STATE)
    {
        LED_Ctr_Info.on_Val = 250;
        LED_Ctr_Info.period_Val = 1500;
        User_LED_ms(&LED_Ctr_Info);
    }
    /*----------------------------运行状态绿灯常亮---------------------------------------------*/
    else if (COM_RUN_STATE == State_Context.state_Value)
    {
        LED_Ctr_Info.on_Val = 1000;
        LED_Ctr_Info.period_Val = 1500;
        User_LED_ms(&LED_Ctr_Info);
    }
    /*-------------------------------故障状态绿灯灭 1HZ-------------------------------------*/
    else
    {
        LED_Ctr_Info.on_Val = 200;
        LED_Ctr_Info.period_Val = 400;
        switch (System_ProtectFlag_Info.all)
        {
        case 1: // 过载  LED闪烁2次
            if (LED_Ctr_Info.cycle_Cnt == 0)
                LED_Ctr_Info.cycle_Val = 2;
            User_LED_ms(&LED_Ctr_Info);
            break;
        case 2: // 母线过欠压  LED闪烁3次
        case 4:
            if (LED_Ctr_Info.cycle_Cnt == 0)
                LED_Ctr_Info.cycle_Val = 3;
            User_LED_ms(&LED_Ctr_Info);
            break;
        case 8: // 输出电压过欠压  LED闪烁4次
        case 16:
            if (LED_Ctr_Info.cycle_Cnt == 0)
                LED_Ctr_Info.cycle_Val = 4;
            User_LED_ms(&LED_Ctr_Info);
            break;
        case 32: // 输出电压短路  LED闪烁5次
            if (LED_Ctr_Info.cycle_Cnt == 0)
                LED_Ctr_Info.cycle_Val = 5;
            User_LED_ms(&LED_Ctr_Info);
            break;
        case 64: // 辅助电源过欠压  LED闪烁6次
        case 128:
            if (LED_Ctr_Info.cycle_Cnt == 0)
                LED_Ctr_Info.cycle_Val = 6;
            User_LED_ms(&LED_Ctr_Info);
            break;
        case 256: // 过温  LED闪烁7次
            if (LED_Ctr_Info.cycle_Cnt == 0)
                LED_Ctr_Info.cycle_Val = 7;
            User_LED_ms(&LED_Ctr_Info);
            break;
        case 512:  //
        case 1024: // 过流  LED闪烁8次
            if (LED_Ctr_Info.cycle_Cnt == 0)
                LED_Ctr_Info.cycle_Val = 8;
            User_LED_ms(&LED_Ctr_Info);
            break;
        case 2048: // 初始化失败  LED闪烁9次
            if (LED_Ctr_Info.cycle_Cnt == 0)
                LED_Ctr_Info.cycle_Val = 9;
            User_LED_ms(&LED_Ctr_Info);
            break;
        case 4096: // DCDC侧故障  LED闪烁10次
            if (LED_Ctr_Info.cycle_Cnt == 0)
                LED_Ctr_Info.cycle_Val = 10;
            User_LED_ms(&LED_Ctr_Info);
            break;
        case 8192: // 通信故障  LED闪烁11次
            if (LED_Ctr_Info.cycle_Cnt == 0)
                LED_Ctr_Info.cycle_Val = 11;
            User_LED_ms(&LED_Ctr_Info);
            break;
        case 16384: // VREF异常  LED闪烁12次
            if (LED_Ctr_Info.cycle_Cnt == 0)
                LED_Ctr_Info.cycle_Val = 12;
            User_LED_ms(&LED_Ctr_Info);
            break;
        default: //  绿灯闪13次
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
Others     : 按键功能程序逻辑处理
*************************************************/
void Key_Calc(Key_Var_t *key)
{
    /*------------------------------------------------------------------------------------*/
    if (key->in == key->ref_Active_Level) // 输入电平与设定有效电平一致
    {
        if (key->release_Cnt > 0)
        {
            key->release_Cnt--;
        }
        if (key->close_Cnt < key->c_Delay_Val) // 按键闭合计数比较
        {
            key->close_Cnt++;
        }
        else
        {
            key->close_Cnt = 0;
            key->out_State = E_KEY_CLOSE; // 表示按键按下
        }
    }
    else // 输入电平与设定有效电平不一致
    {
        if (key->close_Cnt > 0)
        {
            key->close_Cnt--;
        }
        if (key->release_Cnt < key->r_Delay_Val) // 按键松开计数比较
        {
            key->release_Cnt++;
        }
        else
        {
            key->release_Cnt = 0;
            key->out_State = E_KEY_RELEASE; // 表示按键松开
        }
    }
}

/*************************************************
Description: User_Key_Deal
Input      :
Return     :
Others     : 按键状态处理
*************************************************/
uint8_t u8_Key_SW_InvOn_Flag = 0;
uint16_t u16_InvOn_Start_count, u16Inv_RY3_count = 0;
uint16_t u16INV_OFF_Flag = 0;
void User_Key_Deal(void)
{
    if (SysClockBase_ms.sys_1ms == 1)
    {
        // 开关机按钮
        Key_SW_InvOn.in = INV_ON_SWITCH;
        Key_SW_InvOn.Calc(&Key_SW_InvOn);

        if (u16_InvOn_Start_count <= COM_INV_ON_DELAY_TIME)
        {
            u16_InvOn_Start_count++;
        }

        if (u16_InvOn_Start_count >= COM_INV_ON_DELAY_TIME) // 开机延时时间
        {
            // 逆变按钮信号逻辑处理
            if (Key_SW_InvOn.out_State == E_KEY_CLOSE && COM_Ctr_Info.INV_Enable_Flag == 0 && u8_Key_SW_InvOn_Flag == 0)
            {
                u8_Key_SW_InvOn_Flag = 1;         // “0”表示按键松开，“1”表示按键未松开
                COM_Ctr_Info.INV_Enable_Flag = 1; // 开逆变器
                u16INV_OFF_Flag = 0;
            }
            else if (Key_SW_InvOn.out_State == E_KEY_CLOSE && COM_Ctr_Info.INV_Enable_Flag == 1 && u8_Key_SW_InvOn_Flag == 0)
            {
                u8_Key_SW_InvOn_Flag = 1; // “0”表示按键松开，“1”表示按键未松开
                // COM_Ctr_Info.INV_Enable_Flag = 0;//关逆变器
                u16INV_OFF_Flag = 1;
            }
        }

        // INV开关按钮松开标志位
        if (Key_SW_InvOn.out_State == E_KEY_RELEASE)
        {
            u8_Key_SW_InvOn_Flag = 0; // “0”表示按键松开，“1”表示按键未松开
        }
    }

    // 在INV模式时，关逆变输出时直接封管
    // 在PFC模式时，关逆变输出时不封管
    if (u16INV_OFF_Flag == 1)
    {
        if (ADSample_Info.INV_AC_Vol_AD_FIR <= 100 && ADSample_Info.INV_AC_Vol_AD_FIR >= -100)
        {
            if (COM_Ctr_Info.INV_PFC_Mode_Select == INV_MODE) // INV模式：放电
            {
                if (OPERATING_MODE == DEBUG_MODE)
                    COM_Ctr_Info.PWM_Enable = 0;

                COM_Ctr_Info.INV_Enable_Flag = 0; // 关逆变器
                COM_PWM_Disable();                // 在INV模式时关闭PWM输
            }
            else
                COM_Ctr_Info.INV_Enable_Flag = 0; // 关逆变器
        }
    }

    // 开逆变输出继电器
    if (COM_Ctr_Info.INV_Enable_Flag == 1 && System_ProtectFlag_Info.all == 0)
    {
        // if (INV_RY1_STATE == 0)
        // {
        //     u16Inv_RY3_count = 0;
        //     INV_RY1_ENABLE; // 开启逆变器输出
        //     INV_RY3_ENABLE; // 开启逆变器输出
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
        // INV_RY1_DISABLE;//开启逆变器输出
        // INV_RY3_DISABLE;
    }
}

/*************************************************
Description: User_RMS_Deal
Input      :
Return     :
Others     : 交流信号rms处理
*************************************************/
void User_RMS_Deal(void)
{
    // 功率、电压电流有效值计算
    RMS_PQ_Update(&RMS_PQ_Info);

    INV_Ctrl_Info.active_Power = RMS_PQ_Info.out_P; // 有功功率
    PFC_Ctrl_Info.active_Power = RMS_PQ_Info.out_P;
    PFC_Ctrl_Info.RMS_InductorCur = RMS_PQ_Info.out_RMS_CurInduc; // 电感电流有效值

    //    PFC_Ctrl_Info.RMS_Vol           = RMS_PQ_Info.out_RMS_PFC_AC_Vol;//
    PFC_Ctrl_Info.RMS_Vol = (RMS_PQ_Info.out_RMS_PFC_AC_Vol << 2) / COM_REAL_VACIN_RMS_SCAL; //
}

/*************************************************
Description: User_Divider
Input      :
Return     :
Others     : 硬件除法器
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
Others     : 串口1数据处理
*************************************************/
uint8_t *UART1_RXD_B_Temp;
void COM_UART1_Deal(void)
{
    /*------------------------通信数据接收-----------------------------------------------------*/
    /*---------------------8位数据转换为16位格式------------------------------------------*/
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

        // CRC校验码获得处理
        for (int i = 0; i < (UART1_SEND_NUM - 2); i++)
        {
            UART1_Info.RXD_CRC_Calc = crc16_modbus(&UART1_RXD_B_Temp[i], 1, UART1_Info.RXD_CRC_Init);
            UART1_Info.RXD_CRC_Init = UART1_Info.RXD_CRC_Calc;
        }
        UART1_Info.RXD_CRC_Init = 0xffff; // 初始化CRC寄存器初始值

        // 接收的CRC校验码处理
        UART1_Info.RXD_CRC_CkCd = (UART1_RXD_B_Temp[UART1_SEND_NUM - 2]) +
                                  (UART1_RXD_B_Temp[UART1_SEND_NUM - 1] << 8);

        // 计算的CRC校验码与接收的CRC校验码作比较和帧头判断(0X55AA)
        if (UART1_Info.RXD_CRC_Calc == UART1_Info.RXD_CRC_CkCd &&
            UART1_RXD_B_Temp[0] == 0XAA && UART1_RXD_B_Temp[1] == 0X55)
        {
            for (int i = 0; i < (UART1_SEND_NUM >> 1); i++)
            {
                UART1_Info.RXD_W[i] = UART1_RXD_B_Temp[2 * i] + (UART1_RXD_B_Temp[2 * i + 1] << 8);
            }

            /*------------------------------------------------------------------------------------*/
            /*---------------------UART1通信心跳位逻辑--------------------------------------------*/
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
        DMA->DMAEN1 |= 1 << 5; // uart1_TXD                                 //使能传输(UART1)
    }

    if (SysClockBase_ms.sys_1ms == 1)
    {
        /*------------------------------------------------------------------------------------*/
        /*---------------------UART1通信心跳监测----------------------------------------------*/
        COM_Uartx_HeartCheck(&UART1_Info);
    }

    /*------------------------------------------------------------------------------------*/
    /*---------------------UART1通信异常清数据--------------------------------------------*/
    if (UART1_Info.heart_OK_Status == 0) ////UART2接收数据通信异常时 RXD0_Status_Heart_OK心跳状态为 0
    {
        if (System_ProtectFlag_Info.all == 0 && INV_Ctrl_Info.mode_Operate == NORMAL_MODE)
        {
            System_ProtectFlag_Info.all |= E_COMMUNICA_ERR;
        }
        UART1_Info.RXD_W[0] = 0; // 2字节帧头数据；
        UART1_Info.RXD_W[1] = 0; // 前级工作模式：1-放电；2-充电
        UART1_Info.RXD_W[2] = 0; // 前级故障代码
        UART1_Info.RXD_W[3] = 0; // 前级给定PFC母线电压REF
        UART1_Info.RXD_W[4] = 0; // 前级充电状态：1-充电完成；0-需要充电
        UART1_Info.RXD_W[5] = 0;
        UART1_Info.RXD_W[6] = 0; // 前级准备OK信号：1-电源准备OK；0-准备 NOK
    }

    UARTx_DC_Info.err_Code = UART1_Info.RXD_W[2];        // 前级故障代码
    UARTx_DC_Info.mode_State = UART1_Info.RXD_W[1];      // 前级工作模式：1-放电；2-充电
    UARTx_DC_Info.ready_State = UART1_Info.RXD_W[6];     // 前级准备OK信号：1-电源准备OK；0-准备 NOK
    UARTx_DC_Info.vBus_SetVal = UART1_Info.RXD_W[3];     // 前级给定PFC母线电压REF
    UARTx_DC_Info.CHG_FinishState = UART1_Info.RXD_W[4]; // 前级充电状态：1-充电完成；0-需要充电

    // 串口通信，发送数据至DCDC侧
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
Others     : 市电是否OK逻辑处理
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

        UPS_Ctr_Info.V_ACIN_OK = 0; // 市电接入OK标志
        UPS_Ctr_Info.V_ACIN_NOK = 1;
        UPS_Ctr_Info.delta_T = 0;          //
        UPS_Ctr_Info.lock_Phase_OK = 0;    // 锁相OK
        UPS_Ctr_Info.V_ACIN_NOK_Cnt_P = 0; // 正半周市电接入NOK计数
        UPS_Ctr_Info.V_ACIN_NOK_Cnt_N = 0; // 负半周市电接入NOK计数
        UPS_Ctr_Info.V_ACIN_NOK_Cnt = 0;
    }
}

/*************************************************
Description: COM_CHG_INV_Select
Input      :
Return     :
Others     : 充电或者放电功能选择
*************************************************/
void COM_CHG_INV_Select(void)
{
    if (SysClockBase_ms.sys_Mode_1ms == 1)
    {
        if (System_ProtectFlag_Info.all == 0 &&
            State_Context.state_Value <= COM_RUN_STATE &&
            COM_AD_Data_Info.VACIN_Freq_Val_Fir > PFC_START_CHECK_FREQ_DN &&
            COM_AD_Data_Info.VACIN_Freq_Val_Fir < PFC_START_CHECK_FREQ_UP &&
            COM_AD_Data_Info.VACIN_RMS_Val_Fir >= PFC_START_CHECK_AC_VOL_DN_BACK) // 输入电压范围内时，启用控算法
        {
            COM_Ctr_Info.PFC_FREQ_NOKCnt = 0;
            if (COM_Ctr_Info.PFC_FREQ_Cnt < COM_Ctr_Info.PFC_FREQ_TimeVal)
            {
                COM_Ctr_Info.PFC_FREQ_Cnt++;
            }
            if (COM_Ctr_Info.PFC_FREQ_Cnt >= COM_Ctr_Info.PFC_FREQ_TimeVal)
            {
                // 注意：中断调用了硬件除法，外面不可再调用硬件除法函数(User_Divider)
                INV_Ctrl_Info.PWM_Freq_Init = 2000000L * INV_Ctrl_Info.periodDot_Val / INV_Ctrl_Info.PWM_Freq_Init_Temp;
                INV_Ctrl_Info.PWM_Period_Init = 64000000L / (INV_Ctrl_Info.PWM_Freq_Init * 2) - 2; // 计数方式：三角波
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

        // 市电接入判断，设定工作模式为PFC状态
        if (System_ProtectFlag_Info.all == 0 &&
            State_Context.state_Value <= COM_RUN_STATE &&
            COM_AD_Data_Info.VACIN_RMS_Val_Fir > PFC_START_CHECK_AC_VOL_DN &&
            COM_AD_Data_Info.VACIN_RMS_Val_Fir < PFC_START_CHECK_AC_VOL_UP &&
            COM_Ctr_Info.PFC_FREQ_State == 1 &&
            (COM_Ctr_Info.INV_PFC_Mode_Select == 1 || COM_Ctr_Info.INV_PFC_Mode_Select == 0) &&
            COM_AD_Data_Info.VACIN_Freq_Val_Fir > PFC_START_CHECK_FREQ_DN &&
            COM_AD_Data_Info.VACIN_Freq_Val_Fir < PFC_START_CHECK_FREQ_UP) // 输入电压范围内时，启用控算法
        {
            if (COM_Ctr_Info.PFC_AC_Vol_OK_Cnt < COM_Ctr_Info.PFC_AC_Vol_OK_TimeVal)
            {
                COM_Ctr_Info.PFC_AC_Vol_OK_Cnt++;
            }

            if (COM_Ctr_Info.PFC_AC_Vol_OK_Cnt >= COM_Ctr_Info.PFC_AC_Vol_OK_TimeVal)
            {
                UPS_Ctr_Info.V_ACIN_OK = 1; // 市电OK
                UPS_Ctr_Info.V_ACIN_NOK = 0;
            }

            if (COM_Ctr_Info.PFC_AC_Vol_OK_Cnt >= COM_Ctr_Info.PFC_AC_Vol_OK_TimeVal &&
                UPS_Ctr_Info.V_ACIN_OK == 1 &&
                (UPS_Ctr_Info.lock_Phase_OK == 1 || COM_Ctr_Info.INV_Enable_Flag == 0)) // 锁相完成或者未开逆变器
            {
                INV_START_DISABLE;

                // 重新进入初始化阶段
                COM_PWM_Disable(); // 关闭PWM输出、继电器

                // PFC_RY2_ENABLE;//UPS继电器动作     PASS
                INV_RY1_ENABLE; // 开启逆变器输出
                INV_RY3_ENABLE; // 开启逆变器输出

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
                PFC_Ctrl_Info.vBus_Ref_SS = PFC_VBUS_REF_INIT_SS;    // 电压环缓启动参考值给定
                PFC_Ref_Info.u32SS_vBus_Hold = PFC_VBUS_REF_INIT_SS; // PFC电压环缓启动参考值给定
                COM_Ctr_Info.PFC_AC_Vol_OK_Cnt = 0;
                COM_Ctr_Info.INV_PFC_Mode_Select = 2; // 检测市电正常工作模式为PFC模式
            }
        }
        else
        {
            COM_Ctr_Info.PFC_AC_Vol_OK_Cnt = 0;
        }

        // 逆变工作模式处理
        if (System_ProtectFlag_Info.all == 0 &&
            State_Context.state_Value <= COM_RUN_STATE &&
            COM_Ctr_Info.INV_Enable_Flag == 1 &&
            (COM_AD_Data_Info.VACIN_RMS_Val_Fir > PFC_START_CHECK_AC_VOL_UP_BACK ||
             COM_AD_Data_Info.VACIN_RMS_Val_Fir < PFC_START_CHECK_AC_VOL_DN_BACK ||
             COM_Ctr_Info.PFC_FREQ_State == 0 ||
             UPS_Ctr_Info.V_ACIN_NOK == 1) &&
            (COM_Ctr_Info.INV_PFC_Mode_Select == 0 || COM_Ctr_Info.INV_PFC_Mode_Select == 2)) // 输入电压范围内时，启用控算法
        {
            if (COM_Ctr_Info.PFC_AC_Vol_NOK_Cnt < COM_Ctr_Info.PFC_AC_Vol_NOK_TimeVal)
                COM_Ctr_Info.PFC_AC_Vol_NOK_Cnt++;
            if (COM_Ctr_Info.PFC_AC_Vol_NOK_Cnt >= COM_Ctr_Info.PFC_AC_Vol_NOK_TimeVal || UPS_Ctr_Info.V_ACIN_NOK == 1)
            {
                INV_START_DISABLE;

                COM_Ctr_Info.PFC_FREQ_State = 0;

                // 重新进入初始化阶段
                TMM->TMOER1 = _01_TMM_TMIOA0_OUTPUT_DISABLE | _02_TMM_TMIOB0_OUTPUT_DISABLE | _04_TMM_TMIOC0_OUTPUT_DISABLE | _08_TMM_TMIOD0_OUTPUT_DISABLE |
                              _10_TMM_TMIOA1_OUTPUT_DISABLE | _20_TMM_TMIOB1_OUTPUT_DISABLE | _40_TMM_TMIOC1_OUTPUT_DISABLE | _80_TMM_TMIOD1_OUTPUT_DISABLE;
                PORT->P1 &= ~(3 << 4); // 工频管清零

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
                COM_Ctr_Info.INV_PFC_Mode_Select = 1; // 逆变模式

                // 逆变器状态值判断
                if ((COM_AD_Data_Info.auxPower_Val_Fir > COM_AUX_POWER_LVP_VAL &&
                     COM_AD_Data_Info.auxPower_Val_Fir < COM_AUX_POWER_OVP_VAL)) // 判断逆变工作电源在开机检测范围内
                {
                    INV_START_ENABLE; // 发送逆变准备好信号
                }
            }
        }
        else
        {
            COM_Ctr_Info.PFC_AC_Vol_NOK_Cnt = 0;
        }

        // 设定工作模式为空闲状态
        if (System_ProtectFlag_Info.all == 0 &&
            State_Context.state_Value <= COM_RUN_STATE &&
            COM_Ctr_Info.INV_Enable_Flag == 0 &&
            (COM_AD_Data_Info.VACIN_RMS_Val_Fir > PFC_START_CHECK_AC_VOL_UP_BACK ||
             COM_AD_Data_Info.VACIN_RMS_Val_Fir < PFC_START_CHECK_AC_VOL_DN_BACK ||
             COM_Ctr_Info.PFC_FREQ_State == 0 ||
             UPS_Ctr_Info.V_ACIN_NOK == 1) &&
            (COM_Ctr_Info.INV_PFC_Mode_Select == 1 || COM_Ctr_Info.INV_PFC_Mode_Select == 2)) // 输入电压范围内时，启用控算法
        {
            if (COM_Ctr_Info.NO_Mode_OK_Cnt < COM_Ctr_Info.NO_Mode_OK_TimeVal)
            {
                COM_Ctr_Info.NO_Mode_OK_Cnt++;
            }

            if (COM_Ctr_Info.NO_Mode_OK_Cnt >= COM_Ctr_Info.NO_Mode_OK_TimeVal) //
            {
                // 重新进入初始化阶段
                COM_PWM_Disable(); // 关闭PWM输出、继电器

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
                COM_Ctr_Info.INV_PFC_Mode_Select = 0; // 正常工作模式为空闲模式
                INV_START_DISABLE;
            }
        }
        else
        {
            COM_Ctr_Info.NO_Mode_OK_Cnt = 0;
        }
    }
    SysClockBase_ms.sys_Mode_1ms = 0;

    // 根据频率变化实时更新工频周期计数值
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

    // 不在充电模式关PFC_RY2_DISABLE
    if (COM_Ctr_Info.INV_PFC_Mode_Select != PFC_MODE)
    {
        // PFC_RY2_DISABLE;
    }
}

/*************************************************
Description: COM_AD_Data_Deal
Input      :
Return     :
Others     : AD采样真实数据处理
*************************************************/
void COM_AD_Data_Deal(void)
{
    // AD采样真实数据处理,在真实值基础上放大设定倍数
    COM_AD_Data_Info.vRef_Val = ADSample_Info.ref_AD_Fir;             // 基准VREF
    COM_AD_Data_Info.temp_NTC_Val = ADSample_Info.temp_NTC_AD_FIR;    // 温度
    COM_AD_Data_Info.VACIN_RMS_Val = RMS_PQ_Info.out_RMS_PFC_AC_Vol;  // 市电输入电压有效值
    COM_AD_Data_Info.VACOUT_RMS_Val = RMS_PQ_Info.out_RMS_INV_AC_Vol; // 逆变输出电压有效值
    COM_AD_Data_Info.iLoad_RMS_Val = RMS_PQ_Info.out_RMS_CurLoad;     // 负载电流有效值
    COM_AD_Data_Info.iInduc_RMS_Val = RMS_PQ_Info.out_RMS_CurInduc;   // 电感电流有效值

    COM_AD_Data_Info.vBus_Val = ADSample_Info.vBus_AD_FIR * COM_REAL_VBUS_SCAL * COM_VBUS_BASE >> 12;                                 // 母线电压
    COM_AD_Data_Info.auxPower_Val = ADSample_Info.auxPower_AD_FIR * COM_REAL_AUXPOWER_SCAL * COM_AUX_POWER_BASE >> 12;                // 辅助电源电压
    COM_AD_Data_Info.VACIN_Freq_Val = COM_MCU_CLK * COM_REAL_VACIN_FREQ_SCAL / (2 * (TMM->TMGRA0 + 2)) / PFC_Ctrl_Info.AC_Vol_Freq;   // 市电输入电压频率（Hz）
    COM_AD_Data_Info.VACOUT_Freq_Val = COM_MCU_CLK * COM_REAL_VACOUT_FREQ_SCAL / (2 * (TMM->TMGRA0 + 2)) / INV_Ctrl_Info.AC_Vol_Freq; // 逆变输出电压频率（Hz）

    if (COM_AD_Data_Info.VACIN_Freq_Val > 700)
        COM_AD_Data_Info.VACIN_Freq_Val = 700;
    if (COM_AD_Data_Info.VACOUT_Freq_Val > 700)
        COM_AD_Data_Info.VACOUT_Freq_Val = 700;

    COM_AD_Data_Info.vBus_Hold = DFILTER_N(4, COM_AD_Data_Info.vBus_Val, COM_AD_Data_Info.vBus_Hold);
    COM_AD_Data_Info.vBus_Val_Fir = COM_AD_Data_Info.vBus_Hold >> 16; // 母线电压

    COM_AD_Data_Info.vRef_Val_Fir = COM_AD_Data_Info.vRef_Val;         // VRef参考
    COM_AD_Data_Info.temp_NTC_Val_Fir = COM_AD_Data_Info.temp_NTC_Val; // 器件温度
    COM_AD_Data_Info.auxPower_Val_Fir = COM_AD_Data_Info.auxPower_Val; // 辅助电源电压

    COM_AD_Data_Info.VACIN_Freq_Val_Fir = COM_AD_Data_Info.VACIN_Freq_Val;   // 市电输入电压频率
    COM_AD_Data_Info.VACOUT_Freq_Val_Fir = COM_AD_Data_Info.VACOUT_Freq_Val; // 逆变输出电压频率
    COM_AD_Data_Info.VACIN_RMS_Val_Fir = COM_AD_Data_Info.VACIN_RMS_Val;     // 市电输入电压有效值
    COM_AD_Data_Info.VACOUT_RMS_Val_Fir = COM_AD_Data_Info.VACOUT_RMS_Val;   // 逆变输出电压有效值
    COM_AD_Data_Info.iLoad_RMS_Val_Fir = COM_AD_Data_Info.iLoad_RMS_Val;     // 负载电流有效值
    COM_AD_Data_Info.iInduc_RMS_Val_Fir = COM_AD_Data_Info.iInduc_RMS_Val;   // 电感电流有效值

    COM_AD_Data_Info.VACIN_BypassPower = RMS_PQ_Info.out_S;    // 市电输入旁路功率（VA）负载电流有效值*电压有效值
    COM_AD_Data_Info.VACIN_PFC_Power = RMS_PQ_Info.out_P;      // 市电输入PFC功率（W）电感电流*输入电压
    COM_AD_Data_Info.VACOUT_ApparentPower = RMS_PQ_Info.out_S; // 逆变输出视在功率（VA）负载电流有效值*电压有效值
    COM_AD_Data_Info.VACOUT_ActivePower = RMS_PQ_Info.out_P;   // 逆变输出有功功率（W）电感电流*输入电压
}

/*************************************************
Description: COM_Function
Input      :
Return     :
Others     : 用户任务执行
*************************************************/
uint8_t u8_System1msbit_cnt = 0;
void COM_Function(void)
{
    // 获取状态机的状态
    State_Context.ContextPtr(&State_Context);

    // 执行状态机对应的任务
    State_Context_Task();

    // error关闭驱动
    COM_CloseDrive();

    // 交流信号RMS处理
    User_RMS_Deal();

    // LED状态指示
    User_LED_Deal();

    // KEY状态处理
    User_Key_Deal();

    // 串口1数据处理
    COM_UART1_Deal();

    // 充电或者放电功能选择
    COM_CHG_INV_Select();

    // AD采样真实数据处理
    COM_AD_Data_Deal();

    if (SysClockBase_ms.sys_1ms == 1)
    {
        u8_System1msbit_cnt++; // 1ms时钟周期在Main函数存在的周期计数
    }
    if (u8_System1msbit_cnt >= 2)
    {
        SysClockBase_ms.sys_1ms = 0; // 1ms 时钟标记清零
        u8_System1msbit_cnt = 0;     // 1ms时钟周期在Main函数存在的周期计数值清零
    }
}
/*------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------*/
