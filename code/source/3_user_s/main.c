/*******************************************************************************
* Copyright (C) 2019 China Micro Semiconductor Limited Company. All Rights Reserved.
*
* This software is owned and published by:
* CMS LLC, No 2609-10, Taurus Plaza, TaoyuanRoad, NanshanDistrict, Shenzhen, China.
*
* BY DOWNLOADING, INSTALLING OR USING THIS SOFTWARE, YOU AGREE TO BE BOUND
* BY ALL THE TERMS AND CONDITIONS OF THIS AGREEMENT.
*
* This software contains source code for use with CMS
* components. This software is licensed by CMS to be adapted only
* for use in systems utilizing CMS components. CMS shall not be
* responsible for misuse or illegal use of this software for devices not
* supported herein. CMS is providing this software "AS IS" and will
* not be responsible for issues arising from incorrect user implementation
* of the software.
*
* This software may be replicated in part or whole for the licensed use,
* with the restriction that this Disclaimer and Copyright notice must be
* included with each copy of this software, whether used in part or whole,
* at all times.
*/



/*	Local pre-processor symbols('#define')
*****************************************************************************/

/****************************************************************************/
/*	Global variable definitions(declared in header file with 'extern')
*****************************************************************************/


/****************************************************************************/
/*	Local type definitions('typedef')
*****************************************************************************/

/****************************************************************************/
/*	Local variable  definitions('static')
*****************************************************************************/

/****************************************************************************/
/*	Local function prototypes('static')
*****************************************************************************/


/****************************************************************************/
/*	Function implementation - global ('extern') and local('static')
*****************************************************************************/

/****************************************************************************/
/*	include files
*****************************************************************************/
#include "sys_state_machine.h"
#include "user_function.h"
#include "sys_mcu_header.h"
#include "PLL_Ctrl.h"
/****************************************************************************/

/*************************************************
Description: main
Input      : 
Return     : 
Others     : 主调用函数
*************************************************/
int main(void)
{   
    //硬件配置初始化 
    Sys_HardConfigInit();
    //变量初始化
    Sys_Variableinit();
    //锁相参数初始化
    PLL_Ctrl_Param(&PLL_Ctrl_Info_V_ACIN);
    PLL_Ctrl_Param(&PLL_Ctrl_Info_V_ACOUT);
    
    PFC_RY2_ENABLE;//先把继电器开起来
    while(1)
    {		
        //喂狗
        WDT_Restart();          
        
        //用户功能任务调用（LED、KEY......）
        COM_Function();
    }		
}



