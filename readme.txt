2023/03/29
1.优化interrupt.c中的锁相环程序【INV_Lock_Phase();】和INV控制程序【INV_Ctrl();】的时序，即先执行锁相环计算出周期值，再执行INV控制程序更新占空比；

2.优化sys_define_param.h中的母线过压保护时间【COM_VBUS_OVP1_TIME】和母线过压返回值【COM_VBUS_OVP2_VAL_BACK】；

3.优化interrupt.c中的母线过压IO信号【VBUS_OVER_ENABLE】置位逻辑，即母线电压过压保护后置位过压IO信号；

2023/04/04
1.优化通信程序，加入DMA传输，关闭串口接收中断【sys_hardware_init.c---void Sys_IE_Init(void)】，前后级单片机通信波特率修改为38400；
具体更改点：①DMA初始化【sys_hardware_init.c---void Sys_HardConfigInit(void)】；②串口接收逻辑处理优化【user_function.c---void COM_UART1_Deal(void)】；③串口发送逻辑处理优化【user_communication.c---void UART1_INV_DataSend(int32_t *tx_buf)】；④；
2.硬件更改：R47:0Ω；R4:510Ω

2023/04/06
1.优化interrupt.c中void tmm0_interrupt(void)函数的母线过压或者逆变故障时给升压侧发送故障信号逻辑【VBUS_OVER_ENABLE】；
2.优化user_function.c 中void DC_CloseDrive(void)函数的故障关断逻辑(第350行)，现为在任何状态时都会因故障代码进入故障态；


2023/04/15
1.逆变工作模式时的开关频率修改为24KHz；
2.计数方式由锯齿波计数优化为三角波计数【sys_hardware_init.c---void EPWM_ConfigInit(void)】；
3.采样优化为硬件触发采样，通过DMA存取采样数据【sys_hardware_init.c---void ADC_ConfigInit(void)】；
4.硬件减弱电感电流采样滤波电容，反馈电容470pF,RC滤波电容2nF，同时优化限流环；

2023/04/17
1.优化interrupt.c中AD数据处理放入PWM中断处理，减少AD中断执行时间;

2023/04/18
1.优化interrupt.c中AD数据软件滤波去掉（输入电压/输出电压/电感电流）,空载输出电压更稳定;
2.优化interrupt.c中PFC控制中，充电功率800W较小，电感电流由右移7位变为6位，电流环增益加大;

2023/04/19
1.优化interrupt.c中使用PFC_Ctrl_Info.AC_VolBase随输入电压有效值变化;
2.限流环补偿时加入有功功率;

2023/04/21
1.有功功率计算时不再使用绝对值计算，（当电压电流存在相位差时用绝对值计算存在问题）；

2023/04/27
1.UPS锁相优化；

2023/04/28
1.UPS锁相优化，验证完成；

2023/05/05
1.PFC使能PWM驱动放入中断中，防止执行时被中断打断；
2.优化INV_Ctrl_Info.PWM_Period_43Hz和INV_Ctrl_Info.PWM_Period_67Hz 两个变量，在初始化时更新；


2023/05/06
1.使能PWM驱动放在占空比和周期更新之后；

2023/05/08
1.一直检测不在充电模式就关PFC_RY2_DISABLE【void COM_Function(void)】；

2023/05/10
1.优化输出电流计算精度【void RMS_PQ_Calc(RMS_PQ_Var_t *rms_PQ)】；

2023/05/16
1.电感电流采样100pf，放电开关频率21K，新的输出电感1mh，充放电验证完成，确认充电PFC的电流谐波；

2023/05/19
1.PFC模式时，延时发送PFC准备OK的标志给DC侧（待母线电压稳定）；
2.优化PFC电压环变PI逻辑和电压环ref缓启动逻辑；

2023/05/22
1.读取IO口的状态，直接操作寄存器PORT->PREAD6；

2023/05/26
1.增加RY3，维持逆变器输出继电器，降低损耗；


2023/05/30
1.INV侧输出电压采样和输出电感电流采样受前级干扰影响较大，在前级GND和后级的PGND之间加入Y电容后，干扰问题得到解决；

2023/06/08
1.DC和INV侧通信数据增加优化；
2.INV通信传输至DC侧的VBUS数据为真实值的10倍；

2023/06/09
1.整理软件平台，优化部分宏定义；
2.优化输入频率限制功能；

2023/06/15
1.加入PFC独立工作模式：PFC独立工作，给固定的电压环路ref值(注意：需断开主变压器)，【PFC_SINGLE_CTRL_SELECT】

2023/07/11
1.充电时该值不能设置太高【PFC_VBUS_READY_OK_MIN】，尽量小于320V，否则影响充电，低于该值DC侧会直接封波。

2023/07/18
1.PFC限占空比优化(PFC_PWM_DUTY_UP_LIMIT)。

2023/09/03
1.AD基准校准时优化，生成AD校准OK的判断条件优化。

2023/09/19
1.母线电压滤波减弱，滤波重影响母线过压保护。

