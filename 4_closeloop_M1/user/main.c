#include "stm32f10x.h"                  // 引入STM32F10x系列单片机底层驱动库
#include <stdlib.h>                     // 引入标准库，用于atof等函数
#include "MyProject.h"                  // 引入自定义工程头文件，包含外设和FOC相关声明

/************************************************
SimpleMotor开发板
闭环控制  演示
1、力矩模式
2、速度模式
3、位置模式
=================================================
本程序仅供学习，引用代码请标明出处
使用教程：https://blog.csdn.net/loop222/article/details/120471390
         《SimpleFOC移植STM32(四) —— 闭环控制》
创建日期：20230208
作    者：loop222 @郑州
************************************************/
/******************************************************************************/
#define LED_blink    GPIOC->ODR^=(1<<13) // 定义LED翻转宏，操作GPIOC的13引脚输出寄存器取反
/******************************************************************************/
float target;                            // 定义控制目标值（力矩/速度/位置）
/******************************************************************************/
void commander_run(void);                // 声明串口指令处理函数
/******************************************************************************/
void GPIO_Config(void)                   // GPIO初始化配置函数
{
    GPIO_InitTypeDef GPIO_InitStructure; // 定义GPIO初始化结构体变量
	
    // 使能GPIOA、GPIOB、GPIOC时钟和AFIO复用功能时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE); // 禁用JTAG，释放相关引脚
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;         // 配置PC13引脚（LED）
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   // 推挽输出模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;  // 输出速度10MHz
    GPIO_Init(GPIOC, &GPIO_InitStructure);             // 初始化GPIOC13引脚
    GPIO_ResetBits(GPIOC,GPIO_Pin_13);                 // PC13置低，上电点亮LED
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;          // 配置PB9引脚（电机1使能）
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   // 推挽输出模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;  // 输出速度10MHz
    GPIO_Init(GPIOB, &GPIO_InitStructure);             // 初始化GPIOB9引脚
    GPIO_ResetBits(GPIOB,GPIO_Pin_9);                  // PB9置低解除使能（Motor_init中使能）
}
/******************************************************************************/
int main(void)                           // 主函数入口
{
    unsigned int count_i=0;              // 定义循环计数变量
	
    GPIO_Config();                       // 调用GPIO初始化函数
    uart_init(115200);                   // 初始化串口，波特率115200
#if M1_AS5600                            // 条件编译：如果使用AS5600编码器
    I2C_Init_();                         // 初始化I2C总线（AS5600通信）
    printf("AS5600\r\n");                // 串口打印编码器类型
#elif M1_TLE5012B                        // 条件编译：如果使用TLE5012B编码器
    SPI2_Init_();                        // 初始化SPI2总线（TLE5012B通信）
    printf("TLE5012B\r\n");              // 串口打印编码器类型
#endif
    TIM2_PWM_Init();                     // 初始化TIM2，生成PWM波驱动电机
    TIM4_1ms_Init();                     // 初始化TIM4，1ms中断（用于计时）
	
    delay_ms(1000);                      // 延时1秒，等待系统稳定
    MagneticSensor_Init();               // 初始化磁编码器（AS5600/TLE5012B）
	
    voltage_power_supply=12;             // 设置电源供电电压12V
    pole_pairs=7;                        // 设置电机极对数为7（按实际电机配置）
    voltage_limit=6;                     // 设置电压限制6V（<12/1.732=6.9）
    velocity_limit=20;                   // 设置速度限制20rad/s（开环/位置环使用）
    voltage_sensor_align=2.5;            // 设置传感器对齐电压2.5V（云台电机2-3V）
    torque_controller=Type_voltage;      // 设置力矩控制模式为电压模式
    controller=Type_velocity;            // 设置闭环控制模式为速度模式（可改为位置/力矩）
    target=0;                            // 初始化控制目标值为0
	
    Motor_init();                        // 初始化电机硬件和参数
    Motor_initFOC();                     // 初始化FOC算法相关配置
    PID_init();                          // 初始化PID控制器参数（内部已预设）
    printf("Motor ready.\r\n");          // 串口打印电机就绪信息
	
    systick_CountMode();                 // 切换SysTick为计数模式（禁用delay_us/ms）
	
    while(1)                             // 主循环
    {
        count_i++;                       // 计数变量自增
		
        if(time1_cntr>=200)              // 每200ms（1ms中断*200）执行一次
        {
            time1_cntr=0;                // 重置计时变量
            LED_blink;                   // 翻转LED状态（心跳灯）
        }
        if(time2_cntr>=1000)             // 每1000ms执行一次
        {
            time2_cntr=0;                // 重置计时变量
            //printf("%d\r\n",count_i);  // 调试用：打印计数（当前注释）
            count_i=0;                   // 重置计数变量
        }
        move(target);                    // 执行目标值更新（位置/速度/力矩）
        loopFOC();                       // 执行FOC核心算法（电流采样/解耦/PWM输出）
        commander_run();                 // 检查串口指令并处理
    }
}
/******************************************************************************/
void commander_run(void)                 // 串口指令处理函数
{
    if((USART_RX_STA&0x8000)!=0)         // 判断串口接收是否完成（最高位为1表示完成）
    {
        switch(USART_RX_BUF[0])          // 根据接收缓冲区第一个字符判断指令
        {
            case 'H':                    // 指令H：打印测试信息
                printf("Hello World!\r\n"); // 串口打印Hello World
                break;                   // 退出switch
            case 'T':                    // 指令T+数值：设置目标值（如T6.28）
                target=atof((const char *)(USART_RX_BUF+1)); // 转换字符串为浮点型目标值
                printf("RX=%.4f\r\n", target); // 串口回显接收到的目标值
                break;                   // 退出switch
            case 'D':                    // 指令D：禁用电机
                M1_Disable;              // 执行电机禁用操作（宏定义）
                printf("OK!\r\n");       // 串口打印确认信息
                break;                   // 退出switch
            case 'E':                    // 指令E：使能电机
                M1_Enable;               // 执行电机使能操作（宏定义）
                printf("OK!\r\n");       // 串口打印确认信息
                break;                   // 退出switch
        }
        USART_RX_STA=0;                  // 重置串口接收状态标志
    }
}
/******************************************************************************/