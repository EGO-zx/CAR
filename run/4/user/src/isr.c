/*********************************************************************************************************************
* RT1064DVL6A Opensourec Library 即（RT1064DVL6A 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
* 
* 本文件是 RT1064DVL6A 开源库的一部分
* 
* RT1064DVL6A 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
* 
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
* 
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
* 
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
* 
* 文件名称          isr
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 8.32.4 or MDK 5.33
* 适用平台          RT1064DVL6A
* 店铺链接          https://seekfree.taobao.com/
* 
* 修改记录
* 日期              作者                备注
* 2022-09-21        SeekFree            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "isr.h"
#include "main.h"

int16 encoder_data_1 = 0;
int16 encoder_data_2 = 0;
int16 encoder_data_3 = 0;
int16 encoder_data_4 = 0;



void CSI_IRQHandler(void)
{
    CSI_DriverIRQHandler();     // 调用SDK自带的中断函数 这个函数最后会调用我们设置的回调函数
    __DSB();                    // 数据同步隔离
}

void SysTick_Handler(void)
{

}

void PIT_IRQHandler(void)
{
    if (pit_flag_get(PIT_CH0))
    {
        // 获取编码器读数
        encoder_data_1 = -encoder_get_count(ENCODER_1); // 获取编码器计数
        encoder_clear_count(ENCODER_1);                // 清空编码器计数
        encoder_data_2 = +encoder_get_count(ENCODER_2);
        encoder_clear_count(ENCODER_2);

        float target = zebra_motor_target(speed_target);
        if (target <= 0.0f)
        {
            /* 斑马线停车：反向制动，最多 200 ms。
             * 反向占空比按当前转速成比例，速度为 0 或超时后彻底断电。 */
            #define ZEBRA_BRAKE_K       40      /* 每 1 个编码器计数对应的反向占空比 */
            #define ZEBRA_BRAKE_MAX     1200    /* 反向占空比上限（满量程 10000） */
            #define ZEBRA_BRAKE_MIN     1       /* 转速低于此值直接断电 */
            #define ZEBRA_BRAKE_TICKS   40      /* 最多制动 400 ms */
            #define ZEBRA_BRAKE_MIN_PWM 400     /* 低速时保持的最小反向占空比，防止滚动 */
            static uint8_t zebra_brake_ticks = 0;
            int speed1 = encoder_data_1 < 0 ? -encoder_data_1 : encoder_data_1;
            int speed2 = encoder_data_2 < 0 ? -encoder_data_2 : encoder_data_2;
            speed_pwm = 0.0f;
            if (zebra_stop_requested && zebra_brake_ticks < ZEBRA_BRAKE_TICKS &&
                (speed1 > ZEBRA_BRAKE_MIN || speed2 > ZEBRA_BRAKE_MIN))
            {
                int b1 = ZEBRA_BRAKE_K * speed1;
                int b2 = ZEBRA_BRAKE_K * speed2;
                if (b1 > ZEBRA_BRAKE_MAX) b1 = ZEBRA_BRAKE_MAX;
                if (b2 > ZEBRA_BRAKE_MAX) b2 = ZEBRA_BRAKE_MAX;
                /* 低速时按转速比例算出来的制动量太小，车会一直滚；
                 * 给一个最小保持量，把车真正按住。 */
                if (b1 < ZEBRA_BRAKE_MIN_PWM) b1 = ZEBRA_BRAKE_MIN_PWM;
                if (b2 < ZEBRA_BRAKE_MIN_PWM) b2 = ZEBRA_BRAKE_MIN_PWM;
                gpio_set_level(MOTOR1_DIR,
                    MOTOR1_FORWARD_DIR_LEVEL == GPIO_HIGH ? GPIO_LOW : GPIO_HIGH);
                gpio_set_level(MOTOR2_DIR,
                    MOTOR2_FORWARD_DIR_LEVEL == GPIO_HIGH ? GPIO_LOW : GPIO_HIGH);
                pwm_set_duty(MOTOR1_PWM, b1);
                pwm_set_duty(MOTOR2_PWM, b2);
                ++zebra_brake_ticks;
            }
            else
            {
                pwm_set_duty(MOTOR1_PWM, 0);
                pwm_set_duty(MOTOR2_PWM, 0);
            }
        }
        else
        {
            // 编码器值即为速度值
            speed_real = encoder_data_1;
            // PID更新
            speed_pwm = PidLocCtrl(&speed_pid_l, target - speed_real, 1.f);
            // 只考虑一个方向转动
            pwm_set_duty(MOTOR1_PWM, MAX(speed_pwm, 0));
            gpio_set_level(MOTOR1_DIR, MOTOR1_FORWARD_DIR_LEVEL);
            // 编码器值即为速度值
            speed_real = encoder_data_2;
            // PID更新
            speed_pwm = PidLocCtrl(&speed_pid_r, target - speed_real, 1.f);
            // 只考虑一个方向转动
            pwm_set_duty(MOTOR2_PWM, MAX(speed_pwm, 0));
            gpio_set_level(MOTOR2_DIR, MOTOR2_FORWARD_DIR_LEVEL);
        }

        pit_flag_clear(PIT_CH0);
    }

    if(pit_flag_get(PIT_CH1))
    {
        pit_flag_clear(PIT_CH1);
    }
    
    if(pit_flag_get(PIT_CH2))
    {
        pit_flag_clear(PIT_CH2);
    }
    
    if(pit_flag_get(PIT_CH3))
    {
        pit_flag_clear(PIT_CH3);
    }

    __DSB();
}

void LPUART1_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART1))
    {
        // 接收中断
    #if DEBUG_UART_USE_INTERRUPT                        // 如果开启 debug 串口中断
        debug_interrupr_handler();                      // 调用 debug 串口接收处理函数 数据会被 debug 环形缓冲区读取
    #endif                                              // 如果修改了 DEBUG_UART_INDEX 那这段代码需要放到对应的串口中断去
        
    }
        
    LPUART_ClearStatusFlags(LPUART1, kLPUART_RxOverrunFlag);    // 不允许删除
}

void LPUART2_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART2))
    {
        // 接收中断
        
    }
        
    LPUART_ClearStatusFlags(LPUART2, kLPUART_RxOverrunFlag);    // 不允许删除
}

void LPUART3_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART3))
    {
        // 接收中断
        
    }
        
    LPUART_ClearStatusFlags(LPUART3, kLPUART_RxOverrunFlag);    // 不允许删除
}

void LPUART4_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART4))
    {
        // 接收中断 
        if(NULL != flexio_camera_uart_handler)
        {
            flexio_camera_uart_handler();
        }
    }
        
    LPUART_ClearStatusFlags(LPUART4, kLPUART_RxOverrunFlag);    // 不允许删除
}

void LPUART5_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART5))
    {
        // 接收中断
        if(NULL != camera_uart_handler)
        {
            camera_uart_handler();
        }
    }
        
    LPUART_ClearStatusFlags(LPUART5, kLPUART_RxOverrunFlag);    // 不允许删除
}

void LPUART6_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART6))
    {
        // 接收中断
        
    }
        
    LPUART_ClearStatusFlags(LPUART6, kLPUART_RxOverrunFlag);    // 不允许删除
}


void LPUART8_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART8))
    {
        // 接收中断
        if(NULL != wireless_module_uart_handler)
        {
            wireless_module_uart_handler();
        }
        
    }
        
    LPUART_ClearStatusFlags(LPUART8, kLPUART_RxOverrunFlag);    // 不允许删除
}


void GPIO1_Combined_0_15_IRQHandler(void)
{
    if(exti_flag_get(B0))
    {
        exti_flag_clear(B0);// 清除中断标志位
    }
    
}


void GPIO1_Combined_16_31_IRQHandler(void)
{
    if(exti_flag_get(B16))
    {
        exti_flag_clear(B16); // 清除中断标志位
    }

    
}

void GPIO2_Combined_0_15_IRQHandler(void)
{
    // 勿删除此IF语句
    if(NULL != flexio_camera_vsync_handler)
    {
        flexio_camera_vsync_handler();
    }
    
    
    if(exti_flag_get(C0))
    {
        exti_flag_clear(C0);// 清除中断标志位
    }

}


void GPIO2_Combined_16_31_IRQHandler(void)
{
    // -----------------* ToF INT 更新中断 预置中断处理函数 *-----------------
    //tof_module_exti_handler();
    // -----------------* ToF INT 更新中断 预置中断处理函数 *-----------------
    
    if(exti_flag_get(C16))
    {
        exti_flag_clear(C16); // 清除中断标志位
    }
    
    
}




void GPIO3_Combined_0_15_IRQHandler(void)
{

    if(exti_flag_get(D4))
    {
        exti_flag_clear(D4);// 清除中断标志位
    }
}









/*
中断函数名称，用于设置对应功能的中断函数
Sample usage:当前启用了周期定时器中断
void PIT_IRQHandler(void)
{
    //务必清除标志位
    __DSB();
}
记得进入中断后清除标志位
CTI0_ERROR_IRQHandler
CTI1_ERROR_IRQHandler
CORE_IRQHandler
FLEXRAM_IRQHandler
KPP_IRQHandler
TSC_DIG_IRQHandler
GPR_IRQ_IRQHandler
LCDIF_IRQHandler
CSI_IRQHandler
PXP_IRQHandler
WDOG2_IRQHandler
SNVS_HP_WRAPPER_IRQHandler
SNVS_HP_WRAPPER_TZ_IRQHandler
SNVS_LP_WRAPPER_IRQHandler
CSU_IRQHandler
DCP_IRQHandler
DCP_VMI_IRQHandler
Reserved68_IRQHandler
TRNG_IRQHandler
SJC_IRQHandler
BEE_IRQHandler
PMU_EVENT_IRQHandler
Reserved78_IRQHandler
TEMP_LOW_HIGH_IRQHandler
TEMP_PANIC_IRQHandler
USB_PHY1_IRQHandler
USB_PHY2_IRQHandler
ADC1_IRQHandler
ADC2_IRQHandler
DCDC_IRQHandler
Reserved86_IRQHandler
Reserved87_IRQHandler
GPIO1_INT0_IRQHandler
GPIO1_INT1_IRQHandler
GPIO1_INT2_IRQHandler
GPIO1_INT3_IRQHandler
GPIO1_INT4_IRQHandler
GPIO1_INT5_IRQHandler
GPIO1_INT6_IRQHandler
GPIO1_INT7_IRQHandler
GPIO1_Combined_0_15_IRQHandler
GPIO1_Combined_16_31_IRQHandler
GPIO2_Combined_0_15_IRQHandler
GPIO2_Combined_16_31_IRQHandler
GPIO3_Combined_0_15_IRQHandler
GPIO3_Combined_16_31_IRQHandler
GPIO4_Combined_0_15_IRQHandler
GPIO4_Combined_16_31_IRQHandler
GPIO5_Combined_0_15_IRQHandler
GPIO5_Combined_16_31_IRQHandler
WDOG1_IRQHandler
RTWDOG_IRQHandler
EWM_IRQHandler
CCM_1_IRQHandler
CCM_2_IRQHandler
GPC_IRQHandler
SRC_IRQHandler
Reserved115_IRQHandler
GPT1_IRQHandler
GPT2_IRQHandler
PWM1_0_IRQHandler
PWM1_1_IRQHandler
PWM1_2_IRQHandler
PWM1_3_IRQHandler
PWM1_FAULT_IRQHandler
SEMC_IRQHandler
USB_OTG2_IRQHandler
USB_OTG1_IRQHandler
XBAR1_IRQ_0_1_IRQHandler
XBAR1_IRQ_2_3_IRQHandler
ADC_ETC_IRQ0_IRQHandler
ADC_ETC_IRQ1_IRQHandler
ADC_ETC_IRQ2_IRQHandler
ADC_ETC_ERROR_IRQ_IRQHandler
PIT_IRQHandler
ACMP1_IRQHandler
ACMP2_IRQHandler
ACMP3_IRQHandler
ACMP4_IRQHandler
Reserved143_IRQHandler
Reserved144_IRQHandler
ENC1_IRQHandler
ENC2_IRQHandler
ENC3_IRQHandler
ENC4_IRQHandler
TMR1_IRQHandler
TMR2_IRQHandler
TMR3_IRQHandler
TMR4_IRQHandler
PWM2_0_IRQHandler
PWM2_1_IRQHandler
PWM2_2_IRQHandler
PWM2_3_IRQHandler
PWM2_FAULT_IRQHandler
PWM3_0_IRQHandler
PWM3_1_IRQHandler
PWM3_2_IRQHandler
PWM3_3_IRQHandler
PWM3_FAULT_IRQHandler
PWM4_0_IRQHandler
PWM4_1_IRQHandler
PWM4_2_IRQHandler
PWM4_3_IRQHandler
PWM4_FAULT_IRQHandler
Reserved171_IRQHandler
GPIO6_7_8_9_IRQHandler*/



