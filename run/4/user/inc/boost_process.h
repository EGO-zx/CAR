#ifndef BOOST_PROCESS_H
#define BOOST_PROCESS_H

#include "zf_common_headfile.h"

/* 直道加速策略（从 main.c 提出来的独立模块）
 *
 * 两套硬件开关：
 *   开关1（C27）：90000~105000 → 36 / 2500 上限 / 增益 0.30，直道时中线左移
 *   开关2（C26）：90000~110000 → 40 / 3200 上限 / 增益 0.25，
 *                之后 10000 计数内从 40 线性减速到 30
 * 公共规则：
 *   - 环岛（R≠0）期间不加速；出环后走 RING_COOLDOWN_DIST 计数才允许再加速
 *   - 加速段与减速段内舵机慢打（SERVO_SLEW_MAX）
 */

typedef struct
{
    float   steer_gain;   /* 本帧转向增益 */
    uint8_t slow_steer;   /* 1 = 本帧舵机慢打 */
    uint8_t switch_on;    /* 任一加速开关当前为打开状态 */
    uint8_t sw1;          /* 开关1（C27）打开 */
    uint8_t sw2;          /* 开关2（C26）打开 */
} boost_frame_t;

void boost_init(void);          /* 开关引脚初始化 */
void boost_reset(void);         /* 清空状态（一键复位时调用） */

/* 每帧调用一次：内部会直接设置 speed_target 与两个电机的 outmax，
 * 需要中线左移时改写 mid，返回本帧的增益与慢打标志。 */
boost_frame_t boost_update(float enc_distance,
                           uint8_t speed_mode,
                           uint8_t ring_state,
                           uint8_t cross_path_state,
                           uint8_t cross_seen,
                           uint8_t reversing,
                           uint8_t control_paired,
                           uint8_t control_single,
                           uint8_t *mid,
                           const uint8_t *left,
                           const uint8_t *right);

const uint8_t *boost_bias_rows(void);  /* 本帧被左移的行（1 = 已左移） */

/* 减速段的反向制动请求：中断里按这两个值给电机反压 */
extern volatile uint8_t boost_brake_enable;
extern volatile int16_t boost_brake_pwm;

#endif
