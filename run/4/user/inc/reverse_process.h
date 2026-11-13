#ifndef REVERSE_PROCESS_H
#define REVERSE_PROCESS_H

#include "zf_common_headfile.h"

/* 斑马线停车之后倒车回起点：
 *   0 空闲 → 1 等停稳 → 2 倒车中 → 3 刹车 → 4 完成
 */

#define REVERSE_DISTANCE_COUNTS 8000.0f  /* 倒车阶段行驶的编码器计数，之后进入刹车 */
#define REVERSE_PWM_DUTY        1200     /* 倒车时两个电机的 PWM（满量程 10000） */
#define REVERSE_SETTLE_FRAMES   8        /* 连续多少帧速度接近 0 认为停稳 */
#define REVERSE_STEER_GAIN      0.30f    /* 倒车时用中线修正方向的增益 */

/* 刹车强度：与斑马线第一次停车保持一致 */
#define REVERSE_BRAKE_K         50       /* 每个编码器计数对应的正向占空比 */
#define REVERSE_BRAKE_MAX       1600     /* 刹车占空比上限 */
#define REVERSE_BRAKE_MIN_PWM   500      /* 低速时的最小保持量 */
#define REVERSE_BRAKE_FRAMES    20       /* 最多刹 20 帧（约 400 ms） */

extern volatile uint8_t reverse_drive_enable;   /* 1 = 中断里接管电机 */
extern volatile uint8_t reverse_dir_forward;    /* 1 正向（刹车），0 反向（倒车） */
extern volatile int16_t reverse_pwm;            /* 输出占空比 */

uint8_t reverse_state(void);    /* 0 空闲 1 等停稳 2 倒车中 3 刹车 4 完成 */
float   reverse_distance(void); /* 已倒车的编码器计数 */
void    reverse_reset(void);    /* 复位状态机（C14 按键调用） */
void    reverse_update(void);   /* 主循环每帧调用 */

#endif
