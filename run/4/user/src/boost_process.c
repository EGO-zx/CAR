#include "boost_process.h"
#include "pid.h"
#include "img_process.h"

/* ---------------- 参数区 ---------------- */

// 转向增益
#define STEER_GAIN_NORMAL   0.40f   /* 普段 */
#define STEER_GAIN_ZONE     0.30f   /* 开关1 加速区间 */
#define AGGR_GAIN           0.30f   /* 开关2 激进加速区间 */
#define STEER_GAIN_TAIL     0.50f   /* 十字之后 H 档尾段 */
#define RING_STEER_GAIN     0.30f   /* 环岛状态（R≠0） */

// 速度档位
#define SPEED_MODE_L 0
#define SPEED_MODE_M 1
#define SPEED_MODE_H 2
#define SPEED_TARGET_NORMAL 30.0f
#define SPEED_OUTMAX_L      1500.0f
#define SPEED_OUTMAX_M      1800.0f

// 开关1（C27）
#define ZONE_SWITCH_PIN    (C27)
#define ZONE_SWITCH_ACTIVE (GPIO_HIGH)
#define ZONE_DIST_MIN    90000.0f
#define ZONE_DIST_MAX   105000.0f
#define ZONE_TARGET_SPEED   36.0f
#define ZONE_OUTMAX       2500.0f

// 开关2（C26）
#define AGGR_SWITCH_PIN    (C26)
#define AGGR_SWITCH_ACTIVE (GPIO_HIGH)
#define AGGR_DIST_MIN    90000.0f
#define AGGR_DIST_MAX   100000.0f
#define AGGR_TARGET_SPEED   40.0f
#define AGGR_OUTMAX       3200.0f
#define AGGR_DECEL_DIST  10000.0f
#define AGGR_DECEL_OUTMAX 3200.0f

// 十字之后的 H 档尾段
#define TAIL_OUTMAX       2000.0f
#define TAIL_TARGET_SPEED    32.0f

// 环岛退出后的冷却
#define RING_COOLDOWN_DIST 10000.0f

// 中线右移（避险用）：两个开关下，90000~120000 之间生效
#define BIAS_DIST_MIN    90000.0f
#define BIAS_DIST_MAX   120000.0f
#define BIAS_PX              9.0f
#define BIAS_STRAIGHT_PAIRED 18
#define BIAS_STRAIGHT_SINGLE  3
#define BIAS_HOLD_ON          5
#define BIAS_HOLD_MAX        10
#define BIAS_SLEW          0.25f

/* ---------------- 状态 ---------------- */

static uint8_t bias_row[MT9V03X_H];
static uint8_t bias_hold;
static float   bias_px;
static uint8_t ring_seen;
static uint8_t ring_cooling;
static float   ring_cool_base;

volatile uint8_t boost_brake_enable = 0;
volatile int16_t boost_brake_pwm = 0;

/* 减速段反向制动强度 */
#define BOOST_BRAKE_K     120.0f   /* 每超出目标 1 个速度计数对应的反向占空比 */
#define BOOST_BRAKE_MAX   2800     /* 反向占空比上限 */
#define BOOST_BRAKE_MIN_PWM 150    /* 最小保持量（太大会在目标附近来回开关） */
#define BOOST_BRAKE_DEADBAND 0.5f  /* 超出目标多少才开始刹 */

void boost_init(void)
{
    gpio_init(ZONE_SWITCH_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(AGGR_SWITCH_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
}

void boost_reset(void)
{
    int i;
    for (i = 0; i < MT9V03X_H; ++i) bias_row[i] = 0;
    bias_hold = 0;
    bias_px = 0.0f;
    ring_seen = 0;
    ring_cooling = 0;
    ring_cool_base = 0.0f;
}

const uint8_t *boost_bias_rows(void) { return bias_row; }

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
                           const uint8_t *right)
{
    boost_frame_t out;
    float gain = STEER_GAIN_NORMAL;
    float outmax = (speed_mode == SPEED_MODE_L) ? SPEED_OUTMAX_L : SPEED_OUTMAX_M;
    float target_speed = SPEED_TARGET_NORMAL;
    uint8_t sw1 = (gpio_get_level(ZONE_SWITCH_PIN) == ZONE_SWITCH_ACTIVE);
    uint8_t sw2 = (gpio_get_level(AGGR_SWITCH_PIN) == AGGR_SWITCH_ACTIVE);
    uint8_t boost_ok;
    uint8_t in_aggr, in_decel, in_zone;
    int i;

    out.switch_on = (uint8_t)(sw1 || sw2);
    out.sw1 = sw1;
    out.sw2 = sw2;
    boost_brake_enable = 0;
    boost_brake_pwm = 0;

    /* 环岛退出后的冷却：R 回到 0 之后，编码器再走一段才允许提速 */
    if (ring_state != 0)
    {
        ring_seen = 1;
    }
    else if (ring_seen)
    {
        ring_seen = 0;
        ring_cooling = 1;
        ring_cool_base = enc_distance;
    }
    if (ring_cooling && (enc_distance - ring_cool_base) >= RING_COOLDOWN_DIST)
    {
        ring_cooling = 0;
    }

    boost_ok = (uint8_t)(!ring_cooling && ring_state == 0);
    in_aggr = (uint8_t)(sw2 && boost_ok &&
                        enc_distance >= AGGR_DIST_MIN && enc_distance <= AGGR_DIST_MAX);
    in_decel = (uint8_t)(sw2 && boost_ok &&
                         enc_distance > AGGR_DIST_MAX &&
                         enc_distance <= AGGR_DIST_MAX + AGGR_DECEL_DIST);
    in_zone = (uint8_t)(sw1 && boost_ok &&
                        enc_distance >= ZONE_DIST_MIN && enc_distance <= ZONE_DIST_MAX);

    out.slow_steer = (uint8_t)(in_aggr || in_zone || in_decel);

    for (i = 0; i < MT9V03X_H; ++i) bias_row[i] = 0;

    if (in_aggr || in_zone)
    {
        if (in_aggr)
        {
            gain = AGGR_GAIN;
            outmax = AGGR_OUTMAX;
            target_speed = AGGR_TARGET_SPEED;
        }
        else
        {
            gain = STEER_GAIN_ZONE;
            outmax = ZONE_OUTMAX;
            target_speed = ZONE_TARGET_SPEED;
        }
    }
    else
    {
        bias_hold = 0;
        bias_px = 0.0f;

        if (in_decel)
        {
            /* 加速区间结束后的减速段：40 → 30 线性回落，并主动反向制动 */
            float t = (enc_distance - AGGR_DIST_MAX) / AGGR_DECEL_DIST;
            float over;
            if (t < 0.0f) t = 0.0f;
            if (t > 1.0f) t = 1.0f;
            target_speed = AGGR_TARGET_SPEED - (AGGR_TARGET_SPEED - SPEED_TARGET_NORMAL) * t;
            outmax = AGGR_DECEL_OUTMAX;
            /* 实际速度明显高于目标时给反向制动，否则只靠目标速度自然回落 */
            over = speed_real - target_speed;
            if (over > BOOST_BRAKE_DEADBAND)
            {
                int brake = (int)(BOOST_BRAKE_K * over);
                if (brake > BOOST_BRAKE_MAX) brake = BOOST_BRAKE_MAX;
                if (brake < BOOST_BRAKE_MIN_PWM) brake = BOOST_BRAKE_MIN_PWM;
                boost_brake_pwm = (int16_t)brake;
                boost_brake_enable = 1;
            }
        }
        else if (cross_seen && speed_mode == SPEED_MODE_H &&
                 !ring_cooling && ring_state == 0 && !reversing)
        {
            outmax = TAIL_OUTMAX;
            target_speed = TAIL_TARGET_SPEED;
            gain = STEER_GAIN_TAIL;
        }
    }

    /* 中线偏移：两个开关下，90000~120000 之间、直道可靠时把中线左移。
     * 用迟滞 + 逐帧渐变，避免阶跃把舵机“打一下”。 */
    {
        uint8_t bias_zone = (uint8_t)((sw1 || sw2) && boost_ok &&
                                      enc_distance >= BIAS_DIST_MIN &&
                                      enc_distance <= BIAS_DIST_MAX);
        if (bias_zone &&
            control_paired >= BIAS_STRAIGHT_PAIRED &&
            control_single <= BIAS_STRAIGHT_SINGLE &&
            ring_state == 0 && cross_path_state == 0)
        {
            if (bias_hold < BIAS_HOLD_MAX) ++bias_hold;
        }
        else if (bias_hold > 0) --bias_hold;

        {
            float bias_target = (bias_hold >= BIAS_HOLD_ON) ? BIAS_PX : 0.0f;
            bias_px += (bias_target - bias_px) * BIAS_SLEW;
        }
        if (bias_px > 0.5f)
        {
            int shift = (int)(bias_px + 0.5f);
            for (i = 60; i <= 110; ++i)
            {
                int l, r, c;
                if (!img_line_valid[i]) continue;
                l = left[i];
                r = right[i];
                if (r <= l + 24) continue;
                c = (int)mid[i] - shift;      /* 左移 */
                if (c < l + 12) c = l + 12;
                if (c > r - 12) c = r - 12;
                mid[i] = (uint8_t)c;
                bias_row[i] = 1;
            }
        }
    }

    /* 环岛状态（R≠0）统一增益，优先级最高 */
    if (ring_state != 0) gain = RING_STEER_GAIN;

    /* 直接下发速度与动力 */
    speed_target = target_speed;
    if (speed_pid_l.outmax != outmax)
    {
        speed_pid_l.outmax = outmax;
        speed_pid_r.outmax = outmax;
    }

    out.steer_gain = gain;
    return out;
}
