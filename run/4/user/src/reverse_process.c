#include "reverse_process.h"
#include "img_process.h"
#include "isr.h"
#include "pid.h"
#include "main.h"

/* main.c 里定义的中线数组 */
extern uint8_t mid_line[MT9V03X_H];
/* main.c 里定义的舵机中值 */
extern const float servo_motor_duty_middle;

volatile uint8_t reverse_drive_enable = 0;
volatile uint8_t reverse_dir_forward = 0;   /* 1 = 正向（刹车用），0 = 反向（倒车） */
volatile int16_t reverse_pwm = 0;

static uint8_t state = 0;
static uint8_t settle_frames = 0;
static uint8_t brake_frames = 0;
static float   base_distance = 0.0f;
static float   travelled = 0.0f;

uint8_t reverse_state(void) { return state; }
float   reverse_distance(void) { return travelled; }

void reverse_reset(void)
{
    state = 0;
    settle_frames = 0;
    brake_frames = 0;
    base_distance = 0.0f;
    travelled = 0.0f;
    reverse_drive_enable = 0;
    reverse_dir_forward = 0;
    reverse_pwm = 0;
}

/* 倒车时保持直线：用控制窗口的中线偏差修正方向。
 * 倒车时车头朝向相反，舵机修正方向与前进时相反，所以是 +。 */
static void reverse_steer(void)
{
    int rows = 0;
    int sum = 0;
    for (int row = 75; row <= 95; ++row)
    {
        if (!img_line_valid[row]) continue;
        sum += (int)(MT9V03X_W / 2) - mid_line[row];
        ++rows;
    }
    if (rows == 0)
    {
        pwm_set_duty(SERVO_MOTOR_PWM, SERVO_MOTOR_DUTY(servo_motor_duty_middle));
        return;
    }
    float err = (float)sum / (float)rows;
    float duty = servo_motor_duty_middle + err * REVERSE_STEER_GAIN * SERVO_DIR;
    duty = MAX(duty, MIN(SERVO_MOTOR_L_MAX, SERVO_MOTOR_R_MAX));
    duty = MIN(duty, MAX(SERVO_MOTOR_L_MAX, SERVO_MOTOR_R_MAX));
    pwm_set_duty(SERVO_MOTOR_PWM, SERVO_MOTOR_DUTY(duty));
}

void reverse_update(void)
{
    if (state == 0)
    {
        /* 等待斑马线停车请求 */
        if (zebra_stop_requested)
        {
            state = 1;
            settle_frames = 0;
        }
        return;
    }

    if (state == 1)
    {
        /* 等车真正停稳再开始倒，避免带着惯性倒车跑偏 */
        float v = speed_real;
        if (v < 0.0f) v = -v;
        if (v <= 2.0f) ++settle_frames;
        else settle_frames = 0;
        reverse_steer();
        if (settle_frames >= REVERSE_SETTLE_FRAMES)
        {
            base_distance = enc_distance;
            travelled = 0.0f;
            reverse_pwm = REVERSE_PWM_DUTY;
            reverse_dir_forward = 0;
            reverse_drive_enable = 1;
            state = 2;
        }
        return;
    }

    if (state == 2)
    {
        travelled = enc_distance - base_distance;
        if (travelled >= REVERSE_DISTANCE_COUNTS)
        {
            /* 走够倒车距离，切到刹车阶段：正向加电压，强度与第一次停车一致 */
            brake_frames = 0;
            state = 3;
            return;
        }
        reverse_steer();
        return;
    }

    if (state == 3)
    {
        /* 刹车：按当前转速成比例给正向电压，与斑马线停车同一套强度 */
        float v = speed_real;
        if (v < 0.0f) v = -v;
        int brake = (int)(REVERSE_BRAKE_K * v);
        if (brake > REVERSE_BRAKE_MAX) brake = REVERSE_BRAKE_MAX;
        if (brake < REVERSE_BRAKE_MIN_PWM) brake = REVERSE_BRAKE_MIN_PWM;
        reverse_dir_forward = 1;
        reverse_pwm = (int16_t)brake;
        reverse_drive_enable = 1;
        ++brake_frames;
        if (v <= 1.0f || brake_frames >= REVERSE_BRAKE_FRAMES)
        {
            reverse_drive_enable = 0;
            reverse_pwm = 0;
            pwm_set_duty(MOTOR1_PWM, 0);
            pwm_set_duty(MOTOR2_PWM, 0);
            pwm_set_duty(SERVO_MOTOR_PWM, SERVO_MOTOR_DUTY(servo_motor_duty_middle));
            state = 4;
        }
        return;
    }

    /* state == 4：倒车完成，保持停止 */
    reverse_drive_enable = 0;
}
