#include "ring_process.h"
#include "ring_s3_exit.h"
#include "main.h" // 所有引脚信息更改在main.h里改宏

// *************************** 例程硬件连接说明 ***************************
/*

            连接好电机、主板、驱动板、摄像头、编码器
            (请确保全部正确连接后在使用此例程，所有外设均会使用)
*/
// *************************** 例程测试说明 ***************************
/*
            本例程包含功能：
            1.初始化摄像头并采集图像进行大津法二值化
            2.初始化屏幕并显示关键信息
            3.根据偏差计算最基本的舵机打角
            4.运用最基本pid参数和pid算法使电机旋转 (pid参数在pid.c中)


            注意！注意！注意！注意！
            注意！注意！注意！注意！
            注意！注意！注意！注意！

            1.请仔细阅读main.h的所有注释，确保所有宏都和自己的硬件连接对应
            2.若屏幕显示二值化后的图像有缺陷，可能不能正常走直线，
            可以自行更改二值化算法或找光环境良好且均匀的环境测试例程（例程不提供更多的二值化算法）

*/

// **************************** 代码区域 ****************************

// 舵机动作角度中值
const float servo_motor_duty_middle = (SERVO_MOTOR_R_MAX + SERVO_MOTOR_L_MAX) / 2.f; 
//float dir_kp = 0.5;      // 方向比例，原来的0.5搬到这里
//float dir_kd = 0.8;      // 方向微分，新增的
//float last_mid_err = 0;  // 上一次的偏差，Kd要用

void Init()
{

    // 初始化flash, 储存参数. 一个扇区有8页, 一页可以储存4096字节, 一个参数占4个字节, 因此一页最多只能存64个参数
    flash_init();

    // tft180_set_dir(TFT180_CROSSWISE);                                           // 需要先横屏 不然显示不下
    tft180_init();

    // mt9v03x摄像头初始化
    while (1)
    {
        if (mt9v03x_init())
            tft180_show_string(0, 16, "mt9v03x reinit.");
        else
            break;
        system_delay_ms(1000);
    }
    tft180_show_string(0, 16, "init success.");

    // 编码器初始化
    encoder_quad_init(ENCODER_1, ENCODER_1_A, ENCODER_1_B); // 初始化编码器模块与引脚 正交解码编码器模式
    encoder_quad_init(ENCODER_2, ENCODER_2_A, ENCODER_2_B); // 初始化编码器模块与引脚 正交解码编码器模式

    // 电机初始化
    pwm_init(MOTOR1_PWM, 17000, 0); // PWM 通道初始化频率 17KHz 占空比初始为 0
    gpio_init(MOTOR1_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL); // GPIO 初始化为输出 默认上拉输出高
    pwm_init(MOTOR2_PWM, 17000, 0); // PWM 通道初始化频率 17KHz 占空比初始为 0
    gpio_init(MOTOR2_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL); // GPIO 初始化为输出 默认上拉输出高

    // 舵机
    pwm_init(SERVO_MOTOR_PWM, SERVO_MOTOR_FREQ, 0);

    // PID初始化
    My_Pid_Init();

    // 定时器0初始化，10ms可调
    pit_ms_init(PIT_CH0, 10);
}

uint8_t image[MT9V03X_H][MT9V03X_W];
short hist_gram[256];
uint8_t left_line[MT9V03X_H];
uint8_t mid_line[MT9V03X_H];
uint8_t right_line[MT9V03X_H];

// 加权前瞻：以90行为中心，取80~100行；行号越小，看得越远。
// 调前瞻只需修改中心行；半径控制参与平均的范围。
#define STEER_LOOKAHEAD_ROW 90
#define STEER_WINDOW_RADIUS 10

#if STEER_WINDOW_RADIUS < 0 || STEER_LOOKAHEAD_ROW - STEER_WINDOW_RADIUS < 0 || STEER_LOOKAHEAD_ROW + STEER_WINDOW_RADIUS >= MT9V03X_H
#error "Steering window must be inside the camera image"
#endif

// 三角权重：中心行权重11，两端权重1；只累计有效行并重新归一化。
// 双边实测和单边补线均参与；全段无效时返回0，保持上一舵机指令。
static int control_rows, control_error;
static int get_weighted_mid_error(float *error)
{
    control_rows = 0;
    control_error = 0;
    int weighted_error = 0;
    int weight_sum = 0;
    for (int row = STEER_LOOKAHEAD_ROW - STEER_WINDOW_RADIUS;
         row <= STEER_LOOKAHEAD_ROW + STEER_WINDOW_RADIUS; ++row)
    {
        int distance = row - STEER_LOOKAHEAD_ROW;
        int weight;
        if (!img_line_valid[row]) continue;
        if (distance < 0) distance = -distance;
        weight = STEER_WINDOW_RADIUS + 1 - distance;
        ++control_rows;
        weighted_error += ((int)(MT9V03X_W / 2) - mid_line[row]) * weight;
        weight_sum += weight;
    }
    if (weight_sum == 0) return 0;
    *error = (float)weighted_error / weight_sum;
    control_error = (int)*error;
    return 1;
}

#define IMAGE_DISPLAY_W 125
#define IMAGE_DISPLAY_H 100
// 在屏幕分辨率上合成标线，避免单像素中线被缩放采样跳过。
static uint16_t display_image[IMAGE_DISPLAY_H][IMAGE_DISPLAY_W];

/* Five-bit rows, centered on the measured camera coordinate. */
static void draw_ring_mark(s1_point_t point,const uint8_t shape[5],uint16_t color)
{
    if(!point.valid)return;
    int cx=point.x*IMAGE_DISPLAY_W/MT9V03X_W;
    int cy=point.y*IMAGE_DISPLAY_H/MT9V03X_H;
    for(int dy=0;dy<5;++dy)for(int dx=0;dx<5;++dx){
        int x=cx+dx-2,y=cy+dy-2;
        if((shape[dy] & (16u>>dx)) && x>=0 && x<IMAGE_DISPLAY_W && y>=0 && y<IMAGE_DISPLAY_H)
            display_image[y][x]=color;
    }
}

static void display_track_image(void)
{
    int valid_rows = 0, single_rows = 0;
    for (int row = 0; row < MT9V03X_H; ++row)
    {
        if (img_line_valid[row]) ++valid_rows;
        if (img_line_valid[row] == 1) ++single_rows;
    }
    for (int y = 0; y < IMAGE_DISPLAY_H; ++y)
    {
        int row = y * MT9V03X_H / IMAGE_DISPLAY_H;
        for (int x = 0; x < IMAGE_DISPLAY_W; ++x)
            display_image[y][x] = image[row][x * MT9V03X_W / IMAGE_DISPLAY_W] ?
                                  RGB565_WHITE : RGB565_BLACK;
        if (img_edge_valid[row] & 1)
            display_image[y][left_line[row] * IMAGE_DISPLAY_W / MT9V03X_W] = RGB565_BLUE;
        if (img_edge_valid[row] & 2)
            display_image[y][right_line[row] * IMAGE_DISPLAY_W / MT9V03X_W] = RGB565_BLUE;
        if (cross_draw_flags[row] & 1)
            display_image[y][cross_draw_left[row] * IMAGE_DISPLAY_W / MT9V03X_W] = RGB565_GREEN;
        if (cross_draw_flags[row] & 2)
            display_image[y][cross_draw_right[row] * IMAGE_DISPLAY_W / MT9V03X_W] = RGB565_GREEN;
        if (img_line_valid[row])
        {
            int center = mid_line[row] * IMAGE_DISPLAY_W / MT9V03X_W;
            uint16_t color = img_line_valid[row] >= 2 ? RGB565_RED : RGB565_YELLOW;
            for (int x = center - 1; x <= center + 1; ++x)
                if (x >= 0 && x < IMAGE_DISPLAY_W) display_image[y][x] = color;
        }
    }
    // 仅显示本帧角点：候选黄色，四角确认后绿色。
    for (int i = 0; i < 4; ++i)
    {
        if (!(img_cross.corner_mask & (1 << i))) continue;
        int cx = img_cross.corner_x[i] * IMAGE_DISPLAY_W / MT9V03X_W;
        int cy = img_cross.corner_y[i] * IMAGE_DISPLAY_H / MT9V03X_H;
        for (int y = cy - 2; y <= cy + 2; ++y)
            for (int x = cx - 2; x <= cx + 2; ++x)
                if (x >= 0 && x < IMAGE_DISPLAY_W && y >= 0 && y < IMAGE_DISPLAY_H)
                    display_image[y][x] = img_cross.detected ? RGB565_GREEN : RGB565_YELLOW;
    }
    const ring_marks_t *points=ring_frame_marks();
    static const uint8_t up[]={4,14,31,0,0},down[]={0,0,31,14,4};
    static const uint8_t diamond[]={4,10,17,10,4},plus[]={4,4,31,4,4};
    static const uint8_t box[]={31,17,17,17,31};
    /* Display the virtual boundary only; never paint into the camera frame. */
    if(points->anchor.valid && points->shoulder.valid &&
       points->shoulder.y>points->anchor.y){
        for(int row=points->anchor.y;row<=points->shoulder.y;++row){
            int x=points->anchor.x+(points->shoulder.x-points->anchor.x)*
                  (row-points->anchor.y)/(points->shoulder.y-points->anchor.y);
            if(row>=0 && row<MT9V03X_H && x>=0 && x<MT9V03X_W)
                display_image[row*IMAGE_DISPLAY_H/MT9V03X_H][x*IMAGE_DISPLAY_W/MT9V03X_W]=RGB565_GREEN;
        }
    }
    draw_ring_mark(points->candidate,box,RGB565_YELLOW);
    draw_ring_mark(points->anchor,box,RGB565_GREEN);
    draw_ring_mark(points->shoulder,plus,RGB565_BLUE);
    draw_ring_mark(points->middle,diamond,RGB565_MAGENTA);
    draw_ring_mark(points->lower,down,RGB565_GREEN);
    draw_ring_mark(points->upper,up,RGB565_CYAN);
    tft180_show_string(96, 128, "R:");
    tft180_show_int(112, 128, ring_current_state(), 1);
    if(ring_current_state()){
        tft180_show_string(0,112,"C:");
        tft180_show_int(16,112,control_rows,3);
        tft180_show_string(40,112," E:");
        tft180_show_int(64,112,control_error,4);
        tft180_show_string(96,112,"   ");
    }else{
    tft180_show_string(0, 112, "X:");
    tft180_show_int(16, 112, img_cross.detected, 1);
    tft180_show_string(40, 112, "P:");
    tft180_show_int(56, 112, cross_path_state, 1);
    tft180_show_string(80, 112, "M:");
    tft180_show_int(96, 112, img_cross.corner_mask, 2);
    }
    // 一次发送合成画面，避免先擦掉旧线、后叠加新线造成额外闪烁。
    tft180_show_rgb565_image(0, 0, (const uint16 *)display_image,
                            IMAGE_DISPLAY_W, IMAGE_DISPLAY_H,
                            IMAGE_DISPLAY_W, IMAGE_DISPLAY_H, 0);
    // V: 有效中线行数，S: 其中单边补偿行数（总计120行）。
    tft180_show_string(0, 128, "V:");
    tft180_show_int(16, 128, valid_rows, 3);
    tft180_show_string(56, 128, "S:");
    tft180_show_int(72, 128, single_rows, 3);
}

#define DISPLAY_FRAME_INTERVAL 4u

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M); // 不可删除
    debug_init();                  // 调试端口初始化
    system_delay_ms(300);          // 等待主板其他外设上电完成

    Init();
    interrupt_global_enable(0);

    unsigned display_due = DISPLAY_FRAME_INTERVAL;
    while (1)
    {
        //  mt9v03x摄像头
        if (mt9v03x_finish_flag)
        {
            /* Claim the ready buffer and its notification together. */
            uint32_t saved_mask = __get_PRIMASK();
            __disable_irq();
            const void *frame = mt9v03x_image;
            mt9v03x_finish_flag = 0;
            __set_PRIMASK(saved_mask);
            memcpy(image, frame, sizeof(uint8_t) * MT9V03X_H * MT9V03X_W);
            /* A completed buffer may have been returned to CSI during the copy. */
            if (mt9v03x_finish_flag) continue;
            // 获取直方图
            get_hist_gram((uint8_t *)image, MT9V03X_H, MT9V03X_W, hist_gram);
            unsigned char threshold = get_threshold_otsu(hist_gram);
            // 二值化处理
            binaryzation_process((uint8_t *)image, MT9V03X_H, MT9V03X_W, threshold);
            // 边界线寻找
            auxiliary_process((uint8_t *)image, MT9V03X_H, MT9V03X_W, threshold, left_line, mid_line, right_line);

            track_special_process((const uint8_t *)image, MT9V03X_H, MT9V03X_W,
                          left_line, mid_line, right_line,
                          STEER_LOOKAHEAD_ROW-STEER_WINDOW_RADIUS,
                          STEER_LOOKAHEAD_ROW+STEER_WINDOW_RADIUS);

            // 使用一段有效中线的加权偏差；全段无效时保持上一指令。
            float mid_err;
            /* State 4 also steers from its valid island-based midline. */
            if (get_weighted_mid_error(&mid_err))
            {
                //舵机根据误差乘以系数打角
                float tmp_duty = servo_motor_duty_middle - mid_err * 0.5 * SERVO_DIR;
                // 限幅
                tmp_duty = MAX(tmp_duty,MIN(SERVO_MOTOR_L_MAX,SERVO_MOTOR_R_MAX));
                tmp_duty = MIN(tmp_duty,MAX(SERVO_MOTOR_L_MAX,SERVO_MOTOR_R_MAX));
                pwm_set_duty(SERVO_MOTOR_PWM, SERVO_MOTOR_DUTY(tmp_duty));
            }
						//float mid_err = (MT9V03X_W/2) - mid_line[MT9V03X_H-MT9V03X_H/4];
						// 方向PD控制
						//float dir_out = dir_kp * mid_err + dir_kd * (mid_err - last_mid_err);
						//last_mid_err = mid_err;

						//float tmp_duty = servo_motor_duty_middle - dir_out * SERVO_DIR;
						
						
            if (display_due < DISPLAY_FRAME_INTERVAL) ++display_due;
            if (display_due == DISPLAY_FRAME_INTERVAL && !mt9v03x_finish_flag)
            {
                display_track_image();
                tft180_show_float(0, 144, speed_pwm, 4, 2);
                display_due = 0;
            }
        }
    }
}
