#ifndef RING_S1_H
#define RING_S1_H
#include <stdint.h>

/* Keil ARM Compiler 6 / C99. 坐标x向右、y向下，默认120x188。 */
#define S1_FIRST_ROW 60
#define S1_LAST_ROW 110
#define S1_REF_ROW_A 80
#define S1_REF_ROW_B 100
#define S1_LOST_ROWS 8
#define S1_RIGHT_STEP 3
#define S1_LINE_RESIDUAL 4
#define S1_UPPER_X_RANGE 12
#define S1_ARC_WINDOW 4       /* 极值平台两侧各检查4行 */
#define S1_ARC_RISE 2         /* 两侧至少回退2像素 */
#define S1_ARC_MAX_STEP 6     /* 超过此值当作断边，不作趋势反转 */
#define S1_MIN_WIDTH 6
#define S2_CONFIRM_FRAMES 2  /* 连续满足进入条件的帧数 */
#define S2_TRACK_RANGE 16    /* 每帧拐点关联范围 */
#define S2_BOTTOM_MARGIN 12 /* 中点进入画面底部12行时交接上点跟踪 */

typedef struct { int16_t x,y; uint8_t valid; } s1_point_t;
typedef struct {
    uint8_t preidentified, confirmed, lower_missing;
    s1_point_t lower,middle,upper;
} s1_result_t;

typedef struct {
    uint8_t state; /* 0未进入，1状态1，2状态2 */
    uint16_t rows,cols;
    uint8_t upper_tracking; /* 中点接近底边且上点有效后才置1 */
    s1_point_t last_lower,last_upper;
} s1_state_t;

void ring_s1_state_reset(s1_state_t *state);
/* 仅用已confirmed的状态1识别结果建立跟踪基准；成功返回1。
 * 同一环岛只调用一次，重新开始前reset。 */
int ring_s1_enter(s1_state_t *state,int rows,int cols,const s1_result_t *entry);
/* 每帧调用一次；中点接近底边前按中点找上点，之后独立跟踪上点。
 * 不依赖初次预识别门槛，避免中点消失后上点检测被提前跳过。
 * 状态2锁存到reset，并继续跟踪上点；不实现状态2补线。 */
uint8_t ring_s1_update(s1_state_t *state,const uint8_t *image,
                        int rows,int cols,s1_result_t *observation);

/* 输入0黑/非0白；edge_valid bit0左实测、bit1右实测（不可传补线标志）。
 * 单帧检测，不修改输入、不自动切换状态或控制电机。
 * 入口confirmed=预识别通过且上、中、下点有效，满足上点y<中点y<下点y；
 * 中点只有在本帧有效上点下方才有效；lower_missing仅表示本帧没找到下点，
 * 不等同于已经通过，后续状态转换由调用者另外实现。
 */
void ring_s1_detect(const uint8_t *image,int rows,int cols,
                    const uint8_t *left,const uint8_t *right,
                    const uint8_t *edge_valid,s1_result_t *out);
/* 在指定白色参考行上方查找状态1定义的上拐点。 */
s1_point_t ring_find_upper(const uint8_t *image,int rows,int cols,
                           int reference_row,int xmin,int xmax);
/* 仅由调用者在环岛状态1调用，不以本帧三点齐全为条件。
 * track_width直接传入run/3的逐行工作宽度，长度至少rows；0表示无宽度。
 * 右实测边界-width/2，按run/3方式限幅到图像内；成功valid=4，否则0。
 * mid仅在有效行写入；本模块不测量、不保存、不更新宽度。 */
void ring_s1_apply(int rows,int cols,
                   const uint8_t *right,const uint8_t *edge_valid,
                   const uint16_t *track_width,uint8_t *mid,uint8_t *valid);
#endif
