#ifndef RING_S3_EXIT_H
#define RING_S3_EXIT_H
#include <stdint.h>
#include "ring_s2.h"
#define RING_STATE_4 4u
#define S3_EXIT_CONFIRM_FRAMES 3
#define S3_T_ARM_MIN 8
#define S3_T_STEM_ROWS 8
#define S3_T_BAR_ROWS 3
#define S3_EXIT_CORNER_RANGE 12
#define S3_EXIT_TRACK_STEP 12
#define S3_EXIT_TURN_WINDOW 6
#define S3_EXIT_TURN_RETREAT 4
#define S3_EXIT_TURN_STEP_MAX 8
#define S3_EXIT_TURN_SUPPORT 4
typedef struct {
    uint8_t preidentified,corner_valid,confirmed;
    int16_t corner_x,corner_y,bar_row;
} s3_exit_result_t;
typedef struct {
    uint8_t frames;
    uint16_t rows,cols;
    int16_t last_x,last_y;
} s3_exit_tracker_t;
void ring_s3_exit_reset(s3_exit_tracker_t *tracker);
/* 仅在状态3运行；seed_x/seed_row为当前帧近处已确认的白色行驶通路点。
 * 沿相连白区优先寻找右边界斜向转折；标准T形作为兼容分支。
 * 斜向转折不依赖双侧横臂，bar_row=-1表示没有T横臂；0黑/非0白。
 * 返回1表示连续确认，调用者将当前状态设为RING_STATE_4。
 * 输入原始摄像头二值帧，不能包含LCD文字/叠加标记。 */
int ring_s3_exit_update(s3_exit_tracker_t *tracker,uint8_t current_state,
    const uint8_t *image,int rows,int cols,int seed_x,int seed_row,
    s3_exit_result_t *result);
/* 进入状态4后每帧调用，包括从状态3切换的当帧。
 * 使用本帧ring_s2_detect确认的左侧黑色内岛右轮廓+宽度/2。
 * 右路肩拐点仅触发切换，不参与补线；不能传普通left数组替代内岛。
 * 内岛跟踪必须在状态3期间持续更新；失去内岛则中线无效。
 * 有效标记沿用5（内岛单边重建），不重新定义内岛识别算法。 */
void ring_s3_exit_apply(const uint8_t *image,int rows,int cols,
    const s2_result_t *island,const uint16_t *track_width,
    uint8_t *mid,uint8_t *valid);
#endif
