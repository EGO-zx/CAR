#ifndef RING_S4_H
#define RING_S4_H
#include "ring_s1.h"
#include "ring_s3_exit.h"
#define RING_STATE_NORMAL 0u
#define S5_MISSING_FRAMES 3
#define S5_BOTTOM_MARGIN 12
#define RING_STATE_5 5u
#define S4_UPPER_CONFIRM_FRAMES 2

typedef struct {
    s1_point_t candidate; /* 本帧粗定位，只用于限定搜索范围 */
    s1_point_t upper; /* 本帧重新检出的上拐点 */
    s1_point_t last_upper; /* 状态5短时漏检期间保留的搜索中心 */
    uint8_t matches,missing_frames;
    uint16_t rows,cols;
} s4_search_t;

/* 每帧在状态更新后调用。search静态零初始化，新环岛清零。
 * 状态4搜原上拐点并沿内岛补线，确认当帧起进入状态5沿右边补线。
 * 先执行普通巡线。5→0退出当帧保留调用前的mid和valid。
 * island为当前帧结果。其他状态只清除搜索记录，不写中线。 */
void ring_s4_s5_process(s4_search_t *search,uint8_t *state,
    const uint8_t *image,int rows,int cols,const s2_result_t *island,
    const uint8_t *right,const uint8_t *edge_valid,const uint16_t *track_width,
    uint8_t *mid,uint8_t *valid);
#endif
