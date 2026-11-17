#ifndef RING_S4_H
#define RING_S4_H
#include "ring_s1.h"
#include "ring_s3_exit.h"
#define RING_STATE_NORMAL 0u
#define S5_MISSING_FRAMES 3
#define S5_BOTTOM_MARGIN 12
#define RING_STATE_5 5u
#define S4_UPPER_CONFIRM_FRAMES 2
#define S4_ANCHOR_X_PERCENT 10 /* 左侧取点列，占图宽百分比 */
#define S4_BRIDGE_MIN_HEIGHT 6

typedef struct {
    s1_point_t anchor; /* 本帧拉线左端，非上拐点候选 */
    s1_point_t candidate; /* 本帧粗定位，只用于限定搜索范围 */
    s1_point_t upper; /* 本帧重新检出的上拐点 */
    s1_point_t last_upper; /* 状态5短时漏检期间保留的搜索中心 */
    uint8_t matches,missing_frames;
    uint16_t rows,cols;
} s4_search_t;

/* 每帧在状态更新后调用。search静态零初始化，新环岛清零。
 * 状态4从左侧固定列实测边界拉线至右肩，作为虚拟右边界。
 * 确认上拐点当帧起进入状态5沿右边补线。
 * 先执行普通巡线。5→0退出当帧保留调用前的mid和valid。
 * island为当前帧结果。其他状态只清除搜索记录，不写中线。 */
void ring_s4_s5_process(s4_search_t *search,uint8_t *state,
    const uint8_t *image,int rows,int cols,const s2_result_t *island,s1_point_t shoulder,
    const uint8_t *right,const uint8_t *edge_valid,const uint16_t *track_width,
    uint8_t *mid,uint8_t *valid);
void ring_s4_bridge_apply(const uint8_t *image,int rows,int cols,s1_point_t shoulder,
    const uint8_t *right,const uint8_t *flags,const uint16_t *width,
    uint8_t *mid,uint8_t *valid,s1_point_t *anchor);
#endif
