#ifndef RING_S2_H
#define RING_S2_H
#include <stdint.h>
#define S2_MASK_BYTES 8192
#define S2_SPAN_CAPACITY 512
#define S2_ISLAND_MIN_AREA 32
#define S2_ARC_MIN_ROWS 8
#define S2_ARC_MAX_STEP 6
#define S2_MAX_LOST_FRAMES 3

typedef struct { uint16_t y,left,right; } s2_span_t;
/* 约28KB，建议static分配，禁止放在小任务栈中。 */
typedef struct {
    uint16_t rows,cols;
    uint8_t seeded,tracking,lost;
    uint16_t seed_x,seed_y;
    uint32_t previous_area;
    uint8_t previous[S2_MASK_BYTES],visited[S2_MASK_BYTES];
    uint8_t component[S2_MASK_BYTES];
    s2_span_t stack[S2_SPAN_CAPACITY];
} s2_tracker_t;
typedef struct {
    uint8_t found,edge[256],edge_valid[256];
    uint16_t rows,cols;
    uint32_t area;
} s2_result_t;
void ring_s2_reset(s2_tracker_t *tracker);
/* 中点临近底边时，用同一帧状态1已确认的内岛中点(x,y)建立种子。
 * seed后必须立刻用该帧图像detect，随后每帧detect，包括切换前的状态1。
 * 初始种子必须在黑色内岛内，不自动改选附近黑块。 */
int ring_s2_seed(s2_tracker_t *tracker,int rows,int cols,int x,int y);
/* 输入0黑/非0白。四连通黑区识别，允许贴左/底，不接受触顶/右背景。
 * 后续以黑区重叠关联同一内岛；丢失不输出旧边界，连续3帧丢失需重新seed。
 * 返回1仅表示当前帧确认内岛及连续轮廓；不修改状态1和原始搜边数据。 */
int ring_s2_detect(s2_tracker_t *tracker,const uint8_t *image,
                    int rows,int cols,s2_result_t *result);
/* 内岛单边补线，供状态2和退出状态3后的阶段复用。
 * 使用本帧detect结果，不能再直接传普通left数组。
 * 内岛右轮廓+run/3宽度/2；整段必须为白色通路且在画面内。
 * 成功valid=5，否则0，不限幅出画坐标、不换用其他边界。 */
void ring_s2_apply(const uint8_t *image,int rows,int cols,
                    const s2_result_t *result,const uint16_t *track_width,
                    uint8_t *mid,uint8_t *valid);
#endif
