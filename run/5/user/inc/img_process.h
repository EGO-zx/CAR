#ifndef _IMG_PROCESS_H_
#define _IMG_PROCESS_H_
#include "zf_common_headfile.h"

void get_hist_gram(unsigned char* _img,
				   const unsigned short _rows, 
				   const unsigned short _cols, 
				   short* _hist_gram);
unsigned char get_threshold_otsu(const short* _hist_gram);
void binaryzation_process(unsigned char* _img, 
						  const unsigned short _rows, 
						  const unsigned short _cols, 
						  const unsigned int _threshold_value);
// 中线：0无效，1单边补偿，2双边实测，3十字重建，4斑马线桥接；边界：bit0左实测，bit1右实测。
extern uint8_t img_line_valid[256];
extern uint8_t img_edge_valid[256];
void auxiliary_process(uint8_t* _src_pixel_mat, uint8_t _src_rows, uint8_t _src_cols, 
					   unsigned char _threshold_val, 
					   uint8_t* _left_line, uint8_t* _mid_line, uint8_t* _right_line);


// 单帧识别结果；通过状态由cross_process单独管理。
typedef struct {
    uint8_t detected;       // 1：本帧四角及组合验证通过
    uint8_t stage;          // 0无白区，1白区，2已扫角点，3四角齐，4确认
    uint8_t reference_row;
    uint8_t corner_mask;    // bit0上左、bit1上右、bit2下左、bit3下右
    uint8_t corner_x[4], corner_y[4];
} img_cross_info_t;
extern img_cross_info_t img_cross;
void cross_detect(const uint8_t *image, uint8_t rows, uint8_t cols);

extern uint8_t cross_path_state;
extern uint8_t cross_draw_left[256], cross_draw_right[256], cross_draw_flags[256];
void cross_process(const uint8_t *image,uint8_t rows,uint8_t cols,
                   const uint8_t *left,uint8_t *mid,const uint8_t *right,
                   uint8_t control_first,uint8_t control_last);

const uint16_t *img_track_width(void);
// candidate为本帧候选；detected经连续帧确认，短暂丢失时保持。
typedef struct {
    uint8_t candidate, detected, stripes, support_rows;
    uint8_t top, bottom, left, right;
    uint8_t hits, misses;
} img_zebra_info_t;
extern img_zebra_info_t img_zebra;
void zebra_detect(const uint8_t *image, uint8_t rows, uint8_t cols,
                  const uint8_t *left, const uint8_t *right);
// 当前帧有效补线标志；只桥接已确认的斑马线区域。
extern uint8_t zebra_draw_flags[256];
void zebra_process(const uint8_t *image, uint8_t rows, uint8_t cols,
                   const uint8_t *left, uint8_t *mid, const uint8_t *right);

// 主循环置位，10 ms 电机中断执行减速。
extern volatile uint8_t zebra_stop_requested;
extern uint8_t zebra_pass_count;
void zebra_race_update(uint8_t eligible);
float zebra_motor_target(float cruise_target);
/* 复位斑马线计数与停车锁存；复位十字计数与当前通行状态 */
void zebra_reset_counters(void);
void cross_reset_counters(void);
#endif
