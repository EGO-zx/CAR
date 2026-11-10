#ifndef RING_PROCESS_H
#define RING_PROCESS_H
#include "ring_s1.h"
typedef struct { s1_point_t upper,middle,lower,shoulder,candidate; } ring_marks_t;
const ring_marks_t *ring_frame_marks(void);
uint8_t ring_current_state(void);
void track_special_process(const uint8_t *image,uint8_t rows,uint8_t cols,
    const uint8_t *left,uint8_t *mid,const uint8_t *right,
    uint8_t control_first,uint8_t control_last);
#endif
