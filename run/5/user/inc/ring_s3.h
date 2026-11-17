#ifndef RING_S3_H
#define RING_S3_H
#include <stdint.h>
#define S3_CONFIRM_FRAMES 3
typedef struct { uint8_t missing_frames; } s3_entry_t;
void ring_s3_entry_reset(s3_entry_t *entry);
/* 状态1已确认上点。从进入状态2的当帧起，每帧检查上点是否消失。 */
int ring_s3_entry_update(s3_entry_t *entry,uint8_t upper_valid);
#endif
