#include "ring_s3.h"

void ring_s3_entry_reset(s3_entry_t *entry)
{
    entry->missing_frames=0;
}

int ring_s3_entry_update(s3_entry_t *entry,uint8_t upper_valid)
{
    if(upper_valid){
        entry->missing_frames=0;
    }else if(entry->missing_frames<S3_CONFIRM_FRAMES){
        ++entry->missing_frames;
    }
    return entry->missing_frames==S3_CONFIRM_FRAMES;
}
