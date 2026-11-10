#include "ring_s4.h"
#include <string.h>

/* Trace the white approach from the current near-field driving corridor.
 * Stop at a break: isolated bright patches above the road have no lane span. */
static void approach_spans(const uint8_t *image,int rows,int cols,
                           const uint8_t *mid,const uint8_t *valid,
                           int16_t left[256],int16_t right[256])
{
    int seed=rows-1;
    for(int y=0;y<rows;++y){left[y]=-1;right[y]=-1;}
    if(!mid || !valid)return;
    while(seed>=rows*3/4 && (!valid[seed] || mid[seed]>=cols ||
          !image[seed*cols+mid[seed]]))--seed;
    if(seed<rows*3/4)return;
    int l=mid[seed],r=l;
    for(int y=seed;y>=0;--y){
        const uint8_t *row=image+y*cols;
        int x=(l+r)/2;
        if(!row[x]){
            int d=1;
            while(x-d>=l || x+d<=r){
                if(x-d>=l && row[x-d]){x-=d;break;}
                if(x+d<=r && row[x+d]){x+=d;break;}
                ++d;
            }
            if(!row[x])break;
        }
        l=r=x;
        while(l>0 && row[l-1])--l;
        while(r+1<cols && row[r+1])++r;
        if(r-l<5)break;
        left[y]=(int16_t)l;right[y]=(int16_t)r;
    }
}

static s1_point_t search_upper(const uint8_t *image,int rows,int cols,
                               const s2_result_t *island,s1_point_t previous,
                               s1_point_t *candidate,int tracking,const uint8_t *mid,const uint8_t *valid)
{
    s1_point_t missing={0,0,0},best={0,0,0};
    *candidate=missing;
    if(!image || rows<16 || rows>256 || cols<32 || cols>256)return missing;
    int16_t lane_left[256],lane_right[256];
    approach_spans(image,rows,cols,mid,valid,lane_left,lane_right);
    int limit=rows-1;
    if(island && island->found && island->rows==rows && island->cols==cols){
        for(int y=0;y<rows;++y){
            if(island->edge_valid[y]){limit=y;break;}
        }
    }

    int first_x=6,last_x=cols*3/4,first_y=4,last_y=limit-1;
    if(tracking && previous.valid){
        first_x=previous.x-S2_TRACK_RANGE-4;
        last_x=previous.x+S2_TRACK_RANGE+4;
        first_y=previous.y-S2_TRACK_RANGE-3;
        last_y=previous.y+S2_TRACK_RANGE+3;
        if(first_x<6)first_x=6;
        if(first_y<4)first_y=4;
        if(last_y>=limit)last_y=limit-1;
    }
    int best_distance=rows+cols;
    for(int y=last_y;y>=first_y;--y){
        const uint8_t *row=image+y*cols;
        for(int x=first_x;x<=last_x && x<cols-6;++x){
            /* 粗定位：黑段右端，右侧及下方为白区。这里允许短黑段。 */
            if(row[x] || row[x-1] || !row[x+1] || !image[(y+1)*cols+x])continue;
            if(lane_left[y+1]<0 || x<lane_left[y+1] || x>lane_right[y+1])continue;
            s1_point_t seed={(int16_t)x,(int16_t)y,1};
            if(!candidate->valid)*candidate=seed;

            /* 从候选下方的白区向上扫。限制角点离候选的横纵距离，
             * 远处黑块的转折不能用来确认这个候选。 */
            int ref=y+3;
            if(ref>limit)ref=limit;
            if(!image[ref*cols+x])ref=y+1;
            s1_point_t measured=ring_find_upper(image,rows,cols,ref,x-4,x+4);
            if(!measured.valid || measured.y<y-3 || measured.y>y+3)continue;
            int below=measured.y+1;
            if(lane_left[below]<0 || measured.x<lane_left[below] || measured.x>lane_right[below])continue;
            if(!previous.valid){*candidate=seed;return measured;}
            int dx=measured.x-previous.x,dy=measured.y-previous.y;
            if(tracking && (dx < -S2_TRACK_RANGE || dx>S2_TRACK_RANGE ||
                            dy < -S2_TRACK_RANGE || dy>S2_TRACK_RANGE))continue;
            int distance=(dx<0?-dx:dx)+(dy<0?-dy:dy);
            if(distance<best_distance){
                best_distance=distance;best=measured;*candidate=seed;
                if(distance==0)return best;
            }
        }
    }
    return best;
}

void ring_s4_s5_process(s4_search_t *search,uint8_t *state,
    const uint8_t *image,int rows,int cols,const s2_result_t *island,
    const uint8_t *right,const uint8_t *flags,const uint16_t *width,
    uint8_t *mid,uint8_t *valid)
{
    if(!search || !state)return;
    if(*state!=RING_STATE_4 && *state!=RING_STATE_5){
        memset(search,0,sizeof(*search));
        return;
    }
    if(*state==RING_STATE_4){
        if(rows!=search->rows || cols!=search->cols){
            memset(search,0,sizeof(*search));
            if(rows>0 && rows<=256 && cols>0 && cols<=256){
                search->rows=(uint16_t)rows;
                search->cols=(uint16_t)cols;
            }
        }
        s1_point_t previous=search->upper;
        search->upper=search_upper(image,rows,cols,island,previous,&search->candidate,0,mid,valid);
        if(search->upper.valid){
            int dx=search->upper.x-previous.x,dy=search->upper.y-previous.y;
            if(!previous.valid || dx < -S2_TRACK_RANGE || dx>S2_TRACK_RANGE ||
               dy < -S2_TRACK_RANGE || dy>S2_TRACK_RANGE)search->matches=0;
            if(++search->matches>=S4_UPPER_CONFIRM_FRAMES){
                *state=RING_STATE_5;
                search->last_upper=search->upper;
                search->missing_frames=0;
            }
        }else search->matches=0;
    }else if(image && rows==search->rows && cols==search->cols &&
             rows>=16 && rows<=256 && cols>=32 && cols<=256 && search->last_upper.valid){
        search->upper=search_upper(image,rows,cols,NULL,search->last_upper,&search->candidate,1,mid,valid);
        if(search->upper.valid){
            search->last_upper=search->upper;
            search->missing_frames=0;
        }else ++search->missing_frames;
    }else{
        search->upper.valid=0;
        search->candidate.valid=0;
        search->missing_frames=0;
    }
    if(*state==RING_STATE_5){
        if((search->upper.valid && search->upper.y>=rows-S5_BOTTOM_MARGIN) ||
           search->missing_frames>=S5_MISSING_FRAMES){
            *state=RING_STATE_NORMAL;
            memset(search,0,sizeof(*search));
            return; /* 本帧普通巡线已经生成中线，直接交还控制。 */
        }
        ring_s1_apply(rows,cols,right,flags,width,mid,valid);
    }else{
        ring_s3_exit_apply(image,rows,cols,island,width,mid,valid);
    }
}
