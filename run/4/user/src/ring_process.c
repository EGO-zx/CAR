#include "ring_process.h"
#include "img_process.h"
#include "ring_s3.h"
#include "ring_s4.h"
#include <string.h>

static ring_marks_t marks;
const ring_marks_t *ring_frame_marks(void) { return &marks; }
static s1_state_t ring;
static s2_tracker_t island_tracker;
static s2_result_t island;
static s3_entry_t entry3;
static s3_exit_tracker_t exit3;
static s4_search_t search;
uint8_t ring_current_state(void) { return ring.state; }

/* Average four consecutive fully measured straight frames. Zero means
 * that a row did not have two measured edges throughout calibration. */
static uint16_t fixed_width[256];
static uint16_t width_sum[256];
static uint8_t straight_samples;

/* State 1 calibration stores twice the half-width for ring_s1_apply. */
static uint16_t s1_bridge_width[256];

static int bridge_half_at(int y,int a,int b,const uint16_t *samples,int cols)
{
    int half=samples[a]/2;
    if(a!=b)half+=((int)samples[b]/2-half)*(y-a)/(b-a);
    if(half<S1_MIN_WIDTH/2)half=S1_MIN_WIDTH/2;
    if(half>=cols)half=cols-1;
    return half;
}

static void s1_bridge_apply(int rows,int cols,const s1_result_t *o,
    const uint8_t *right,uint8_t *mid)
{
    uint16_t samples[256]={0};
    int first=-1,second=-1,previous=-1,last=-1;
    if(o->middle.valid && o->lower.valid && o->middle.y>=0 &&
       o->lower.y<rows && o->middle.y<o->lower.y &&
       o->middle.x>=0 && o->middle.x<cols && o->lower.x>=0 && o->lower.x<cols){
        for(int y=o->middle.y;y<=o->lower.y;++y){
            int l=o->middle.x+(o->lower.x-o->middle.x)*(y-o->middle.y)/
                (o->lower.y-o->middle.y);
            int r=right[y];
            cross_draw_left[y]=(uint8_t)l;
            cross_draw_flags[y]=1;
            if(!(img_edge_valid[y]&2) || r>=cols || r-l<S1_MIN_WIDTH)continue;
            samples[y]=(uint16_t)(2*(r-(l+r)/2));
            if(first<0)first=y;
            else if(second<0)second=y;
            previous=last;
            last=y;
        }
    }
    if(first>=0){
        /* Interpolate holes and extend the nearest measured slope at each end. */
        int a=first,b=second<0?first:second;
        for(int y=0;y<rows;++y){
            while(y>b && b<last){
                a=b;
                do{++b;}while(!samples[b]);
            }
            if(y>last && previous>=0){a=previous;b=last;}
            s1_bridge_width[y]=samples[y]?samples[y]:
                (uint16_t)(2*bridge_half_at(y,a,b,samples,cols));
        }
    }
    ring_s1_apply(rows,cols,right,img_edge_valid,s1_bridge_width,mid,img_line_valid);
    /* A bridge row with rejected geometry must not steer from that row. */
    for(int y=0;y<rows;++y)
        if(cross_draw_flags[y] && !samples[y])img_line_valid[y]=0;
}

static void measure_straight(int rows,const uint8_t *left,const uint8_t *right)
{
    if(straight_samples>=4 || rows<=110)return;
    for(int y=60;y<=110;++y){
        int l=left[60]+((int)left[110]-left[60])*(y-60)/50;
        int r=right[60]+((int)right[110]-right[60])*(y-60)/50;
        if(img_edge_valid[y]!=3 || right[y]-left[y]<6 ||
           left[y]<l-3 || left[y]>l+3 || right[y]<r-3 || right[y]>r+3){
            straight_samples=0;
            return;
        }
    }
    for(int y=0;y<rows;++y){
        int width=(int)right[y]-left[y];
        if(img_edge_valid[y]!=3 || width<6)width_sum[y]=0;
        else if(!straight_samples)width_sum[y]=width;
        else if(width_sum[y])width_sum[y]+=width;
    }
    if(++straight_samples==4)
        for(int y=0;y<rows;++y)fixed_width[y]=(width_sum[y]+2)/4;
}

/* Left opening ROI: left eighth of the image, bottom quarter. */
static uint8_t opening_seen,closed_frames;

static int opening_closed(const uint8_t *image,int rows,int cols)
{
    int white_run=0;
    int occupied=island.found;
    for(int y=3*rows/4;y<rows;++y){
        white_run=0;
        for(int x=0;x<cols/8;++x){
            int pos=y*cols+x;
            if(image[pos]){
                if(++white_run>=3)opening_seen=1;
                occupied=0;
            }else white_run=0;
            if(!(island_tracker.previous[pos>>3] & (1u<<(pos&7))))occupied=0;
        }
    }
    if(!opening_seen || !occupied){closed_frames=0;return 0;}
    return ++closed_frames>=S2_CONFIRM_FRAMES;
}

static void finish_ring(void)
{
    opening_seen=closed_frames=0;
    ring_s1_state_reset(&ring);
    ring_s2_reset(&island_tracker);
    ring_s3_entry_reset(&entry3);
    ring_s3_exit_reset(&exit3);
    memset(&search,0,sizeof(search));
    memset(s1_bridge_width,0,sizeof(s1_bridge_width));
}

/* Re-seed from the measured left shoulder below a confirmed exit corner.
 * The component detector must still confirm a bounded, curved inner island. */
static void reacquire_at_exit(const uint8_t *image,int rows,int cols,
    const uint8_t *left,const uint8_t *right,const s3_exit_result_t *junction)
{
    if(island.found || !junction->confirmed)return;
    for(int y=junction->corner_y+2;y<rows;++y){
        int x=left[y],r=right[y];
        if((img_edge_valid[y]&3)!=3 || x<2 || r>=cols || r-x<12 ||
           r<junction->corner_x-16 || r>junction->corner_x+16)continue;
        if(image[y*cols+x] || image[y*cols+x-1] || !image[y*cols+x+1])continue;
        /* Only the legacy T branch supplies a horizontal arm. */
        if(junction->bar_row>=0 && (junction->bar_row>=rows || x<S3_T_ARM_MIN ||
           !image[junction->bar_row*cols+x-S3_T_ARM_MIN]))continue;
        ring_s2_seed(&island_tracker,rows,cols,x,y);
        if(!ring_s2_detect(&island_tracker,image,rows,cols,&island))
            ring_s2_reset(&island_tracker);
        return;
    }
}

void track_special_process(const uint8_t *image,uint8_t rows,uint8_t cols,
    const uint8_t *left,uint8_t *mid,const uint8_t *right,
    uint8_t control_first,uint8_t control_last)
{
    memset(&marks,0,sizeof(marks));
    /* Do not retain S1 bridge overlays after changing state. */
    memset(cross_draw_flags,0,sizeof(cross_draw_flags));
    s1_result_t observation={0};
    const uint16_t *width=fixed_width;
    if(ring.state==0){
        /* Let a cross already being traversed finish before recognizing a ring. */
        if(cross_path_state==0){
            ring_s1_detect(image,rows,cols,left,right,img_edge_valid,&observation);
            marks.upper=observation.upper; marks.middle=observation.middle; marks.lower=observation.lower;
            ring_s1_enter(&ring,rows,cols,&observation);
        }
        if(ring.state==0){
            cross_process(image,rows,cols,left,mid,right,control_first,control_last);
            if(!cross_path_state && !observation.preidentified)measure_straight(rows,left,right);
            else if(straight_samples<4)straight_samples=0;
            return;
        }
        if(straight_samples<4)straight_samples=0;
        memset(s1_bridge_width,0,sizeof(s1_bridge_width));
        memset(&img_cross,0,sizeof(img_cross));
        memset(cross_draw_flags,0,sizeof(cross_draw_flags));
    }else if(ring.state<=2){
        ring_s1_update(&ring,image,rows,cols,&observation);
        marks.upper=observation.upper; marks.middle=observation.middle; marks.lower=observation.lower;
    }

    if(ring.state<=4)ring_s2_detect(&island_tracker,image,rows,cols,&island);
    if(ring.state==1 && observation.middle.valid &&
       (!island.found || !island.edge_valid[observation.middle.y] ||
        island.edge[observation.middle.y]!=observation.middle.x)){
        /* Seed from the measured middle while visible, including the entry frame. */
        ring_s2_seed(&island_tracker,rows,cols,observation.middle.x,observation.middle.y);
        ring_s2_detect(&island_tracker,image,rows,cols,&island);
    }
    if(ring.state==1 && opening_closed(image,rows,cols))ring.state=2;
    if(ring.state==2 && ring_s3_entry_update(&entry3,observation.upper.valid)){
        ring.state=3;
    }
    int entering_four=0;
    if(ring.state==3 || ring.state==RING_STATE_4){
        int row=rows-1;
        while(row>=control_first && (!img_line_valid[row] || mid[row]>=cols ||
              !image[row*cols+mid[row]]))--row;
        s3_exit_result_t exit_result;
        /* Keep confirming the exit in S4: after three lost island frames the
         * component tracker needs a fresh seed, not just another detect call. */
        if(ring_s3_exit_update(&exit3,3,image,rows,cols,
                row>=control_first?mid[row]:-1,row,&exit_result)){
            reacquire_at_exit(image,rows,cols,left,right,&exit_result);
            if(ring.state==3){
                entering_four=1;
                ring.state=RING_STATE_4;
            }
        }
        if(exit_result.corner_valid)
            marks.shoulder=(s1_point_t){exit_result.corner_x,exit_result.corner_y,1};
    }

    switch(ring.state){
    case 1: s1_bridge_apply(rows,cols,&observation,right,mid);break;
    case 2:
    case 3:
        ring_s2_apply(image,rows,cols,&island,s1_bridge_width,mid,img_line_valid);
        break;
    case RING_STATE_4:
    case RING_STATE_5:
        if(entering_four){
            ring_s4_bridge_apply(image,rows,cols,marks.shoulder,right,img_edge_valid,
                                 width,mid,img_line_valid,&search.anchor);
            marks.anchor=search.anchor;
            break;
        }
        ring_s4_s5_process(&search,&ring.state,image,rows,cols,&island,marks.shoulder,right,
                           img_edge_valid,width,mid,img_line_valid);
        marks.upper=search.upper; marks.candidate=search.candidate;
        marks.anchor=search.anchor;
        if(ring.state==0)finish_ring();
        break;
    }
}
