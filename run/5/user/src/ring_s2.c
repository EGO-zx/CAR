#include "ring_s2.h"
#include <string.h>
static int marked(const uint8_t *map,int p){return (map[p>>3]>>(p&7))&1;}
static void mark(uint8_t *map,int p){map[p>>3]|=(uint8_t)(1u<<(p&7));}
static int abs_s2(int v){return v<0?-v:v;}
void ring_s2_reset(s2_tracker_t *t){if(t)memset(t,0,sizeof(*t));}
int ring_s2_seed(s2_tracker_t *t,int h,int w,int x,int y)
{
    if(!t || h<8 || h>256 || w<8 || w>256 || x<0 || x>=w || y<0 || y>=h)return 0;
    ring_s2_reset(t);t->rows=(uint16_t)h;t->cols=(uint16_t)w;
    t->seed_x=(uint16_t)x;t->seed_y=(uint16_t)y;t->seeded=1;return 1;
}
/* 每个黑色水平段只入栈一次；固定工作区满则整帧拒绝，不使用残缺连通域。 */
static int push_run(s2_tracker_t *t,const uint8_t *im,int w,int y,int x,int *size)
{
    int l=x,r=x;
    while(l>0 && im[y*w+l-1]==0)--l;
    while(r+1<w && im[y*w+r+1]==0)++r;
    if(*size>=S2_SPAN_CAPACITY)return 0;
    for(int k=l;k<=r;++k){mark(t->visited,y*w+k);mark(t->component,y*w+k);}
    s2_span_t *s=&t->stack[(*size)++];
    s->y=(uint16_t)y;s->left=(uint16_t)l;s->right=(uint16_t)r;return 1;
}
static int failed(s2_tracker_t *t)
{
    if(++t->lost>=S2_MAX_LOST_FRAMES){t->seeded=0;t->tracking=0;}
    return 0;
}
int ring_s2_detect(s2_tracker_t *t,const uint8_t *im,int h,int w,s2_result_t *o)
{
    if(o)memset(o,0,sizeof(*o));
    if(!t || !o)return 0;
    if(!t->seeded)return 0;
    if(!im || h!=t->rows || w!=t->cols || h<8 || h>256 || w<8 || w>256)return failed(t);
    memset(t->visited,0,sizeof(t->visited));
    uint32_t best_overlap=0,second_overlap=0,best_area=0;
    int best_seed=-1;
    /* 两遍：先选与历史内岛一致的连通域，再重建其mask。 */
    for(int pass=0;pass<2;++pass){
        if(pass && best_seed<0)return failed(t);
        if(pass)memset(t->visited,0,sizeof(t->visited));
        for(int seed=pass?best_seed:0;seed<h*w;++seed){
            if(im[seed] || marked(t->visited,seed))continue;
            memset(t->component,0,sizeof(t->component));
            int size=0,touches_background=0,contains_seed=0;
            uint32_t area=0,overlap=0;
            int16_t edge[256];
            for(int y=0;y<h;++y)edge[y]=-1;
            if(!push_run(t,im,w,seed/w,seed%w,&size))return failed(t);
            while(size){
                s2_span_t span=t->stack[--size];
                int y=span.y,l=span.left,r=span.right;
                area+=(uint32_t)(r-l+1);
                if(y==0 || r==w-1)touches_background=1;
                if(y==t->seed_y && l<=t->seed_x && r>=t->seed_x)contains_seed=1;
                if(r>edge[y])edge[y]=(int16_t)r;
                for(int x=l;x<=r;++x)if(marked(t->previous,y*w+x))++overlap;
                for(int d=-1;d<=1;d+=2){
                    int yy=y+d;if(yy<0 || yy>=h)continue;
                    for(int x=l;x<=r;++x)
                        if(!im[yy*w+x] && !marked(t->visited,yy*w+x))
                            if(!push_run(t,im,w,yy,x,&size))return failed(t);
                }
            }
            if(touches_background || area<S2_ISLAND_MIN_AREA)continue;
            /* 仅保留最长连续黑左白右轮廓；断边不能连接到另一个黑块。 */
            int start=0,len=0,best_start=0,best_len=0,prev=-1;
            for(int y=0;y<h;++y){
                int x=edge[y];
                int ok=x>=2 && x+3<w && !im[y*w+x-1] && !im[y*w+x-2] &&
                       im[y*w+x+1] && im[y*w+x+2] && im[y*w+x+3];
                if(!ok){len=0;prev=-1;continue;}
                if(!len || abs_s2(x-prev)>S2_ARC_MAX_STEP){start=y;len=0;}
                ++len;prev=x;
                if(len>best_len){best_len=len;best_start=start;}
            }
            if(best_len<S2_ARC_MIN_ROWS)continue;
            int minx=w,maxx=-1;
            for(int y=best_start;y<best_start+best_len;++y){
                if(edge[y]<minx)minx=edge[y];if(edge[y]>maxx)maxx=edge[y];
            }
            if(maxx-minx<3)continue; /* 拒绝纯竖边/小噪声，保留部分可见弧段。 */
            if(!t->tracking){if(!contains_seed)continue;overlap=area;}
            else {
                uint32_t smaller=area<t->previous_area?area:t->previous_area;
                if(overlap<16 || overlap*4<smaller)continue;
            }
            if(!pass){
                if(overlap>best_overlap){second_overlap=best_overlap;best_overlap=overlap;best_seed=seed;best_area=area;}
                else if(overlap>second_overlap)second_overlap=overlap;
            }else{
                /* 两个候选接近时不猜测身份。 */
                if(second_overlap && second_overlap*4>=best_overlap*3)return failed(t);
                memcpy(t->previous,t->component,sizeof(t->previous));
                t->previous_area=best_area;t->tracking=1;t->lost=0;
                o->rows=(uint16_t)h;o->cols=(uint16_t)w;o->area=area;o->found=1;
                for(int y=best_start;y<best_start+best_len;++y){o->edge[y]=(uint8_t)edge[y];o->edge_valid[y]=1;}
                return 1;
            }
        }
    }
    return failed(t);
}
void ring_s2_apply(const uint8_t *im,int h,int w,const s2_result_t *o,
                    const uint16_t *widths,uint8_t *mid,uint8_t *valid)
{
    if(!valid || h<1 || h>256)return;
    memset(valid,0,(unsigned)h);
    if(!im || !o || !widths || !mid || w<1 || w>256 || !o->found || o->rows!=h || o->cols!=w)return;
    for(int y=0;y<h;++y){
        int l=o->edge[y],width=widths[y],m=l+width/2,clear=1;
        if(!o->edge_valid[y] || l>=w || width<6 || width>w*2 || m>=w || im[y*w+l])continue;
        for(int x=l+1;x<=m;++x)if(!im[y*w+x]){clear=0;break;}
        if(clear){mid[y]=(uint8_t)m;valid[y]=5;}
    }
}
