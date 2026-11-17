#include "ring_s3_exit.h"
#include <string.h>
void ring_s3_exit_apply(const uint8_t *image,int rows,int cols,
    const s2_result_t *island,const uint16_t *track_width,
    uint8_t *mid,uint8_t *valid)
{
    ring_s2_apply(image,rows,cols,island,track_width,mid,valid);
}

static int absolute_exit(int v){return v<0?-v:v;}
static int white_exit(const uint8_t *im,int w,int x,int y){return im[y*w+x]!=0;}
void ring_s3_exit_reset(s3_exit_tracker_t *t){if(t)memset(t,0,sizeof(*t));}

/* 从T横臂内向下扫描：右侧黑路肩的上沿，向左退到竖向行驶通路。
 * 与状态1左下角镜像；验证水平上沿、竖向边界和上白下黑极性。 */
static int right_shoulder(const uint8_t *im,int h,int w,int ref,int expected,
                           int *px,int *py)
{
    int16_t edge[256];int best=-1;
    for(int x=0;x<w;++x){
        edge[x]=-1;
        if(!white_exit(im,w,x,ref))continue;
        for(int y=ref+1;y<h-1;++y)
            if(!white_exit(im,w,x,y) && !white_exit(im,w,x,y+1)){
                edge[x]=(int16_t)y;break;
            }
    }
    int first=expected-S3_EXIT_CORNER_RANGE,last=expected+S3_EXIT_CORNER_RANGE;
    if(first<6)first=6;if(last>w-7)last=w-7;
    for(int x=first;x<=last;++x){
        int y=edge[x],outer=0,retreat=0,arm=0,polarity=0;
        if(y<4 || y>=h-7 || y-ref>S3_EXIT_CORNER_RANGE)continue;
        for(int k=1;k<=6;++k){
            int a=edge[x+k],b=edge[x-k];
            if(a>=0 && absolute_exit(a-y)<=2+k)++outer;
            if(k>=3 && (b<0 || b-y>=5))++retreat;
            for(int dx=-3;dx<=3;++dx)
                if(white_exit(im,w,x+dx-1,y+k) && !white_exit(im,w,x+dx+1,y+k)){
                    ++arm;break;
                }
        }
        for(int dx=1;dx<=3;++dx)for(int dy=1;dy<=3;++dy)
            if(white_exit(im,w,x+dx,y-dy) && !white_exit(im,w,x+dx,y+dy))++polarity;
        if(outer<4 || retreat<3 || arm<3 || polarity<6)continue;
        int score=outer+retreat+arm+polarity;
        if(score>best){best=score;*px=x;*py=y;}
    }
    return best>=0;
}

/* A pointed shoulder has a local minimum in x, supported on both sides.
 * Reject smooth bends by requiring substantial retreat within six rows. */
static int right_turn(const uint8_t *im,int h,int w,const int16_t *edge,
    const s3_exit_tracker_t *t,int *px,int *py)
{
    int best=-32767,found=0,n=S3_EXIT_TURN_WINDOW;
    for(int y=n;y<h-n;++y){
        int x=edge[y],up_support=0,down_support=0,ok=1;
        if(x<2 || x+3>=w)continue;
        for(int k=1;k<=n;++k){
            int a=edge[y-k],b=edge[y+k];
            int an=edge[y-k+1],bn=edge[y+k-1];
            if(a<0 || b<0 || an<0 || bn<0 || a<x-1 || b<x-1 ||
               a-an < -1 || b-bn < -1 || a-an>S3_EXIT_TURN_STEP_MAX ||
               b-bn>S3_EXIT_TURN_STEP_MAX){ok=0;break;}
            if(a>=x+2)++up_support;
            if(b>=x+2)++down_support;
        }
        if(!ok || up_support<S3_EXIT_TURN_SUPPORT || down_support<S3_EXIT_TURN_SUPPORT ||
           edge[y-n]-x<S3_EXIT_TURN_RETREAT || edge[y+n]-x<S3_EXIT_TURN_RETREAT)continue;
        int polarity=0;
        for(int yy=y-2;yy<=y+2;++yy){
            int r=edge[yy];
            if(r<1 || r+3>=w)continue;
            const uint8_t *row=im+yy*w;
            if(row[r-1] && row[r] && !row[r+1] && !row[r+2] && !row[r+3])++polarity;
        }
        if(polarity<4)continue;
        int score=edge[y-n]+edge[y+n]-2*x;
        if(t->frames){
            int dx=absolute_exit(x+1-t->last_x),dy=absolute_exit(y-t->last_y);
            if(dx>S3_EXIT_TRACK_STEP || dy>S3_EXIT_TRACK_STEP)continue;
            score-=dx+dy;
        }
        if(!found || score>best){found=1;best=score;*px=x+1;*py=y;}
    }
    return found;
}

int ring_s3_exit_update(s3_exit_tracker_t *t,uint8_t state,
    const uint8_t *im,int h,int w,int seed_x,int seed_y,s3_exit_result_t *o)
{
    if(o){memset(o,0,sizeof(*o));o->bar_row=-1;}
    if(!t)return 0;
    if(state!=3){ring_s3_exit_reset(t);return 0;}
    if(!o || !im || h<24 || h>256 || w<32 || w>256 || seed_x<0 || seed_x>=w ||
       seed_y<S3_T_STEM_ROWS+S3_T_BAR_ROWS || seed_y>=h){t->frames=0;return 0;}
    if(t->rows!=h || t->cols!=w){ring_s3_exit_reset(t);t->rows=(uint16_t)h;t->cols=(uint16_t)w;}
    if(!white_exit(im,w,seed_x,seed_y)){t->frames=0;return 0;}
    int16_t left[256],right[256];
    for(int y=0;y<h;++y)left[y]=right[y]=-1;
    int cx=seed_x,top=seed_y,prev_l=seed_x,prev_r=seed_x;
    /* Search within the previous span when its center meets a slanted shoulder.
     * Never jump a row or use a disconnected bright patch. */
    for(int y=seed_y;y>=0;--y){
        if(!white_exit(im,w,cx,y)){
            int next=-1;
            for(int d=1;cx-d>=prev_l || cx+d<=prev_r;++d){
                if(cx-d>=prev_l && white_exit(im,w,cx-d,y)){next=cx-d;break;}
                if(cx+d<=prev_r && white_exit(im,w,cx+d,y)){next=cx+d;break;}
            }
            if(next<0)break;
            cx=next;
        }
        int l=cx,r=cx;
        while(l>0 && white_exit(im,w,l-1,y))--l;
        while(r+1<w && white_exit(im,w,r+1,y))++r;
        if(r-l<5)break;
        left[y]=(int16_t)l;right[y]=(int16_t)r;top=y;
        prev_l=l;prev_r=r;
        cx=(l+r)/2;
    }
    int found=0,corner_x=0,corner_y=0;
    int16_t measured_right[256];
    for(int y=0;y<h;++y)measured_right[y]=right[y]==w-1?-1:right[y];
    found=right_turn(im,h,w,measured_right,t,&corner_x,&corner_y);
    if(found)o->preidentified=1;
    /* Preserve detection of a square shoulder whose upper arm touches frame. */
    for(int y=seed_y-S3_T_STEM_ROWS;!found && y>=top+S3_T_BAR_ROWS-1;--y){
        int lower=y+S3_T_STEM_ROWS;
        int sl=left[lower],sr=right[lower];
        if(sr>=w-1 || sr-sl<12)continue;
        /* Both T arms must open, but the left shoulder can be much closer
         * to the car than the right one. Require three supporting stem rows. */
        int left_support=0;
        for(int row=lower;row<=seed_y;++row){
            if(left[row]>=left[y]+S3_T_ARM_MIN && left[row]+12<sr){
                if(++left_support==3){sl=left[row];break;}
            }else left_support=0;
        }
        if(left_support<3)continue;
        /* 横臂必须向当前通路两侧展开；仅一侧开口不是T。 */
        int band=1;
        for(int k=0;k<S3_T_BAR_ROWS;++k)
            if(left[y-k]>sl-S3_T_ARM_MIN || right[y-k]<sr+S3_T_ARM_MIN)band=0;
        if(!band)continue;
        /* 竖向通路需持续存在，排除单行缺口或两块无关白区。 */
        int stem=1;
        for(int k=1;k<=S3_T_STEM_ROWS;++k)
            if(left[y+k]<0 || right[y+k]-left[y+k]<12)stem=0;
        if(!stem)continue;
        /* 横臂上方为黑色封顶，而非十字继续向上延伸的白色竖臂。 */
        int cap=0;
        for(int yy=y-S3_T_BAR_ROWS;yy>=top-S3_EXIT_CORNER_RANGE && yy>=2;--yy){
            int black=0;
            for(int x=sl;x<=sr;++x)
                if(!white_exit(im,w,x,yy) && !white_exit(im,w,x,yy-1))++black;
            if(black*5>=(sr-sl+1)*4){cap=1;break;}
        }
        if(!cap)continue;
        o->preidentified=1;o->bar_row=(int16_t)y;
        if(right_shoulder(im,h,w,y,sr+1,&corner_x,&corner_y)){found=1;break;}
    }
    if(!found){t->frames=0;return 0;}
    o->corner_valid=1;o->corner_x=(int16_t)corner_x;o->corner_y=(int16_t)corner_y;
    if(t->frames && (absolute_exit(corner_x-t->last_x)>S3_EXIT_TRACK_STEP ||
                     absolute_exit(corner_y-t->last_y)>S3_EXIT_TRACK_STEP))t->frames=0;
    t->last_x=(int16_t)corner_x;t->last_y=(int16_t)corner_y;
    if(t->frames<S3_EXIT_CONFIRM_FRAMES)++t->frames;
    o->confirmed=t->frames>=S3_EXIT_CONFIRM_FRAMES;
    return o->confirmed;
}
