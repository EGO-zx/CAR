#include "ring_s1.h"
#include <string.h>

static int absolute(int v){return v<0?-v:v;}
static int white(const uint8_t *im,int h,int w,int x,int y)
{return x>=0 && x<w && y>=0 && y<h && im[y*w+x]!=0;}
static void set_point(s1_point_t *p,int x,int y)
{p->x=(int16_t)x;p->y=(int16_t)y;p->valid=1;}

/* 同一内岛右轮廓，从下往上：x增大 -> 最大值平台 -> x减小。
 * 必须两侧都有连续趋势支持，不接受孤立尖峰、断边或单调斜线。 */
static s1_point_t arc_corner(const int16_t *edge,int h,int window)
{
    s1_point_t best={0,0,0};
    for(int y=window;y<h-window;++y){
        if(edge[y]<0)continue;
        int end=y;
        while(end+1<h && edge[end+1]==edge[y])++end;
        if(end+window>=h)break;
        int ok=1,rising=0,falling=0;
        for(int k=1;k<=window;++k){
            int above=edge[y-k],anear=edge[y-k+1];
            int below=edge[end+k],bnear=edge[end+k-1];
            if(above<0 || below<0 || above>anear || below>bnear ||
               anear-above>S1_ARC_MAX_STEP || bnear-below>S1_ARC_MAX_STEP){ok=0;break;}
            if(anear>above)++falling;
            if(bnear>below)++rising;
        }
        if(ok && rising>=2 && falling>=2 &&
           edge[y]-edge[y-window]>=S1_ARC_RISE &&
           edge[y]-edge[end+window]>=S1_ARC_RISE &&
           (!best.valid || edge[y]>best.x))set_point(&best,edge[y],(y+end)/2);
        y=end;
    }
    return best;
}

/* 借鉴十字：从白色参考横线纵扫黑边，再判断横向转折和纵向边界。
 * 左侧下角：左侧轮廓连续，右侧向下退；角下左黑右白。
 * 左侧上角：左侧轮廓连续，右侧向上退；角上左黑右白。
 */
static s1_point_t corner(const uint8_t *im,int h,int w,int ref,int upper,
                          int xmin,int xmax)
{
    int16_t edge[256];
    s1_point_t p={0,0,0};
    int best=-1;
    for(int x=0;x<w;++x){
        edge[x]=-1;
        if(!white(im,h,w,x,ref))continue;
        for(int y=ref+(upper?-1:1);y>0 && y<h-1;y+=upper?-1:1)
            if(!white(im,h,w,x,y) && !white(im,h,w,x,y+(upper?-1:1)))
            {edge[x]=(int16_t)y;break;}
    }
    if(xmin<6)xmin=6;
    if(xmax>w-7)xmax=w-7;
    for(int x=xmin;x<=xmax;++x){
        int y=edge[x],outer=0,retreat=0,arm=0,polarity=0;
        if(y<4 || y>=h-4)continue;
        for(int k=1;k<=6;++k){
            int a=edge[x-k],b=edge[x+k];
            if(a>=0 && absolute(a-y)<=2+k)++outer;
            if(k>=3 && (b<0 || (upper?y-b:b-y)>=5))++retreat;
            int yy=y+(upper?-k:k);
            for(int dx=-3;dx<=3;++dx)
                if(yy>=0 && yy<h && !white(im,h,w,x+dx-1,yy) && white(im,h,w,x+dx+1,yy))
                {++arm;break;}
        }
        /* 在角点黑区一侧取小块检查，不只依赖单像素。
         * 下角上白下黑；上角上黑下白。 */
        for(int dx=-3;dx<=-1;++dx)for(int dy=1;dy<=3;++dy){
            int above=white(im,h,w,x+dx,y-dy),below=white(im,h,w,x+dx,y+dy);
            if(upper?(!above && below):(above && !below))++polarity;
        }
        if(outer<4 || retreat<3 || arm<3 || polarity<6)continue;
        int score=outer+retreat+arm+polarity;
        if(score>best){best=score;set_point(&p,x,y);}
    }
    return p;
}

s1_point_t ring_find_upper(const uint8_t *image,int rows,int cols,
                           int reference_row,int xmin,int xmax)
{
    s1_point_t missing={0,0,0};
    if(!image || rows<8 || rows>256 || cols<16 || cols>256 ||
       reference_row<1 || reference_row>=rows)return missing;
    return corner(image,rows,cols,reference_row,1,xmin,xmax);
}

static s1_point_t find_middle(const uint8_t *im,int h,int w,int ref,int *top_out,int entry_search)
{
    /* 从下方白区逐列向上找白-黑-白，同一连续黑段组构成内岛候选。
     * 允许黑块贴左画框；拒绝触顶未重新进入白区的背景。
     * 邻列黑段必须重叠，且上下轮廓变化<=6像素。 */
    int count=0,prevtop=-1,prevbottom=-1,extent=0,bestextent=0;
    int rightmost=-1,island_top=0;
    int16_t working_edge[256],island_edge[256];
    for(int y=0;y<h;++y)working_edge[y]=island_edge[y]=-1;
    for(int x=2;x<w*3/4;++x){
        int bottom=-1,top=-1;
        if(white(im,h,w,x,ref)){
            int y=ref-1;
            while(y>1 && white(im,h,w,x,y))--y;
            if(y>1){bottom=y;while(y>1 && !white(im,h,w,x,y))--y; if(y>1)top=y+1;}
        }
        if(top<0 || bottom-top+1<3 || !white(im,h,w,x,top-2)){
            count=extent=0;prevtop=prevbottom=-1;continue;
        }
        if(!count || top>prevbottom || bottom<prevtop || absolute(top-prevtop)>6 || absolute(bottom-prevbottom)>6){
            count=extent=0;
            for(int y=0;y<h;++y)working_edge[y]=-1;
        }
        ++count;extent+=bottom-top+1;prevtop=top;prevbottom=bottom;
        for(int y=top;y<=bottom;++y)working_edge[y]=(int16_t)x;
        if(count>=6 && extent>bestextent){
            bestextent=extent;rightmost=x;island_top=top;
            memcpy(island_edge,working_edge,(unsigned)h*sizeof(island_edge[0]));
        }
    }
    if(rightmost<0){s1_point_t none={0,0,0};return none;}
    *top_out=island_top;
    s1_point_t point=arc_corner(island_edge,h,S1_ARC_WINDOW);
    /* A distant arc may span only a few rows. Still require two measured
     * steps and a two-pixel retreat on each side of the same peak. */
    if(entry_search && !point.valid)point=arc_corner(island_edge,h,2);
    return point;
}

void ring_s1_detect(const uint8_t *im,int h,int w,const uint8_t *left,
                    const uint8_t *right,const uint8_t *flags,s1_result_t *o)
{
    if(!o)return;
    memset(o,0,sizeof(*o));o->lower_missing=1;
    if(!im || !left || !right || !flags || h<=S1_LAST_ROW || h>256 || w<32 || w>256)return;
    (void)left;
    /* 预识别：右边允许少量短缺口，有效测量拟合为直线；左边连续丢失。 */
    int n=0,sy=0,sx=0,syy=0,syx=0,run=0,bestlen=0,start=0,beststart=0;
    int previous=-1,gap=0;
    for(int y=S1_FIRST_ROW;y<=S1_LAST_ROW;++y){
        if(flags[y]&2){
            if(previous>=0 && absolute(right[y]-right[previous])>S1_RIGHT_STEP*(y-previous))return;
            previous=y;
            gap=0;
            ++n;sy+=y;sx+=right[y];syy+=y*y;syx+=y*right[y];
        }else if(++gap>2)return;
        if(!(flags[y]&1)){
            if(!run)start=y;
            if(++run>bestlen){bestlen=run;beststart=start;}
        }else run=0;
    }
    if(n<48 || bestlen<S1_LOST_ROWS)return;
    if((flags[S1_REF_ROW_A]&3)!=2 && (flags[S1_REF_ROW_B]&3)!=2)return;
    int denominator=n*syy-sy*sy;
    int slope=(n*syx-sy*sx)*256/denominator,origin=(sx*256-slope*sy)/n;
    for(int y=S1_FIRST_ROW;y<=S1_LAST_ROW;++y)
        if((flags[y]&2) && absolute(right[y]*256-origin-slope*y)>S1_LINE_RESIDUAL*256)return;
    o->preidentified=1;
    int ref=beststart+bestlen/2;
    o->lower=corner(im,h,w,ref,0,6,w*3/4);
    o->lower_missing=!o->lower.valid;

    int island_top=0;
    o->middle=find_middle(im,h,w,ref,&island_top,1);
    if(!o->middle.valid)return;
    int rightmost=o->middle.x;
    /* 在内岛最右端上方白区找上角，横向只允许靠近中点。 */
    int upper_ref=island_top-2;
    if(upper_ref>1)o->upper=corner(im,h,w,upper_ref,1,
        rightmost-S1_UPPER_X_RANGE,rightmost+S1_UPPER_X_RANGE);
    /* A middle is usable only below a currently measured upper corner. */
    if(!o->upper.valid || o->middle.y<=o->upper.y){o->middle.valid=0;return;}
    if(!o->lower.valid || o->middle.y>=o->lower.y)return;
    o->confirmed=1;
}

void ring_s1_state_reset(s1_state_t *state)
{
    if(state)memset(state,0,sizeof(*state));
}

static int point_in_frame(s1_point_t p,int h,int w)
{
    return p.valid && p.x>=0 && p.x<w && p.y>=0 && p.y<h;
}

int ring_s1_enter(s1_state_t *state,int h,int w,const s1_result_t *entry)
{
    if(!state || state->state || !entry || !entry->confirmed ||
       h<=S1_LAST_ROW || h>256 || w<32 || w>256 ||
       !point_in_frame(entry->lower,h,w) || !point_in_frame(entry->middle,h,w) ||
       !point_in_frame(entry->upper,h,w) || entry->middle.y<=entry->upper.y ||
       entry->middle.y>=entry->lower.y)return 0;
    state->rows=(uint16_t)h;state->cols=(uint16_t)w;
    state->upper_tracking=entry->upper.valid && entry->middle.y>=h-S2_BOTTOM_MARGIN;
    state->last_lower=entry->lower;state->last_upper=entry->upper;
    state->state=1;
    return 1;
}

/* 不依赖内岛/中点提供上点参考行，围绕上一帧位置关联同一上角。
 * 下点只搜索原位置附近。 */
static s1_point_t track_corner(const uint8_t *im,int h,int w,
                               s1_point_t previous,int upper)
{
    s1_point_t best={0,0,0};
    if(!previous.valid)return best;
    int distance=32767;
    int first,last,xmin,xmax;
    {
        first=previous.y-S2_TRACK_RANGE;
        last=previous.y+S2_TRACK_RANGE+2;
        if(first<1)first=1;
        if(last>h-2)last=h-2;
        xmin=previous.x-S2_TRACK_RANGE;xmax=previous.x+S2_TRACK_RANGE;
    }
    for(int ref=first;ref<=last;ref+=2){
        s1_point_t p=corner(im,h,w,ref,upper,xmin,xmax);
        if(!p.valid)continue;
        int dx=absolute(p.x-previous.x),dy=absolute(p.y-previous.y);
        if(dx>S2_TRACK_RANGE || dy>S2_TRACK_RANGE || (!upper && p.y<previous.y-2))continue;
        if(dx+dy<distance){best=p;distance=dx+dy;}
    }
    return best;
}

uint8_t ring_s1_update(s1_state_t *state,const uint8_t *im,
                        int h,int w,s1_result_t *o)
{
    if(o)memset(o,0,sizeof(*o));
    if(!state)return 0;
    if(state->state!=1 && state->state!=2)return state->state;
    if(!o || !im || h!=state->rows || w!=state->cols ||
       h<=S1_LAST_ROW || h>256 || w<32 || w>256){
        return state->state;
    }
    if(state->state==2){
        o->upper=track_corner(im,h,w,state->last_upper,1);
        if(o->upper.valid)state->last_upper=o->upper;
        return state->state;
    }
    o->lower=track_corner(im,h,w,state->last_lower,0);
    o->lower_missing=!o->lower.valid;
    /* 从画面底部白区向上找内岛，避免沿用入口参考行漏掉下移的中点。 */
    int island_top=0;
    o->middle=find_middle(im,h,w,h-2,&island_top,0);
    if(!state->upper_tracking){
        /* 交接前仍使用原状态1方法：中点限制上点横向搜索范围。 */
        int upper_ref=island_top-2;
        if(o->middle.valid && upper_ref>1)
            o->upper=corner(im,h,w,upper_ref,1,
                o->middle.x-S1_UPPER_X_RANGE,o->middle.x+S1_UPPER_X_RANGE);
        if(!o->upper.valid)
            o->upper=track_corner(im,h,w,state->last_upper,1);
        if(o->middle.valid && o->upper.valid && o->upper.y<o->middle.y &&
           o->middle.y>=h-S2_BOTTOM_MARGIN){
            state->upper_tracking=1;
            state->last_upper=o->upper;
        }
    }else{
        /* 已交接后，中点消失不再阻断上点搜索；只使用真实检出的新位置。 */
        o->upper=track_corner(im,h,w,state->last_upper,1);
    }
    /* Do not expose an unverified middle to display or island reseeding. */
    if(!o->upper.valid || o->middle.y<=o->upper.y)o->middle.valid=0;
    o->confirmed=o->lower.valid && o->middle.valid && o->upper.valid &&
                 o->upper.y<o->middle.y && o->middle.y<o->lower.y;
    if(o->lower.valid)state->last_lower=o->lower;
    if(o->upper.valid)state->last_upper=o->upper;
    return state->state;
}

void ring_s1_apply(int h,int w,const uint8_t *right,
                   const uint8_t *flags,const uint16_t *track_width,uint8_t *mid,uint8_t *valid)
{
    if(!valid || h<1 || h>256)return;
    memset(valid,0,(unsigned)h);
    if(!right || !flags || !track_width || !mid || w<1 || w>256)return;
    for(int y=0;y<h;++y){
        int width=track_width[y],r=right[y];
        if(!(flags[y]&2) || r>=w || width<S1_MIN_WIDTH || width>w*2)continue;
        /* 与run/3单边补偿一致，使用现有宽度并将中线限幅到图像内。
         * 环岛状态1只跟随右实测边界，不受左边界是否被检出影响。 */
        int m=r-width/2;
        if(m<0)m=0;
        if(m>=w)m=w-1;
        mid[y]=(uint8_t)m;valid[y]=4;
    }
}
