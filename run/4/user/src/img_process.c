/*
 *@第十九届东南大学智能车竞赛基础四轮组部分开源学习代码
 *
 * 【本文件在 main.c 中的调用顺序】
 * 1. get_hist_gram：统计当前灰度图像中，每个灰度值出现多少次。
 * 2. get_threshold_otsu：根据直方图，用大津法选择整幅图共用的阈值。
 * 3. binaryzation_process：以阈值为界，把灰度图原地转换成黑白图。
 * 4. auxiliary_process：逐行寻找黑色左右边界，计算白色赛道的中线。
 * run/3 的 main.c 使用有效中线加权控制舵机。
 *
 * 【图像存储与坐标】
 * 像素按行连续存放，二维坐标 (行 i, 列 j) 对应一维下标 i * 列数 + j。
 * 第 0 行是图像顶部，最后一行是底部；第 0 列在左侧，列号向右增加。
 * 当前例程图像为 120 行、188 列，灰度范围为 0（黑）到 255（白）。
 * 边界数组中的下标表示行号，数组元素表示该行边界或中线的列号。
 */

#include "img_process.h"
#include "string.h"

/* 7条黑条斑马线：原始二值图检测，不改边界、中线或控制。
 * 6条可见时要求更多支持行；不以固定跳变总数判定远近。
 * 所有阈值为120x188图像的初值，须用实拍帧验证。 */
#define ZEBRA_STRIPES 7
#define ZEBRA_MIN_ROWS 3
#define ZEBRA_PARTIAL_ROWS 5
#define ZEBRA_ANCHOR_DISTANCE 24
#define ZEBRA_CONFIRM_FRAMES 3
#define ZEBRA_RELEASE_FRAMES 5
#define ZEBRA_FRAME_DY 24
#define ZEBRA_FRAME_DX 16
img_zebra_info_t img_zebra;
volatile uint8_t zebra_stop_requested;
uint8_t zebra_pass_count;
static uint8_t zebra_rows, zebra_cols;
static int zebra_prev_y, zebra_prev_x, zebra_prev_width;

static int zebra_abs(int value) { return value < 0 ? -value : value; }

// 两次识别之间要求连续空帧，防止同一条斑马线因短时漏检重复计数。
#define ZEBRA_REARM_FRAMES 12
void zebra_race_update(uint8_t eligible)
{
    static uint8_t armed = 1, clear_frames;
    if (zebra_stop_requested) return;
    if (!eligible || img_zebra.candidate)
    {
        clear_frames = 0;
        if (eligible && armed && img_zebra.hits >= ZEBRA_CONFIRM_FRAMES)
        {
            ++zebra_pass_count;
            armed = 0;
        }
        return;
    }
    if (armed) return;
    ++clear_frames;
    if (zebra_pass_count == 2 && clear_frames >= ZEBRA_RELEASE_FRAMES)
        zebra_stop_requested = 1;
    else if (zebra_pass_count == 1 && clear_frames >= ZEBRA_REARM_FRAMES)
        armed = 1;
}

// PIT 每10 ms 调用一次：600 ms 内降至巡航目标的25%，随后切断驱动。
#define ZEBRA_SLOW_TICKS 60
float zebra_motor_target(float cruise_target)
{
    static uint8_t ticks;
    if (!zebra_stop_requested) return cruise_target;
    if (ticks == ZEBRA_SLOW_TICKS) return 0.0f;
    ++ticks;
    return cruise_target * (1.0f - 0.75f * ticks / ZEBRA_SLOW_TICKS);
}

/* 必须在邻近完整赛道内：使用当前帧正常双边行，避开被黑条切碎的行。
 * 连续白区验证也排除把两根黑条之间的小白缝当作完整赛道。 */
static int zebra_lane(const uint8_t *image, int rows, int cols, int y,
                      int first, int last, const uint8_t *left, const uint8_t *right)
{
    int span = last - first + 1;
    for (int d = 1; d <= ZEBRA_ANCHOR_DISTANCE; ++d)
        for (int side = -1; side <= 1; side += 2)
        {
            int row = y + side * d, white = 0;
            int l, r, width;
            if (row < 0 || row >= rows || img_edge_valid[row] != 3) continue;
            l = left[row]; r = right[row]; width = r - l - 1;
            if (l >= r || r >= cols || width < 28) continue;
            if (span * 100 < width * 45 || span * 100 > width * 110) continue;
            if (first < l - width / 10 || last > r + width / 10) continue;
            for (int x = l + 1; x < r; ++x)
                if (image[row * cols + x]) ++white;
            if (white * 100 >= width * 95) return 1;
        }
    return 0;
}

void zebra_detect(const uint8_t *image, uint8_t rows, uint8_t cols,
                  const uint8_t *left, const uint8_t *right)
{
    int chain = 0, strong = 0, previous_n = 0;
    int previous_centers[ZEBRA_STRIPES] = {0};
    int chain_left = 0, chain_right = 0;
    uint8_t latched = img_zebra.detected, hits = img_zebra.hits, misses = img_zebra.misses;
    if (rows != zebra_rows || cols != zebra_cols || !image || !left || !right || rows < 8 || cols < 28)
    {
        latched = hits = misses = 0;
        zebra_rows = rows; zebra_cols = cols;
    }
    memset(&img_zebra, 0, sizeof(img_zebra));
    if (!image || !left || !right || rows < 8 || cols < 28) return;
    for (int y = rows / 6; y < rows; ++y)
    {
        int starts[ZEBRA_STRIPES], ends[ZEBRA_STRIPES];
        int n = 0, overflow = 0, minw = cols, maxw = 0, ming = cols, maxg = 0;
        int total = 0, valid = 1;
        // 仅统计两侧都有白像素的黑段，左右外部黑背景不计入。
        for (int x = 1; x < cols - 1; ++x)
        {
            int start, width;
            if (image[y * cols + x] || !image[y * cols + x - 1]) continue;
            start = x;
            while (x < cols && !image[y * cols + x]) ++x;
            if (x == cols) break;
            if (n == ZEBRA_STRIPES) { overflow = 1; break; }
            starts[n] = start; ends[n] = x - 1; width = x - start;
            if (width < minw) minw = width;
            if (width > maxw) maxw = width;
            total += width;
            if (n)
            {
                int gap = start - ends[n - 1] - 1;
                if (gap < ming) ming = gap;
                if (gap > maxg) maxg = gap;
            }
            ++n;
        }
        if (overflow || n < ZEBRA_STRIPES - 1) valid = 0;
        if (valid)
        {
            int span = ends[n - 1] - starts[0] + 1;
            // 保留远处1像素黑条，靠多行连续性排除孤立像素。
            if (maxw > minw * 3 || maxg > ming * 3 || span < 24 ||
                total * 100 < span * 15 || total * 100 > span * 70 ||
                !zebra_lane(image, rows, cols, y, starts[0], ends[n - 1], left, right)) valid = 0;
        }
        if (!valid) { chain = strong = previous_n = 0; continue; }
        if (n != previous_n) chain = strong = 0;
        else
            for (int k = 0; k < n; ++k)
                if (zebra_abs(starts[k] + ends[k] - previous_centers[k]) > 6)
                { chain = strong = 0; break; }
        if (!chain) { chain_left = starts[0]; chain_right = ends[n - 1]; }
        if (starts[0] < chain_left) chain_left = starts[0];
        if (ends[n - 1] > chain_right) chain_right = ends[n - 1];
        ++chain;
        if (n == ZEBRA_STRIPES) ++strong;
        previous_n = n;
        for (int k = 0; k < n; ++k) previous_centers[k] = starts[k] + ends[k];
        if (chain >= (strong ? ZEBRA_MIN_ROWS : ZEBRA_PARTIAL_ROWS) && chain > img_zebra.support_rows)
        {
            img_zebra.candidate = 1;
            img_zebra.stripes = n;
            img_zebra.support_rows = chain;
            img_zebra.top = y - chain + 1; img_zebra.bottom = y;
            img_zebra.left = chain_left;
            img_zebra.right = chain_right;
        }
    }
    if (img_zebra.candidate)
    {
        int y = (img_zebra.top + img_zebra.bottom) / 2;
        int x = (img_zebra.left + img_zebra.right) / 2;
        int width = img_zebra.right - img_zebra.left + 1;
        int same = zebra_abs(y - zebra_prev_y) <= ZEBRA_FRAME_DY &&
                   zebra_abs(x - zebra_prev_x) <= ZEBRA_FRAME_DX &&
                   width * 2 >= zebra_prev_width && width <= zebra_prev_width * 2;
        if (!same) { hits = 0; latched = 0; }
        if (hits < ZEBRA_CONFIRM_FRAMES) ++hits;
        if (hits >= ZEBRA_CONFIRM_FRAMES) latched = 1;
        zebra_prev_y = y; zebra_prev_x = x; zebra_prev_width = width;
        misses = 0;
    }
    else
    {
        hits = 0;
        if (misses < ZEBRA_RELEASE_FRAMES) ++misses;
        if (misses >= ZEBRA_RELEASE_FRAMES) latched = 0;
    }
    img_zebra.detected = latched; img_zebra.hits = hits; img_zebra.misses = misses;
}
       
uint8_t zebra_draw_flags[256];

// 寻找未被黑条割裂的完整赛道，拒绝细缝、赛道外和不连续白区。
static int zebra_anchor(const uint8_t *image, int rows, int cols,
                        const uint8_t *left, const uint8_t *right, int start, int step)
{
    int span = img_zebra.right - img_zebra.left + 1;
    for (int d = 0; d < ZEBRA_ANCHOR_DISTANCE; ++d)
    {
        int y = start + d * step, white = 0, l, r, w;
        if (y < 0 || y >= rows) break;
        if (img_edge_valid[y] != 3) continue;
        l = left[y]; r = right[y]; w = r - l - 1;
        if (w < 28 || r >= cols || span * 100 < w * 45 || span * 100 > w * 110) continue;
        if (img_zebra.left < l - w / 10 || img_zebra.right > r + w / 10) continue;
        for (int x = l + 1; x < r; ++x) if (image[y * cols + x]) ++white;
        if (white * 100 >= w * 95) return y;
    }
    return -1;
}

void zebra_process(const uint8_t *image, uint8_t rows, uint8_t cols,
                   const uint8_t *left, uint8_t *mid, const uint8_t *right)
{
    int a, b, first, last, ca, cb;
    memset(zebra_draw_flags, 0, sizeof(zebra_draw_flags));
    if (!image || !left || !mid || !right || !img_zebra.candidate ||
        !img_zebra.detected || cross_path_state || img_zebra.bottom >= rows ||
        img_zebra.right >= cols) return;
    first = img_zebra.top; last = img_zebra.bottom;
    a = zebra_anchor(image, rows, cols, left, right, first - 1, -1);
    b = zebra_anchor(image, rows, cols, left, right, last + 1, 1);
    // 近端移出图像时，用远端两条完整赛道行的方向向下延伸。
    if (a >= 0 && b < 0)
    {
        b = a;
        a = zebra_anchor(image, rows, cols, left, right, b - 4, -1);
    }
    else if (a < 0 && b >= 0)
        a = zebra_anchor(image, rows, cols, left, right, b + 4, 1);
    // 无可靠依据时不能继续使用条纹间白缝生成的错误中线。
    for (int y = first; y <= last; ++y) img_line_valid[y] = 0;
    if (a < 0 || b < 0 || a == b) return;
    ca = (left[a] + right[a]) / 2; cb = (left[b] + right[b]) / 2;
    if (zebra_abs(cb - ca) > zebra_abs(b - a) ||
        first < (a < b ? a : b) - ZEBRA_ANCHOR_DISTANCE ||
        last > (a > b ? a : b) + ZEBRA_ANCHOR_DISTANCE) return;
    for (int y = first; y <= last; ++y)
    {
        int center = ca + (cb - ca) * (y - a) / (b - a);
        if (center < 0 || center >= cols) return;
    }
    for (int y = first; y <= last; ++y)
    {
        mid[y] = ca + (cb - ca) * (y - a) / (b - a);
        img_line_valid[y] = 4;
        zebra_draw_flags[y] = 1;
    }
}



/*
 * 二值化：把灰度图转换成仅含 0 和 255 的黑白图。
 * _img：图像首地址，直接覆盖原图，不额外分配输出图像。
 * _rows / _cols：图像行数 / 列数。
 * _threshold_value：大津法得到的灰度阈值。
 * 规则：灰度严格大于阈值 -> 白色；小于或等于阈值 -> 黑色。
 */
void binaryzation_process(unsigned char* _img,
                          const unsigned short _rows,
                          const unsigned short _cols,
                          const unsigned int _threshold_value)
{
    // 第一步：求像素总数。当前 120 * 188 = 22560，能用 unsigned short 保存。
    // 若以后提高分辨率，需检查总像素数是否超过该类型的表示范围。
    unsigned short _size = _rows * _cols;
    // 第二步：按内存顺序遍历全部像素，无需分别循环行和列。
    for (unsigned short _i = 0; _i < _size; ++_i)
    {
        // *(_img + _i) 等价于 _img[_i]，表示第 _i 个像素。
        if (*(_img + _i) > _threshold_value)
            *(_img + _i) = 255; // 较亮区域变为白色，预期对应赛道内部。
        else
            *(_img + _i) = 0;   // 较暗区域变为黑色，预期对应赛道边界。
    }
}

/*
 * 灰度直方图：统计 0~255 共 256 个灰度级各自的像素数量。
 * _img：尚未二值化的灰度图；_rows / _cols：行数 / 列数。
 * _hist_gram：调用者提供的至少 256 个 short 元素的输出数组。
 * 例如 _hist_gram[200] = 100，表示有 100 个像素的灰度值为 200。
 */
void get_hist_gram(unsigned char* _img,
                   const unsigned short _rows,
                   const unsigned short _cols,
                   short* _hist_gram)
{
    // 第一步：每一帧重新统计，因此必须先清空全部 256 个计数。
    // memset 的长度单位是字节，必须乘以每个元素的大小。
    // 当前文件已使用完整清零；若只写 256，会漏掉后一半 short 元素。
    // 这里的 _hist_gram 是指针，不能用 sizeof(_hist_gram) 代替数组总大小。
    memset(_hist_gram, 0,  256 * sizeof(*_hist_gram));
    // 第二步：遍历每一行。
    for (unsigned short _i = 0; _i < _rows; ++_i)
    {
        // 第三步：遍历该行的每一列。
        for (unsigned short _j = 0; _j < _cols; ++_j)
        {
            // 先读出像素灰度，再以灰度为下标，让对应计数加 1。
            // 当前整帧共 22560 像素，单个计数可由当前平台的 short 保存。
            _hist_gram[_img[_i * _cols + _j]]++;
        }
    }
}

/*
 * 大津法（Otsu）：寻找使两类像素的“类间方差”最大的阈值。
 * 对每个候选阈值 Y，将灰度 <= Y 的像素归入暗类，> Y 的归入亮类。
 * 类间方差 = 暗类占比 * 亮类占比 * (暗类平均灰度 - 亮类平均灰度)^2。
 * 数值越大，表示按这一阈值分出的两类在平均亮度上分离得越明显。
 * 输入：当前帧的 256 项灰度直方图。返回：选出的阈值（0~255）。
 * 前提：直方图计数非负，且至少有一个像素；本函数未处理空图像输入。
 */
unsigned char get_threshold_otsu(const short* _hist_gram)
{
    // Y 用来遍历灰度级；amount 是整幅图的像素总数。X 为原代码未使用变量。
    int X, Y, amount = 0;
    // back 表示暗类，fore 表示亮类，并非实际识别出的物体前景和背景。
    // pixel_back / fore：各类像素数量。
    // pixel_integral_back / fore：各类像素的灰度总和。
    // pixel_integral：整幅图的灰度总和。
    int pixel_back = 0, pixel_fore = 0, pixel_integral_back = 0, pixel_integral_fore = 0, pixel_integral = 0;
    // Omega：像素数量占比；Micro：平均灰度。
    // SigmaB：目前找到的最大类间方差；Sigma：当前候选阈值的类间方差。
    double OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // 类间方差;
    int pixel_min_value, pixel_max_value;
    int threshold = 0;

    // 第一步：从低到高找到实际出现过的最小灰度值。
    // 循环体是空语句“;”，搜索通过循环条件和变量递增完成。
    for (pixel_min_value = 0; pixel_min_value < 256 && _hist_gram[pixel_min_value] == 0; pixel_min_value++)
        ;
    // 第二步：从高到低找到最大灰度值，缩小后续阈值搜索范围。
    for (pixel_max_value = 255; pixel_max_value > pixel_min_value && _hist_gram[pixel_max_value] == 0; pixel_max_value--)
        ;
    // 特殊情况：全图只有一个灰度值，直接返回它。
    // 根据本文件的二值化规则，此时全部像素都会变成黑色。
    if (pixel_max_value == pixel_min_value)
        return pixel_max_value; // 图像中只有一个颜色
    // 特殊情况：最大与最小灰度只差 1，取较小值就能分开两类。
    // 此判断要求两种灰度相邻，并非适用于任意“只有两种灰度”的图像。
    if (pixel_min_value + 1 == pixel_max_value)
        return pixel_min_value; // 图像中只有二个颜色

    // 第三步：把有效灰度范围内的计数相加，得到总像素数。
    for (Y = pixel_min_value; Y <= pixel_max_value; Y++)
        amount += _hist_gram[Y]; // 像素总数

    // 第四步：求灰度总和。某灰度级的贡献 = 该灰度级像素数 * 灰度值。
    pixel_integral = 0;
    for (Y = pixel_min_value; Y <= pixel_max_value; Y++)
        pixel_integral += _hist_gram[Y] * Y;
    // 方差不小于 0，初值设为 -1，让第一个有效候选可以更新结果。
    SigmaB = -1;
    // 第五步：逐个尝试阈值。不到最大灰度，以保证亮类仍有像素。
    for (Y = pixel_min_value; Y < pixel_max_value; Y++)
    {
        // 阈值上移到 Y 时，把灰度恰为 Y 的像素累加进暗类。
        pixel_back = pixel_back + _hist_gram[Y];
        // 亮类数量可由总数减暗类数量得到，无需再遍历图像。
        pixel_fore = amount - pixel_back;
        // 转为 double 后做除法，避免整数除法丢失小数部分。
        OmegaBack = (double)pixel_back / amount;
        OmegaFore = (double)pixel_fore / amount;
        // 同样累计暗类灰度总和，再相减求亮类灰度总和。
        pixel_integral_back += _hist_gram[Y] * Y;
        pixel_integral_fore = pixel_integral - pixel_integral_back;
        // 各类灰度总和 / 各类像素数 = 各类平均灰度。
        MicroBack = (double)pixel_integral_back / pixel_back;
        MicroFore = (double)pixel_integral_fore / pixel_fore;
        // 根据两类占比和平均灰度差，计算当前划分的类间方差。
        Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);
        // 第六步：发现更大的方差时，保存方差及其对应的阈值。
        // 使用严格大于，因此若多个候选方差相同，保留先遇到的较小阈值。
        if (Sigma > SigmaB)
        {
            SigmaB = Sigma;
            threshold = Y;
        }
    }
    return threshold;
}

/* 当前帧锚点补线：
 * 1. 独立跟踪白色走廊及左右实测边界；补出的中线不参与搜边。
 * 2. 邻近行一致的双边界作为锚点，优先使用当前帧宽度变化趋势。
 * 3. 单边行：可见边界 +/- 当前宽度模型/2，而非固定直道宽度/2。
 * measured_width 仅记录双边实测；track_width 是锚点推算的工作模型。
 * 工作模型只能被双边锚点更新，单边补线不会自我训练或随时间漂移。
 * 没有新锚点时保留模型，不按帧数过期。重启/尺寸变化后重新学习。
 * 45cm 不能直接换成像素；本方法是图像局部近似，不是标定后的鸟瞰几何。
 */
#define RESEED_DISTANCE 24
#define MAX_ROW_STEP 8
#define MIN_TRACK_WIDTH 6
#define WIDTH_FIT_SPAN 12
#define WIDTH_FIT_POINTS 8
#define WIDTH_EXTRAP_ROWS 32
#define WIDTH_SLOPE_LIMIT_Q8 (4 * 256)
uint8_t img_line_valid[256]; /* 0无效；1单边补偿；2双边实测 */
uint8_t img_edge_valid[256]; /* bit0左边实测，bit1右边实测；不表示中线有效 */
static uint8_t measured_width[256];
static uint16_t track_width[256];
const uint16_t *img_track_width(void) { return track_width; }
static uint8_t last_rows, last_cols;
static int bottom_seed = -1;

static int clip_column(int x, int cols)
{
    return x < 0 ? 0 : (x >= cols ? cols - 1 : x);
}

static int abs_int(int x)
{
    return x < 0 ? -x : x;
}

/* 在锚点的完整赛道一侧拟合宽度 w(row)，不用单边/推算宽度参与拟合。
 * 采用多个点的一阶最小二乘，减轻单行像素抖动；Q8避免浮点运算。 */
static int fit_width_slope(int anchor, int direction, int rows,
                           const uint8_t* paired, const uint8_t* left,
                           const uint8_t* right, int* slope_q8)
{
    int n = 0, sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (int step = 0; step <= WIDTH_FIT_SPAN && n < WIDTH_FIT_POINTS; ++step)
    {
        int row = anchor + direction * step;
        int x, width;
        if (row < 0 || row >= rows) break;
        if (!paired[row]) continue;
        x = row - anchor;
        width = right[row] - left[row];
        ++n;
        sx += x;
        sy += width;
        sxx += x * x;
        sxy += x * width;
    }
    if (n < 3 || n * sxx == sx * sx) return 0;
    *slope_q8 = (n * sxy - sx * sy) * 256 / (n * sxx - sx * sx);
    if (*slope_q8 > WIDTH_SLOPE_LIMIT_Q8) *slope_q8 = WIDTH_SLOPE_LIMIT_Q8;
    if (*slope_q8 < -WIDTH_SLOPE_LIMIT_Q8) *slope_q8 = -WIDTH_SLOPE_LIMIT_Q8;
    return 1;
}

void auxiliary_process(uint8_t* image, uint8_t rows, uint8_t cols,
                       unsigned char threshold,
                       uint8_t* left_line, uint8_t* mid_line, uint8_t* right_line)
{
    uint8_t paired[256] = {0};
    int seed, slope = 0, previous_seed = -1, missing = 0;
    (void)threshold;
    memset(img_line_valid, 0, sizeof(img_line_valid));
    memset(img_edge_valid, 0, sizeof(img_edge_valid));
    if (!image || !left_line || !mid_line || !right_line || !rows || cols < 7)
    {
        return;
    }
    if (rows != last_rows || cols != last_cols)
    {
        memset(measured_width, 0, sizeof(measured_width));
        memset(track_width, 0, sizeof(track_width));
        bottom_seed = -1;
        last_rows = rows;
        last_cols = cols;
    }
    seed = bottom_seed >= 0 ? bottom_seed : cols / 2;
    bottom_seed = -1;

    /* 第一遍只搜边。搜索起点来自画面内白色走廊，而非推算的赛道中线。 */
    for (int row = rows - 1; row >= 0; --row)
    {
        const uint8_t* pixels = image + row * cols;
        int predicted = clip_column(seed + slope, cols);
        int start = -1, distance_best = cols + 1;
        int left = 0, right = cols - 1, flags = 0;
        left_line[row] = 0;
        right_line[row] = cols - 1;
        mid_line[row] = predicted; /* 无效行只保留有界坐标，消费者必须检查标志。 */
        for (int x = 1; x < cols - 1; ++x)
        {
            int distance = abs_int(x - predicted);
            if (pixels[x - 1] == 255 && pixels[x] == 255 && pixels[x + 1] == 255 &&
                distance < distance_best)
            {
                start = x;
                distance_best = distance;
            }
        }
        if (start < 0 || (row != rows - 1 && distance_best > RESEED_DISTANCE))
        {
            if (++missing <= 4) seed = predicted;
            else slope = 0;
            previous_seed = -1;
            continue;
        }
        for (int x = start; x >= 2; --x)
            if (pixels[x] == 0 && pixels[x - 1] == 0 && pixels[x - 2] == 0)
            {
                left = x;
                flags |= 1;
                break;
            }
        for (int x = start; x + 2 < cols; ++x)
            if (pixels[x] == 0 && pixels[x + 1] == 0 && pixels[x + 2] == 0)
            {
                right = x;
                flags |= 2;
                break;
            }
        left_line[row] = left;
        right_line[row] = right;
        img_edge_valid[row] = flags;
        seed = (left + right) / 2; /* 仅用于搜索，不是补线输出。 */
        slope = previous_seed >= 0 ? seed - previous_seed : 0;
        if (slope > MAX_ROW_STEP) slope = MAX_ROW_STEP;
        if (slope < -MAX_ROW_STEP) slope = -MAX_ROW_STEP;
        previous_seed = seed;
        missing = 0;
        if (row == rows - 1 && flags) bottom_seed = seed;
    }

    /* 双边锚点需要至少两个邻近行支持。不再因旧宽度不同而永久拒绝新宽度。 */
    for (int row = 0; row < rows; ++row)
    {
        int width = right_line[row] - left_line[row];
        int support = 0;
        if (img_edge_valid[row] != 3 || width < MIN_TRACK_WIDTH) continue;
        for (int delta = -2; delta <= 2; ++delta)
        {
            int other = row + delta;
            int other_width;
            if (!delta || other < 0 || other >= rows || img_edge_valid[other] != 3) continue;
            other_width = right_line[other] - left_line[other];
            if (other_width >= MIN_TRACK_WIDTH &&
                abs_int(other_width - width) <= 8 + width / 5 &&
                abs_int(left_line[other] - left_line[row]) <= RESEED_DISTANCE * abs_int(delta) &&
                abs_int(right_line[other] - right_line[row]) <= RESEED_DISTANCE * abs_int(delta))
                ++support;
        }
        if (support >= 2) paired[row] = 1;
    }

    /* 第二遍从当前帧锚点生成宽度模型。历史实测只用于远距离的透视比例。 */
    for (int row = 0; row < rows; ++row)
    {
        int above = row - 1, below = row + 1, width = 0;
        if (paired[row])
        {
            track_width[row] = right_line[row] - left_line[row];
            continue;
        }
        while (above >= 0 && !paired[above]) --above;
        while (below < rows && !paired[below]) ++below;
        if (above >= 0 && below < rows)
        {
            int wa = right_line[above] - left_line[above];
            int wb = right_line[below] - left_line[below];
            width = wa + (wb - wa) * (row - above) / (below - above);
        }
        else if (above >= 0 || below < rows)
        {
            int anchor = above >= 0 ? above : below;
            int direction = above >= 0 ? -1 : 1;
            int anchor_width = right_line[anchor] - left_line[anchor];
            int delta = row - anchor;
            int slope_q8 = 0;
            if (fit_width_slope(anchor, direction, rows, paired, left_line, right_line, &slope_q8))
            {
                int limited_delta = delta;
                int reference;
                if (limited_delta > WIDTH_EXTRAP_ROWS) limited_delta = WIDTH_EXTRAP_ROWS;
                if (limited_delta < -WIDTH_EXTRAP_ROWS) limited_delta = -WIDTH_EXTRAP_ROWS;
                width = anchor_width + slope_q8 * limited_delta / 256;
                reference = anchor + limited_delta;
                /* 不无限线性外推。远处保持已学的透视比例；没有标定依据则不补。 */
                if (limited_delta != delta)
                    width = measured_width[row] && measured_width[reference] ?
                        width * measured_width[row] / measured_width[reference] : 0;
            }
            else if (measured_width[row] && measured_width[anchor])
                width = anchor_width * measured_width[row] / measured_width[anchor];
        }
        if (width >= MIN_TRACK_WIDTH && width <= cols * 2)
            track_width[row] = width;
        /* 没有当前帧依据时保留先前锚点模型；绝不由单边结果累积更新。 */
    }

    /* 第三遍输出中线，并且只用可靠双边行更新实测宽度参考。 */
    for (int row = 0; row < rows; ++row)
    {
        int flags = img_edge_valid[row], center;
        if (paired[row])
        {
            int width = right_line[row] - left_line[row];
            center = (left_line[row] + right_line[row]) / 2;
            measured_width[row] = measured_width[row] ?
                (3 * measured_width[row] + width + 2) / 4 : width;
            img_line_valid[row] = 2;
        }
        else if ((flags == 1 || flags == 2) && track_width[row])
        {
            center = flags == 1 ? left_line[row] + track_width[row] / 2 :
                                  right_line[row] - track_width[row] / 2;
            img_line_valid[row] = 1;
        }
        else continue;
        mid_line[row] = clip_column(center, cols);
    }
}

/* 十字识别：只读二值图，不读写基础巡线模型；通过期间关联上一有效上角点。
 * 坐标：x向右，y向下；四角顺序：上左、上右、下左、下右。
 * 轮廓法参考用户提供的方法：纵扫交界，再找向入口/出口退去的转折。
 */
#define CROSS_WHITE_PERCENT 75
#define CROSS_BAND_ROWS 4
#define CROSS_CORNER_DEPTH 5
#define CROSS_CORNER_WINDOW 6
#define CROSS_MIN_WIDTH 12
#define CROSS_MIN_HEIGHT 6
img_cross_info_t img_cross;
static int16_t cross_upper[256], cross_lower[256];
/* 仅在四角首次确认后启用；保存最后通过成对验证的上角点。
 * 这是候选关联依据，不是旧角点补线：输出始终来自当前图像。 */
#define CROSS_TRACK_X_STEP 16
#define CROSS_TRACK_Y_STEP 24
static uint8_t cross_upper_tracking;
static uint8_t cross_upper_x[2], cross_upper_y[2];
static int cross_initial_upper_width;
/* 四角确认时记录角点外侧的路口边缘形状，整次通过固定该模板。
 * 比较相对坐标，允许角点平移及左右不同高，防止位置关联逐帧漂移到弯道。 */
#define CROSS_SHOULDER_SPAN 18
#define CROSS_SHOULDER_MIN 6
#define CROSS_SHOULDER_TOL 4
static int16_t cross_shoulder[2][CROSS_SHOULDER_SPAN+1];

static void cross_capture_shoulder(int cols)
{
    for(int side=0;side<2;++side)
        for(int k=1;k<=CROSS_SHOULDER_SPAN;++k)
        {
            int x=img_cross.corner_x[side]+(side?k:-k);
            cross_shoulder[side][k]=x>=0 && x<cols && cross_upper[x]>=0?
                cross_upper[x]-img_cross.corner_y[side]:32767;
        }
}

static int cross_same_shoulder(const int16_t *edge,int cols,int side,int x,int y)
{
    int compared=0,matched=0;
    for(int k=3;k<=CROSS_SHOULDER_SPAN;++k)
    {
        int q=x+(side?k:-k),reference=cross_shoulder[side][k];
        if(reference==32767 || q<0 || q>=cols) continue;
        ++compared;
        if(edge[q]>=0 && abs_int((edge[q]-y)-reference)<=CROSS_SHOULDER_TOL)
            ++matched;
    }
    return compared>=CROSS_SHOULDER_MIN && matched>=CROSS_SHOULDER_MIN &&
           matched*4>=compared*3;
}
/* 下角点已接近底部后失去可靠观测，才锁定出画；左右独立。 */
#define CROSS_LOWER_BOTTOM_MARGIN 16
#define CROSS_LOWER_X_STEP 16
#define CROSS_LOWER_Y_STEP 24
#define CROSS_LOWER_BACK_STEP 6
static uint8_t cross_lower_x[2],cross_lower_y[2];
static uint8_t cross_lower_near[2],cross_lower_gone[2];

static void cross_start_lower(int rows)
{
    for(int side=0;side<2;++side)
    {
        cross_lower_x[side]=img_cross.corner_x[side+2];
        cross_lower_y[side]=img_cross.corner_y[side+2];
        cross_lower_near[side]=cross_lower_y[side]>=rows-CROSS_LOWER_BOTTOM_MARGIN;
        cross_lower_gone[side]=0;
    }
}

static void cross_track_lower(int rows,int *x,int *y)
{
    for(int side=0;side<2;++side)
    {
        int i=side+2;
        int valid=!cross_lower_gone[side] && x[i]>=0 &&
            abs_int(x[i]-cross_lower_x[side])<=CROSS_LOWER_X_STEP &&
            y[i]>=cross_lower_y[side]-CROSS_LOWER_BACK_STEP &&
            y[i]<=cross_lower_y[side]+CROSS_LOWER_Y_STEP &&
            x[side]>=0 && y[i]-y[side]>=CROSS_MIN_HEIGHT &&
            abs_int(x[i]-x[side])<=y[i]-y[side];
        if(valid)
        {
            cross_lower_x[side]=x[i];cross_lower_y[side]=y[i];
            if(y[i]>=rows-CROSS_LOWER_BOTTOM_MARGIN)cross_lower_near[side]=1;
        }
        else
        {
            if(cross_lower_near[side])cross_lower_gone[side]=1;
            x[i]=-1;y[i]=0;
            img_cross.corner_mask&=~(1<<i);
            img_cross.corner_x[i]=img_cross.corner_y[i]=0;
        }
    }
}

static void cross_remember_upper(void)
{
    for(int i=0;i<2;++i)
    {cross_upper_x[i]=img_cross.corner_x[i];cross_upper_y[i]=img_cross.corner_y[i];}
}

/* 下角点是否存在不影响上角点成对验证。保留左右高度差。 */
static int cross_upper_pair_valid(const int *x,const int *y)
{
    if(x[0]<0 || x[1]<0 || x[1]-x[0]<CROSS_MIN_WIDTH) return 0;
    if(!cross_upper_tracking) return 1;
    int width=x[1]-x[0],previous=cross_upper_x[1]-cross_upper_x[0];
    if(width*4<previous*3 || width*4>previous*5 ||
       width*4<cross_initial_upper_width*3) return 0;
    for(int i=0;i<2;++i)
        if(abs_int(x[i]-cross_upper_x[i])>CROSS_TRACK_X_STEP ||
           abs_int(y[i]-cross_upper_y[i])>CROSS_TRACK_Y_STEP) return 0;
    return 1;
}

/* 外侧轮廓允许倾斜；内侧须退去，且角点后有真实纵向黑白边界。 */
static int cross_find_corner(const uint8_t *image, const int16_t *edge,
                             int rows, int cols, int direction, int upper,
                             int *cy)
{
    int best=-1, best_score=-10000;
    int begin=direction>0?CROSS_CORNER_WINDOW:cols-1-CROSS_CORNER_WINDOW;
    int end=direction>0?cols*3/4:cols/4;
    for(int x=begin;direction>0?x<end:x>end;x+=direction)
    {
        int y=edge[x], outer=0, inner=0, arm=0;
        if(y<0) continue;
        int side=direction>0?0:1;
        if(upper && cross_upper_tracking &&
           (abs_int(x-cross_upper_x[side])>CROSS_TRACK_X_STEP ||
            abs_int(y-cross_upper_y[side])>CROSS_TRACK_Y_STEP)) continue;
        if(upper && cross_upper_tracking &&
           !cross_same_shoulder(edge,cols,side,x,y)) continue;
        if(!upper && cross_upper_tracking &&
           (cross_lower_gone[side] ||
            abs_int(x-cross_lower_x[side])>CROSS_LOWER_X_STEP ||
            y<cross_lower_y[side]-CROSS_LOWER_BACK_STEP ||
            y>cross_lower_y[side]+CROSS_LOWER_Y_STEP)) continue;
        for(int k=1;k<=CROSS_CORNER_WINDOW;++k)
        {
            int a=edge[x-direction*k], b=edge[x+direction*k];
            if(a>=0 && abs_int(a-y)<=2+k) ++outer;
            /* 缺失值仅作为白色延伸证据，不参与坐标差值。 */
            if(k>=3 && (b<0 || (upper?y-b:b-y)>=CROSS_CORNER_DEPTH)) ++inner;
        }
        if(outer<4 || inner<3) continue;
        for(int k=1;k<=CROSS_CORNER_WINDOW;++k)
        {
            int yy=y+(upper?-k:k);
            if(yy<0 || yy>=rows) break;
            /* 允许边界倾斜；只寻找角点邻近的、朝通道内侧黑到白的边。 */
            for(int dx=-3;dx<=3;++dx)
            {
                int xx=x+dx, outside=xx-direction, inside=xx+direction;
                if(outside<0 || outside>=cols || inside<0 || inside>=cols) continue;
                if(!image[yy*cols+outside] && image[yy*cols+inside])
                {++arm;break;}
            }
        }
        if(arm<3) continue;
        int score=(outer+inner+arm)*4;
        if(upper && cross_upper_tracking)
            score-=2*abs_int(x-cross_upper_x[side])+abs_int(y-cross_upper_y[side]);
        if(!upper && cross_upper_tracking)
            score-=2*abs_int(x-cross_lower_x[side])+abs_int(y-cross_lower_y[side]);
        if(score>best_score) {best=x;*cy=y;best_score=score;}
    }
    return best;
}

void cross_detect(const uint8_t *image, uint8_t rows, uint8_t cols)
{
    int start=-1,best_start=-1,best_len=0;
    int xx[4],yy[4]={0,0,0,0};
    memset(&img_cross,0,sizeof(img_cross));
    if(!image || rows<16 || cols<32) return;
    /* 全高度预筛，前方出现宽白区即可启动，不等下方丢边。
     * 检查中央连续白走廊，图像边缘不算实测边界。 */
    for(int y=0;y<rows;++y)
    {
        int left=cols/2,right=cols/2,opened=0;
        if(image[y*cols+cols/2])
        {
            while(left>0 && image[y*cols+left-1]) --left;
            while(right+1<cols && image[y*cols+right+1]) ++right;
            opened=(right-left+1)*100>=cols*CROSS_WHITE_PERCENT;
        }
        if(opened) {if(start<0)start=y;}
        if((!opened || y==rows-1) && start>=0)
        {
            int length=y-start+(opened?1:0);
            if(length>best_len) {best_len=length;best_start=start;}
            start=-1;
        }
    }
    if(best_len<CROSS_BAND_ROWS) return;
    img_cross.stage=1;
    int ref=best_start+best_len/2;
    img_cross.reference_row=ref;
    for(int x=0;x<cols;++x)
    {
        cross_upper[x]=cross_lower[x]=-1;
        if(!image[ref*cols+x]) continue;
        for(int y=ref-1;y>=1;--y)
            if(!image[y*cols+x] && !image[(y-1)*cols+x])
            {cross_upper[x]=y;break;}
        for(int y=ref+1;y+1<rows;++y)
            if(!image[y*cols+x] && !image[(y+1)*cols+x])
            {cross_lower[x]=y;break;}
    }
    img_cross.stage=2;
    xx[0]=cross_find_corner(image,cross_upper,rows,cols, 1,1,&yy[0]);
    xx[1]=cross_find_corner(image,cross_upper,rows,cols,-1,1,&yy[1]);
    xx[2]=cross_find_corner(image,cross_lower,rows,cols, 1,0,&yy[2]);
    xx[3]=cross_find_corner(image,cross_lower,rows,cols,-1,0,&yy[3]);
    for(int i=0;i<4;++i) if(xx[i]>=0)
    {
        img_cross.corner_mask|=1<<i;
        img_cross.corner_x[i]=xx[i];img_cross.corner_y[i]=yy[i];
    }
    if(!cross_upper_pair_valid(xx,yy))
    {
        // 失败候选不得显示为有效上角点，也不得进入拟合或更新跟踪参考。
        img_cross.corner_mask&=~3;
        img_cross.corner_x[0]=img_cross.corner_x[1]=0;
        img_cross.corner_y[0]=img_cross.corner_y[1]=0;
        return;
    }
    if(cross_upper_tracking)
    {
        cross_remember_upper();
        cross_track_lower(rows,xx,yy);
    }
    if(img_cross.corner_mask!=15) return;
    img_cross.stage=3;
    /* 不限制左右角点同高；只检查左右次序和同侧上下关系。 */
    if(xx[1]-xx[0]<CROSS_MIN_WIDTH || xx[3]-xx[2]<CROSS_MIN_WIDTH ||
       yy[2]-yy[0]<CROSS_MIN_HEIGHT || yy[3]-yy[1]<CROSS_MIN_HEIGHT) return;
    /* 入口到出口的中心连接应处在白区，容忍少量散点。 */
    int top=(yy[0]+yy[1])/2,bottom=(yy[2]+yy[3])/2;
    int a=(xx[0]+xx[1])/2,b=(xx[2]+xx[3])/2,white=0;
    for(int y=top;y<=bottom;++y)
    {
        int x=a+(b-a)*(y-top)/(bottom-top);
        if(image[y*cols+x]) ++white;
    }
    if(white*100<(bottom-top+1)*90) return;
    img_cross.detected=1;img_cross.stage=4;
}

/* 十字通过状态独立于本帧四角判定。仅保留状态，不冻结历史路径。 */
#define CROSS_FIT_SPAN 10
#define CROSS_FIT_MIN_POINTS 4
#define CROSS_PATH_SLOPE_Q8 256 /* x(y): 每行最多横移1像素，允许零斜率。 */
uint8_t cross_path_state; /* 0普通，1上下连接，2双侧拟合，3混合，4本帧依据不足 */
uint8_t cross_draw_left[256], cross_draw_right[256], cross_draw_flags[256];
static uint8_t cross_pass_active, cross_rows, cross_cols;
static uint8_t cross_upper_missing_frames;
static uint8_t cross_completed_passes;

static void cross_reset_pass(void)
{
    cross_pass_active = cross_path_state = cross_upper_tracking = 0;
    cross_upper_missing_frames = 0;
}

/* 从当前远端角点沿真实出口边界向上跟踪，普通补线数据不参与拟合。 */
static int cross_fit_exit(const uint8_t *image,int rows,int cols,int side,int *slope,int *origin)
{
    int x0=img_cross.corner_x[side],y0=img_cross.corner_y[side],previous=x0;
    int n=0,sd=0,sx=0,sdd=0,sdx=0,dir=side?-1:1;
    (void)rows;
    for(int k=1;k<=CROSS_FIT_SPAN && y0-k>=0;++k)
    {
        int y=y0-k,best=-1,distance=1000;
        for(int x=previous-3;x<=previous+3;++x)
        {
            if(x<2 || x>=cols-2) continue;
            if(!image[y*cols+x] && !image[y*cols+x-dir] &&
               image[y*cols+x+dir] && image[y*cols+x+2*dir] && abs_int(x-previous)<distance)
            {best=x;distance=abs_int(x-previous);}
        }
        if(best<0) break;
        previous=best;
        int d=-k;
        ++n;sd+=d;sx+=best;sdd+=d*d;sdx+=d*best;
    }
    if(n<CROSS_FIT_MIN_POINTS || n*sdd==sd*sd) return 0;
    *slope=(n*sdx-sd*sx)*256/(n*sdd-sd*sd);
    if(abs_int(*slope)>CROSS_PATH_SLOPE_Q8) return 0;
    *origin=(sx*256-(*slope)*sd)/n;
    return abs_int(*origin-x0*256)<=3*256;
}

void cross_process(const uint8_t *image,uint8_t rows,uint8_t cols,
                   const uint8_t *left,uint8_t *mid,const uint8_t *right,
                   uint8_t control_first,uint8_t control_last)
{
    int slope[2]={0,0},origin[2]={0,0},end[2]={0,0},ok[2]={0,0},fits=0;
    memset(cross_draw_flags,0,sizeof(cross_draw_flags));
    if(!image || !left || !mid || !right || rows<16 || cols<32 ||
       control_first>control_last || control_last>=rows)
    {cross_reset_pass();return;}
    if(rows!=cross_rows || cols!=cross_cols)
    {cross_reset_pass();cross_rows=rows;cross_cols=cols;}
    /* 每次启动只处理两个十字；完成后跳过预筛和角点扫描。 */
    if (cross_completed_passes >= 2) return;
    cross_detect(image,rows,cols);
    for(int y=0;y<rows;++y) {cross_draw_left[y]=left[y];cross_draw_right[y]=right[y];}
    if(!cross_pass_active)
    {
        cross_path_state=0;
        if(!img_cross.detected) return;
        cross_pass_active=1;
        cross_upper_tracking=1;
        cross_capture_shoulder(cols);
        cross_start_lower(rows);
        cross_initial_upper_width=img_cross.corner_x[1]-img_cross.corner_x[0];
        cross_remember_upper();
    }
    /* 双上角连续两帧消失时，在覆盖普通中线前交还控制。 */
    if ((img_cross.corner_mask & 3) == 0)
    {
        if (++cross_upper_missing_frames >= 2)
        {
            ++cross_completed_passes;
            cross_reset_pass();
            return;
        }
    }
    else cross_upper_missing_frames = 0;

    // 两个当前上角点必须一起有效；禁止单个错点与普通边线拼成细通道。
    if((img_cross.corner_mask&3)!=3)
    {
        cross_path_state=4;
        for(int y=control_first;y<=control_last;++y) img_line_valid[y]=0;
        return;
    }
    for(int side=0;side<2;++side)
    {
        if(!(img_cross.corner_mask&(1<<side))) continue;
        int x=img_cross.corner_x[side],y=img_cross.corner_y[side],lower=side+2;
        origin[side]=x*256;
        if(img_cross.corner_mask&(1<<lower))
        {
            int dy=img_cross.corner_y[lower]-y;
            if(dy<CROSS_MIN_HEIGHT) continue;
            slope[side]=((int)img_cross.corner_x[lower]-x)*256/dy;
            if(abs_int(slope[side])>CROSS_PATH_SLOPE_Q8) continue;
            end[side]=img_cross.corner_y[lower];ok[side]=1;
        }
        else
        {
            if(!cross_fit_exit(image,rows,cols,side,&slope[side],&origin[side])) continue;
            end[side]=rows-1;ok[side]=1;++fits;
        }
        for(int row=y;row<=end[side];++row)
        {
            int px=(origin[side]+slope[side]*(row-y))/256;
            if(px<0 || px>=cols) continue;
            if(side) cross_draw_right[row]=px; else cross_draw_left[row]=px;
            cross_draw_flags[row]|=1<<side;
        }
    }
    cross_path_state=ok[0]&&ok[1]?(fits==0?1:(fits==2?2:3)):4;
    for(int y=0;y<rows;++y)
    {
        int flags=cross_draw_flags[y];
        int l=cross_draw_left[y],r=cross_draw_right[y];
        /* 左右角点不等高时，未补一侧必须是当前实测边界。 */
        if(flags && ((flags|img_edge_valid[y])&3)==3 && r-l>=6)
        {
            mid[y]=(l+r)/2;img_line_valid[y]=3;
        }
        else if(flags || (y>=control_first && y<=control_last && cross_path_state==4))
            img_line_valid[y]=0;
    }
}
