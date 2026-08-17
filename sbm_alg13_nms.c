// ============================================================================
// @file      sbm_alg13_nms.c
// @brief     NMS 后处理（PS 端 C 语言模块，SBM.Alg.13）
// @details
//   与 line2Dup.cpp matchClass() 语义一致：
//     - score = raw * 100 / (4 * N_features)   （归一化，0~100）
//     - IoU NMS 阈值 0.5；级 0 细化：16×16 单元 similarityLocal 窗口
//     - 级 0 线性内存布局与 4.2.6 节一致：块序 + 线性寻址
//   依赖：PL 累加引擎 Top-32 候选（AXI4-Lite）、级 0 线性内存（DDR3 HP 映射）
// @note 修正记录（坐标换算，审核 H2 项）：
//   原版将候选 cell 坐标直接 ×2，缺 cell→像素的 ×T+offset 换算（偏小约 8 倍）。
//   修正为与 C++ matchClass 完全一致的链：
//     级 1 单元 c → 级 1 像素 c*T1+offset1（T1=8，offset1=3）
//       → 级 0 像素 ×2+1 → 级 0 单元 /T0（T0=4）→ 16×16 单元细化
//       → 原图像素 (cx-8+bdx)*T0+offset0（offset0=1）
//   同时修正：级 0 细化窗口为 16×16 单元 = 64×64 像素（T0=4），原“72×72”口径有误。
// @author    WorkBuddy
// @date      2026-08-14
// ============================================================================
#include <stdint.h>
#include <string.h>

/// PL 候选 FIFO 深度
#define CAND_MAX    32
/// 进入级 0 细化的最大候选数
#define TOP_K       8
/// 最终输出 Top-N
#define TOP_N       5
/// IoU 抑制阈值
#define IOU_TH      0.5f
/// 最低归一化得分（0~100）
#define MIN_SCORE   20
/// 级 0 扩散范围
#define T0          4
/// 级 0 细化半径（单元），窗口 16×16 单元
#define R0          8
/// 级 1 扩散范围（粗层）
#define T1          8
/// 级 1 像素偏移 = T1/2 + (T1%2-1)
#define OFFSET1     3
/// 级 0 像素偏移 = T0/2 + (T0%2-1)
#define OFFSET0     1

/// @brief PL 候选条目（AXI4-Lite 格式，级 1 单元坐标）
typedef struct {
	uint8_t  score_raw;   ///< 原始得分（PL 累加引擎输出）
	uint16_t x, y;        ///< 级 1 单元坐标
} cand_t;

/// @brief 特征条目（级 0 原图像素坐标）
typedef struct {
	uint16_t x, y;        ///< 级 0 原图像素坐标
	uint8_t  label;       ///< 量化方向标签（0~7）
} feat_t;

/// @brief 最终匹配结果
typedef struct {
	float score;          ///< 归一化得分 0~100
	int   cx, cy;         ///< 原图中心坐标
	int   w, h;           ///< 目标范围
	float theta;          ///< 旋转角度
} result_t;

// ---- 寄存器访问（裸机直接寻址，或封装为 xil_io 宏） ----
static volatile uint32_t *const REG_BASE = (volatile uint32_t *)0xA0000000U;
#define REG_CAND_CNT  (REG_BASE[0x20/4])   ///< 候选条数
#define REG_CAND_RD   (REG_BASE[0x24/4])   ///< 读即弹出
#define REG_MAX_SCORE (REG_BASE[0x28/4])   ///< 本模板最高得分

// ---- 级 0 线性内存寻址（与 4.2.6 节 linearize 同构，T=T0，x/y 为级 0 像素坐标） ----
/**
 * @brief   级 0 线性内存指针计算
 * @param[in]  base  级 0 线性内存基址
 * @param[in]  ori   方向（0~7）
 * @param[in]  x     级 0 像素 x
 * @param[in]  y     级 0 像素 y
 * @param[in]  wc0   级 0 每行单元数
 * @return  指向 (ori,x,y) 处响应值的指针
 */
static inline const uint8_t *lm0_ptr(const uint8_t *base, int ori,
                                     int x, int y, int wc0)
{
	int block = (y % T0) * T0 + (x % T0);      // 块号
	int cell  = (y / T0) * wc0 + (x / T0);     // 单元号
	return base + ((uint64_t)ori * T0 * T0 + block) * (uint64_t)wc0 * wc0 + cell;
}

// ---- 级 0 细化：16×16 单元窗口重算得分，返回窗口最大值位置（相对窗口左上角） ----
// 语义与 C++ similarityLocal 一致：dst[dy][dx] = 模板左上角位于
// (cx-8+dx, cy-8+dy) 单元处的得分（特征像素坐标 + 候选左上角像素坐标）
/**
 * @brief   级 0 细化窗口重算得分
 * @details 在 (cx,cy) 周围 16×16 单元窗口内重算模板相似度，返回最大值相对位置。
 * @param[in]  lm0     级 0 线性内存基址
 * @param[in]  feats   特征数组
 * @param[in]  nf      特征数
 * @param[in]  cx      细化窗口中心 x（级 0 单元）
 * @param[in]  cy      细化窗口中心 y（级 0 单元）
 * @param[in]  wc0     级 0 每行单元数
 * @param[in]  cells0  级 0 单元总数（未使用，保留接口一致性）
 * @param[out] best_dx 窗口内最佳 x 偏移
 * @param[out] best_dy 窗口内最佳 y 偏移
 * @return  归一化最高得分（0~100）
 */
static float similarity_local(const uint8_t *lm0, const feat_t *feats, int nf,
                              int cx, int cy, int wc0, int cells0,
                              int *best_dx, int *best_dy)
{
	uint8_t acc[16][16];
	memset(acc, 0, sizeof(acc));
	for (int f = 0; f < nf; f++) {
		int fx = feats[f].x, fy = feats[f].y, ori = feats[f].label & 7;
		for (int dy = 0; dy < 16; dy++) {
			for (int dx = 0; dx < 16; dx++) {
				// 特征落在图像上的像素坐标 = 模板内坐标 + 候选窗口左上角
				int px = fx + (cx - 8 + dx) * T0;
				int py = fy + (cy - 8 + dy) * T0;
				if (px < 0 || py < 0 || px >= wc0 * T0 || py >= wc0 * T0) continue;
				acc[dy][dx] += *lm0_ptr(lm0, ori, px, py, wc0);
			}
		}
	}
	uint8_t mx = 0; int bdx = 0, bdy = 0;
	for (int dy = 0; dy < 16; dy++)
		for (int dx = 0; dx < 16; dx++)
			if (acc[dy][dx] > mx) {
				mx = acc[dy][dx];
				bdx = dx; bdy = dy;
			}
	*best_dx = bdx; *best_dy = bdy;
	return (float)mx * 100.0f / (float)(4 * nf);
}

// ---- IoU（粗层单元坐标系，候选框 = 模板在级 1 的跨度） ----
/**
 * @brief   两候选框 IoU 计算
 * @param[in] a   候选框 a（级 1 单元坐标）
 * @param[in] b   候选框 b（级 1 单元坐标）
 * @param[in] w1  模板级 1 宽度（单元）
 * @param[in] h1  模板级 1 高度（单元）
 * @return  IoU 重叠比（0~1）
 */
static float iou(const cand_t *a, const cand_t *b, int w1, int h1)
{
	int x0 = a->x, y0 = a->y, x1 = b->x, y1 = b->y;
	int ix = (x0 < x1 ? x0 : x1), iy = (y0 < y1 ? y0 : y1);
	int iw = (x0 < x1 ? x0 + w1 - x1 : x1 + w1 - x0);
	int ih = (y0 < y1 ? y0 + h1 - y1 : y1 + h1 - y0);
	if (iw <= 0 || ih <= 0) return 0.0f;
	float inter = (float)iw * (float)ih;
	float uni = 2.0f * (float)w1 * (float)h1 - inter;
	return inter / uni;
}

// ---- 主入口：候选读取→归一化→NMS→级 0 细化→排序输出 ----
/**
 * @brief   NMS 后处理主入口
 * @param[in]      cands    候选数组（调用方已通过 REG_CAND_CNT/REG_CAND_RD 装载）
 * @param[in]      cand_cnt 候选条数
 * @param[in]      feats    特征数组
 * @param[in]      nf       特征数
 * @param[in]      lm0      级 0 线性内存基址
 * @param[in]      wc0      级 0 每行单元数
 * @param[in]      cells0   级 0 单元总数
 * @param[in]      w1       模板级 1 宽度（单元）
 * @param[in]      h1       模板级 1 高度（单元）
 * @param[out]     out      输出结果数组
 * @param[in]      out_max  输出数组容量
 * @return  实际输出结果数（≤ out_max）
 */
int sbm_alg13_nms(cand_t *cands, int cand_cnt, const feat_t *feats, int nf,
                  const uint8_t *lm0, int wc0, int cells0,
                  int w1, int h1, result_t *out, int out_max)
{
	// 第一步：读候选（调用方已通过 REG_CAND_CNT/REG_CAND_RD 装载 cands[]）
	// 第二步：归一化 + 低分过滤
	float score[CAND_MAX]; int keep[CAND_MAX], nkeep = 0;
	for (int i = 0; i < cand_cnt; i++) {
		score[i] = (float)cands[i].score_raw * 100.0f / (float)(4 * nf);
		if (score[i] >= MIN_SCORE) keep[nkeep++] = i;
	}
	// 第三步：按得分降序插入排序 + IoU NMS
	int order[CAND_MAX], nord = 0;
	for (int i = 0; i < nkeep; i++) {                 // 插入排序（候选量级小）
		int j = nord;
		while (j > 0 && score[keep[i]] > score[order[j - 1]]) {
			order[j] = order[j - 1]; j--;
		}
		order[j] = keep[i]; nord++;
	}
	int surv[CAND_MAX], nsurv = 0;
	for (int i = 0; i < nord; i++) {
		int suppressed = 0;
		for (int j = 0; j < nsurv; j++)
			if (iou(&cands[order[i]], &cands[surv[j]], w1, h1) >= IOU_TH) {
				suppressed = 1; break;
			}
		if (!suppressed) surv[nsurv++] = order[i];
		if (nsurv >= TOP_K) break;                    // 最多 TOP_K 进细化
	}
	// 第四步：级 0 细化（坐标链与 C++ matchClass 一致）
	//   级 1 单元 → 级 1 像素（c*T1+OFFSET1）→ 级 0 像素（×2+1）→ 级 0 单元（/T0）
	int n = 0;
	for (int i = 0; i < nsurv && n < out_max; i++) {
		int bdx, bdy;
		int x1 = cands[surv[i]].x * T1 + OFFSET1;     // 级 1 像素坐标
		int y1 = cands[surv[i]].y * T1 + OFFSET1;
		int cx = (x1 * 2 + 1) / T0;                   // 级 0 单元坐标（细化中心）
		int cy = (y1 * 2 + 1) / T0;
		float s = similarity_local(lm0, feats, nf, cx, cy, wc0, cells0, &bdx, &bdy);
		out[n].score = s;
		out[n].cx = (cx - 8 + bdx) * T0 + OFFSET0;    // 级 0 单元→原图像素
		out[n].cy = (cy - 8 + bdy) * T0 + OFFSET0;
		out[n].w  = w1 * T1 * 2; out[n].h = h1 * T1 * 2;   // 模板跨度换算原图像素
		out[n].theta = 0.0f;                          // 单角度匹配，旋转由多模板覆盖
		n++;
	}
	// 第五步：排序输出 Top-N（得分降序，简单插入排序）
	for (int i = 1; i < n; i++) {
		result_t t = out[i]; int j = i - 1;
		while (j >= 0 && out[j].score < t.score) { out[j + 1] = out[j]; j--; }
		out[j + 1] = t;
	}
	return n < out_max ? n : out_max;
}
