// ============================================================================
// @file      sbm_gauss_h.v
// @brief     7 抽头水平高斯滤波
// @details
//   核系数 [2, 7, 14, 18, 14, 7, 2] / 64（σ ≈ 1.37），与 OpenCV
//   GaussianBlur(7×7, σ=0) 自动推导核完全一致（修订：原二项核
//   [1,6,15,20,15,6,1]/64 尾系数偏差 1/32，已对齐 OpenCV）。
//
//   边界：BORDER_REPLICATE（与 line2Dup.cpp quantizedOrientations 一致）
//     - 左边界：行首指示 i_row_start=1 时，7 个抽头整体装载行首像素。
//     - 右边界：行末后本模块内部自动补 3 拍、以末像素持续右移复制。
//   中心对齐：因果窗口输出 = 中心窗口滞后 3 列，故每行丢弃前 3 个因果输出，
//     每行恰好输出 IMG_W 个有效像素（列 0..IMG_W-1），无气泡。
//   延迟：2 拍（移位寄存器 1 拍 + 加法树 1 拍），吞吐 1 像素/时钟。
//   舍入：四舍五入，(sum + 32) >> 6。全部移位实现，0 DSP。
// @param[in]  clk        时钟
// @param[in]  rst_n      低有效异步复位
// @param[in]  i_valid    输入有效（每行 IMG_W 个像素）
// @param[in]  i_row_start 行首指示（本拍为行首像素，与 i_valid 对齐）
// @param[in]  i_data     输入像素（8bit）
// @param[out] o_valid    输出有效（每行 IMG_W 个，与 o_data 对齐）
// @param[out] o_row_start 输出行首标记（与 o_valid 对齐）
// @param[out] o_data     输出像素（8bit）
// @author    WorkBuddy
// @date      2026-08-14
// ============================================================================
`include "sbm_geometry.vh"
module sbm_gauss_h #(
	parameter IMG_W = `SBM_L0_W    ///< 图像宽度（级 0, 16T 对齐, 由 sbm_geometry.vh 派生）
)(
	input  wire       clk,
	input  wire       rst_n,
	input  wire       i_valid,       ///< 输入有效（每行 IMG_W 个像素）
	input  wire       i_row_start,   ///< 行首指示（本拍为行首像素，与 i_valid 对齐）
	input  wire [7:0] i_data,
	output wire       o_valid,       ///< 输出有效（每行 IMG_W 个，与 o_data 对齐）
	output wire       o_row_start,   ///< 输出行首标记（与 o_valid 对齐）
	output wire [7:0] o_data
);

	// ==================== 输入行内计数与右边界复制（行尾补 3 拍末像素） ====================
	reg  [12:0] pix_cnt;             ///< 0..IMG_W-1
	reg  [1:0]  flush_c;             ///< 行尾复制剩余拍（0..3）
	reg  [7:0]  last_pix;            ///< 行末像素（右边界复制源）
	wire        w_in_valid = i_valid || (flush_c != 2'd0);
	wire [7:0]  w_in_data  = (flush_c != 2'd0) ? last_pix : i_data;

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			pix_cnt <= 13'd0; flush_c <= 2'd0; last_pix <= 8'd0;
		end else begin
			if (i_valid) begin
				last_pix <= i_data;
				if (pix_cnt == IMG_W-1) begin
					pix_cnt <= 13'd0;
					flush_c <= 2'd3;             // 行尾补 3 拍复制
				end else
					pix_cnt <= pix_cnt + 13'd1;
			end else if (flush_c != 2'd0)
				flush_c <= flush_c - 2'd1;       // 复制期逐拍递减
		end
	end

	// ==================== 7 级抽头（行首整体装载 = 左边界复制） ====================
	reg [7:0] tap [0:6];
	integer   i;
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			for (i = 0; i < 7; i = i + 1) tap[i] <= 8'd0;
		end else if (w_in_valid) begin
			if (i_row_start) begin      // 左边界复制：整体装载
				for (i = 0; i < 7; i = i + 1) tap[i] <= w_in_data;
			end else begin              // 正常右移
				for (i = 0; i < 6; i = i + 1) tap[i] <= tap[i+1];
				tap[6] <= w_in_data;
			end
		end
	end

	// ==================== 加法树：[2,7,14,18,14,7,2]/64（OpenCV 口径） ====================
	// 2=2, 7=8-1, 14=16-2, 18=16+2，全部移位实现，0 DSP
	wire [9:0]  w_t06 = tap[0] + tap[6];                        // 0..510
	wire [9:0]  w_t15 = tap[1] + tap[5];
	wire [9:0]  w_t24 = tap[2] + tap[4];
	wire [10:0] w_2t  = w_t06 << 1;                            // *2
	wire [12:0] w_7t  = (w_t15 << 3) - w_t15;                  // *7
	wire [13:0] w_14t = (w_t24 << 4) - (w_t24 << 1);           // *14
	wire [13:0] w_18t = (tap[3] << 4) + (tap[3] << 1);        // *18
	wire [13:0] w_sum = w_2t + w_7t + w_14t + w_18t;          // 14bit, 最大 16320

	reg  [13:0] sum;                       // 2 拍流水：数据延迟 2 拍
	reg         vld_d1, vld_d2;            // 有效延迟 2 拍（与数据严格对齐）
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			sum <= 14'd0; vld_d1 <= 1'b0; vld_d2 <= 1'b0;
		end else begin
			sum    <= w_sum;
			vld_d1 <= w_in_valid;
			vld_d2 <= vld_d1;
		end
	end

	// ==================== 行内周期计数（含复制拍）：0..IMG_W+2，与 vld_d2 同延迟 ====================
	reg  [12:0] w_cycle;                   ///< 输入周期（0..IMG_W+2）
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) w_cycle <= 13'd0;
		else if (i_row_start) w_cycle <= 13'd0;
		else if (w_in_valid) w_cycle <= w_cycle + 13'd1;
	end
	reg  [12:0] w_cycle_d1, w_cycle_d2;
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin w_cycle_d1 <= 13'd0; w_cycle_d2 <= 13'd0; end
		else begin w_cycle_d1 <= w_cycle; w_cycle_d2 <= w_cycle_d1; end
	end

	// ==================== 输出：丢弃每行前 3 个因果窗口结果（列 -3..-1），发射列 0..IMG_W-1 ====================
	assign o_valid     = vld_d2 && (w_cycle_d2 >= 13'd3) && (w_cycle_d2 < IMG_W + 13'd3);
	assign o_row_start = o_valid && (w_cycle_d2 == 13'd3);
	assign o_data      = (sum + 14'd32) >> 6;   // 四舍五入到 8bit，无溢出

endmodule
